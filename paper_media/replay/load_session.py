"""Adapters: session logs -> one unified, time sorted event list.

Every source has its own adapter so a field rename touches one function.
Sources and the fields each adapter reads (verified against the writers):

  events_<server>.jsonl          robot_backend/events/bus.py (EventBus.publish),
    (scene, motion, gripper,      envelope from events/schema.py:
     watchdog, executor_monitor)  event_type, wall_time_iso, payload, seq
  mcp_calls_<role>.jsonl          mcp_servers/role_context.py (log_tool_call):
                                  wall_time_iso (call START), elapsed_sec,
                                  server, tool, pid, arguments, ok, result
  orchestrator/*.jsonl            cell_orchestrator/node.py (_log):
                                  type, wall_ns (grant time), step_id, phase, waited_sec
  frames/look_*.png, exec_look_*  look() files, located through the look event's
                                  payload.paths.rgb and payload.wall_ns
  frames/<label>_<a>_<b>/capture.json   capture_frames(): frames[].file, frames[].wall_ns
  frames/<camera>/frames.jsonl    paper_media/recorder/frame_recorder.py
  bag/ (rosbag2, mcap)            /joint_states, /tf, /tf_static; time = the bag's
                                  receive timestamp (system clock ns), which the
                                  cell's manifest designates as the correlation field

Clock rule: cross-process ordering uses wall clock only. wall_time_iso is used
where a record has it. Fields that are wall clock epoch nanoseconds (orchestrator
wall_ns, look payload wall_ns, capture frames wall_ns, bag receive time) are
converted to the same UTC ISO form. The envelope timestamp_ns is monotonic and
is never read. Message header stamps are simulation time and are never read.
"""
from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Optional

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
RG2_CALIBRATION_CSV = REPO_ROOT / "ros2_ws" / "src" / "isaac_trajectory_bridge" / "config" / "rg2_calibration_table.csv"
GRIPPER_JOINT = "finger_joint"   # the URDF joint the effort clauses are defined on (contracts/loader.py)

EXECUTOR_SERVERS = frozenset({"motion", "scene", "gripper", "executor_monitor"})
WATCHDOG_SERVERS = frozenset({"watchdog"})


# ---------------------------------------------------------------------------
# time helpers
# ---------------------------------------------------------------------------

def iso_to_epoch(iso: str) -> float:
    return datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp()


def epoch_to_iso(s: float) -> str:
    return datetime.fromtimestamp(s, tz=timezone.utc).isoformat()


def wall_ns_to_iso(ns: int) -> str:
    """For fields that are WALL CLOCK epoch nanoseconds only (never timestamp_ns)."""
    return epoch_to_iso(int(ns) / 1e9)


def short(text: Any, n: int = 80) -> str:
    s = " ".join(str(text).split())
    return s if len(s) <= n else s[: n - 3].rstrip() + "..."


# ---------------------------------------------------------------------------
# model
# ---------------------------------------------------------------------------

@dataclass
class Event:
    t: float                     # seconds since session start
    wall_time_iso: str
    lane: str                    # executor | bus | monitors | watchdog
    kind: str
    label: str
    detail: str
    source_file: str
    raw: Any = field(repr=False, default=None)
    image: Optional[str] = None  # look_frame: path of the captured image
    end_t: Optional[float] = None  # tool calls: return time

    @property
    def epoch(self) -> float:
        return iso_to_epoch(self.wall_time_iso)


@dataclass
class ParseIssue:
    source_file: str
    line: Optional[int]
    reason: str


@dataclass
class FrameIndex:
    """Images ordered by wall clock, for 'nearest earlier frame' lookup."""
    epochs: list[float]
    paths: list[str]
    source: str                  # recorder:<camera> | evidence

    def at(self, epoch: float) -> Optional[str]:
        i = bisect.bisect_right(self.epochs, epoch) - 1
        return self.paths[i] if i >= 0 else None

    def __len__(self) -> int:
        return len(self.paths)


@dataclass
class Signal:
    name: str
    t: list[float]               # session seconds
    v: list[float]
    unit: str
    source: str


@dataclass
class Session:
    dir: Path
    t0_epoch: float
    t0_source: str
    events: list[Event]
    issues: list[ParseIssue]
    agent_config: dict[str, dict]
    manifest: Optional[dict]
    registry_provenance: dict[str, dict]
    evidence_frames: FrameIndex
    filtered_records: dict[str, int]

    def t(self, epoch: float) -> float:
        return epoch - self.t0_epoch

    def epoch(self, t: float) -> float:
        return self.t0_epoch + t

    @property
    def duration(self) -> float:
        return max((e.end_t or e.t) for e in self.events) if self.events else 0.0

    def first(self, kind: str, after: float = -math.inf) -> Optional[Event]:
        return next((e for e in self.events if e.kind == kind and e.t >= after), None)


# ---------------------------------------------------------------------------
# readers
# ---------------------------------------------------------------------------

def read_jsonl(path: Path, issues: list[ParseIssue]) -> list[tuple[int, dict]]:
    out: list[tuple[int, dict]] = []
    if not path.exists():
        return out
    with path.open(encoding="utf-8") as fh:
        for i, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError as exc:
                issues.append(ParseIssue(path.name, i, f"bad JSON: {exc}"))
                continue
            if not isinstance(rec, dict):
                issues.append(ParseIssue(path.name, i, "record is not an object"))
                continue
            out.append((i, rec))
    return out


# A draft event: (iso, lane, kind, label, detail, extras)
Draft = tuple[str, str, str, str, str, dict]


# ---- adapter: events_<server>.jsonl envelopes ------------------------------

def _verdict_label(verdict: Any, scope: str) -> str:
    v = str(verdict or "").lower()
    word = {"conformant": "PASS", "non_conformant": "FAIL", "inconclusive": "INCONCLUSIVE"}.get(v, v.upper() or "?")
    return f"{word} {scope}".strip()


def adapt_envelope(rec: dict, source_file: str) -> list[Draft]:
    """One /watchdog/events envelope -> zero or more draft events."""
    et = rec.get("event_type")
    iso = rec.get("wall_time_iso")
    p = rec.get("payload") if isinstance(rec.get("payload"), dict) else {}
    if not et or not iso:
        raise ValueError("envelope without event_type or wall_time_iso")
    agent_lane = "executor" if source_file in ("events_executor_monitor.jsonl", "events_scene.jsonl",
                                               "events_motion.jsonl", "events_gripper.jsonl") else "watchdog"
    out: list[Draft] = []
    if et == "begin_step":
        sid = p.get("step_id", "?")
        objs = ", ".join(p.get("object_ids") or [])
        out.append((iso, "executor", "step_start", f"begin {sid}", f"Step {sid} announced. Objects: {objs or 'none'}.", {}))
        out.append((iso, "bus", "begin_step", f"begin_step {sid}", f"begin_step {sid}", {}))
    elif et == "end_step":
        sid, outc = p.get("step_id", "?"), p.get("claimed_outcome", "?")
        detail = json.dumps(p.get("claim_detail") or {}, default=str)
        out.append((iso, "executor", "step_claim", f"{sid}: {outc}", f"Executor claims {outc} for {sid}. {short(detail, 220)}", {}))
        out.append((iso, "bus", "end_step", f"end_step {sid}", f"end_step {sid} claimed {outc}", {}))
    elif et == "phase":
        out.append((iso, "bus", "phase", f"{p.get('phase', '?')} {p.get('step_id', '')}".strip(), json.dumps(p, default=str), {}))
    elif et == "run_ended":
        st = p.get("status", "?")
        out.append((iso, "executor", "run_end", f"run ended: {st}", short(p.get("reason", ""), 400), {}))
        out.append((iso, "bus", "run_ended", f"run_ended {st}", short(p.get("reason", ""), 200), {}))
    elif et in ("skill_start", "skill_end"):
        name = p.get("skill_name", "?")
        extra = "" if et == "skill_start" else f" ok={p.get('ok')}"
        out.append((iso, "bus", et, f"{et.replace('_', ' ')} {name}", f"{et} {name}{extra}", {}))
    elif et == "watchdog_ready":
        scope = p.get("scope", "?")
        tgt = p.get("step_id") or ""
        ph = p.get("phase") or ""
        out.append((iso, "bus", "ready", f"ready {scope} {tgt} {ph}".strip(), json.dumps(p, default=str), {}))
    elif et == "monitor_armed":
        family = p.get("monitor_type") or "?"
        mid = p.get("monitor_id", "?")
        name = mid if family == "authored" else family
        obj = p.get("object_id") or "scene"
        thr = p.get("threshold")
        params = p.get("params") or {}
        detail = (f"Armed {mid} ({family}) on {obj}. Threshold {thr} {p.get('comparison', '')}, "
                  f"source {p.get('threshold_source', '?')}. Purpose {p.get('purpose', 'clause')}. "
                  f"Bound parameters: {short(json.dumps(params, default=str), 300)}")
        out.append((iso, "monitors", "arm", f"arm {name} {obj}", detail, {}))
    elif et == "monitor_fire":
        mid, obj = p.get("monitor_id", "?"), p.get("object_id") or "scene"
        val, thr = p.get("triggering_value"), p.get("threshold")
        detail = (f"Monitor {mid} fired on {obj}: value {val} vs threshold {thr}. "
                  f"Purpose {p.get('purpose', 'clause')}. Action {p.get('action_taken')}. "
                  f"trigger_wall_ns {p.get('trigger_wall_ns')}")
        out.append((iso, "monitors", "fire", f"FIRE {mid} {obj}", detail, {}))
        if p.get("halt_result") is not None or str(p.get("action_taken", "")).lower() == "halt":
            out.append((iso, "watchdog", "halt", f"halt after {mid}", json.dumps(p.get("halt_result"), default=str), {}))
    elif et == "monitor_disarmed":
        out.append((iso, "monitors", "disarm", f"disarm {p.get('monitor_id', '?')}", f"reason {p.get('reason')}", {}))
    elif et == "look":
        role = p.get("role") or ("executor" if agent_lane == "executor" else "watchdog")
        lane = "executor" if role == "executor" else "watchdog"
        out.append((iso, lane, "look", f"look() {p.get('look_id', '')}".strip(), short(p.get("reason", ""), 400), {}))
        rgb = (p.get("paths") or {}).get("rgb")
        if rgb:
            fiso = wall_ns_to_iso(p["wall_ns"]) if p.get("wall_ns") else iso
            out.append((fiso, lane, "look_frame", f"frame {p.get('look_id', '')}".strip(),
                        short(p.get("reason", ""), 200), {"image_ref": rgb}))
    elif et in ("verdict", "run_verdict"):
        scope = p.get("step_id") or ("run" if et == "run_verdict" else "")
        out.append((iso, agent_lane, et, _verdict_label(p.get("verdict"), scope),
                    short(p.get("explanation", ""), 1200), {}))
    elif et == "observation":
        out.append((iso, agent_lane, "observation", "observation", short(p.get("summary", ""), 600), {}))
    elif et == "frames_captured":
        out.append((iso, agent_lane, "frames_captured", f"captured {p.get('frame_count', '?')} frames",
                    short(p.get("label", ""), 200), {}))
    else:
        out.append((iso, "bus", et, et, short(json.dumps(p, default=str), 200), {}))
    return out


# ---- adapter: mcp_calls_<role>.jsonl ---------------------------------------

_WAKE_REASON = {"begin_step": "arming", "monitor_fire": "fire", "end_step": "end_step",
                "timeout": "timeout", "terminated": "terminated"}


def filter_calls(recs: list[tuple[int, dict]], servers: frozenset[str]) -> tuple[list[tuple[int, dict]], int]:
    """Keep the role's own servers and, per server, that server's modal pid.

    Some executor logs contain records from another run's watchdog server
    (different pid, minutes to days later). Both filters are needed.
    """
    by_server: dict[str, Counter] = defaultdict(Counter)
    for _, r in recs:
        if r.get("server") in servers:
            by_server[r["server"]][r.get("pid")] += 1
    modal = {s: c.most_common(1)[0][0] for s, c in by_server.items()}
    kept = [(i, r) for i, r in recs if r.get("server") in servers and r.get("pid") == modal.get(r.get("server"))]
    return kept, len(recs) - len(kept)


def adapt_mcp_call(rec: dict, role: str) -> list[Draft]:
    start_iso = rec.get("wall_time_iso")
    if not start_iso:
        raise ValueError("call record without wall_time_iso")
    tool = rec.get("tool", "?")
    elapsed = float(rec.get("elapsed_sec") or 0.0)
    end_iso = epoch_to_iso(iso_to_epoch(start_iso) + elapsed)
    result = rec.get("result") if isinstance(rec.get("result"), dict) else {}
    if role == "watchdog" and tool == "wait_for_event":
        wake = str(result.get("wake") or ("error" if rec.get("ok") is False else "?"))
        reason = _WAKE_REASON.get(wake, wake)
        detail = f"wait_for_event returned after {elapsed:.2f} s, wake={wake}"
        if wake == "monitor_fire" and isinstance(result.get("fire"), dict):
            f = result["fire"]
            detail += f" ({f.get('monitor_id')} on {f.get('object_id') or 'scene'})"
        elif isinstance(result.get("event"), dict):
            env = result["event"].get("envelope") or {}
            pl = env.get("payload") or {}
            if pl.get("step_id"):
                detail += f" (step {pl.get('step_id')})"
        return [(end_iso, "watchdog", "wake", f"wake: {reason}", detail, {"call_start_iso": start_iso})]
    args = rec.get("arguments")
    arg_text = short(json.dumps(args, default=str), 240) if args else ""
    ok = rec.get("ok")
    detail = f"{tool}({arg_text}) ok={ok} in {elapsed:.2f} s"
    if rec.get("error"):
        detail += f". Error: {short(rec.get('error'), 200)}"
    return [(start_iso, role, "tool_call", tool, detail, {"end_iso": end_iso, "server": rec.get("server")})]


# ---- adapter: orchestrator log ---------------------------------------------

def adapt_orchestrator(rec: dict) -> list[Draft]:
    typ = rec.get("type")
    ns = rec.get("wall_ns")
    if typ is None or ns is None:
        raise ValueError("orchestrator record without type or wall_ns")
    iso = wall_ns_to_iso(ns)
    step = rec.get("step_id") or ""
    phase = rec.get("phase") or ""
    tgt = f"{step} {phase}".strip()
    if typ == "gate_open":
        return [(iso, "bus", "gate_open", f"gate open {tgt}", f"Gate released after {rec.get('waited_sec', 0):.2f} s", {})]
    if typ == "gate_timeout":
        return [(iso, "bus", "timeout", f"gate timeout {tgt}",
                 f"Gate timed out after {rec.get('waited_sec', 0):.1f} s; executor proceeded unmonitored at this step.", {})]
    if typ in ("abort", "gate_aborted"):
        return [(iso, "watchdog", "halt", "run aborted", short(rec.get("detail") or rec.get("abort_detail") or "", 300), {})]
    if typ in ("orchestrator_start", "run_ready", "gate_unguarded_by_config", "gate_unguarded_latched", "latch_cleared"):
        return [(iso, "bus", typ, typ.replace("_", " "), json.dumps({k: v for k, v in rec.items() if k != "wall_ns"}, default=str), {})]
    return [(iso, "bus", typ, typ, json.dumps(rec, default=str), {})]


# ---------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------

def resolve_session_image(session_dir: Path, ref: str) -> Optional[Path]:
    """A look() path may be absolute from the recording machine; fall back to frames/<name>."""
    p = Path(ref)
    if p.is_file():
        return p
    q = session_dir / "frames" / p.name
    return q if q.is_file() else None


def load_recorder_frames(session_dir: Path, camera: str) -> FrameIndex:
    folder = session_dir / "frames" if camera in ("", "default") else session_dir / "frames" / camera
    issues: list[ParseIssue] = []
    rows = []
    for _, r in read_jsonl(folder / "frames.jsonl", issues):
        f = folder / str(r.get("file", ""))
        if r.get("wall_time_iso") and f.is_file():
            rows.append((iso_to_epoch(r["wall_time_iso"]), str(f)))
    rows.sort()
    return FrameIndex([a for a, _ in rows], [b for _, b in rows], f"recorder:{camera or 'default'}")


def load_evidence_frames(session_dir: Path, events: list[Event], issues: list[ParseIssue]) -> FrameIndex:
    """look() frames plus capture_frames() windows, both on the wall clock."""
    rows: list[tuple[float, str]] = []
    for e in events:
        if e.kind == "look_frame" and e.image:
            rows.append((e.epoch, e.image))
    frames_dir = session_dir / "frames"
    if frames_dir.is_dir():
        for cap in sorted(frames_dir.glob("*/capture.json")):
            try:
                data = json.loads(cap.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                issues.append(ParseIssue(str(cap.relative_to(session_dir)), None, f"capture.json unreadable: {exc}"))
                continue
            for fr in data.get("frames") or []:
                f = cap.parent / str(fr.get("file", ""))
                if fr.get("wall_ns") and f.is_file():
                    rows.append((int(fr["wall_ns"]) / 1e9, str(f)))
    rows.sort()
    return FrameIndex([a for a, _ in rows], [b for _, b in rows], "evidence")


# ---------------------------------------------------------------------------
# load
# ---------------------------------------------------------------------------

def _json_or_none(path: Path) -> Optional[dict]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def load_session(session_dir: Path | str) -> Session:
    sd = Path(session_dir).expanduser().resolve()
    if not sd.is_dir():
        raise FileNotFoundError(f"session directory not found: {sd}")
    issues: list[ParseIssue] = []
    drafts: list[tuple[Draft, str, dict]] = []   # (draft, source_file, raw)
    filtered: dict[str, int] = {}

    for path in sorted(sd.glob("events_*.jsonl")):
        for ln, rec in read_jsonl(path, issues):
            try:
                for d in adapt_envelope(rec, path.name):
                    drafts.append((d, path.name, rec))
            except (ValueError, KeyError, TypeError) as exc:
                issues.append(ParseIssue(path.name, ln, str(exc)))

    for role, servers in (("executor", EXECUTOR_SERVERS), ("watchdog", WATCHDOG_SERVERS)):
        path = sd / f"mcp_calls_{role}.jsonl"
        recs, dropped = filter_calls(read_jsonl(path, issues), servers)
        if dropped:
            filtered[path.name] = dropped
        for ln, rec in recs:
            try:
                for d in adapt_mcp_call(rec, role):
                    drafts.append((d, path.name, rec))
            except (ValueError, KeyError, TypeError) as exc:
                issues.append(ParseIssue(path.name, ln, str(exc)))

    orch_start: Optional[float] = None
    for path in sorted((sd / "orchestrator").glob("*.jsonl")) if (sd / "orchestrator").is_dir() else []:
        for ln, rec in read_jsonl(path, issues):
            try:
                if rec.get("type") == "orchestrator_start" and rec.get("wall_ns"):
                    s = int(rec["wall_ns"]) / 1e9
                    orch_start = s if orch_start is None else min(orch_start, s)
                for d in adapt_orchestrator(rec):
                    drafts.append((d, f"orchestrator/{path.name}", rec))
            except (ValueError, KeyError, TypeError) as exc:
                issues.append(ParseIssue(f"orchestrator/{path.name}", ln, str(exc)))

    if not drafts:
        raise ValueError(f"no parseable events in {sd}")
    epochs = [iso_to_epoch(d[0]) for d, _, _ in drafts]
    if orch_start is not None:
        t0, t0_src = orch_start, "orchestrator_start"
    else:
        t0, t0_src = min(epochs), "earliest_event"

    events: list[Event] = []
    for (iso, lane, kind, label, detail, extra), src, raw in drafts:
        ep = iso_to_epoch(iso)
        image = None
        if kind == "look_frame":
            img = resolve_session_image(sd, extra["image_ref"])
            if img is None:
                issues.append(ParseIssue(src, None, f"look frame not found on disk: {Path(extra['image_ref']).name}"))
                continue
            image = str(img)
        end_t = iso_to_epoch(extra["end_iso"]) - t0 if "end_iso" in extra else None
        events.append(Event(t=ep - t0, wall_time_iso=iso, lane=lane, kind=kind, label=label, detail=detail,
                            source_file=src, raw=raw, image=image, end_t=end_t))
    events.sort(key=lambda e: (e.t, e.lane, e.kind))

    # look frames on disk that no look event names
    named = {Path(e.image).name for e in events if e.image}
    for f in sorted((sd / "frames").glob("*look_*_rgb.png")) if (sd / "frames").is_dir() else []:
        if f.name not in named:
            issues.append(ParseIssue(f"frames/{f.name}", None, "look frame without a look event; no timestamp, skipped"))

    agent_config = {r: c for r in ("executor", "watchdog")
                    if (c := _json_or_none(sd / f"agent_config_{r}.json")) is not None}
    provenance = {r: c for r in ("executor", "watchdog")
                  if (c := _json_or_none(sd / f"registry_provenance_{r}.json")) is not None}
    session = Session(dir=sd, t0_epoch=t0, t0_source=t0_src, events=events, issues=issues,
                      agent_config=agent_config, manifest=_json_or_none(sd / "manifest.json"),
                      registry_provenance=provenance, evidence_frames=FrameIndex([], [], "evidence"),
                      filtered_records=filtered)
    session.evidence_frames = load_evidence_frames(sd, events, issues)
    return session


# ---------------------------------------------------------------------------
# continuous signals (rosbag2)
# ---------------------------------------------------------------------------

def _rg2_width_table() -> Optional[tuple[list[float], list[float]]]:
    """master_angle_rad -> true_width_m, open_to_close rows, monotone as the controller uses it."""
    if not RG2_CALIBRATION_CSV.exists():
        return None
    rows = []
    with RG2_CALIBRATION_CSV.open(newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if r.get("direction", "open_to_close") not in ("open_to_close", ""):
                continue
            try:
                a, w = float(r["master_angle_rad"]), float(r["true_width_m"])
            except (KeyError, ValueError):
                continue
            if math.isfinite(a) and math.isfinite(w) and w >= 0:
                rows.append((a, w))
    if len(rows) < 2:
        return None
    rows.sort()
    angles = [a for a, _ in rows]
    widths = [w for _, w in rows]
    for i in range(1, len(widths)):           # same smoothing as utils/rg2_geometry.py
        widths[i] = min(widths[i], widths[i - 1])
    return angles, widths


def _interp(x: float, xp: list[float], fp: list[float]) -> float:
    if x <= xp[0]:
        return fp[0]
    if x >= xp[-1]:
        return fp[-1]
    i = bisect.bisect_left(xp, x)
    x0, x1, y0, y1 = xp[i - 1], xp[i], fp[i - 1], fp[i]
    return y1 if x1 == x0 else y0 + (x - x0) * (y1 - y0) / (x1 - x0)


def _quat_rotate(q: tuple[float, float, float, float], v: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v); v' = v + w*t + cross(q.xyz, t)
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


class _TfTree:
    """Minimal TF lookup over recorded /tf and /tf_static, using latest-at-or-before per link."""

    def __init__(self) -> None:
        self.links: dict[str, list[tuple[float, str, tuple, tuple]]] = defaultdict(list)
        self.times: dict[str, list[float]] = {}
        self.static: dict[str, tuple[str, tuple, tuple]] = {}

    def add(self, epoch: float, parent: str, child: str, tr: tuple, q: tuple, is_static: bool) -> None:
        if is_static:
            self.static[child] = (parent, tr, q)
        else:
            self.links[child].append((epoch, parent, tr, q))

    def finalize(self) -> None:
        for child, v in self.links.items():
            v.sort(key=lambda x: x[0])
            self.times[child] = [x[0] for x in v]

    def _link(self, child: str, epoch: float):
        if child in self.static:
            return self.static[child]
        seq = self.links.get(child)
        if not seq:
            return None
        i = bisect.bisect_right(self.times[child], epoch) - 1
        return None if i < 0 else seq[i][1:]

    def position(self, frame: str, epoch: float) -> Optional[tuple[str, tuple[float, float, float]]]:
        """(root frame, position of `frame` in that root)."""
        p = (0.0, 0.0, 0.0)
        chain = []
        cur = frame
        for _ in range(64):
            link = self._link(cur, epoch)
            if link is None:
                break
            chain.append(link)
            cur = link[0]
        # chain[0] is frame's own link; walk upward: p_parent = R p_child + t
        for parent, tr, q in chain:
            p = _quat_rotate(q, p)
            p = (p[0] + tr[0], p[1] + tr[1], p[2] + tr[2])
        return cur, p


def available_signal_names() -> list[str]:
    return ["effort_finger", "gripper_width_true", "joint_effort:<joint>", "joint_position:<joint>",
            "tf_dist:<frame_a>:<frame_b>"]


def load_signals(session: Session, names: Iterable[str], sample_hz: float = 20.0,
                 warn=lambda m: print(f"warning: {m}", file=sys.stderr)) -> dict[str, Signal]:
    """Continuous signals from the session bag, on session time. Unknown or absent signals warn and are skipped."""
    names = [n for n in names if n]
    if not names:
        return {}
    bag = session.dir / "bag"
    if not (bag / "metadata.yaml").exists():
        warn(f"no rosbag2 directory at {bag}; signals {names} unavailable")
        return {}
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        warn("rosbags is not installed (pip install rosbags); signals unavailable")
        return {}

    want_joints: dict[str, tuple[str, str]] = {}   # signal -> (joint, field)
    tf_pairs: dict[str, tuple[str, str]] = {}
    for n in names:
        if n == "effort_finger":
            want_joints[n] = (GRIPPER_JOINT, "effort")
        elif n == "gripper_width_true":
            want_joints[n] = (GRIPPER_JOINT, "position")
        elif n.startswith("joint_effort:"):
            want_joints[n] = (n.split(":", 1)[1], "effort")
        elif n.startswith("joint_position:"):
            want_joints[n] = (n.split(":", 1)[1], "position")
        elif n.startswith("tf_dist:") and n.count(":") == 2:
            _, a, b = n.split(":")
            tf_pairs[n] = (a, b)
        else:
            warn(f"unknown signal {n!r}; known: {available_signal_names()}")

    series: dict[str, tuple[list[float], list[float]]] = {n: ([], []) for n in want_joints}
    tf = _TfTree()
    topics = set()
    if want_joints:
        topics.add("/joint_states")
    if tf_pairs:
        topics |= {"/tf", "/tf_static"}
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        missing = topics - {c.topic for c in conns}
        if missing:
            warn(f"bag has no {sorted(missing)}")
        for conn, ts_ns, raw in reader.messages(connections=conns):
            epoch = ts_ns / 1e9   # bag receive time, system clock (the manifest's correlation field)
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == "/joint_states":
                names_list = list(msg.name)
                for sig, (joint, fld) in want_joints.items():
                    if joint in names_list:
                        arr = getattr(msg, fld)
                        i = names_list.index(joint)
                        if len(arr) > i:
                            series[sig][0].append(epoch - session.t0_epoch)
                            series[sig][1].append(float(arr[i]))
            else:
                for tr in msg.transforms:
                    t = tr.transform.translation
                    r = tr.transform.rotation
                    tf.add(epoch, tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"),
                           (t.x, t.y, t.z), (r.x, r.y, r.z, r.w), conn.topic == "/tf_static")
    out: dict[str, Signal] = {}
    for sig, (ts, vs) in series.items():
        if not ts:
            warn(f"signal {sig}: joint {want_joints[sig][0]!r} not found in /joint_states")
            continue
        unit = "effort" if want_joints[sig][1] == "effort" else "rad"
        src = f"/joint_states {want_joints[sig][0]}.{want_joints[sig][1]}"
        if sig == "gripper_width_true":
            table = _rg2_width_table()
            if table is None:
                warn(f"RG2 calibration table not found at {RG2_CALIBRATION_CSV}; gripper_width_true unavailable")
                continue
            vs = [_interp(v, table[0], table[1]) * 1000.0 for v in vs]
            unit, src = "mm", src + " via rg2_calibration_table.csv"
        out[sig] = Signal(sig, ts, vs, unit, src)
    if tf_pairs:
        tf.finalize()
        t_start, t_end = 0.0, session.duration
        n = max(2, int((t_end - t_start) * sample_hz))
        for sig, (a, b) in tf_pairs.items():
            ts, vs = [], []
            for k in range(n):
                t = t_start + (t_end - t_start) * k / (n - 1)
                pa, pb = tf.position(a, session.epoch(t)), tf.position(b, session.epoch(t))
                if pa and pb and pa[0] == pb[0] and pa[0] not in (a, b):
                    ts.append(t)
                    vs.append(1000.0 * math.dist(pa[1], pb[1]))
            if not ts:
                warn(f"signal {sig}: frames {a!r} and {b!r} share no TF root in the bag")
                continue
            out[sig] = Signal(sig, ts, vs, "mm", f"/tf distance {a} to {b}")
    return out


# ---------------------------------------------------------------------------
# story
# ---------------------------------------------------------------------------

STORY_DEFAULTS: dict[str, Any] = {
    "title": "", "subtitle": "", "camera": "main", "window": None,
    "speed": {"idle": 8.0, "event": 1.0, "event_pad_s": 1.5, "hold_s": 2.0},
    "key_events": ["arm", "fire", "verdict", "halt", "look_frame"],
    "prompt_excerpts": [], "captions": [], "signals": [], "thresholds": [],
    "figure_frames_at": None, "executor_tools_hide": [], "registry_paths": {},
    "run_id": None, "scenario": None, "fault_class": None, "assembly": None, "trial": None,
    # packaging (paper_media/outputs/<media_id>/)
    "media_id": None, "kind": None, "tags": [], "notes": "",
    "figure_frame_cameras": [], "video_camera": None,
}
_KNOWN_STORY_KEYS = set(STORY_DEFAULTS) | {"session_dir"}


@dataclass
class Story:
    path: Path
    session_dir: Path
    data: dict

    def __getattr__(self, name: str) -> Any:
        if name in ("path", "session_dir", "data"):
            raise AttributeError(name)
        return self.data.get(name)


def load_story(path: Path | str, warn=lambda m: print(f"warning: {m}", file=sys.stderr)) -> Story:
    p = Path(path).expanduser().resolve()
    raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    if "session_dir" not in raw:
        raise ValueError(f"{p}: story has no session_dir")
    for k in raw:
        if k not in _KNOWN_STORY_KEYS:
            warn(f"{p.name}: unknown story key {k!r} ignored")
    data = json.loads(json.dumps(STORY_DEFAULTS))
    for k, v in raw.items():
        if k == "speed" and isinstance(v, dict):
            data["speed"].update(v)
        else:
            data[k] = v
    sd = Path(str(raw["session_dir"])).expanduser()
    if not sd.is_absolute():
        sd = (p.parent / sd).resolve()
    return Story(path=p, session_dir=sd, data=data)


def visible_events(session: Session, story: Story) -> list[Event]:
    hide = set(story.executor_tools_hide or [])
    w = story.window
    out = []
    for e in session.events:
        if e.kind == "tool_call" and e.lane == "executor" and e.label in hide:
            continue
        if w and not (float(w[0]) <= e.t <= float(w[1])):
            continue
        out.append(e)
    return out


def anchor_time(session: Session, spec: dict) -> Optional[float]:
    """at_t (seconds), or at_event: the first event of that kind in any lane.

    The excerpt's own `lane` says where the card is SHOWN, not where the
    anchoring event lives (an arm event is in the monitors lane, its excerpt
    belongs to the WatchDog panel).
    """
    if spec.get("at_t") is not None:
        return float(spec["at_t"])
    kind = spec.get("at_event")
    if kind:
        e = session.first(kind)
        return e.t if e else None
    return None


# ---------------------------------------------------------------------------
# registry thresholds
# ---------------------------------------------------------------------------

def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _dig(d: Any, dotted: str) -> Any:
    for part in dotted.split("."):
        if not isinstance(d, dict) or part not in d:
            return None
        d = d[part]
    return d


def registry_for_role(session: Session, story: Story, role: str,
                      warn=lambda m: print(f"warning: {m}", file=sys.stderr)) -> Optional[dict]:
    """Registry the given role loaded in this run, or None.

    Order: session manifest.json (embedded registry, the cell's load), then
    registry_provenance_<role>.json (path + sha), then story registry_paths.
    A file whose sha differs from the one recorded for the run is rejected:
    its values would describe a different registry than the run used.
    """
    man = session.manifest
    prov = session.registry_provenance.get(role)
    if man and isinstance(man.get("registry"), dict) and prov is None:
        # The manifest embeds the registry the cell loaded. The WatchDog loads that
        # one. The executor does too unless its config records an override.
        override = (session.agent_config.get("executor") or {}).get("executor_registry_override")
        if role == "watchdog" or (role == "executor" and override is None):
            return man["registry"]
    if prov and prov.get("registry_path"):
        p = Path(prov["registry_path"]).expanduser()
        if not p.is_file():
            warn(f"{role} registry {p} recorded for this run no longer exists")
        elif prov.get("registry_sha256") and _sha256(p) != prov["registry_sha256"]:
            warn(f"{role} registry {p} has changed since the run (sha differs from registry_provenance_{role}.json); not used")
        else:
            return yaml.safe_load(p.read_text(encoding="utf-8"))
    sp = (story.registry_paths or {}).get(role)
    if sp:
        p = Path(str(sp)).expanduser()
        if p.is_file():
            warn(f"{role} registry taken from the story ({p}); its match to the run is NOT verified")
            return yaml.safe_load(p.read_text(encoding="utf-8"))
        warn(f"story registry_paths.{role} {p} does not exist")
    return None


def resolve_thresholds(session: Session, story: Story,
                       warn=lambda m: print(f"warning: {m}", file=sys.stderr)) -> list[dict]:
    """[{signal, value, label, source}] for every resolvable threshold line."""
    out = []
    for th in story.thresholds or []:
        sig, label = th.get("signal"), th.get("label", "")
        vf = th.get("value_from", th.get("value"))
        if isinstance(vf, (int, float)):
            out.append({"signal": sig, "value": float(vf), "label": label, "source": "story number"})
            continue
        role = {"watchdog_registry": "watchdog", "executor_registry": "executor"}.get(str(vf))
        value = None
        if vf == "armed_monitor":
            # the threshold the WatchDog bound when it armed this monitor (first arming)
            mid = th.get("monitor_id")
            for env in read_jsonl(Path(session.dir) / "events_watchdog.jsonl", []):
                p = env[1].get("payload") or {}
                if env[1].get("event_type") == "monitor_armed" and p.get("monitor_id") == mid \
                        and isinstance(p.get("threshold"), (int, float)):
                    value = float(p["threshold"])
                    src = p.get("threshold_source") or "?"
                    break
            if value is not None:
                out.append({"signal": sig, "value": value, "label": label,
                            "source": f"monitor_armed {mid} threshold ({src})"})
                continue
            warn(f"threshold {label!r}: no monitor_armed record for {mid!r}")
        elif role:
            reg = registry_for_role(session, story, role, warn)
            obj, fld = th.get("object"), th.get("field")
            if reg is not None and obj and fld:
                value = _dig(_dig(reg, "objects") or {}, f"{obj}.{fld}")
                if value is None:
                    warn(f"threshold {label!r}: objects.{obj}.{fld} not in the {role} registry")
            elif reg is not None:
                warn(f"threshold {label!r}: value_from {vf} needs 'object' and 'field' keys")
            if value is not None:
                out.append({"signal": sig, "value": float(value), "label": label, "source": f"{role} registry objects.{obj}.{fld}"})
                continue
        else:
            warn(f"threshold {label!r}: unknown value_from {vf!r}")
        fb = th.get("fallback_value")
        if isinstance(fb, (int, float)):
            warn(f"threshold {label!r}: registry value not resolvable, using story fallback_value {fb}")
            out.append({"signal": sig, "value": float(fb), "label": label, "source": "story fallback_value"})
        else:
            warn(f"threshold {label!r}: skipped (no registry value and no fallback_value)")
    return out


# ---------------------------------------------------------------------------
# run metadata (web export)
# ---------------------------------------------------------------------------

def run_metadata(session: Session, story: Story,
                 warn=lambda m: print(f"warning: {m}", file=sys.stderr)) -> dict:
    """scenario, fault class, assembly, trial: session manifest first, then the story.

    No session in this campaign has a manifest.json, so in practice the story
    supplies these. agent_config_watchdog.json's scenario is used only as a
    last resort for the scenario field.
    """
    man = session.manifest or {}
    meta = {}
    for key, man_key in (("scenario", "scenario"), ("fault_class", "fault_class"),
                         ("assembly", "assembly"), ("trial", "trial")):
        v = man.get(man_key) if man else None
        if v is None:
            v = getattr(story, key)
        meta[key] = v
    if meta["scenario"] is None:
        sc = (session.agent_config.get("watchdog") or {}).get("scenario")
        if sc:
            meta["scenario"] = str(sc).upper()
    if meta["scenario"] is not None:
        meta["scenario"] = str(meta["scenario"]).upper()
    for k, v in meta.items():
        if v is None and k != "fault_class":
            warn(f"run metadata {k!r} not found in the session or the story")
    return meta


# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------

def session_summary(session_dir: Path | str, camera: str = "main", out=sys.stdout) -> Session:
    s = load_session(session_dir)
    w = lambda *a: print(*a, file=out)  # noqa: E731
    w(f"session   {s.dir}")
    w(f"t0        {epoch_to_iso(s.t0_epoch)} (from {s.t0_source})")
    if s.events:
        w(f"span      {s.events[0].t:.2f} s to {max((e.end_t or e.t) for e in s.events):.2f} s, {len(s.events)} events")
    counts = Counter((e.lane, e.kind) for e in s.events)
    for lane in ("executor", "bus", "monitors", "watchdog"):
        kinds = sorted((k, n) for (ln, k), n in counts.items() if ln == lane)
        w(f"  {lane:<9} " + ", ".join(f"{k}={n}" for k, n in kinds))
    rec = load_recorder_frames(s.dir, camera)
    w(f"frames    recorder:{camera} {len(rec)} | evidence (look + capture_frames) {len(s.evidence_frames)}")
    w(f"manifest  {'present' if s.manifest else 'absent'} | registry provenance: {sorted(s.registry_provenance) or 'none'}")
    w(f"agents    " + ", ".join(f"{r}={c.get('model')}/{c.get('effort')}" for r, c in s.agent_config.items()))
    bag = s.dir / "bag" / "metadata.yaml"
    if bag.exists():
        info = (yaml.safe_load(bag.read_text(encoding="utf-8")) or {}).get("rosbag2_bagfile_information", {})
        topics = [t["topic_metadata"]["name"] for t in info.get("topics_with_message_count", [])]
        w(f"bag       {info.get('storage_identifier')} topics {topics}")
    else:
        w("bag       absent")
    if s.filtered_records:
        w(f"filtered  foreign-server or foreign-pid call records dropped: {s.filtered_records}")
    w(f"unparsed  {len(s.issues)}")
    for i in s.issues[:50]:
        w(f"  {i.source_file}:{i.line if i.line is not None else '-'}  {i.reason}")
    if len(s.issues) > 50:
        w(f"  ... {len(s.issues) - 50} more")
    return s
