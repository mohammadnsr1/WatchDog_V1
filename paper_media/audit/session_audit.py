#!/usr/bin/env python3
"""Read-only integrity audit of WatchDog session directories.

    python -m paper_media.audit.session_audit <sessions_root> [--out report.json] [--session <id>]

Standard library only. The bag checks use `rosbags` when it is importable and
are skipped with a warning otherwise. Nothing is written except the JSON report.

Every check yields a status: ok, warn (suspicious, results probably usable) or
corrupt (a recorded value or a score may be wrong). A session's status is the
worst of its checks. The JSON carries per-check details so every flag can be
traced to records.

Clock rule: ordering across processes uses wall clock only (wall_time_iso, or
fields that are wall clock epoch ns: orchestrator wall_ns, fire trigger_wall_ns,
look wall_ns, bag receive time). The envelope timestamp_ns is per process and
is never compared across files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

# ---------------------------------------------------------------------------
# expectations (from the code: mcp_servers/*, events/schema.py, cell_orchestrator/node.py)
# ---------------------------------------------------------------------------

ROLE_SERVERS = {
    "executor": {"motion", "scene", "gripper", "executor_monitor"},
    "watchdog": {"watchdog"},
}
# Tools that command the robot or declare executor progress. The WatchDog must never call any.
COMMAND_TOOLS = {
    "move_to_named_safe_joint_pose", "execute_approach_and_preposition", "move_grasp_center_to_pose",
    "move_tcp_to_pose", "lift", "retreat_tcp", "grasp", "release", "open_gripper", "set_gripper_width",
    "set_gripper_joint_position", "clear_octomap", "set_permitted_contacts", "reset_object_state",
    "recover_scene", "sync_scene", "begin_step", "end_step", "mark_phase", "end_run",
}
FIRE_REQUIRED = ("monitor_id", "trigger_wall_ns", "triggering_value", "threshold", "action_taken")
ARM_REQUIRED = ("monitor_id", "monitor_type")
EXEC_EVENT_FILES = ("events_scene.jsonl", "events_motion.jsonl", "events_gripper.jsonl")

STATUS_RANK = {"ok": 0, "skip": 0, "warn": 1, "corrupt": 2}

# Files the operator workflow legitimately writes after the run (labels, scorer output,
# leak audit, manual metadata copies). Anything else touched after run end is suspicious.
EXPECTED_POST_RUN_FILES = {
    "score.json", "score_s4.json", "observation_score.json", "intruder_leak_audit.json",
    "claude_cli_version.txt", "executor.settings.json", "watchdog.settings.json",
    "ground_truth_injections.jsonl", "operator_faults.jsonl",
}
EXPECTED_POST_RUN_GLOBS = ("gt_*.yaml", "*.yaml", "*.yml", "*.md", "*.txt")


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def parse_iso(s: Any) -> tuple[Optional[float], Optional[str]]:
    """(epoch seconds, problem). Problem is None for a valid UTC ISO 8601 timestamp."""
    if not isinstance(s, str) or not s:
        return None, "missing"
    try:
        dt = datetime.fromisoformat(s.replace("Z", "+00:00"))
    except ValueError:
        return None, "unparseable"
    if dt.tzinfo is None:
        return None, "no_timezone"
    if dt.utcoffset().total_seconds() != 0:
        return dt.timestamp(), "not_utc"
    return dt.timestamp(), None


def iso(epoch: Optional[float]) -> Optional[str]:
    return None if epoch is None else datetime.fromtimestamp(epoch, tz=timezone.utc).isoformat()


def read_jsonl(path: Path) -> tuple[list[tuple[int, dict]], list[str]]:
    recs, bad = [], []
    if not path.exists():
        return recs, bad
    with path.open(encoding="utf-8", errors="replace") as fh:
        for i, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError as exc:
                bad.append(f"line {i}: {exc.msg}")
                continue
            if isinstance(r, dict):
                recs.append((i, r))
            else:
                bad.append(f"line {i}: not an object")
    return recs, bad


def read_json(path: Path) -> Optional[dict]:
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
        return d if isinstance(d, dict) else None
    except (OSError, json.JSONDecodeError):
        return None


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def yaml_scalar_fields(text: str, wanted_leaf: str) -> dict[str, str]:
    """Minimal YAML reader: {dotted.path: value} for every `wanted_leaf: value` line.

    Indentation-based, mappings only, which is enough for registry and label files.
    It does not understand lists, anchors or flow style; values under a list are
    reported with the list's parent path.
    """
    out: dict[str, str] = {}
    stack: list[tuple[int, str]] = []
    for raw in text.splitlines():
        line = raw.split(" #", 1)[0].rstrip()
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip(" "))
        body = line.strip()
        if body.startswith("- "):
            continue
        m = re.match(r"^([A-Za-z0-9_.\-]+):\s*(.*)$", body)
        if not m:
            continue
        key, val = m.group(1), m.group(2).strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if val == "" or val in ("|", ">", ">-", "|-"):
            stack.append((indent, key))
            continue
        if key == wanted_leaf:
            out[".".join([k for _, k in stack] + [key])] = val.strip("'\"")
    return out


def label_fields(session_dir: Path) -> dict[str, Any]:
    """scenario / class / run_id from the session's own label file (gt_*.yaml), top-level keys only."""
    out: dict[str, Any] = {}
    labels = sorted(session_dir.glob("gt_*.yaml"))
    if not labels:
        return out
    out["label_file"] = labels[0].name
    if len(labels) > 1:
        out["label_files"] = [p.name for p in labels]
    for line in labels[0].read_text(encoding="utf-8", errors="replace").splitlines():
        m = re.match(r"^(scenario|class|run_id|task|expected_final_verdict):\s*(.+?)\s*(#.*)?$", line)
        if m:
            out[m.group(1)] = m.group(2).strip("'\"")
    return out


class Report:
    def __init__(self) -> None:
        self.checks: dict[str, dict] = {}

    def add(self, name: str, status: str, details: Optional[list] = None, **info: Any) -> None:
        details = details or []
        self.checks[name] = {"status": status, "count": len(details), "details": details[:200], **info}
        if len(details) > 200:
            self.checks[name]["details_truncated"] = len(details) - 200

    @property
    def status(self) -> str:
        worst = max((STATUS_RANK[c["status"]] for c in self.checks.values()), default=0)
        return {0: "clean", 1: "warnings", 2: "corrupted"}[worst]


# ---------------------------------------------------------------------------
# pass 1: dominant process ids per (session, log, server), for foreign-record matching
# ---------------------------------------------------------------------------

def dominant_pids(session_dir: Path) -> dict[tuple[str, str], int]:
    out = {}
    for role in ROLE_SERVERS:
        recs, _ = read_jsonl(session_dir / f"mcp_calls_{role}.jsonl")
        by: dict[str, Counter] = defaultdict(Counter)
        for _, r in recs:
            by[str(r.get("server"))][r.get("pid")] += 1
        for server, c in by.items():
            if server in ROLE_SERVERS[role]:
                out[(role, server)] = c.most_common(1)[0][0]
    return out


def build_pid_index(sessions: list[Path]) -> dict[int, list[dict]]:
    idx: dict[int, list[dict]] = defaultdict(list)
    for sd in sessions:
        for (role, server), pid in dominant_pids(sd).items():
            idx[pid].append({"session": sd.name, "role": role, "server": server})
    return idx


# ---------------------------------------------------------------------------
# pass 2: per-session audit
# ---------------------------------------------------------------------------

def audit_session(sd: Path, pid_index: dict[int, list[dict]], args) -> dict:
    rep = Report()
    sid = sd.name
    label = label_fields(sd)
    cfg = {r: read_json(sd / f"agent_config_{r}.json") for r in ("executor", "watchdog")}

    # ---- load -------------------------------------------------------------
    events: dict[str, list[tuple[int, dict]]] = {}
    bad_json: list[str] = []
    for p in sorted(sd.glob("events_*.jsonl")):
        recs, bad = read_jsonl(p)
        events[p.name] = recs
        bad_json += [f"{p.name} {b}" for b in bad]
    calls: dict[str, list[tuple[int, dict]]] = {}
    for role in ROLE_SERVERS:
        recs, bad = read_jsonl(sd / f"mcp_calls_{role}.jsonl")
        calls[role] = recs
        bad_json += [f"mcp_calls_{role}.jsonl {b}" for b in bad]
    orch: list[dict] = []
    orch_files = sorted((sd / "orchestrator").glob("*.jsonl")) if (sd / "orchestrator").is_dir() else []
    for p in orch_files:
        recs, bad = read_jsonl(p)
        orch += [r for _, r in recs]
        bad_json += [f"orchestrator/{p.name} {b}" for b in bad]
    rep.add("json_parse", "corrupt" if bad_json else "ok", bad_json)

    def env_of(fname: str, et: str) -> list[dict]:
        return [r for _, r in events.get(fname, []) if r.get("event_type") == et]

    scene = [r for _, r in events.get("events_scene.jsonl", [])]
    run_ended = [r for r in scene if r.get("event_type") == "run_ended"]
    orch_start = [r["wall_ns"] / 1e9 for r in orch if r.get("type") == "orchestrator_start" and r.get("wall_ns")]
    guard_modes = sorted({str(r.get("guard_mode")) for r in orch if r.get("type") == "orchestrator_start"})

    # ---- scenario detection (labels that exist, in order of authority) ----
    scen_sources = {}
    if label.get("scenario"):
        scen_sources["label_file"] = str(label["scenario"]).upper()
    if cfg["watchdog"] and cfg["watchdog"].get("scenario"):
        scen_sources["agent_config_watchdog"] = str(cfg["watchdog"]["scenario"]).upper()
    if cfg["executor"] and "s4" in str(cfg["executor"].get("mcp_config", "")).lower():
        scen_sources["executor_mcp_config"] = "S4"
    if guard_modes == ["unguarded"]:
        scen_sources["orchestrator_guard_mode"] = "S4"
    distinct = set(scen_sources.values())
    scenario = scen_sources.get("label_file") or (distinct.pop() if len(distinct) == 1 else None)
    is_s4 = scenario == "S4"
    cls = label.get("class")
    rep.add("scenario_labels", "warn" if len(set(scen_sources.values())) > 1 or scenario is None else "ok",
            [] if len(set(scen_sources.values())) <= 1 and scenario else [scen_sources], sources=scen_sources)

    # ---- expected files ---------------------------------------------------
    missing, present = [], {}
    expect = ["mcp_calls_executor.jsonl", "events_scene.jsonl", "events_motion.jsonl", "events_gripper.jsonl",
              "agent_config_executor.json"]
    expect += ["events_executor_monitor.jsonl"] if is_s4 else ["mcp_calls_watchdog.jsonl", "events_watchdog.jsonl",
                                                               "agent_config_watchdog.json"]
    for f in expect:
        (present.__setitem__(f, True) if (sd / f).exists() else missing.append(f))
    if not orch_files:
        missing.append("orchestrator/*.jsonl")
    if not (sd / "bag" / "metadata.yaml").exists():
        missing.append("bag/metadata.yaml")
    if not label:
        missing.append("gt_*.yaml (label)")
    optional = {
        "manifest.json": (sd / "manifest.json").exists(),
        "registry_provenance_executor.json": (sd / "registry_provenance_executor.json").exists(),
        "registry_provenance_watchdog.json": (sd / "registry_provenance_watchdog.json").exists(),
        "frames/": (sd / "frames").is_dir(),
        "claude_cli_version.txt": (sd / "claude_cli_version.txt").exists(),
    }
    rep.add("expected_files", "warn" if missing else "ok", missing, optional=optional)

    # ---- session span -----------------------------------------------------
    starts = list(orch_start)
    for r in ("executor", "watchdog"):
        if cfg[r] and cfg[r].get("launched_wall_time_iso"):
            e, _ = parse_iso(cfg[r]["launched_wall_time_iso"])
            if e:
                starts.append(e)
    ends = []
    for r in run_ended:
        e, _ = parse_iso(r.get("wall_time_iso"))
        if e:
            ends.append(e)
    verdict_file = "events_executor_monitor.jsonl" if is_s4 else "events_watchdog.jsonl"
    for r in env_of(verdict_file, "run_verdict"):
        e, _ = parse_iso(r.get("wall_time_iso"))
        if e:
            ends.append(e)
    span = (min(starts) - args.span_pad_s, max(ends) + args.span_pad_s) if starts and ends else None
    # The run itself, unpadded: first launch or orchestrator start, to the executor's run_ended.
    _re = [parse_iso(r.get("wall_time_iso"))[0] for r in run_ended]
    run_window = (min(starts), max(e for e in _re if e is not None)) if starts and any(e is not None for e in _re) else None

    # ---- time sanity ------------------------------------------------------
    bad_time, outside, disorder, gaps = [], [], [], []
    all_epochs: list[float] = []

    def check_series(name: str, pts: list[tuple[int, Optional[str], str]], order_tol: float) -> None:
        prev = None
        for ln, ts, kind in pts:
            e, prob = parse_iso(ts)
            if prob:
                bad_time.append(f"{name}:{ln} {kind} wall_time_iso {prob} ({ts!r})")
                if e is None:
                    continue
            all_epochs.append(e)
            if span and not (span[0] <= e <= span[1]):
                outside.append(f"{name}:{ln} {kind} at {ts}")
            if prev is not None and e < prev - order_tol:
                disorder.append(f"{name}:{ln} {kind} {prev - e:.3f} s before the previous record")
            prev = e if prev is None else max(prev, e)

    for fname, recs in events.items():
        check_series(fname, [(ln, r.get("wall_time_iso"), str(r.get("event_type"))) for ln, r in recs], args.order_tol_s)
    for role, recs in calls.items():
        # call records are appended on RETURN, stamped with the call START: order by end time
        pts = []
        for ln, r in recs:
            e, prob = parse_iso(r.get("wall_time_iso"))
            end = iso(e + float(r.get("elapsed_sec") or 0.0)) if e is not None and prob is None else r.get("wall_time_iso")
            pts.append((ln, end, f"{r.get('server')}.{r.get('tool')}"))
        check_series(f"mcp_calls_{role}.jsonl", pts, args.order_tol_s)
    all_epochs.sort()
    for a, b in zip(all_epochs, all_epochs[1:]):
        if b - a > args.gap_s and (not span or (span[0] <= a <= span[1])):
            gaps.append(f"{b - a:.1f} s gap after {iso(a)}")
    rep.add("time_format", "corrupt" if bad_time else "ok", bad_time)
    rep.add("time_outside_span", "warn" if outside else "ok", outside,
            span=[iso(span[0]), iso(span[1])] if span else None)
    rep.add("time_out_of_order", "warn" if disorder else "ok", disorder)
    rep.add("time_large_gaps", "warn" if gaps else "ok", gaps, threshold_s=args.gap_s)

    # ---- foreign records + role separation ----------------------------------
    foreign, role_viol = [], []
    for role, recs in calls.items():
        by: dict[str, Counter] = defaultdict(Counter)
        for _, r in recs:
            by[str(r.get("server"))][r.get("pid")] += 1
        dom = {s: c.most_common(1)[0][0] for s, c in by.items()}
        grouped: dict[tuple, list[int]] = defaultdict(list)
        for ln, r in recs:
            server, pid = str(r.get("server")), r.get("pid")
            if server not in ROLE_SERVERS[role]:
                grouped[("foreign_server", server, pid)].append(ln)
            elif pid != dom[server]:
                grouped[("foreign_pid", server, pid)].append(ln)
            if role == "watchdog" and r.get("tool") in COMMAND_TOOLS:
                role_viol.append(f"mcp_calls_watchdog.jsonl:{ln} WatchDog called command tool {r.get('tool')}")
        for (why, server, pid), lines in grouped.items():
            first = next(r for ln, r in recs if ln == lines[0])
            last = next(r for ln, r in recs if ln == lines[-1])
            matches = [m for m in pid_index.get(pid, []) if m["session"] != sid]
            entry = {"file": f"mcp_calls_{role}.jsonl", "reason": why, "server": server, "pid": pid,
                     "n_records": len(lines), "first_line": lines[0], "last_line": lines[-1],
                     "first_wall_time_iso": first.get("wall_time_iso"), "last_wall_time_iso": last.get("wall_time_iso"),
                     "matches_dominant_pid_of": matches}
            # Where the foreign calls sit relative to this run. Calls made DURING the run by the
            # other role's server are a role separation violation; calls made after it ended
            # are contamination from a process outside the run (see foreign_mcp_records).
            starts_in = [parse_iso(r.get("wall_time_iso"))[0] for ln, r in recs if ln in set(lines)]
            in_run = [t for t in starts_in if t is not None and run_window and run_window[0] <= t <= run_window[1]]
            entry["n_during_run"] = len(in_run)
            entry["n_after_run"] = sum(1 for t in starts_in if t is not None and run_window and t > run_window[1])
            entry["n_before_run"] = sum(1 for t in starts_in if t is not None and run_window and t < run_window[0])
            foreign.append(entry)
            if why == "foreign_server" and in_run:
                role_viol.append(f"mcp_calls_{role}.jsonl: {len(in_run)} calls to server {server} (pid {pid}) "
                                 f"made during the run")
    rep.add("foreign_mcp_records", "corrupt" if foreign else "ok", foreign)
    rep.add("role_separation", "corrupt" if role_viol else "ok", role_viol)

    # ---- step structure ---------------------------------------------------
    begins: Counter = Counter()
    ends_c: Counter = Counter()
    step_issues = []
    t_run_end = parse_iso(run_ended[0].get("wall_time_iso"))[0] if run_ended else None
    for r in scene:
        p = r.get("payload") or {}
        if r.get("event_type") == "begin_step":
            begins[p.get("step_id")] += 1
        elif r.get("event_type") == "end_step":
            ends_c[p.get("step_id")] += 1
            e, _ = parse_iso(r.get("wall_time_iso"))
            if t_run_end is not None and e is not None and e > t_run_end + 0.001:
                step_issues.append(f"end_step {p.get('step_id')} after run_ended")
    for s in set(begins) | set(ends_c):
        if begins[s] != ends_c[s]:
            step_issues.append(f"step {s}: {begins[s]} begin_step vs {ends_c[s]} end_step")
        if begins[s] > 1:
            # cell_orchestrator keys gate events by (step_id, phase) and never resets them,
            # so a repeated step id passes its gate at once, before any re-arming.
            step_issues.append(f"step {s} begun {begins[s]} times; repeats pass the orchestrator gate without waiting")
    if not run_ended:
        step_issues.append("no run_ended event")
    if len(run_ended) > 1:
        step_issues.append(f"{len(run_ended)} run_ended events")
    rep.add("step_structure", "warn" if step_issues else "ok", step_issues)

    # ---- fires --------------------------------------------------------------
    mon_file = "events_executor_monitor.jsonl" if is_s4 else "events_watchdog.jsonl"
    fires = [r.get("payload") or {} for r in env_of(mon_file, "monitor_fire")]
    actions = Counter(str(f.get("action_taken")) for f in fires)
    halts = sum(1 for f in fires if f.get("halt_result") is not None)
    null_fields = [f"{f.get('monitor_id')}: missing {k}" for f in fires for k in FIRE_REQUIRED if f.get(k) is None]
    # delivery: the adjudicating role's call whose RETURN carries the same (monitor_id, trigger_wall_ns)
    deliver_role = "executor" if is_s4 else "watchdog"
    deliveries: dict[tuple, float] = {}
    wake_ends: list[float] = []
    for _, r in calls.get(deliver_role, []):
        if r.get("server") not in ROLE_SERVERS[deliver_role] or r.get("tool") not in ("wait_for_event", "get_monitor_fires"):
            continue
        e, prob = parse_iso(r.get("wall_time_iso"))
        if e is None:
            continue
        end = e + float(r.get("elapsed_sec") or 0.0)
        wake_ends.append(end)
        res = r.get("result") if isinstance(r.get("result"), dict) else {}
        carried = ([res["fire"]] if isinstance(res.get("fire"), dict) else []) + list(res.get("additional_fires") or []) + list(res.get("fires") or [])
        for f in carried:
            if isinstance(f, dict):
                k = (f.get("monitor_id"), f.get("trigger_wall_ns"))
                deliveries[k] = min(deliveries.get(k, end), end)
    late = []
    for f in fires:
        k = (f.get("monitor_id"), f.get("trigger_wall_ns"))
        if k[1] is None:
            continue
        t_fire = int(k[1]) / 1e9
        if k in deliveries:
            if deliveries[k] - t_fire > args.fire_wake_timeout_s:
                late.append(f"{k[0]} at {iso(t_fire)} delivered after {deliveries[k] - t_fire:.1f} s")
        else:
            after_end = t_run_end is not None and t_fire > t_run_end
            late.append(f"{k[0]} at {iso(t_fire)} never delivered to the {deliver_role}"
                        + (" (fired after run_ended)" if after_end else ""))
    fire_status = "ok"
    if null_fields:
        fire_status = "corrupt"
    elif late or halts or set(actions) - {"flag"}:
        fire_status = "warn"
    rep.add("fires", fire_status, null_fields + late, n_fires=len(fires), action_taken=dict(actions),
            n_halt_result_non_null=halts, timeout_s=args.fire_wake_timeout_s)

    # ---- verdicts -----------------------------------------------------------
    verdicts = [r.get("payload") or {} for r in env_of(mon_file, "verdict")]
    v_steps = Counter(v.get("step_id") for v in verdicts)
    begun = set(begins)
    unknown = [f"verdict for step {s!r} that was never begun" for s in v_steps if s not in begun]
    no_verdict = sorted(s for s in begun if s not in v_steps)
    n_run_verdicts = len(env_of(mon_file, "run_verdict"))
    issues = unknown + ([f"no run_verdict"] if n_run_verdicts == 0 else []) + \
        ([f"{n_run_verdicts} run_verdict events"] if n_run_verdicts > 1 else [])
    rep.add("verdicts", "warn" if issues else "ok", issues, steps_without_step_verdict=no_verdict,
            note="step verdicts are expected only where the agent chose to record one; a missing one is not by itself an error")

    # ---- arming -------------------------------------------------------------
    arms = env_of(mon_file, "monitor_armed")
    arm_issues = []
    pre_wake = 0
    wake_times = sorted(wake_ends)
    for r in arms:
        p = r.get("payload") or {}
        miss = [k for k in ARM_REQUIRED if p.get(k) in (None, "")]
        if p.get("purpose", "clause") == "clause" and p.get("threshold") is None:
            miss.append("threshold")
        if not p.get("params") and p.get("monitor_type") != "authored":
            miss.append("params")
        if miss:
            arm_issues.append(f"{p.get('monitor_id')}: missing {miss}")
        if not is_s4:
            e, _ = parse_iso(r.get("wall_time_iso"))
            if e is not None and wake_times:
                prior = [w for w in wake_times if w <= e]
                if not prior:
                    pre_wake += 1
    rep.add("arming", "warn" if arm_issues else "ok", arm_issues, n_arms=len(arms),
            n_armed_before_first_wake=pre_wake,
            note="arming before the first wake is normal run-start bystander arming; counted, not flagged")

    # ---- S4 / guard mode ----------------------------------------------------
    gate_types = Counter(r.get("type") for r in orch if str(r.get("type", "")).startswith("gate_"))
    g_issues = []
    if is_s4:
        if calls["watchdog"]:
            g_issues.append(f"S4 session has {len(calls['watchdog'])} mcp_calls_watchdog records")
        if events.get("events_watchdog.jsonl"):
            g_issues.append("S4 session has events_watchdog.jsonl")
        wd_in_exec = sum(1 for _, r in calls["executor"] if r.get("server") == "watchdog")
        if wd_in_exec:
            g_issues.append(f"S4 executor log has {wd_in_exec} records from server watchdog")
        if gate_types.get("gate_unguarded_latched"):
            g_issues.append("gate_unguarded_latched in an S4 session (expected unguarded_by_config)")
        if gate_types.get("gate_open") or gate_types.get("gate_timeout"):
            g_issues.append(f"S4 session has guarded gate records {dict(gate_types)}")
        if guard_modes != ["unguarded"]:
            g_issues.append(f"orchestrator guard_mode {guard_modes}, expected ['unguarded']")
    else:
        for t in ("gate_unguarded_by_config", "gate_unguarded_latched"):
            if gate_types.get(t):
                g_issues.append(f"{gate_types[t]} {t} records in a guarded (non S4) session")
        if gate_types.get("gate_aborted"):
            g_issues.append(f"{gate_types['gate_aborted']} gate_aborted records")
    rep.add("guard_mode", "corrupt" if g_issues else "ok", g_issues, gate_types=dict(gate_types), guard_modes=guard_modes)

    # ---- registry provenance ------------------------------------------------
    prov_issues, efforts = [], {}
    for role in ("executor", "watchdog"):
        prov = read_json(sd / f"registry_provenance_{role}.json")
        if not prov:
            continue
        p = Path(str(prov.get("registry_path", ""))).expanduser()
        if not p.is_file():
            prov_issues.append(f"{role}: recorded registry {p} no longer exists")
            continue
        cur = sha256(p)
        if prov.get("registry_sha256") and cur != prov["registry_sha256"]:
            prov_issues.append(f"{role}: {p} changed since the run (sha {cur[:12]} != recorded {str(prov['registry_sha256'])[:12]})")
            continue
        efforts[role] = {k.split(".")[1]: v for k, v in
                         yaml_scalar_fields(p.read_text(encoding="utf-8"), "max_clamp_effort").items()
                         if k.startswith("objects.")}
    c4 = {}
    if str(cls) == "4":
        c4["executor_known"] = "executor" in efforts
        c4["watchdog_known"] = "watchdog" in efforts
        if not ("executor" in efforts and "watchdog" in efforts):
            prov_issues.append("Class 4: the session alone cannot establish both roles' max_clamp_effort "
                               f"(provenance present for {sorted(efforts) or 'neither'})")
        else:
            diff = {o: (efforts['watchdog'].get(o), efforts['executor'].get(o)) for o in
                    set(efforts["executor"]) | set(efforts["watchdog"])
                    if efforts["watchdog"].get(o) != efforts["executor"].get(o)}
            c4["differing_objects_watchdog_vs_executor"] = diff
            lower = [o for o, (w, e) in diff.items() if w is not None and e is not None and float(w) < float(e)]
            if not lower:
                prov_issues.append("Class 4: no object has a lower max_clamp_effort for the WatchDog than the executor")
    rep.add("registry_provenance", "warn" if prov_issues else "ok", prov_issues,
            roles_with_provenance=sorted(efforts), class4=c4 or None)

    # ---- look() frame paths -------------------------------------------------
    look_issues = []
    sd_res = sd.resolve()
    for fname, recs in events.items():
        for ln, r in recs:
            if r.get("event_type") != "look":
                continue
            for view, path in ((r.get("payload") or {}).get("paths") or {}).items():
                pth = Path(str(path))
                inside = False
                try:
                    inside = pth.resolve().is_relative_to(sd_res) and pth.exists()
                except (OSError, ValueError):
                    pass
                if inside:
                    continue
                by_name = (sd / "frames" / pth.name).exists()
                look_issues.append(f"{fname}:{ln} {view} path {path} does not resolve inside the session"
                                   + ("; a file with that name exists in frames/" if by_name else "; file missing"))
    rep.add("look_paths", "warn" if look_issues else "ok", look_issues)

    # ---- files written after the run ---------------------------------------
    # mcp_calls_<role>.jsonl and registry_provenance_executor.json resolve the session through
    # ~/.watchdog/current_session on EVERY write, so any process with a task-path env var set
    # (a test run, a stray server) appends into the last session the pointer names.
    late = []
    if span:
        cutoff = span[1]
        for p in sorted(sd.iterdir()):
            if not p.is_file() or p.name in EXPECTED_POST_RUN_FILES or any(p.match(g) for g in EXPECTED_POST_RUN_GLOBS):
                continue
            mt = p.stat().st_mtime
            if mt > cutoff:
                late.append(f"{p.name} modified {iso(mt)}, {mt - (span[1] - args.span_pad_s):.0f} s after the run ended")
    rep.add("late_file_writes", "warn" if late else "ok", late,
            note="mtime survives copies poorly; treat as a lead, confirm against record timestamps")

    # ---- bag ----------------------------------------------------------------
    if args.skip_bag:
        rep.add("bag", "skip", [], note="--skip-bag")
    else:
        name, status, details, info = _bag_check(sd, span)
        rep.add(name, status, details, **info)

    return {"session": sid, "status": rep.status, "scenario": scenario, "class": cls,
            "label": label, "checks": rep.checks}


def _bag_check(sd: Path, span) -> tuple:
    meta = sd / "bag" / "metadata.yaml"
    if not meta.exists():
        return ("bag", "warn", ["no bag/metadata.yaml"], {})
    text = meta.read_text(encoding="utf-8", errors="replace")
    issues = []
    m0 = re.search(r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)", text)
    m1 = re.search(r"\bduration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    bag_span = None
    if m0 and m1:
        b0 = int(m0.group(1)) / 1e9
        bag_span = (b0, b0 + int(m1.group(1)) / 1e9)
        if span and (bag_span[1] < span[0] or bag_span[0] > span[1]):
            issues.append(f"bag span {iso(bag_span[0])}..{iso(bag_span[1])} does not overlap the session span")
    else:
        issues.append("bag metadata lacks starting_time or duration")
    topics = re.findall(r"name:\s*(/\S+)", text)
    if "/tf" not in topics:
        issues.append("bag has no /tf")
    if "/joint_states" not in topics:
        issues.append("bag has no /joint_states")
    info: dict = {"topics": topics, "bag_span": [iso(bag_span[0]), iso(bag_span[1])] if bag_span else None}
    try:
        from rosbags.highlevel import AnyReader  # optional
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        info["joint_check"] = "skipped: rosbags not importable"
        return ("bag", "warn" if issues else "ok", issues, info)
    try:
        with AnyReader([sd / "bag"], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
            conns = [c for c in reader.connections if c.topic == "/joint_states"]
            seen = found = with_effort = 0
            for conn, _ts, raw in reader.messages(connections=conns):
                msg = reader.deserialize(raw, conn.msgtype)
                seen += 1
                names = list(msg.name)
                if "finger_joint" in names:
                    found += 1
                    if len(msg.effort) > names.index("finger_joint"):
                        with_effort += 1
                if seen >= 300:
                    break
            info["joint_check"] = {"messages_sampled": seen, "with_finger_joint": found, "with_finger_effort": with_effort}
            if seen and not with_effort:
                issues.append("finger_joint effort absent in sampled /joint_states")
    except Exception as exc:  # noqa: BLE001  report, do not crash the audit
        issues.append(f"bag did not open: {type(exc).__name__}: {exc}")
    return ("bag", "warn" if issues else "ok", issues, info)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="session_audit", description=__doc__.split("\n\n")[0])
    ap.add_argument("sessions_root", type=Path)
    ap.add_argument("--out", type=Path, default=None, help="write the JSON report here")
    ap.add_argument("--session", action="append", default=None, help="audit only this session id (repeatable)")
    ap.add_argument("--fire-wake-timeout-s", type=float, default=10.0)
    ap.add_argument("--gap-s", type=float, default=180.0)
    ap.add_argument("--order-tol-s", type=float, default=1.0)
    ap.add_argument("--span-pad-s", type=float, default=300.0)
    ap.add_argument("--skip-bag", action="store_true")
    args = ap.parse_args(argv)

    root = args.sessions_root.expanduser()
    if not root.is_dir():
        print(f"error: {root} is not a directory", file=sys.stderr)
        return 2
    all_sessions = sorted(p for p in root.iterdir() if p.is_dir() and re.match(r"^\d{8}T\d{6}Z$", p.name))
    targets = [p for p in all_sessions if not args.session or p.name in set(args.session)]
    if not targets:
        print("error: no matching session directories", file=sys.stderr)
        return 2
    print(f"indexing process ids across {len(all_sessions)} sessions ...", file=sys.stderr)
    pid_index = build_pid_index(all_sessions)
    results = []
    for i, sd in enumerate(targets, 1):
        try:
            results.append(audit_session(sd, pid_index, args))
        except Exception as exc:  # noqa: BLE001  one broken session must not hide the rest
            results.append({"session": sd.name, "status": "corrupted", "scenario": None, "class": None,
                            "checks": {"audit_crash": {"status": "corrupt", "count": 1,
                                                       "details": [f"{type(exc).__name__}: {exc}"]}}})
        if i % 10 == 0:
            print(f"  {i}/{len(targets)}", file=sys.stderr)

    groups: dict[str, list[str]] = {"clean": [], "warnings": [], "corrupted": []}
    per_check: dict[str, Counter] = defaultdict(Counter)
    for r in results:
        groups[r["status"]].append(r["session"])
        for name, c in r["checks"].items():
            per_check[name][c["status"]] += 1
    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "sessions_root": str(root.resolve()),
        "parameters": {k: v for k, v in vars(args).items() if k not in ("sessions_root", "out")},
        "summary": {"groups": {k: len(v) for k, v in groups.items()},
                    "per_check": {k: dict(v) for k, v in sorted(per_check.items())}},
        "groups": groups,
        "sessions": results,
    }
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=1, default=str), encoding="utf-8")

    print(f"\nSession audit: {len(results)} sessions under {root}")
    for g in ("clean", "warnings", "corrupted"):
        print(f"  {g:<10} {len(groups[g])}")
    print("\nPer check (ok / warn / corrupt / skip):")
    for name, c in sorted(per_check.items()):
        print(f"  {name:<22} {c['ok']:>4} {c['warn']:>5} {c['corrupt']:>7} {c['skip']:>5}")
    if groups["corrupted"]:
        print("\nCorrupted sessions and the checks that flagged them:")
        for r in results:
            if r["status"] == "corrupted":
                bad = [n for n, c in r["checks"].items() if c["status"] == "corrupt"]
                print(f"  {r['session']}  {r.get('scenario') or '?'}  class {r.get('class') or '?'}  {', '.join(bad)}")
    if args.out:
        print(f"\nFull report: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
