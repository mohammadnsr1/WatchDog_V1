"""Readers for one session directory: orchestrator log, MCP call logs, event logs.

All timestamps are converted to wall-clock seconds since the Unix epoch (UTC).
Sources and their stamps:
  orchestrator  wall_ns (epoch ns, grant time) + waited_sec (monotonic delta)
  mcp_calls_*   wall_time_iso = call START; elapsed_sec = duration; return = start + elapsed
  events_*      wall_time_iso (envelope, epoch); monitor_fire payload trigger_wall_ns (epoch ns)
The envelope timestamp_ns is monotonic and is never used.
"""
from __future__ import annotations

import json
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from .config import DELIVERY_TOOLS, EXECUTOR_SERVERS, WATCHDOG_SERVERS


def iso_to_s(iso: str) -> float:
    return datetime.fromisoformat(iso).timestamp()


def read_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    out = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


@dataclass
class Call:
    server: str
    tool: str
    pid: int
    start_s: float
    elapsed_s: float
    ok: Optional[bool]
    result: Any
    arguments: Any

    @property
    def end_s(self) -> float:
        return self.start_s + self.elapsed_s

    def carried_fires(self) -> list[dict]:
        """Fire payloads carried in this call's return (wait_for_event / get_monitor_fires)."""
        if self.tool not in DELIVERY_TOOLS or not isinstance(self.result, dict):
            return []
        r = self.result
        fires: list[dict] = []
        if isinstance(r.get("fire"), dict):
            fires.append(r["fire"])
        fires.extend(f for f in (r.get("additional_fires") or []) if isinstance(f, dict))
        fires.extend(f for f in (r.get("fires") or []) if isinstance(f, dict))
        return fires


@dataclass
class CallLog:
    calls: list[Call]
    n_raw: int
    n_dropped_foreign_server: int
    n_dropped_foreign_pid: int
    pid_by_server: dict[str, int]
    foreign_servers: dict[str, int]


def _to_call(rec: dict) -> Call:
    return Call(
        server=str(rec.get("server")), tool=str(rec.get("tool")), pid=int(rec.get("pid", -1)),
        start_s=iso_to_s(rec["wall_time_iso"]), elapsed_s=float(rec.get("elapsed_sec") or 0.0),
        ok=rec.get("ok"), result=rec.get("result"), arguments=rec.get("arguments"),
    )


def load_call_log(path: Path, allowed_servers: frozenset[str]) -> CallLog:
    """Filter to the role's own servers AND, per server, that server's modal pid (D1).

    The contamination observed in the campaign is cross-pid (a watchdog server from another
    run wrote into an executor log), so both filters are applied independently.
    """
    recs = read_jsonl(path)
    by_server: dict[str, Counter] = defaultdict(Counter)
    foreign: Counter = Counter()
    for r in recs:
        s = str(r.get("server"))
        if s in allowed_servers:
            by_server[s][int(r.get("pid", -1))] += 1
        else:
            foreign[s] += 1
    pid_by_server = {s: c.most_common(1)[0][0] for s, c in by_server.items()}
    calls: list[Call] = []
    n_pid = 0
    for r in recs:
        s = str(r.get("server"))
        if s not in allowed_servers:
            continue
        if int(r.get("pid", -1)) != pid_by_server[s]:
            n_pid += 1
            continue
        calls.append(_to_call(r))
    calls.sort(key=lambda c: c.start_s)
    return CallLog(calls=calls, n_raw=len(recs), n_dropped_foreign_server=sum(foreign.values()),
                   n_dropped_foreign_pid=n_pid, pid_by_server=pid_by_server, foreign_servers=dict(foreign))


@dataclass
class OrchestratorLog:
    path: Optional[Path]
    records: list[dict]
    n_logs_found: int
    discarded_paths: list[str]
    unresolved: bool                     # >1 log carrying gates: overhead undefined
    start_s: Optional[float]

    def gates(self) -> list[dict]:
        return [r for r in self.records if r.get("type", "").startswith("gate_")]


_GATE_TYPES = {"gate_open", "gate_timeout", "gate_unguarded_by_config",
               "gate_unguarded_latched", "gate_aborted"}


def load_orchestrator(session_dir: Path, expected_gates: Optional[int]) -> OrchestratorLog:
    """R2 rule: never concatenate. Pick the log with the latest orchestrator_start whose gate
    count reconciles; if more than one log carries gates the session is unresolved."""
    paths = sorted((session_dir / "orchestrator").glob("*.jsonl")) if (session_dir / "orchestrator").exists() else []
    parsed = []
    for p in paths:
        recs = read_jsonl(p)
        starts = [r["wall_ns"] / 1e9 for r in recs if r.get("type") == "orchestrator_start"]
        n_gates = sum(1 for r in recs if r.get("type") in _GATE_TYPES)
        parsed.append((p, recs, max(starts) if starts else None, n_gates))
    if not parsed:
        return OrchestratorLog(None, [], 0, [], False, None)
    with_gates = [x for x in parsed if x[3] > 0]
    unresolved = len(with_gates) > 1
    cands = [x for x in parsed if expected_gates is None or x[3] == expected_gates] or parsed
    chosen = max(cands, key=lambda x: (x[2] or 0.0))
    discarded = [str(x[0]) for x in parsed if x[0] != chosen[0]]
    return OrchestratorLog(chosen[0], chosen[1], len(parsed), discarded, unresolved, chosen[2])


@dataclass
class Session:
    session_id: str
    dir: Path
    orchestrator: OrchestratorLog
    executor_calls: CallLog
    watchdog_calls: Optional[CallLog]
    events_scene: list[dict]
    events_motion: list[dict]
    events_gripper: list[dict]
    monitor_events: list[dict]          # events_watchdog.jsonl (S1-S3) or events_executor_monitor.jsonl (S4)
    monitor_events_file: Optional[str]
    agent_config: dict[str, dict] = field(default_factory=dict)   # role -> agent_config_<role>.json
    files_read: list[Path] = field(default_factory=list)

    # ---- derived markers -------------------------------------------------
    def begin_step_plus_phase(self) -> int:
        return sum(1 for e in self.events_scene if e.get("event_type") in ("begin_step", "phase"))

    def run_ended_s(self) -> Optional[float]:
        ends = [e for e in self.events_scene if e.get("event_type") == "run_ended"]
        return iso_to_s(ends[0]["wall_time_iso"]) if ends else None

    def first_begin_step_s(self) -> Optional[float]:
        bs = [e for e in self.events_scene if e.get("event_type") == "begin_step"]
        return iso_to_s(bs[0]["wall_time_iso"]) if bs else None

    def executor_first_call_s(self) -> Optional[float]:
        return self.executor_calls.calls[0].start_s if self.executor_calls.calls else None

    def fires(self) -> list[dict]:
        """monitor_fire payloads, with the envelope wall_time_iso attached as _envelope_s."""
        out = []
        for e in self.monitor_events:
            if e.get("event_type") == "monitor_fire" and isinstance(e.get("payload"), dict):
                p = dict(e["payload"])
                p["_envelope_s"] = iso_to_s(e["wall_time_iso"])
                out.append(p)
        return out


def load_session(session_dir: Path, scenario: str) -> Session:
    sid = session_dir.name
    files: list[Path] = []

    def jl(name: str) -> list[dict]:
        p = session_dir / name
        if p.exists():
            files.append(p)
        return read_jsonl(p)

    scene = jl("events_scene.jsonl")
    expected = sum(1 for e in scene if e.get("event_type") in ("begin_step", "phase"))
    orch = load_orchestrator(session_dir, expected)
    if orch.path:
        files.append(orch.path)
    ex_path = session_dir / "mcp_calls_executor.jsonl"
    files.append(ex_path)
    ex = load_call_log(ex_path, EXECUTOR_SERVERS)
    wd_path = session_dir / "mcp_calls_watchdog.jsonl"
    wd = None
    if wd_path.exists():
        files.append(wd_path)
        wd = load_call_log(wd_path, WATCHDOG_SERVERS)
    if scenario == "S4":
        mon_file = "events_executor_monitor.jsonl"
    else:
        mon_file = "events_watchdog.jsonl"
    mon = jl(mon_file)
    cfg: dict[str, dict] = {}
    for role in ("executor", "watchdog"):
        p = session_dir / f"agent_config_{role}.json"
        if p.exists():
            files.append(p)
            cfg[role] = json.loads(p.read_text(encoding="utf-8"))
    return Session(
        session_id=sid, dir=session_dir, orchestrator=orch, executor_calls=ex, watchdog_calls=wd,
        events_scene=scene, events_motion=jl("events_motion.jsonl"), events_gripper=jl("events_gripper.jsonl"),
        monitor_events=mon, monitor_events_file=mon_file if (session_dir / mon_file).exists() else None,
        agent_config=cfg, files_read=files,
    )
