"""WatchDog gate overhead (2c) and the task-time decomposition (A2).

One gate wait = the interval the executor's RequestGo was blocked, which the orchestrator
records on the grant as ``waited_sec`` (monotonic, same process).  The request time is
``wall_ns/1e9 - waited_sec``.  Gate timeouts (120 s, then proceed) are real blocking and
count in the total (D2) but not in the median / p95 / max.

Run boundaries (D1):
  run_wall_s          executor's first own-server MCP call  ->  run_ended
  run_wall_stepping_s first begin_step                       ->  run_ended
  watchdog_postrun_tail_s  run_ended -> last watchdog MCP call return (S1-S3)
"""
from __future__ import annotations

from typing import Any, Optional

from .config import COMMITTED_TASK_GATES, GATED_SCENARIOS
from .fractions import median, p95
from .loader import Run
from .sessionlogs import Session


def compute_gates(run: Run, s: Session) -> None:
    d = run.derived
    o = s.orchestrator
    d["n_orchestrator_logs"] = o.n_logs_found
    d["orchestrator_log_used"] = str(o.path) if o.path else None
    d["orchestrator_logs_discarded"] = ";".join(o.discarded_paths)
    d["multi_orchestrator_unresolved"] = o.unresolved
    d["expected_gates_begin_step_plus_phase"] = s.begin_step_plus_phase()
    d["n_executor_calls_dropped_contamination"] = s.executor_calls.n_dropped_foreign_server + s.executor_calls.n_dropped_foreign_pid
    d["executor_contamination_servers"] = ";".join(f"{k}:{v}" for k, v in sorted(s.executor_calls.foreign_servers.items()))
    d["executor_pid_by_server"] = ";".join(f"{k}:{v}" for k, v in sorted(s.executor_calls.pid_by_server.items()))
    d["n_watchdog_calls_dropped_contamination"] = (
        (s.watchdog_calls.n_dropped_foreign_server + s.watchdog_calls.n_dropped_foreign_pid) if s.watchdog_calls else None)

    # ---- run boundaries, all scenarios --------------------------------------
    t_first_call = s.executor_first_call_s()
    t_first_step = s.first_begin_step_s()
    t_end = s.run_ended_s()
    d["executor_first_call_iso_s"] = t_first_call
    d["first_begin_step_s"] = t_first_step
    d["run_ended_s"] = t_end
    if t_end is None:
        run.set_na("run_wall_s", "no_run_ended_event")
        run.set_na("run_wall_stepping_s", "no_run_ended_event")
    else:
        if t_first_call is None:
            run.set_na("run_wall_s", "no_executor_mcp_calls")
        else:
            d["run_wall_s"] = t_end - t_first_call
        if t_first_step is None:
            run.set_na("run_wall_stepping_s", "no_begin_step_event")
        else:
            d["run_wall_stepping_s"] = t_end - t_first_step

    gates = o.gates()
    n_total = len(gates)
    d["n_gate_records"] = n_total
    d["gate_count_anomaly"] = "" if n_total == d["expected_gates_begin_step_plus_phase"] else (
        f"gate_records={n_total}!=begin_step+phase={d['expected_gates_begin_step_plus_phase']}")
    d["gate_count_vs_committed_task"] = "" if n_total == COMMITTED_TASK_GATES else f"differs_from_committed_{COMMITTED_TASK_GATES}:{n_total}"

    if run.scenario not in GATED_SCENARIOS:
        for col in ("n_gates", "n_gate_timeouts", "gate_wait_total_s", "gate_wait_total_ex_timeout_s",
                    "gate_wait_median_s", "gate_wait_p95_s", "gate_wait_max_s",
                    "gate_overhead_fraction", "gate_overhead_fraction_ex_timeout", "watchdog_postrun_tail_s"):
            run.set_na(col, "no_gates_in_scenario")
        d["timeout_step_ids"] = ""
        d["n_unguarded_gate_records"] = sum(1 for g in gates if g["type"] == "gate_unguarded_by_config")
        _decompose(run, blocking=0.0, blocking_is_zero_by_construction=True)
        return

    if o.unresolved:
        for col in ("n_gates", "n_gate_timeouts", "gate_wait_total_s", "gate_wait_total_ex_timeout_s",
                    "gate_wait_median_s", "gate_wait_p95_s", "gate_wait_max_s",
                    "gate_overhead_fraction", "gate_overhead_fraction_ex_timeout"):
            run.set_na(col, "multi_orchestrator_unresolved")
        _decompose(run, blocking=None, blocking_is_zero_by_construction=False)
        return

    waits = [(g["type"], float(g.get("waited_sec", 0.0)), g.get("step_id"), g.get("phase"), g["wall_ns"] / 1e9)
             for g in gates if g["type"] in ("gate_open", "gate_timeout")]
    timeouts = [w for w in waits if w[0] == "gate_timeout"]
    normal = [w[1] for w in waits if w[0] == "gate_open"]
    d["n_gates"] = len(waits)
    d["n_gate_timeouts"] = len(timeouts)
    d["timeout_step_ids"] = ";".join(f"{w[2]}{('/' + w[3]) if w[3] else ''}" for w in timeouts)
    d["gate_wait_total_s"] = sum(w[1] for w in waits)
    d["gate_wait_total_ex_timeout_s"] = sum(normal)
    d["gate_wait_median_s"] = median(normal)
    d["gate_wait_p95_s"] = p95(normal)
    d["gate_wait_max_s"] = max(normal) if normal else None
    d["_gate_waits"] = list(normal)        # per-gate waits (ex timeout), pooled by tables
    if not normal:
        for col in ("gate_wait_median_s", "gate_wait_p95_s", "gate_wait_max_s"):
            run.set_na(col, "no_non_timeout_gates")
    # request-time sanity: every gate request must fall inside the run window
    if t_first_call is not None:
        early = [w for w in waits if (w[4] - w[1]) < t_first_call - 0.5]
        if early:
            run.flags["gate_request_before_executor_first_call"] = str(len(early))
    rw = d.get("run_wall_s")
    if rw is None or rw <= 0:
        run.set_na("gate_overhead_fraction", "run_wall_s_unavailable")
        run.set_na("gate_overhead_fraction_ex_timeout", "run_wall_s_unavailable")
    else:
        d["gate_overhead_fraction"] = d["gate_wait_total_s"] / rw
        d["gate_overhead_fraction_ex_timeout"] = d["gate_wait_total_ex_timeout_s"] / rw
    # watchdog post-run tail
    if s.watchdog_calls and s.watchdog_calls.calls and t_end is not None:
        d["watchdog_postrun_tail_s"] = s.watchdog_calls.calls[-1].end_s - t_end
    else:
        run.set_na("watchdog_postrun_tail_s", "no_watchdog_calls_or_run_end")
    _decompose(run, blocking=d["gate_wait_total_s"], blocking_is_zero_by_construction=False)


def _decompose(run: Run, blocking: Optional[float], blocking_is_zero_by_construction: bool) -> None:
    """A2 columns.  Blocking and post-run adjudication are separate costs; never summed."""
    d = run.derived
    rw = d.get("run_wall_s")
    d["total_task_time_s"] = rw
    if rw is None:
        run.set_na("total_task_time_s", run.na_reasons.get("run_wall_s", "run_wall_s_unavailable"))
    if blocking is None:
        run.set_na("watchdog_blocking_s", run.na_reasons.get("gate_wait_total_s", "gate_wait_unavailable"))
        run.set_na("executor_working_s", "gate_wait_unavailable")
    else:
        d["watchdog_blocking_s"] = blocking
        d["watchdog_blocking_is_zero_by_construction"] = blocking_is_zero_by_construction
        if rw is None:
            run.set_na("executor_working_s", "run_wall_s_unavailable")
        else:
            d["executor_working_s"] = rw - blocking
    ttv = run.results.get("time_to_verdict_s")
    if run.scenario in GATED_SCENARIOS:
        try:
            d["watchdog_adjudication_s"] = float(ttv) if ttv not in (None, "", "None") else None
        except ValueError:
            d["watchdog_adjudication_s"] = None
        if d["watchdog_adjudication_s"] is None:
            run.set_na("watchdog_adjudication_s", "time_to_verdict_s_blank")
    else:
        # S4: the executor adjudicates itself; its time_to_verdict_s is a different quantity
        # (not anchored on run_ended) and is passed through under its original name only.
        run.set_na("watchdog_adjudication_s", "no_watchdog_role_s4_time_to_verdict_not_comparable")
