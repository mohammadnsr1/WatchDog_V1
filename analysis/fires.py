"""Fire delivery latency and serialization cost (2d).

fire_delivery_latency_s = (return of the first adjudicating-agent call whose result carried
the fire) - trigger_wall_ns.  Return = call start + elapsed_sec (D4).  Fires are matched to
deliveries EXACTLY on (monitor_id, trigger_wall_ns); there is no nearest-timestamp fallback.
A fire that matches nothing is either explained (fired after the last delivering call's
return and before run_ended) or is a hard failure.

frac_fires_during_motion = fires whose trigger_wall_ns lies inside the open interval of an
executor motion/gripper MCP call (D5), with a breakdown by server.
"""
from __future__ import annotations

from typing import Optional

from .config import ADJUDICATING_ROLE, DELIVERY_TOOLS, MOTION_INTERVAL_SERVERS
from .fractions import median, p95
from .loader import Run
from .sessionlogs import Call, Session


class DeliveryMatchError(RuntimeError):
    pass


def _delivery_calls(run: Run, s: Session) -> tuple[list[Call], str, str]:
    role = ADJUDICATING_ROLE.get(run.scenario, "")
    if role == "watchdog":
        calls = s.watchdog_calls.calls if s.watchdog_calls else []
        mech = "blocking_wait"
    else:
        calls = s.executor_calls.calls
        mech = "poll"
    dc = [c for c in calls if c.tool in DELIVERY_TOOLS]
    tools = ";".join(sorted({c.tool for c in dc}))
    return dc, mech, tools


def compute_fires(run: Run, s: Session) -> None:
    d = run.derived
    fires = s.fires()
    d["log_monitor_events_file"] = s.monitor_events_file
    d["log_n_fires"] = len(fires)
    d["log_n_fires_clause"] = sum(1 for f in fires if (f.get("purpose") or "clause") == "clause")
    d["log_n_fires_observation"] = sum(1 for f in fires if f.get("purpose") == "observation")
    d["delivery_match_method"] = "exact_monitor_id_plus_trigger_wall_ns"
    dc, mech, tools = _delivery_calls(run, s)
    d["delivery_mechanism"] = mech
    d["delivery_tools"] = tools
    d["n_delivery_calls"] = len(dc)

    # index: key -> earliest delivering call return
    first_return: dict[tuple, float] = {}
    for c in dc:
        for f in c.carried_fires():
            k = (f.get("monitor_id"), f.get("trigger_wall_ns"))
            if k not in first_return or c.end_s < first_return[k]:
                first_return[k] = c.end_s
    keys = [(f.get("monitor_id"), f.get("trigger_wall_ns")) for f in fires]
    if len(set(keys)) != len(keys):
        raise DeliveryMatchError(f"{run.batch_id}/{run.run_id}: duplicate (monitor_id, trigger_wall_ns) among fires")
    missing_stamp = [k for k in keys if k[1] is None]
    last_return = max((c.end_s for c in dc), default=None)
    t_end = s.run_ended_s()
    latencies: list[float] = []
    undelivered: list[str] = []
    for f, k in zip(fires, keys):
        if k[1] is None:
            undelivered.append(f"{k[0]}:no_trigger_wall_ns")
            continue
        t_fire = k[1] / 1e9
        if k in first_return:
            latencies.append(first_return[k] - t_fire)
            continue
        if last_return is not None and t_fire > last_return and (t_end is None or t_fire <= t_end + 1e-6):
            undelivered.append(f"{k[0]}:fired_after_last_wait_before_run_end")
        elif last_return is not None and t_fire > last_return and t_end is not None and t_fire > t_end:
            undelivered.append(f"{k[0]}:fired_after_run_end")
        elif not dc:
            undelivered.append(f"{k[0]}:no_delivery_calls_in_log")
        else:
            raise DeliveryMatchError(
                f"{run.batch_id}/{run.run_id}: fire {k} matched no delivering call although calls "
                f"continued after it (last return {last_return:.3f}, fire {t_fire:.3f})")
    d["n_fires_delivered"] = len(latencies)
    d["n_fires_undelivered"] = len(undelivered)
    d["undelivered_fire_reasons"] = ";".join(undelivered)
    d["n_fires_missing_trigger_stamp"] = len(missing_stamp)
    d["_fire_latencies"] = list(latencies)      # underscore: pooled by tables, not written to CSV
    neg = [x for x in latencies if x < 0]
    if neg:
        run.flags["negative_delivery_latency"] = str(len(neg))
    if latencies:
        d["fire_delivery_median_s"] = median(latencies)
        d["fire_delivery_p95_s"] = p95(latencies)
        d["fire_delivery_max_s"] = max(latencies)
        d["fire_delivery_latencies_s"] = ";".join(f"{x:.3f}" for x in latencies)
    else:
        reason = "no_fires_in_run" if not fires else "no_fire_delivered"
        for col in ("fire_delivery_median_s", "fire_delivery_p95_s", "fire_delivery_max_s"):
            run.set_na(col, reason)
        d["fire_delivery_latencies_s"] = ""

    # ---- serialization: fires inside motion/gripper call intervals ------------
    intervals = [(c.start_s, c.end_s, c.server, c.tool) for c in s.executor_calls.calls
                 if c.server in MOTION_INTERVAL_SERVERS]
    d["motion_interval_servers"] = ";".join(sorted(MOTION_INTERVAL_SERVERS))
    d["motion_tool_set"] = ";".join(sorted({f"{sv}:{t}" for _, _, sv, t in intervals}))
    d["n_motion_intervals"] = len(intervals)
    in_any = in_motion = in_gripper = 0
    hit_tools: list[str] = []
    for f, k in zip(fires, keys):
        if k[1] is None:
            continue
        t = k[1] / 1e9
        hits = [(sv, tl) for (a, b, sv, tl) in intervals if a < t < b]
        if hits:
            in_any += 1
            hit_tools.append(",".join(f"{sv}:{tl}" for sv, tl in hits))
            if any(sv == "motion" for sv, _ in hits):
                in_motion += 1
            if any(sv == "gripper" for sv, _ in hits):
                in_gripper += 1
    stamped = len(fires) - len(missing_stamp)
    d["n_fires_during_motion"] = in_any
    d["n_fires_during_motion_server"] = in_motion
    d["n_fires_during_gripper_server"] = in_gripper
    d["fires_during_motion_tools"] = ";".join(hit_tools)
    d["n_fires_stamped"] = stamped
    if stamped == 0:
        for col in ("frac_fires_during_motion", "frac_fires_during_motion_server", "frac_fires_during_gripper_server"):
            run.set_na(col, "no_fires_in_run" if not fires else "no_stamped_fires")
    else:
        d["frac_fires_during_motion"] = in_any / stamped
        d["frac_fires_during_motion_server"] = in_motion / stamped
        d["frac_fires_during_gripper_server"] = in_gripper / stamped
