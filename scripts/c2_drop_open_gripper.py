#!/usr/bin/env python3
"""Operator-side Class 2 fault: open the gripper mid-transport (drop).

Waits until the WatchDog releases STEP's gate (watchdog_ready, scope=step,
for STEP -- the moment the executor starts moving on it), waits DELAY_S
more, then calls /rg2_controller/open -- the same Trigger service the
executor's own release uses -- so the carried part falls. The executor is
not told; whatever it does next is part of the run.

Run it in a ROS-sourced terminal after the bringup is up and BEFORE the
executor starts, with the COMMITTED task (plain ./run_executor.sh 2; this is
its own fault, not r2's). The fault record goes to
<session>/operator_faults.jsonl (wall time, step, delay, service result).

  source /opt/ros/jazzy/setup.bash
  python3 scripts/c2_drop_open_gripper.py                      # defaults below
  python3 scripts/c2_drop_open_gripper.py --step place_approach_line_green --delay 4
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger

OPEN_SERVICE = "/rg2_controller/open"
MAX_SESSION_AGE_S = 15 * 60
POLL_S = 0.2


def session_dir() -> Path:
    sd = Path((Path.home() / ".watchdog" / "current_session").read_text().strip())
    if not sd.is_dir():
        sys.exit(f"current_session names {sd}, which is not a directory")
    age = time.time() - sd.stat().st_mtime
    if age > MAX_SESSION_AGE_S:
        sys.exit(f"{sd} last modified {age/60:.1f} min ago -- stale pointer? "
                 "Start the bringup for this run first.")
    return sd


def events(path: Path):
    if not path.exists():
        return []
    out = []
    for line in path.read_text().splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            pass
    return out


def released(sd: Path, step: str) -> bool:
    return any(e.get("event_type") == "watchdog_ready"
               and (e.get("payload") or {}).get("scope") == "step"
               and (e.get("payload") or {}).get("step_id") == step
               for e in events(sd / "events_watchdog.jsonl"))


def run_ended(sd: Path) -> bool:
    return any(e.get("event_type") == "run_ended"
               for e in events(sd / "events_scene.jsonl"))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", default="place_approach_u_orange",
                    help="drop during this step (default: place_approach_u_orange)")
    ap.add_argument("--delay", type=float, default=3.0,
                    help="seconds after the step's gate release (default 3.0)")
    a = ap.parse_args()

    sd = session_dir()
    print(f"[drop] session {sd}\n[drop] waiting for the gate release of {a.step}, "
          f"then +{a.delay:g}s -> {OPEN_SERVICE}")
    rclpy.init()
    node = rclpy.create_node("operator_c2_drop")
    client = node.create_client(Trigger, OPEN_SERVICE)
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            sys.exit(f"{OPEN_SERVICE} unavailable -- is the bringup up?")
        while not released(sd, a.step):
            if run_ended(sd):
                print("[drop] run ended before the step was released -- nothing done")
                return
            time.sleep(POLL_S)
        t_release = datetime.now(timezone.utc)
        time.sleep(a.delay)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        res = future.result()
        rec = {"wall_time_iso": datetime.now(timezone.utc).isoformat(),
               "fault": "drop_open_gripper", "step_id": a.step,
               "delay_s": a.delay, "release_seen_iso": t_release.isoformat(),
               "service": OPEN_SERVICE,
               "success": bool(res.success) if res else None,
               "message": res.message if res else "no response"}
        with open(sd / "operator_faults.jsonl", "a") as fh:
            fh.write(json.dumps(rec) + "\n")
        print(f"[drop] gripper opened -> {rec['success']} {rec['message']!r}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
