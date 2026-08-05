"""Rung 2 (primitive 2/4) — RelativeFrameMonitor, grasp retention.

Contract sourcing: green_cube's own PickContract already contains the exact
clause needed, with NO role-reuse required this time (unlike the
stationarity test) —

    object_tcp_offset_m LE position_tol   (an EFFECT clause: "object-to-TCP
    positional offset must stay within position tolerance after lift,
    confirming the object is tracking the gripper")

That is precisely what a grasp-retention relative-frame check verifies, just
evaluated continuously during the hold instead of once after a claimed
success. max_delta_m = position_tol (0.005 m for green_cube).

Because simulating a genuine grip failure requires a real physical slip,
this test grasps green_cube for real, then asks you to pull it out of the
gripper by hand in Isaac Sim — same manual-disturbance pattern as the
stationarity test, for the same reason (a scripted "slip" isn't a real test
of the monitor once the earlier stiffness fix made grasps reliably firm).

Run (full stack + Isaac Sim up, workspace sourced, from test/):
    python3 watchdog_rung2_relative_frame_test.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from robot_backend.contracts.loader import load_pick_contract
from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import grasp_skill, release_skill
from robot_backend.skills.maintenance_skills import (
    set_permitted_contacts as set_permitted_contacts_skill,
)
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.utils.geometry import make_pose
from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.monitors import RelativeFrameMonitor
from robot_backend.watchdog.timebase import wall_ns

from watchdog_smoke_test import WatchDogHarness

WORLD_FRAME = "world"
GREEN = "green_cube"
SAFE_POSE_NAME = "ready"
GRASP_WIDTH_M = 0.04
APPROACH_HEIGHT_M = 0.10
PARENT_FRAME = "tcp"
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}

DEFAULT_QUIET_SEC = 6.0
DEFAULT_PROMPT_SEC = 25.0
POLL_SEC = 0.2
SETTLE_SEC = 1.0


def _pose(x, y, z):
    return make_pose(position={"x": x, "y": y, "z": z},
                      orientation=dict(TOP_DOWN_QUAT), frame_id=WORLD_FRAME)


def _find_clause(contract, field: str):
    for clause in contract.preconditions + contract.effects + contract.invariants:
        if clause.field == field:
            return clause
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet-sec", type=float, default=DEFAULT_QUIET_SEC)
    ap.add_argument("--prompt-sec", type=float, default=DEFAULT_PROMPT_SEC)
    ap.add_argument("--log-dir", default=str(Path.home() / ".watchdog"))
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    if not rclpy.ok():
        rclpy.init()

    harness = WatchDogHarness(log_dir)
    engine = MonitorEngine(event_bus=harness.event_bus, halt_client=None,
                           logger=harness.node.get_logger())
    harness.recorder.engine = engine

    exec_node = Node("watchdog_rung2_relframe_exec")
    configure_bus(node=exec_node, log_path=str(log_dir / "rung2_relframe_exec.jsonl"))
    backend = create_ros_moveit_backend(node=exec_node)

    ok = True
    grasped = False
    try:
        print("waiting for recorder buffers to fill...")
        time.sleep(2.0)

        # Scene setup, same as every other executor-driving test: MoveIt cannot
        # attach an object that is not in the world collision scene.
        backend.scene.sync_collision_objects([GREEN])
        set_permitted_contacts_skill(backend, registry=harness.recorder.registry)

        child_frame = harness.recorder.registry["objects"][GREEN]["tf_frame"]

        # grasp_width_m is instruction-sourced, not registry-sourced (see
        # agents/tasks/single_step_pick.{executor,watchdog}.yaml's steps:
        # block); this manual test
        # script has no task YAML of its own, so it stands in with GRASP_WIDTH_M.
        step_spec = {
            "step_id": "rung2_relative_frame_test",
            "object_id": GREEN,
            "grasp_width_m": GRASP_WIDTH_M,
        }
        contract = load_pick_contract(GREEN, step_spec)
        clause = _find_clause(contract, "object_tcp_offset_m")
        if clause is None:
            print("FAIL: green_cube's contract has no object_tcp_offset_m clause.")
            return 1
        max_delta_m = clause.threshold
        print(f"max_delta_m = {max_delta_m} m, sourced from green_cube's own "
              f"contract clause: field={clause.field!r} "
              f"comparison={clause.comparison.value!r} threshold={clause.threshold} "
              f"(\"{clause.description}\")")

        print(f"\ngrasping {GREEN}...")
        pose = harness.recorder.latest_object_pose(GREEN)
        if pose is None:
            print(f"FAIL: no buffered pose for {GREEN}.")
            return 1
        p = pose["pose"]["position"]

        move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            backend, _pose(p["x"], p["y"], p["z"] + APPROACH_HEIGHT_M),
            gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            backend, _pose(p["x"], p["y"], p["z"]), gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        registry = harness.recorder.registry
        res = grasp_skill(backend, object_id=GREEN, registry=registry)
        if not res.get("ok"):
            print(f"FAIL: grasp failed, cannot test grasp retention: "
                  f"{res.get('reason')}")
            return 1
        grasped = True
        print("grasp confirmed. settling before arming...")
        time.sleep(2.0)  # let the hold stabilize before capturing the reference offset

        clause_dict = clause.model_dump(mode="json")
        monitor = RelativeFrameMonitor(
            params={"parent_frame": PARENT_FRAME, "child_frame": child_frame,
                     "max_delta_m": max_delta_m},
            step_id="rung2_relative_frame_test",
            recorder=harness.recorder,
            clause=clause_dict,
            halt_on_fire=False,
            monitor_id=None,
            armed_wall_ns=wall_ns(),
        )
        armed = engine.arm(monitor)
        print(f"armed monitor_id={armed['monitor_id']} "
              f"reference={armed['reference_values']}")

        print(f"\nPhase A: quiet window ({args.quiet_sec:.0f}s) — do NOT touch "
              f"{GREEN}. It should stay firmly held.")
        t_end = time.monotonic() + args.quiet_sec
        spurious = []
        while time.monotonic() < t_end:
            spurious.extend(engine.pop_fires())
            time.sleep(POLL_SEC)
        if spurious:
            print(f"  FAIL: {len(spurious)} spurious fire(s) while genuinely "
                  f"held — either the grasp isn't as firm as expected, or "
                  f"max_delta_m={max_delta_m} is tighter than normal hold "
                  f"jitter. First: {spurious[0]}")
            ok = False
        else:
            print("  PASS: no fires while held normally.")

        if not ok:
            print("\nStopping before Phase B.")
            return 1

        print(f"\nPhase B: go pull {GREEN} out of the gripper by hand in "
              f"Isaac Sim NOW (simulate a slip). Waiting up to "
              f"{args.prompt_sec:.0f}s for a fire...")
        t_end = time.monotonic() + args.prompt_sec
        fires = []
        while time.monotonic() < t_end:
            fires.extend(engine.pop_fires())
            if fires:
                break
            time.sleep(POLL_SEC)

        if not fires:
            print(f"  FAIL: no fire within {args.prompt_sec:.0f}s. Either "
                  f"{GREEN} wasn't pulled, or the pull was smaller than "
                  f"max_delta_m={max_delta_m} m.")
            return 1

        fire = fires[0]
        print(f"  fire observed: triggering_value={fire['triggering_value']:.4f} m "
              f"(threshold {fire['threshold']} m, comparison {fire['comparison']!r})")

        snapshot = engine.snapshot()
        still_armed = any(m["monitor_id"] == armed["monitor_id"] for m in snapshot)
        print(f"  auto-disarm after fire: "
              f"{'FAIL (still armed)' if still_armed else 'PASS'}")
        ok &= not still_armed

        time.sleep(0.3)
        events = harness.recorder.get_events_window(
            fire["trigger_wall_ns"] - 1_000_000_000, wall_ns())
        fire_events = [e for e in events if e["event_type"] == "monitor_fire"]
        loopback_ok = len(fire_events) > 0
        print(f"  monitor_fire observed on /watchdog/events: "
              f"{'PASS' if loopback_ok else 'FAIL'} ({len(fire_events)} found)")
        ok &= loopback_ok

        print(f"\nRUNG 2 / relative_frame: {'PASS' if ok else 'FAIL'}")
        return 0 if ok else 1
    finally:
        if grasped:
            try:
                release_skill(backend, object_id=GREEN)
                move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
            except Exception as exc:  # noqa: BLE001
                print(f"cleanup release/home failed (non-fatal): {exc}")
        shutdown_bus()
        harness.shutdown()
        try:
            exec_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())