"""Rung 3 — halt_on_fire=True: monitor fire composed with halt authority.

Composes what Rung 1 and Rung 2 validated separately:
  Rung 1: halt cancels MoveGroup and the arm actually stops.
  Rung 2: each monitor primitive fires on a real violation (flag only).
  Rung 3: a monitor firing AUTOMATICALLY halts the arm, with no agent in the
          loop — the whole point of halt_on_fire.

SCENARIO — why stationarity on red_cube during an arm motion:
  Halt authority is deliberately narrow: HaltClient cancels MoveGroup goals
  and nothing else. It CANNOT stop a gripper close. So the violation must
  occur while the ARM is moving, or there is nothing for the halt to cancel.
  A stationarity monitor on red_cube (a bystander object nobody should be
  touching) while the arm executes an unrelated motion is exactly that: an
  invariant violated by an external disturbance, mid-motion.

  red_cube is never grasped here, only observed — so this test is unaffected
  by red's known grasp-offset / link-collision issue.

The arm loops between two poses above green_cube so there is sustained motion
while you disturb red by hand in Isaac Sim.

WHAT IS CHECKED (all six must pass):
  1. Pre-disturbance: the arm is CONFIRMED moving before the fire — otherwise
     "the arm stopped" proves nothing (the Rung 1 lesson).
  2. The fire records action_taken == "halt", not "flag".
  3. The arm ACTUALLY stopped, measured from the recorder's own joint buffer.
  4. Event ordering: the halt event precedes the monitor_fire event on
     /watchdog/events. MonitorEngine._handle_fire calls halt BEFORE building
     the fire payload and BEFORE setting wake_event — this check proves the
     halt did not wait on any agent-facing bookkeeping.
  5. The motion thread was preempted (MoveGroup goal cancelled out from under
     it) — independent corroboration that the cancel reached the executor.
  6. The monitor auto-disarmed after firing (one-shot semantics).

Run (full stack + Isaac Sim up, workspace sourced, from test/):
    python3 watchdog_rung3_halt_on_fire_test.py
    python3 watchdog_rung3_halt_on_fire_test.py --prompt-sec 40
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from robot_backend.contracts.loader import load_pick_contract
from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.maintenance_skills import (
    set_permitted_contacts as set_permitted_contacts_skill,
)
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.utils.geometry import make_pose
from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.monitors import StationarityMonitor
from robot_backend.watchdog.timebase import wall_ns

from watchdog_smoke_test import WatchDogHarness, _max_joint_travel

WORLD_FRAME = "world"
GREEN = "green_cube"
RED = "red_cube"
SAFE_POSE_NAME = "ready"
GRASP_WIDTH_M = 0.04
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
STEP_ID = "rung3_halt_on_fire_test"

# Two poses above green_cube; the arm shuttles between them so there is
# sustained motion to interrupt. Vertical travel only — no lateral sweep
# toward red_cube, which must stay undisturbed except by your hand.
LOW_Z_OFFSET_M = 0.08
HIGH_Z_OFFSET_M = 0.25

DEFAULT_QUIET_SEC = 4.0
DEFAULT_PROMPT_SEC = 30.0
POLL_SEC = 0.1
PRE_HALT_WINDOW_SEC = 0.8
PRE_HALT_MOTION_MIN_RAD = 0.02
STOP_SETTLE_SEC = 0.4
STOP_OBSERVE_SEC = 1.5
STOP_EPSILON_RAD = 0.01


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
    # THE Rung 3 difference: the engine gets the halt client, so a fire can
    # halt autonomously. Rung 2 passed halt_client=None (flag only).
    engine = MonitorEngine(
        event_bus=harness.event_bus,
        halt_client=harness.halt_client,
        logger=harness.node.get_logger(),
    )
    harness.recorder.engine = engine

    exec_node = Node("watchdog_rung3_exec")
    configure_bus(node=exec_node, log_path=str(log_dir / "rung3_exec.jsonl"))
    backend = create_ros_moveit_backend(node=exec_node)

    stop_flag = threading.Event()
    motion: dict = {}
    thread: threading.Thread | None = None
    ok = True

    try:
        print("waiting for recorder buffers to fill...")
        time.sleep(2.0)
        rec = harness.recorder
        registry = rec.registry

        backend.scene.sync_collision_objects([GREEN, RED])
        set_permitted_contacts_skill(backend, registry=registry)

        green = rec.latest_object_pose(GREEN)
        red = rec.latest_object_pose(RED)
        if green is None or red is None:
            print("FAIL: missing buffered pose for green_cube or red_cube.")
            return 1
        gp = green["pose"]["position"]
        rp = red["pose"]["position"]
        print(f"{RED} baseline: x={rp['x']:.4f} y={rp['y']:.4f} z={rp['z']:.4f}")

        contract = load_pick_contract(RED)
        clause = _find_clause(contract, "object_position_error_m")
        if clause is None:
            print(f"FAIL: {RED}'s contract has no object_position_error_m clause.")
            return 1
        epsilon_m = clause.threshold
        print(f"epsilon_m = {epsilon_m} m, sourced from {RED}'s own contract "
              f"clause: field={clause.field!r} threshold={clause.threshold}")

        print("\nmoving to start pose...")
        move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
        time.sleep(1.0)

        monitor = StationarityMonitor(
            params={"object_id": RED, "epsilon_m": epsilon_m},
            step_id=STEP_ID,
            recorder=rec,
            clause=clause.model_dump(mode="json"),
            halt_on_fire=True,          # <-- Rung 3
            monitor_id=None,
            armed_wall_ns=wall_ns(),
        )
        armed = engine.arm(monitor)
        mid = armed["monitor_id"]
        print(f"armed monitor_id={mid} halt_on_fire=True "
              f"reference={armed['reference_values']}")

        # --- Phase A: no spurious fire while nothing is touched ---
        print(f"\nPhase A: quiet window ({args.quiet_sec:.0f}s), arm stationary, "
              f"do NOT touch {RED}.")
        t_end = time.monotonic() + args.quiet_sec
        spurious = []
        while time.monotonic() < t_end:
            spurious.extend(engine.pop_fires())
            time.sleep(POLL_SEC)
        if spurious:
            print(f"  FAIL: {len(spurious)} spurious fire(s) — and each one "
                  f"issued a HALT. Fix before continuing.")
            return 1
        print("  PASS: no fires while undisturbed.")

        # --- start the shuttle motion ---
        low = _pose(gp["x"], gp["y"], gp["z"] + LOW_Z_OFFSET_M)
        high = _pose(gp["x"], gp["y"], gp["z"] + HIGH_Z_OFFSET_M)

        def _motion_loop() -> None:
            targets = [high, low]
            i = 0
            while not stop_flag.is_set():
                try:
                    move_grasp_center_to_pose_skill(
                        backend, targets[i % 2], gripper_width_m=GRASP_WIDTH_M)
                except Exception as exc:  # noqa: BLE001
                    # A cancelled goal surfacing here is the SUCCESS signal:
                    # the halt reached the executor. Stop; do not re-issue
                    # motion after a halt.
                    motion["preempted_wall_ns"] = wall_ns()
                    motion["error"] = f"{type(exc).__name__}: {exc}"
                    return
                i += 1
            motion["clean_exit"] = True

        print(f"\nstarting shuttle motion between z+{LOW_Z_OFFSET_M:.2f} and "
              f"z+{HIGH_Z_OFFSET_M:.2f} above {GREEN}...")
        thread = threading.Thread(target=_motion_loop, daemon=True)
        thread.start()
        time.sleep(2.0)  # let the first motion get underway

        # --- check 1: confirm the arm is genuinely moving ---
        pre_end = wall_ns()
        pre_start = pre_end - int(PRE_HALT_WINDOW_SEC * 1_000_000_000)
        pre_travel, pre_joint, pre_n = _max_joint_travel(rec, pre_start, pre_end)
        was_moving = pre_travel >= PRE_HALT_MOTION_MIN_RAD
        print(f"  [1] arm confirmed moving: {'PASS' if was_moving else 'FAIL'} "
              f"({pre_travel:.4f} rad on {pre_joint}, {pre_n} samples)")
        if not was_moving:
            print("      -> the shuttle motion isn't producing movement; "
                  "check pose reachability. Aborting: a stop-check on a "
                  "stationary arm proves nothing.")
            return 1
        ok &= was_moving

        # --- trigger ---
        print(f"\nGO DRAG {RED} IN ISAAC SIM NOW — the arm should stop on its "
              f"own. Waiting up to {args.prompt_sec:.0f}s...")
        t_end = time.monotonic() + args.prompt_sec
        fires = []
        while time.monotonic() < t_end:
            fires.extend(engine.pop_fires())
            if fires:
                break
            time.sleep(POLL_SEC)

        if not fires:
            print(f"  FAIL: no fire within {args.prompt_sec:.0f}s.")
            return 1

        fire = fires[0]
        fire_wall = fire["trigger_wall_ns"]
        print(f"\n  fire: triggering_value={fire['triggering_value']:.4f} m "
              f"(threshold {fire['threshold']} m)")

        # --- check 2: the fire actually halted rather than just flagged ---
        halted = fire.get("action_taken") == "halt"
        print(f"  [2] action_taken == 'halt': {'PASS' if halted else 'FAIL'} "
              f"(got {fire.get('action_taken')!r})")
        ok &= halted

        # --- check 3: arm actually stopped ---
        time.sleep(STOP_SETTLE_SEC)
        obs_start = wall_ns()
        time.sleep(STOP_OBSERVE_SEC)
        obs_end = wall_ns()
        travel, joint, n = _max_joint_travel(rec, obs_start, obs_end)
        stopped = travel <= STOP_EPSILON_RAD
        print(f"  [3] arm actually stopped: {'PASS' if stopped else 'FAIL'} "
              f"(max travel {travel:.4f} rad on {joint}, {n} samples, "
              f"epsilon {STOP_EPSILON_RAD})")
        ok &= stopped

        # --- check 4: halt was published BEFORE the fire event ---
        events = rec.get_events_window(fire_wall - 2_000_000_000, wall_ns())
        halt_idx = next(
            (i for i, e in enumerate(events)
             if e["event_type"] == "halt" and e["payload"].get("monitor_id") == mid),
            None)
        fire_idx = next(
            (i for i, e in enumerate(events)
             if e["event_type"] == "monitor_fire"
             and e["payload"].get("monitor_id") == mid),
            None)
        if halt_idx is None or fire_idx is None:
            ordering_ok = False
            detail = (f"halt_event={'found' if halt_idx is not None else 'MISSING'}, "
                      f"fire_event={'found' if fire_idx is not None else 'MISSING'}")
        else:
            ordering_ok = halt_idx < fire_idx
            detail = f"halt at index {halt_idx}, monitor_fire at index {fire_idx}"
        print(f"  [4] halt precedes monitor_fire on /watchdog/events: "
              f"{'PASS' if ordering_ok else 'FAIL'} ({detail})")
        if not ordering_ok and halt_idx is not None and fire_idx is not None:
            print("      -> halt is being issued AFTER the agent-facing fire "
                  "bookkeeping. The design requires halt first so it never "
                  "waits on the LLM.")
        ok &= ordering_ok

        # --- check 5: the motion thread was preempted ---
        thread.join(timeout=10.0)
        preempted = "preempted_wall_ns" in motion
        print(f"  [5] motion thread preempted by the cancel: "
              f"{'PASS' if preempted else 'FAIL'}")
        if preempted:
            lag_ms = (motion["preempted_wall_ns"] - fire_wall) / 1e6
            print(f"      {motion['error']}")
            print(f"      preempt observed {lag_ms:+.0f} ms after the fire")
        else:
            print("      -> the executor never saw its goal cancelled; the "
                  "arm may have stopped for an unrelated reason.")
        ok &= preempted

        # --- check 6: auto-disarm ---
        still_armed = any(m["monitor_id"] == mid for m in engine.snapshot())
        print(f"  [6] auto-disarm after fire: "
              f"{'FAIL (still armed)' if still_armed else 'PASS'}")
        ok &= not still_armed

        print(f"\nRUNG 3: {'PASS' if ok else 'FAIL'}")
        return 0 if ok else 1
    finally:
        stop_flag.set()
        if thread is not None:
            thread.join(timeout=10.0)
        try:
            move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
        except Exception as exc:  # noqa: BLE001
            print(f"cleanup home failed (non-fatal): {exc}")
        shutdown_bus()
        harness.shutdown()
        try:
            exec_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\nremember to drag {RED} back to its original spot.")


if __name__ == "__main__":
    sys.exit(main())
