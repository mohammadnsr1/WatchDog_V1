"""Rung 2 (primitive 4/4) — EffortEventMonitor, fully scripted via real grasps.

Two phases, both using real grasp_skill calls on green_cube — no manual
disturbance needed, since the guarded-move close reliably produces a real,
sustained effort signal (per the earlier grasp/effort work).

Phase A — COMPLIANCE check, using the real safety clause:
    green_cube's contract invariant is
        grasp_effort_instantaneous LE max_clamp_effort   (max_clamp_effort = 4.0)
    max_clamp_effort is the conformance BOUND (spec.limits.max_clamp_effort) —
    what the object tolerates — distinct from force_threshold_n, the guarded
    close's stop TRIGGER, which cannot bound achievable effort. The monitor is
    armed with comparison="le", threshold=max_clamp_effort — the same
    comparison and threshold as the clause itself, copied unchanged — and it
    fires exactly when that invariant is violated. A normal, healthy grasp is
    EXPECTED to pass this (no fire): at the configured 3 mm clearance, settled
    effort is ~3.42, under the 4.0 bound. So Phase A's correct, designed
    outcome is NO FIRE — that is what "the grasp is conformant with its own
    effort invariant" looks like. This is a genuine test of the monitor's
    specificity (it doesn't cry wolf on a compliant grasp), not a placeholder.

Phase B — MECHANISM check, using a deliberately lower threshold:
    Re-arm on the same joint with threshold = max_clamp_effort * 0.5, well
    above the RG2's idle noise floor (~0.03) but comfortably below what a
    real contact grasp reaches. This should fire, proving the monitor
    actually detects a real crossing (rather than being silently broken in a
    way Phase A's "no fire" result couldn't distinguish from "not working").

sustained_ticks defaults to 3 (see monitors.py) for both phases — this test
does not attempt to probe the debounce boundary itself; that's covered by
the offline unit tests. This test validates real-hardware behavior end to
end: does a real grasp correctly comply with its own safety invariant, and
does the monitor mechanism actually fire when it should.

Run (full stack up, workspace sourced, from test/):
    python3 watchdog_rung2_effort_event_test.py
"""

from __future__ import annotations

import argparse
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
from robot_backend.watchdog.monitors import EffortEventMonitor
from robot_backend.watchdog.timebase import wall_ns

from watchdog_smoke_test import WatchDogHarness

WORLD_FRAME = "world"
GREEN = "green_cube"
SAFE_POSE_NAME = "ready"
GRASP_WIDTH_M = 0.04
APPROACH_HEIGHT_M = 0.10
FINGER_JOINT = "finger_joint"
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}

HOLD_SEC = 2.0
SETTLE_SEC = 1.0
PHASE_B_THRESHOLD_FACTOR = 0.5


def _pose(x, y, z):
    return make_pose(position={"x": x, "y": y, "z": z},
                      orientation=dict(TOP_DOWN_QUAT), frame_id=WORLD_FRAME)


def _find_clause(contract, field: str):
    for clause in contract.preconditions + contract.effects + contract.invariants:
        if clause.field == field:
            return clause
    return None


def _do_grasp(backend, registry, recorder) -> bool:
    pose = recorder.latest_object_pose(GREEN)
    if pose is None:
        print(f"FAIL: no buffered pose for {GREEN}.")
        return False
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
    res = grasp_skill(backend, object_id=GREEN, registry=registry)
    if not res.get("ok"):
        print(f"FAIL: grasp failed: {res.get('reason')}")
        return False
    return True


def _run_phase(harness, engine, backend, registry, *, threshold: float,
               clause: dict | None, expect_fire: bool, label: str) -> tuple[bool, list]:
    print(f"\n{label}: threshold={threshold} comparison=le, "
          f"expect_fire={expect_fire}")
    if not _do_grasp(backend, registry, harness.recorder):
        return False, []

    monitor = EffortEventMonitor(
        params={"joint_name": FINGER_JOINT, "comparison": "le",
                 "threshold": threshold, "sustained_ticks": 3},
        step_id="rung2_effort_event_test",
        clause=clause,
        halt_on_fire=False,
        monitor_id=None,
        armed_wall_ns=wall_ns(),
    )
    armed = engine.arm(monitor)
    print(f"  armed monitor_id={armed['monitor_id']}, holding {HOLD_SEC:.0f}s...")

    fires = []
    t_end = time.monotonic() + HOLD_SEC
    while time.monotonic() < t_end:
        fires.extend(engine.pop_fires())
        time.sleep(0.05)

    fired = bool(fires)
    ok = fired == expect_fire
    if fired:
        fire = fires[0]
        print(f"  fire observed: triggering_value={fire['triggering_value']:.4f} "
              f"(threshold {fire['threshold']})")
    else:
        print("  no fire.")
    print(f"  result: {'PASS' if ok else 'FAIL'} "
          f"(expected fire={expect_fire}, got fire={fired})")

    if not fired:
        engine.disarm(armed["monitor_id"], "phase_complete_no_fire")

    release_skill(backend, object_id=GREEN)
    move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
    time.sleep(SETTLE_SEC)
    return ok, fires


def main() -> int:
    ap = argparse.ArgumentParser()
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

    exec_node = Node("watchdog_rung2_effort_exec")
    configure_bus(node=exec_node, log_path=str(log_dir / "rung2_effort_exec.jsonl"))
    backend = create_ros_moveit_backend(node=exec_node)

    ok = True
    try:
        print("waiting for recorder buffers to fill...")
        time.sleep(2.0)
        registry = harness.recorder.registry

        # Scene setup, same as every other executor-driving test: MoveIt cannot
        # attach an object that is not in the world collision scene. Both
        # phases re-grasp, but release_skill re-syncs green each time, so once
        # here is enough.
        backend.scene.sync_collision_objects([GREEN])
        set_permitted_contacts_skill(backend, registry=registry)

        # grasp_width_m is instruction-sourced, not registry-sourced (see
        # agents/tasks/single_step_pick.{executor,watchdog}.yaml's steps:
        # block); this manual test
        # script has no task YAML of its own, so it stands in with GRASP_WIDTH_M.
        step_spec = {
            "step_id": "rung2_effort_event_test",
            "object_id": GREEN,
            "grasp_width_m": GRASP_WIDTH_M,
        }
        contract = load_pick_contract(GREEN, step_spec)
        clause = _find_clause(contract, "grasp_effort_instantaneous")
        if clause is None:
            print("FAIL: green_cube's contract has no grasp_effort_instantaneous clause.")
            return 1
        max_clamp_effort = clause.threshold
        print(f"max_clamp_effort = {max_clamp_effort}, sourced from "
              f"green_cube's own contract clause: field={clause.field!r} "
              f"comparison={clause.comparison.value!r} "
              f"(\"{clause.description}\")")

        ok_a, _ = _run_phase(
            harness, engine, backend, registry,
            threshold=max_clamp_effort,
            clause=clause.model_dump(mode="json"),
            expect_fire=False,
            label="Phase A (compliance check, real safety threshold)",
        )
        ok &= ok_a
        if not ok_a:
            print("\n  If this unexpectedly fired: the grasp's instantaneous "
                  "effort is exceeding its own safety threshold even in "
                  "normal operation — that's worth investigating on its own, "
                  "separately from this test.")

        threshold_b = max_clamp_effort * PHASE_B_THRESHOLD_FACTOR
        ok_b, fires_b = _run_phase(
            harness, engine, backend, registry,
            threshold=threshold_b,
            clause={
                "field": "grasp_effort_instantaneous",
                "comparison": "le",
                "threshold": threshold_b,
                "description": (
                    f"Mechanism check only: {PHASE_B_THRESHOLD_FACTOR} x "
                    f"green_cube's real max_clamp_effort, deliberately "
                    f"lowered to confirm the monitor fires on a real "
                    f"crossing rather than being silently inert."
                ),
                "unit": "sim_effort",
            },
            expect_fire=True,
            label="Phase B (mechanism check, lowered threshold)",
        )
        ok &= ok_b

        if ok_b and fires_b:
            fire = fires_b[0]
            snapshot = engine.snapshot()
            still_armed = any(
                m.get("monitor_id") == fire.get("monitor_id") for m in snapshot
            )
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

        print(f"\nRUNG 2 / effort_event: {'PASS' if ok else 'FAIL'}")
        return 0 if ok else 1
    finally:
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