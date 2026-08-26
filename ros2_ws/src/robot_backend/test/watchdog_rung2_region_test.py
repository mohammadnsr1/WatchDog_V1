"""Rung 2 (primitive 3/4) — RegionMonitor, "stay_outside" keepout zone.

Fully scripted, unlike the stationarity and relative-frame tests — no manual
disturbance needed. The violation is a real grasp-and-stack of green_cube
directly on top of red_cube (the same Task-A motion from
pick_place_stress_test.py), which deterministically drives green_cube's pose
into a keepout box built around red_cube.

Contract sourcing is different in kind here: there is no existing Clause
for "keepout region" in PickContract (region/keepout has no representation
in the current schema — confirmed absent in the earlier status report). The
region's geometry is instead derived directly from registry data that IS
authoritative: each object's resolved collision AABB. half_extents are built
from red_cube's and green_cube's half-widths plus a small margin, so the box
just barely encloses the real contact geometry — a genuine registry-sourced
parameter, just not routed through a Clause object. The `clause` argument
passed to arm_monitor is a plain descriptive dict for provenance, not a
PickContract Clause instance (arm_monitor's clause parameter accepts any
dict).

Phase A (baseline, green_cube at its normal resting spot): confirm NO fire —
sanity-checks the box geometry itself doesn't misfire on the object's own
resting pose.
Phase B (scripted stack onto red_cube): confirm a fire as green_cube's
placement drives it into the keepout box.

Run (full stack up, workspace sourced, from test/):
    python3 watchdog_rung2_region_test.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.registry import Registry, load_registry
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
from robot_backend.watchdog.monitors import RegionMonitor
from robot_backend.watchdog.timebase import wall_ns

from watchdog_smoke_test import WatchDogHarness

WORLD_FRAME = "world"
GREEN = "green_cube"
RED = "red_cube"
SAFE_POSE_NAME = "ready"
GRASP_WIDTH_M = 0.04
APPROACH_HEIGHT_M = 0.10
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}

KEEPOUT_MARGIN_M = 0.01     # small buffer beyond the exact contact geometry
BASELINE_QUIET_SEC = 3.0
SETTLE_SEC = 1.0


def _pose(x, y, z):
    return make_pose(position={"x": x, "y": y, "z": z},
                      orientation=dict(TOP_DOWN_QUAT), frame_id=WORLD_FRAME)


def _resolved_registry() -> Registry:
    from ament_index_python.packages import get_package_share_directory

    path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    return load_registry(path)


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

    exec_node = Node("watchdog_rung2_region_exec")
    configure_bus(node=exec_node, log_path=str(log_dir / "rung2_region_exec.jsonl"))
    backend = create_ros_moveit_backend(node=exec_node)

    ok = True
    grasped = False
    try:
        print("waiting for recorder buffers to fill...")
        time.sleep(2.0)
        registry = harness.recorder.registry
        resolved_registry = _resolved_registry()

        # Scene setup. sync is required before attach; set_permitted_contacts
        # is required because Phase B deliberately stacks green on red, which
        # is exactly the declared acm_allowed_object_pairs entry.
        backend.scene.sync_collision_objects([GREEN, RED])
        set_permitted_contacts_skill(backend, registry=registry)

        red_pose = harness.recorder.latest_object_pose(RED)
        green_pose = harness.recorder.latest_object_pose(GREEN)
        if red_pose is None or green_pose is None:
            print("FAIL: missing buffered pose for red_cube or green_cube.")
            return 1
        rp = red_pose["pose"]["position"]
        gp0 = green_pose["pose"]["position"]

        _, red_half = resolved_registry.objects[RED].resolve_aabb()
        _, green_half = resolved_registry.objects[GREEN].resolve_aabb()
        center = {"x": rp["x"], "y": rp["y"], "z": rp["z"]}
        half_extents = {
            axis: red_half[axis] + green_half[axis] + KEEPOUT_MARGIN_M
            for axis in ("x", "y", "z")
        }
        print(f"keepout box around {RED}: center={center} "
              f"half_extents={half_extents} "
              f"(derived from {RED}/{GREEN} resolved AABBs + "
              f"{KEEPOUT_MARGIN_M} m margin)")

        clause = {
            "field": f"{GREEN}_outside_{RED}_keepout",
            "comparison": "gt",
            "threshold": 0.0,
            "description": (
                f"{GREEN} must stay outside a keepout box around {RED}'s "
                f"resting pose; box half-extents derived from both cubes' "
                f"registry resolved AABBs plus a fixed margin, not "
                f"from a PickContract Clause (region/keepout has no clause "
                f"representation in the current schema)."
            ),
            "unit": "m",
        }

        monitor = RegionMonitor(
            params={"object_id": GREEN, "center": center,
                     "half_extents": half_extents, "mode": "stay_outside"},
            clause=clause,
            halt_on_fire=False,
            monitor_id=None,
            armed_wall_ns=wall_ns(),
        )
        armed = engine.arm(monitor)
        print(f"armed monitor_id={armed['monitor_id']}")

        print(f"\nPhase A: baseline check ({BASELINE_QUIET_SEC:.0f}s), "
              f"{GREEN} at its resting pose x={gp0['x']:.3f} y={gp0['y']:.3f} — "
              f"should be well outside the keepout box.")
        t_end = time.monotonic() + BASELINE_QUIET_SEC
        spurious = []
        while time.monotonic() < t_end:
            spurious.extend(engine.pop_fires())
            time.sleep(0.2)
        if spurious:
            print(f"  FAIL: {len(spurious)} spurious fire(s) at baseline — the "
                  f"box geometry itself is wrong for this scene layout. First: "
                  f"{spurious[0]}")
            ok = False
        else:
            print("  PASS: no fire at baseline.")

        if not ok:
            print("\nStopping before Phase B.")
            return 1

        print(f"\nPhase B: scripted stack — grasping {GREEN} and placing it "
              f"directly on top of {RED} (same motion as pick_place_stress_test "
              f"Task A). This should drive {GREEN} into the keepout box.")
        move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            backend, _pose(gp0["x"], gp0["y"], gp0["z"] + APPROACH_HEIGHT_M),
            gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            backend, _pose(gp0["x"], gp0["y"], gp0["z"]), gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        res = grasp_skill(backend, object_id=GREEN, registry=registry)
        if not res.get("ok"):
            print(f"FAIL: grasp failed, cannot complete the stack: "
                  f"{res.get('reason')}")
            return 1
        grasped = True

        stack_z = rp["z"] + red_half["z"] + green_half["z"]
        move_grasp_center_to_pose_skill(
            backend, _pose(gp0["x"], gp0["y"], gp0["z"] + APPROACH_HEIGHT_M),
            gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            backend, _pose(rp["x"], rp["y"], stack_z + APPROACH_HEIGHT_M),
            gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)

        fires = list(engine.pop_fires())  # check before the final descend too
        move_grasp_center_to_pose_skill(
            backend, _pose(rp["x"], rp["y"], stack_z), gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        fires.extend(engine.pop_fires())

        if not fires:
            print("  FAIL: no fire despite stacking onto red_cube. Check the "
                  "keepout box dimensions against the actual stack geometry.")
            return 1

        fire = fires[0]
        print(f"  fire observed: triggering_value={fire['triggering_value']:.4f} "
              f"(clearance into the box; <=0 means inside)")

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

        print(f"\nRUNG 2 / region: {'PASS' if ok else 'FAIL'}")
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
        print(f"\nremember: {GREEN} is now stacked on {RED}. Reposition before "
              f"other tests that assume green_cube's original resting pose.")


if __name__ == "__main__":
    sys.exit(main())
