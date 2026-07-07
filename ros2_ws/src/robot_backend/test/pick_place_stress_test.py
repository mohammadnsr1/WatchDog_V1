"""Repeatable pick-and-place stress test — no agent, no shared-node deadlock.

Structure that avoids the rclpy "two things spinning one node" footgun:
  - The BACKEND owns its own node(s) (created via create_ros_moveit_backend(None)).
    The motion/gripper/scene SKILLS spin those nodes themselves via their internal
    spin_until_future_complete when called. The test never spins them.
  - The TEST owns a SEPARATE node just for the TF listener, spun continuously by a
    background MultiThreadedExecutor thread (the calibrate_rg2_width.py pattern),
    so cube poses stay fresh without the test hand-spinning anything.

Per cycle:
  1. sync scene   2. safe pose
  3. pick green, place ON TOP of red
  4. pick green, place 5 cm RIGHT of red (-y)
Loops N times; prints PASS/FAIL per cycle and a final tally.

Run (full stack up, workspace sourced):
    python3 pick_place_stress_test.py --cycles 20
"""

from __future__ import annotations

import argparse
import threading
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import tf2_ros

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import grasp_skill, release_skill
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.skills.maintenance_skills import set_permitted_contacts as set_permitted_contacts_skill
from robot_backend.utils.geometry import make_pose

# ------------------------- CONFIG -------------------------
WORLD_FRAME = "world"            # set to "World" if that's the cube parent frame
GREEN = "green_cube"
RED = "red_cube"
SAFE_POSE_NAME = "ready"
CUBE_HEIGHT_M = 0.05
GRASP_WIDTH_M = 0.04
APPROACH_HEIGHT_M = 0.10
RIGHT_OFFSET_M = 0.05
SETTLE_SEC = 1.0
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
# ----------------------------------------------------------


def _load_registry() -> dict:
    from pathlib import Path
    import yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        return {}
    try:
        with path.open("r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except OSError:
        return {}


def _pose(x, y, z):
    return make_pose(
        position={"x": x, "y": y, "z": z},
        orientation=dict(TOP_DOWN_QUAT),
        frame_id=WORLD_FRAME,
    )


class TFReader:
    """Owns its own node + listener, spun by a background executor thread."""

    def __init__(self):
        self.node = Node("stress_test_tf")
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self.node)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def wait_ready(self, timeout=15.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.buf.can_transform(WORLD_FRAME, GREEN, rclpy.time.Time()) and \
               self.buf.can_transform(WORLD_FRAME, RED, rclpy.time.Time()):
                return True
            time.sleep(0.1)
        return False

    def pose(self, child) -> dict:
        tf = self.buf.lookup_transform(WORLD_FRAME, child, rclpy.time.Time())
        t = tf.transform.translation
        return {"x": t.x, "y": t.y, "z": t.z}

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


class Cycle:
    def __init__(self, backend, tf, registry):
        self.b = backend
        self.tf = tf
        self.registry = registry

    def _pick(self):
        g = self.tf.pose(GREEN)
        move_grasp_center_to_pose_skill(
            self.b, _pose(g["x"], g["y"], g["z"] + APPROACH_HEIGHT_M),
            gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            self.b, _pose(g["x"], g["y"], g["z"]), gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        res = grasp_skill(self.b, object_id=GREEN, registry=self.registry)
        if not res.get("ok"):
            raise RuntimeError(f"grasp failed: {res.get('reason', res)}")

    def _place(self, x, y, z):
        cur = self.tf.pose(GREEN)
        move_grasp_center_to_pose_skill(
            self.b, _pose(cur["x"], cur["y"], cur["z"] + APPROACH_HEIGHT_M),
            gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            self.b, _pose(x, y, z + APPROACH_HEIGHT_M), gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            self.b, _pose(x, y, z), gripper_width_m=GRASP_WIDTH_M)
        time.sleep(SETTLE_SEC)
        rel = release_skill(self.b, object_id=GREEN)
        if not rel.get("ok"):
            raise RuntimeError(f"release failed: {rel}")
        time.sleep(SETTLE_SEC)

    def run(self):
        try:
            self.b.scene.sync_collision_objects([GREEN, RED])
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)

            # Task A: green on top of red
            self._pick()
            red = self.tf.pose(RED)
            self._place(red["x"], red["y"], red["z"] + CUBE_HEIGHT_M)
            time.sleep(SETTLE_SEC)
            g = self.tf.pose(GREEN)
            if abs(g["x"] - red["x"]) > 0.03 or abs(g["y"] - red["y"]) > 0.03:
                return False, f"stack misplaced green={g} red={red}"
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)

            # Task B: green 5cm right (-y) of red
            self._pick()
            red = self.tf.pose(RED)
            tx, ty, tz = red["x"], red["y"] - RIGHT_OFFSET_M, CUBE_HEIGHT_M / 2.0
            self._place(tx, ty, tz)
            time.sleep(SETTLE_SEC)
            g = self.tf.pose(GREEN)
            if abs(g["x"] - tx) > 0.03 or abs(g["y"] - ty) > 0.03:
                return False, f"right-place misplaced green={g} target=({tx:.3f},{ty:.3f})"
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)
            return True, "ok"
        except Exception as exc:  # noqa: BLE001
            return False, f"{type(exc).__name__}: {exc}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=10)
    args = ap.parse_args()

    if not rclpy.ok():
        rclpy.init()

    tf = TFReader()
    if not tf.wait_ready():
        print(f"ERROR: cube TF not available under parent '{WORLD_FRAME}'. "
              "Check WORLD_FRAME / frame names / that the full stack is up.")
        tf.shutdown()
        return 1

    backend = create_ros_moveit_backend(node=None)  # backend owns its own node(s)
    registry = _load_registry()
    if not registry:
        print("WARN: registry empty; grasp width falls back to 0 and will fail.")

    set_permitted_contacts_skill(backend, registry=registry)

    passes = 0
    fails = []
    for i in range(1, args.cycles + 1):
        print(f"\n===== CYCLE {i}/{args.cycles} =====")
        ok, msg = Cycle(backend, tf, registry).run()
        print(f"CYCLE {i}: {'PASS' if ok else 'FAIL'} — {msg}")
        passes += int(ok)
        if not ok:
            fails.append((i, msg))

    print("\n" + "=" * 50)
    print(f"RESULT: {passes}/{args.cycles} passed")
    for i, msg in fails:
        print(f"  cycle {i} FAIL: {msg}")

    tf.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if passes == args.cycles else 1


if __name__ == "__main__":
    sys.exit(main())
