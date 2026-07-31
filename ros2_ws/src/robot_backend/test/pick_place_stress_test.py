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
  4. pick green, place RIGHT_OFFSET_M RIGHT of red (-y)
Loops N times; prints PASS/FAIL per cycle and a final tally.

Robustness (mirrors grasp_effort_test.py, now that the grasp is repeatable):
  - Place retreats from the KNOWN pick pose, not the attached object's TF
    (once grasped, green's TF rides with the gripper, so reading it back is
    circular). Deterministic up-and-over.
  - Success checks read the object pose only after it has SETTLED (two
    consecutive stable reads), so a still-settling cube is not misread as a
    misplacement.
  - Failures are labeled by phase (A:pick, A:place, A:check, B:pick, ...) and
    grasp failures carry full provenance, so a FAIL says which step broke and
    why instead of just surfacing a bare exception.
  - Clean executor/thread teardown to avoid the "Destroyable ... destruction
    was requested" shutdown noise.

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

from robot_backend.registry import Registry, load_registry
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import grasp_skill, release_skill
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.skills.maintenance_skills import (
    set_permitted_contacts as set_permitted_contacts_skill,
)
from robot_backend.utils.geometry import make_pose

# ------------------------- CONFIG -------------------------
WORLD_FRAME = "world"            # set to "World" if that's the cube parent frame
GREEN = "green_cube"
RED = "red_cube"
SAFE_POSE_NAME = "ready"
GRASP_WIDTH_M = 0.04
APPROACH_HEIGHT_M = 0.10
RIGHT_OFFSET_M = 0.10           # Task B: place green this far in -y from red
SETTLE_SEC = 1.0
SETTLE_EPSILON_M = 0.002        # pose-settled threshold for success checks
SETTLE_TIMEOUT_SEC = 3.0
PLACE_Z_TOLERANCE_M = 0.01      # z-tolerance for stack/rest height checks
XY_TOLERANCE_M = 0.03           # x/y-tolerance for placement checks
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
# ----------------------------------------------------------


def _load_registry() -> Registry | None:
    from pathlib import Path
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        return None
    try:
        return load_registry(path)
    except (OSError, ValueError):
        return None


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
        # Order matters: stop the executor, let the spin thread exit, THEN
        # destroy the node. Destroying the node while the background thread is
        # still spinning it is what produces the "cannot use Destroyable
        # because destruction was requested" / "exception was never retrieved"
        # noise at process exit.
        self.executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()


class Cycle:
    def __init__(self, backend, tf, registry: dict, resolved_registry: Registry):
        self.b = backend
        self.tf = tf
        self.registry = registry
        self.resolved_registry = resolved_registry

    def _object_height_m(self, object_id: str) -> float:
        _, half_extents = self.resolved_registry.objects[object_id].resolve_aabb()
        return 2.0 * half_extents["z"]

    def _wait_settled(self, object_id: str) -> dict:
        """Return the object pose once it stops moving.

        Reads the pose repeatedly and returns as soon as two consecutive
        reads agree within SETTLE_EPSILON_M on all axes, or the last read at
        SETTLE_TIMEOUT_SEC. This prevents a still-settling cube (physics has
        not caught up right after release) from being misread as a
        misplacement by the success checks.
        """
        prev = self.tf.pose(object_id)
        end = time.time() + SETTLE_TIMEOUT_SEC
        while time.time() < end:
            time.sleep(0.3)
            cur = self.tf.pose(object_id)
            if (abs(cur["x"] - prev["x"]) < SETTLE_EPSILON_M
                    and abs(cur["y"] - prev["y"]) < SETTLE_EPSILON_M
                    and abs(cur["z"] - prev["z"]) < SETTLE_EPSILON_M):
                return cur
            prev = cur
        return prev

    def _pick(self) -> dict:
        """Pick green from its current pose. Returns the grasp pose used.

        The returned pose is the pre-grasp green location; _place uses it as
        the retreat reference instead of reading green's TF back while it is
        attached (attached, green's frame rides with the gripper).
        """
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
            prov = res.get("provenance", {})
            raise RuntimeError(
                f"grasp failed: {res.get('reason', res)} | "
                f"stop_reason={prov.get('stop_reason')!r} "
                f"final_effort={prov.get('final_effort')} "
                f"final_width_m={prov.get('final_width_m')} "
                f"asymmetry={prov.get('asymmetry_detected')}"
            )
        return g

    def _place(self, pick_pose: dict, x, y, z):
        """Place green at (x, y, z), retreating from the known pick pose.

        The first up-move references pick_pose (deterministic) rather than
        green's attached TF, which rides with the gripper once grasped. Target
        z is the object-center resting height, so opening the gripper leaves
        the cube resting (no freefall / re-rotation).
        """
        move_grasp_center_to_pose_skill(
            self.b,
            _pose(pick_pose["x"], pick_pose["y"], pick_pose["z"] + APPROACH_HEIGHT_M),
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
        phase = "init"
        try:
            phase = "sync"
            self.b.scene.sync_collision_objects([GREEN, RED])
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)

            # Task A: green on top of red
            phase = "A:pick"
            pick_pose = self._pick()
            red = self.tf.pose(RED)
            stack_z = (
                red["z"]
                + self._object_height_m(RED) / 2.0
                + self._object_height_m(GREEN) / 2.0
            )
            phase = "A:place"
            self._place(pick_pose, red["x"], red["y"], stack_z)
            phase = "A:check"
            g = self._wait_settled(GREEN)
            if (abs(g["x"] - red["x"]) > XY_TOLERANCE_M
                    or abs(g["y"] - red["y"]) > XY_TOLERANCE_M
                    or abs(g["z"] - stack_z) > PLACE_Z_TOLERANCE_M):
                return False, (
                    f"[A:check] stack misplaced green={g} red={red} "
                    f"expected_z={stack_z:.4f} "
                    f"dz={g['z'] - stack_z:+.4f}"
                )
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)

            # Task B: green RIGHT_OFFSET_M to the right (-y) of red, on the table
            phase = "B:pick"
            pick_pose = self._pick()
            red = self.tf.pose(RED)
            tx = red["x"]
            ty = red["y"] - RIGHT_OFFSET_M
            tz = self._object_height_m(GREEN) / 2.0
            phase = "B:place"
            self._place(pick_pose, tx, ty, tz)
            phase = "B:check"
            g = self._wait_settled(GREEN)
            if (abs(g["x"] - tx) > XY_TOLERANCE_M
                    or abs(g["y"] - ty) > XY_TOLERANCE_M
                    or abs(g["z"] - tz) > PLACE_Z_TOLERANCE_M):
                return False, (
                    f"[B:check] right-place misplaced green={g} "
                    f"target=({tx:.3f},{ty:.3f},{tz:.3f}) "
                    f"dz={g['z'] - tz:+.4f}"
                )
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)
            return True, "ok"
        except Exception as exc:  # noqa: BLE001
            return False, f"[{phase}] {type(exc).__name__}: {exc}"


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
    resolved_registry = _load_registry()
    if resolved_registry is None:
        print("WARN: registry empty; grasp width falls back to 0 and will fail.")
        registry = {}
    else:
        registry = resolved_registry.raw_dict

    set_permitted_contacts_skill(backend, registry=registry)

    passes = 0
    fails = []
    for i in range(1, args.cycles + 1):
        print(f"\n===== CYCLE {i}/{args.cycles} =====")
        if resolved_registry is None:
            ok, msg = False, "[init] validated registry unavailable"
        else:
            ok, msg = Cycle(backend, tf, registry, resolved_registry).run()
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
