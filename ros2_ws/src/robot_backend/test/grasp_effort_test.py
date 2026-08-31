"""Grasp-only stress test with continuous finger-joint effort logging.

Isolates the PICK section of pick_place_stress_test.py so the RG2 grasp /
guarded-move behavior can be characterized on its own, with the finger_joint
effort sampled continuously for the entire run (not just at the grasp
plateau). Use this to tell apart the two failure modes:

  1. The grasp never really engages  -> effort spikes briefly then falls;
     stop_reason may still be "force_threshold" on a transient.
  2. The grasp engages but slips      -> effort plateaus at grasp, then decays
     during the post-grasp static hold (or during the optional lift).

Node-ownership follows the same rule as pick_place_stress_test.py:
  - The BACKEND owns its own node(s); the motion/gripper skills spin those
    themselves. The test never spins the backend node.
  - The TEST owns TWO separate background-spun nodes: one TF listener
    (cube poses) and one /joint_states subscriber (effort trace). Both are
    read-only extra subscribers, safe to run alongside the skills.

Per cycle (pick only):
  1. sync scene   2. safe pose
  3. approach green from APPROACH_HEIGHT_M above
  4. descend to grasp pose
  5. grasp_skill (torque mode)  -> log full provenance
  6. static hold for HOLD_SEC   -> watch effort decay
  7. (optional --lift) lift LIFT_HEIGHT_M, hold under load, then place the
     cube back at its original resting pose so the next cycle is repeatable
  8. release (cube already resting on table), return to safe pose

Placing the cube back before release (rather than opening in mid-air) keeps
the cube's pose/orientation identical from cycle to cycle. An air-release
drops the cube, it resettles rotated, and the following grasp starts from a
different state — which masks whether the grasp itself is repeatable.

Console prints per-cycle grasp provenance + effort-window summaries.
A full per-sample effort trace for every cycle is written to --csv
(default grasp_effort_trace.csv) with a phase label per row, so you can
plot effort vs time and see exactly where it lets go.

The grasp width is instruction-sourced and matters: it becomes grasp_skill's
expected_width_m, which is what makes the probe's commanded floor
width - GRASP_CLEARANCE_M. Omit it and the floor is FULLY CLOSED, so the
effort measured is "squeezed all the way into the part" rather than "effort at
the configured clearance". Known objects default; anything else needs
--width_m. Asymmetric parts also need their instruction grasp_center_offset,
loaded automatically from agents/tasks/<object>_pick.executor.yaml.

Run (full stack up, workspace sourced):
    python3 grasp_effort_test.py --cycles 10
    python3 grasp_effort_test.py --cycles 10 --lift
    python3 grasp_effort_test.py --cycles 5 --object red_cube --csv red_trace.csv
    python3 grasp_effort_test.py --cycles 5 --object u_green --lift \\
            --csv u_green_trace.csv
"""

from __future__ import annotations

import argparse
import csv
import threading
import time
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
import tf2_ros

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import (
    GRASP_CLEARANCE_M,
    grasp_skill,
    release_skill,
)
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.skills.maintenance_skills import (
    set_permitted_contacts as set_permitted_contacts_skill,
)
from robot_backend.utils.geometry import compose_poses, make_pose

# ------------------------- CONFIG -------------------------
WORLD_FRAME = "world"            # set to "World" if that's the cube parent frame
GREEN = "green_cube"
RED = "red_cube"
SAFE_POSE_NAME = "ready"
# grasp_width_m is instruction-sourced (agents/tasks/*.yaml's steps: block),
# not registry-sourced. Kept here only as --object defaults so the common
# cases need no --width_m. u_green is 0.042, not the pre-2x 0.021: its
# registry entry points at the vertex-baked 2x mesh. Mirrors
# rg2_effort_sweep.KNOWN_OBJECT_WIDTHS_M -- keep the two in sync.
KNOWN_OBJECT_WIDTHS_M = {"red_cube": 0.06, "green_cube": 0.04, "u_green": 0.042}
APPROACH_HEIGHT_M = 0.10
SETTLE_SEC = 1.0
HOLD_SEC = 2.0                   # static post-grasp hold, effort watched here
LIFT_HEIGHT_M = 0.10            # only used with --lift
FINGER_JOINT_NAME = "finger_joint"   # RG2 master finger joint in /joint_states
JOINT_STATES_TOPIC = "/joint_states"
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
# ----------------------------------------------------------


def _load_registry() -> dict:
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


def _load_grasp_center_offset(object_id: str, task_path: Path | None = None) -> dict | None:
    """Load T_object_grasp_center for object_id from its executor task YAML.

    grasp_center_offset is instruction-sourced (see agents/tasks/*.yaml's
    steps: block), not registry-sourced, and is NOT identity for u_green --
    its grasp point sits on the U's off-center right arm, not the object's TF
    origin. Grasping such a part top-down at its raw TF position contacts the
    wrong geometry, so the measured effort does not transfer to the real task.

    Defaults to agents/tasks/<object_id>_pick.executor.yaml; returns None (the
    caller then falls back to a plain top-down grasp at the raw TF position)
    if that file or a matching steps: entry does not exist -- true for
    red_cube/green_cube, which have no task file of their own.

    Mirrors rg2_effort_sweep._load_grasp_center_offset; keep the two in sync.
    """
    import yaml

    if task_path is None:
        repo_root = Path(__file__).resolve().parents[4]
        task_path = repo_root / "agents" / "tasks" / f"{object_id}_pick.executor.yaml"
    if not task_path.is_file():
        return None
    with task_path.open("r", encoding="utf-8") as fh:
        task = yaml.safe_load(fh) or {}
    for entry in task.get("steps") or []:
        if isinstance(entry, dict) and entry.get("object_id") == object_id:
            offset = entry.get("grasp_center_offset")
            if isinstance(offset, dict):
                return offset
    return None


def _grasp_center_pose(object_pose: dict, offset: dict | None) -> dict:
    """Compose the instruction grasp_center_offset onto the object's live pose.

    With no offset, fall back to a plain top-down grasp at the object's raw TF
    position, ignoring the object's own orientation -- this file's original
    behaviour, correct for the symmetric cubes.
    """
    if offset is None:
        return make_pose(
            position=dict(object_pose["position"]),
            orientation=dict(TOP_DOWN_QUAT),
            frame_id=WORLD_FRAME,
        )
    offset_pose = make_pose(
        position=offset.get("position", {"x": 0.0, "y": 0.0, "z": 0.0}),
        orientation=offset.get(
            "orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        ),
    )
    return compose_poses(object_pose, offset_pose)


def _raised(pose: dict, dz: float) -> dict:
    """Copy of pose translated by dz in world z, orientation preserved.

    Orientation must be preserved: with an offset-composed grasp pose the
    approach/lift poses share the grasp orientation, so rebuilding them from
    TOP_DOWN_QUAT would rotate the wrist between approach and grasp.
    """
    return make_pose(
        position={**pose["position"], "z": pose["position"]["z"] + dz},
        orientation=dict(pose["orientation"]),
        frame_id=WORLD_FRAME,
    )


class TFReader:
    """Owns its own node + listener, spun by a background executor thread."""

    def __init__(self):
        self.node = Node("grasp_test_tf")
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self.node)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def wait_ready(self, object_id, timeout=15.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.buf.can_transform(WORLD_FRAME, object_id, rclpy.time.Time()):
                return True
            time.sleep(0.1)
        return False

    def pose(self, child) -> dict:
        """Full 6-DOF pose of child in WORLD_FRAME.

        Orientation is returned, not just position: an asymmetric part's
        instruction grasp_center_offset is expressed relative to the object's
        own orientation, so composing it needs the rotation too.
        """
        tf = self.buf.lookup_transform(WORLD_FRAME, child, rclpy.time.Time())
        t = tf.transform.translation
        r = tf.transform.rotation
        return make_pose(
            position={"x": t.x, "y": t.y, "z": t.z},
            orientation={"x": r.x, "y": r.y, "z": r.z, "w": r.w},
            frame_id=WORLD_FRAME,
        )

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


class EffortLogger:
    """Continuously sample finger_joint effort/position from /joint_states.

    Owns its own node + subscription, spun by a background executor thread —
    a read-only extra subscriber, independent of the backend node the skills
    spin. Every received message is timestamped (monotonic, relative to
    start) and stored. Call phase() to label the current point in time so the
    trace can be segmented into approach / descend / grasp / hold / lift.
    """

    def __init__(self, finger_joint_name: str = FINGER_JOINT_NAME):
        self._finger = finger_joint_name
        self.node = Node("grasp_test_effort")
        self._sub = self.node.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._cb, 50
        )
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        # each sample: (t_rel, effort, position, cycle, phase)
        self._samples: list[tuple[float, float, float, int, str]] = []
        self._cycle = 0
        self._phase = "idle"
        self._seen_finger = False
        self._thread.start()

    def _cb(self, msg: JointState) -> None:
        names = list(msg.name)
        if self._finger not in names:
            return
        idx = names.index(self._finger)
        effort = float(msg.effort[idx]) if msg.effort and idx < len(msg.effort) else float("nan")
        position = float(msg.position[idx]) if msg.position and idx < len(msg.position) else float("nan")
        t_rel = time.monotonic() - self._t0
        with self._lock:
            self._seen_finger = True
            self._samples.append((t_rel, effort, position, self._cycle, self._phase))

    def wait_ready(self, timeout=10.0) -> bool:
        """Block until at least one message carrying finger_joint arrives."""
        end = time.time() + timeout
        while time.time() < end:
            with self._lock:
                if self._seen_finger:
                    return True
            time.sleep(0.05)
        return False

    def phase(self, cycle: int, phase: str) -> None:
        """Label all subsequent samples with (cycle, phase)."""
        with self._lock:
            self._cycle = cycle
            self._phase = phase

    def window_stats(self, cycle: int, phase: str) -> dict:
        """min / max / mean / first / last effort for one (cycle, phase)."""
        with self._lock:
            efforts = [
                s[1] for s in self._samples
                if s[3] == cycle and s[4] == phase and s[1] == s[1]  # drop NaN
            ]
        if not efforts:
            return {"n": 0, "min": None, "max": None, "mean": None,
                    "first": None, "last": None}
        return {
            "n": len(efforts),
            "min": min(efforts),
            "max": max(efforts),
            "mean": sum(efforts) / len(efforts),
            "first": efforts[0],
            "last": efforts[-1],
        }

    def dump_csv(self, path: str) -> int:
        with self._lock:
            rows = list(self._samples)
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_rel_s", "effort", "position", "cycle", "phase"])
            for t_rel, effort, position, cycle, phase in rows:
                w.writerow([f"{t_rel:.4f}", f"{effort:.6f}", f"{position:.6f}", cycle, phase])
        return len(rows)

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()


class GraspCycle:
    def __init__(self, backend, tf, effort, registry, object_id, do_lift,
                 grasp_width_m, grasp_center_offset=None):
        self.b = backend
        self.tf = tf
        self.effort = effort
        self.registry = registry
        self.object_id = object_id
        self.do_lift = do_lift
        # The object's true width at contact, instruction-sourced. Passed to
        # grasp_skill as expected_width_m, which is what makes the probe's
        # commanded floor width - GRASP_CLEARANCE_M instead of 0.0. Without
        # it the safety floor is FULLY CLOSED and the measured effort is
        # "squeezed all the way into the part", not "effort at the configured
        # clearance" -- a different and higher number, useless for choosing
        # force_threshold_n and liable to crush the part.
        self.grasp_width_m = float(grasp_width_m)
        self.grasp_center_offset = grasp_center_offset

    def _phase_summary(self, cycle: int, phase: str) -> str:
        s = self.effort.window_stats(cycle, phase)
        if s["n"] == 0:
            return f"    {phase:<8} (no samples)"
        return (
            f"    {phase:<8} n={s['n']:<3} "
            f"min={s['min']:.4f} max={s['max']:.4f} mean={s['mean']:.4f} "
            f"first={s['first']:.4f} last={s['last']:.4f}"
        )

    def run(self, cycle: int):
        oid = self.object_id
        try:
            self.effort.phase(cycle, "sync")
            self.b.scene.sync_collision_objects([oid])
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)

            # --- approach ---
            # grasp_pose is the object's live pose with the instruction
            # grasp_center_offset composed on (identity fallback = plain
            # top-down at the raw TF position). Held for the whole cycle: once
            # grasped, the object's TF rides with the gripper, so re-reading it
            # would be circular.
            self.effort.phase(cycle, "approach")
            grasp_pose = _grasp_center_pose(
                self.tf.pose(oid), self.grasp_center_offset
            )
            move_grasp_center_to_pose_skill(
                self.b, _raised(grasp_pose, APPROACH_HEIGHT_M),
                gripper_width_m=self.grasp_width_m)
            time.sleep(SETTLE_SEC)

            # --- descend ---
            self.effort.phase(cycle, "descend")
            move_grasp_center_to_pose_skill(
                self.b, grasp_pose, gripper_width_m=self.grasp_width_m)
            time.sleep(SETTLE_SEC)

            # --- grasp (torque mode) ---
            self.effort.phase(cycle, "grasp")
            res = grasp_skill(self.b, object_id=oid, registry=self.registry,
                              expected_width_m=self.grasp_width_m)
            prov = res.get("provenance", {})

            # --- static hold: watch for effort decay = slipping grip ---
            self.effort.phase(cycle, "hold")
            time.sleep(HOLD_SEC)

            # --- optional lift + hold under load, then place back ---
            if self.do_lift and res.get("ok"):
                # Lift straight up from the known grasp pose p. Using p (not a
                # fresh TF read) is deterministic: once grasped, the object's
                # TF rides up with the gripper, so re-reading it mid-lift is
                # circular and laggy.
                self.effort.phase(cycle, "lift")
                move_grasp_center_to_pose_skill(
                    self.b, _raised(grasp_pose, LIFT_HEIGHT_M),
                    gripper_width_m=self.grasp_width_m)
                time.sleep(SETTLE_SEC)
                self.effort.phase(cycle, "lifthold")
                time.sleep(HOLD_SEC)

                # Place the cube back down at its original resting pose BEFORE
                # releasing, so the next cycle starts from the identical cube
                # pose/orientation. Target z is exactly the grasp-center height
                # the descend used, so the cube is resting on the table when
                # the gripper opens (no freefall, no re-rotation).
                self.effort.phase(cycle, "place")
                move_grasp_center_to_pose_skill(
                    self.b, grasp_pose, gripper_width_m=self.grasp_width_m)
                time.sleep(SETTLE_SEC)

            # --- release + home (not scored; just leaves a clean state) ---
            # Without --lift the cube was never raised, so release-in-place
            # already leaves it on the table. With --lift it was just set back
            # down above. Either way the gripper opens on a table-resting cube.
            self.effort.phase(cycle, "release")
            release_skill(self.b, object_id=oid)
            move_to_named_safe_joint_pose_skill(self.b, SAFE_POSE_NAME)

            # --- assemble report ---
            ok = bool(res.get("ok"))
            report_lines = [
                f"  grasp ok={ok} "
                f"stop_reason={prov.get('stop_reason')!r} "
                f"final_effort={prov.get('final_effort')} "
                f"final_width_m={prov.get('final_width_m')} "
                f"expected_width_m={prov.get('expected_width_m')} "
                f"force_threshold={prov.get('force_threshold')} "
                f"asymmetry={prov.get('asymmetry_detected')}",
                self._phase_summary(cycle, "descend"),
                self._phase_summary(cycle, "grasp"),
                self._phase_summary(cycle, "hold"),
            ]
            if self.do_lift:
                report_lines.append(self._phase_summary(cycle, "lift"))
                report_lines.append(self._phase_summary(cycle, "lifthold"))

            if not ok:
                report_lines.insert(1, f"  FAIL reason: {res.get('reason')}")
            return ok, "\n".join(report_lines)
        except Exception as exc:  # noqa: BLE001
            return False, f"  {type(exc).__name__}: {exc}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=10)
    ap.add_argument("--object", default=GREEN, help="registry object_id to grasp")
    ap.add_argument("--width_m", type=float, default=None,
                    help="expected grasp width in metres at contact "
                         "(instruction-sourced; not in the registry). Passed to "
                         "grasp_skill as expected_width_m, which sets the probe's "
                         "commanded floor to width - GRASP_CLEARANCE_M. Defaults "
                         "to the --object's known width if it is red_cube, "
                         "green_cube or u_green.")
    ap.add_argument("--task", default=None,
                    help="path to the executor task YAML providing "
                         "grasp_center_offset (defaults to "
                         "agents/tasks/<object>_pick.executor.yaml)")
    ap.add_argument("--no-offset", action="store_true",
                    help="ignore any task-file grasp_center_offset and grasp "
                         "top-down at the object's raw TF position")
    ap.add_argument("--lift", action="store_true",
                    help="after grasp, lift LIFT_HEIGHT_M and hold to watch effort under load")
    ap.add_argument("--csv", default="grasp_effort_trace.csv",
                    help="path for the per-sample effort trace")
    ap.add_argument("--finger-joint", default=FINGER_JOINT_NAME,
                    help="name of the RG2 master finger joint in /joint_states")
    args = ap.parse_args()

    # Resolve the grasp width BEFORE bringing anything up: a missing width is
    # the one input that silently changes what this test measures (the probe
    # floor becomes fully-closed), so fail loudly instead of defaulting.
    if args.width_m is not None:
        grasp_width_m = args.width_m
    elif args.object in KNOWN_OBJECT_WIDTHS_M:
        grasp_width_m = KNOWN_OBJECT_WIDTHS_M[args.object]
    else:
        print(f"ERROR: --width_m not given and {args.object!r} has no known "
              f"default width. grasp_width_m is instruction-sourced -- read it "
              f"from agents/tasks/{args.object}_pick.executor.yaml's steps: "
              f"block and pass --width_m explicitly.")
        return 1

    offset = None if args.no_offset else _load_grasp_center_offset(
        args.object, Path(args.task) if args.task else None
    )
    print(f"object={args.object}  width_m={grasp_width_m}  "
          f"clearance={GRASP_CLEARANCE_M} -> probe floor "
          f"{max(0.0, grasp_width_m - GRASP_CLEARANCE_M):.4f} m")
    if offset is not None:
        print(f"using instruction grasp_center_offset: {offset}")
    else:
        print("no grasp_center_offset: plain top-down grasp at the raw TF position")

    if not rclpy.ok():
        rclpy.init()

    tf = TFReader()
    if not tf.wait_ready(args.object):
        print(f"ERROR: TF for '{args.object}' not available under parent "
              f"'{WORLD_FRAME}'. Check frame names / that the full stack is up.")
        tf.shutdown()
        return 1

    effort = EffortLogger(finger_joint_name=args.finger_joint)
    if not effort.wait_ready():
        print(f"ERROR: no /joint_states message carrying '{args.finger_joint}' "
              f"within timeout. Check --finger-joint name and that the "
              f"controller is publishing joint states.")
        effort.shutdown()
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
        print(f"\n===== CYCLE {i}/{args.cycles} (grasp {args.object}) =====")
        ok, msg = GraspCycle(
            backend, tf, effort, registry, args.object, args.lift,
            grasp_width_m=grasp_width_m, grasp_center_offset=offset,
        ).run(i)
        print(f"CYCLE {i}: {'PASS' if ok else 'FAIL'}")
        print(msg)
        passes += int(ok)
        if not ok:
            fails.append(i)

    print("\n" + "=" * 60)
    print(f"RESULT: {passes}/{args.cycles} grasps confirmed ok")
    if fails:
        print(f"  failed cycles: {fails}")

    n = effort.dump_csv(args.csv)
    print(f"  wrote {n} effort samples to {args.csv}")
    print("  columns: t_rel_s, effort, position, cycle, phase")
    print("  phases: sync, approach, descend, grasp, hold"
          + (", lift, lifthold, place" if args.lift else "") + ", release")

    effort.shutdown()
    tf.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if passes == args.cycles else 1


if __name__ == "__main__":
    sys.exit(main())