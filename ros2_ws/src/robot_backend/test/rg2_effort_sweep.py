"""Characterize RG2 grasp effort as a function of clearance x force_threshold.

WHY THIS EXISTS
---------------
Observed: settled clamp effort parks at ~3.5 regardless of what
force_threshold is set to (it settled at ~3.4 when force_threshold was 0.2,
and Phase A of the effort-event monitor test never saw effort reach 4.0 even
though the grasp "succeeded"). The hypothesis this sweep tests:

    CEILING LAW — against a RIGID object the fingers physically stall at the
    object's true width. The probe keeps commanding toward
    target_width_m = grasp_width_m - GRASP_CLEARANCE_M, which is INSIDE the
    object, so steady-state tracking error saturates at the clearance itself.
    Since this is position control, torque ~= stiffness x tracking_error.
    Therefore CLEARANCE, not force_threshold, sets the achievable effort
    ceiling. force_threshold can only stop the close EARLY (below the
    ceiling); it can never push effort above it.

Predictions if the ceiling law holds:
  - settled effort scales ~linearly with clearance, and is ~flat across
    force_threshold once force_threshold exceeds the ceiling for that
    clearance.
  - stop_reason becomes the diagnostic: "force_threshold" means the threshold
    was reachable at that clearance (stopped early, below the ceiling);
    "effort_plateau" means it was NOT reachable and the close stalled at the
    ceiling instead. The boundary between them, per clearance row, IS the
    ceiling.
  - final_width_m stays ~= the object's true width in every cell, because a
    rigid cube does not compress. This is the executor's blind spot: its
    width check cannot distinguish a gentle grasp from a crushing one.

This characterizes the CONTROLLER, not grasp_skill: it calls
_call_guarded_move directly so target_width_m and force_threshold are set
explicitly per cell. No monkeypatching of GRASP_CLEARANCE_M, no attach/detach
in the loop, no registry corruption. The arm parks at the grasp pose once and
stays there; only the gripper cycles.

Output: a printed table plus a CSV (--csv) with one row per cell.

Run (full stack up, workspace sourced, from test/):
    python3 rg2_effort_sweep.py
    python3 rg2_effort_sweep.py --object red_cube
    python3 rg2_effort_sweep.py --clearances 0.001,0.003,0.005 --thresholds 1.0,3.5,8.0
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import (
    _call_guarded_move,
    _resolve_guarded_move_action_name,
)
from robot_backend.skills.motion_skills import (
    move_to_named_safe_joint_pose_skill,
    move_grasp_center_to_pose_skill,
)
from robot_backend.utils.geometry import make_pose

# EffortLogger already does exactly the phase-labelled effort buffering this
# needs; reuse it rather than reimplementing (sibling module in test/).
from grasp_effort_test import EffortLogger

WORLD_FRAME = "world"
SAFE_POSE_NAME = "ready"
APPROACH_HEIGHT_M = 0.10
TOP_DOWN_QUAT = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
FINGER_JOINT = "finger_joint"

DEFAULT_CLEARANCES = [0.001, 0.002, 0.003, 0.004, 0.005]
DEFAULT_THRESHOLDS = [0.5, 1.0, 2.0, 3.5, 5.0, 8.0]

SETTLE_SEC = 1.0
HOLD_SEC = 1.5          # settled-plateau measurement window, per cell
POST_OPEN_SEC = 0.6


def _pose(x, y, z):
    return make_pose(position={"x": x, "y": y, "z": z},
                      orientation=dict(TOP_DOWN_QUAT), frame_id=WORLD_FRAME)


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


class TFReader:
    """Minimal TF reader; own node, background-spun (same pattern as the tests)."""

    def __init__(self, world_frame: str):
        import threading
        import tf2_ros
        from rclpy.executors import MultiThreadedExecutor
        self.world_frame = world_frame
        self.node = Node("rg2_sweep_tf")
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self.node)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def wait_ready(self, child: str, timeout: float = 15.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.buf.can_transform(self.world_frame, child, rclpy.time.Time()):
                return True
            time.sleep(0.1)
        return False

    def pose(self, child: str) -> dict:
        tf = self.buf.lookup_transform(self.world_frame, child, rclpy.time.Time())
        t = tf.transform.translation
        return {"x": t.x, "y": t.y, "z": t.z}

    def shutdown(self):
        self.executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="green_cube")
    ap.add_argument("--clearances", default=None,
                     help="comma-separated metres, e.g. 0.001,0.003,0.005")
    ap.add_argument("--thresholds", default=None,
                     help="comma-separated effort units, e.g. 1.0,3.5,8.0")
    ap.add_argument("--csv", default="rg2_effort_sweep.csv")
    ap.add_argument("--trace-csv", default=None,
                     help="optional per-sample effort trace")
    args = ap.parse_args()

    clearances = ([float(v) for v in args.clearances.split(",")]
                   if args.clearances else DEFAULT_CLEARANCES)
    thresholds = ([float(v) for v in args.thresholds.split(",")]
                   if args.thresholds else DEFAULT_THRESHOLDS)
    oid = args.object

    if not rclpy.ok():
        rclpy.init()

    registry = _load_registry()
    if not registry:
        print("ERROR: registry empty.")
        return 1
    true_width = float(registry["objects"][oid]["spec"]["grasp"]["grasp_width_m"])
    print(f"object={oid}  registry grasp_width_m={true_width}  "
          f"(the object's true width; the probe's commanded floor is "
          f"true_width - clearance, i.e. inside the object)")

    tf = TFReader(WORLD_FRAME)
    if not tf.wait_ready(oid):
        print(f"ERROR: no TF for {oid}.")
        tf.shutdown()
        return 1

    effort_log = EffortLogger(finger_joint_name=FINGER_JOINT)
    if not effort_log.wait_ready():
        print(f"ERROR: no /joint_states carrying {FINGER_JOINT}.")
        effort_log.shutdown()
        tf.shutdown()
        return 1

    exec_node = Node("rg2_sweep_exec")
    backend = create_ros_moveit_backend(node=exec_node)
    action_name = _resolve_guarded_move_action_name(backend)
    gripper_node = backend.gripper.node

    rows: list[dict] = []
    try:
        p = tf.pose(oid)
        print("\nmoving to grasp pose (once; the arm stays here for the whole sweep)...")
        move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            backend, _pose(p["x"], p["y"], p["z"] + APPROACH_HEIGHT_M),
            gripper_width_m=true_width)
        time.sleep(SETTLE_SEC)
        move_grasp_center_to_pose_skill(
            backend, _pose(p["x"], p["y"], p["z"]), gripper_width_m=true_width)
        time.sleep(SETTLE_SEC)

        total = len(clearances) * len(thresholds)
        cell = 0
        print(f"\nsweeping {len(clearances)} clearances x {len(thresholds)} "
              f"thresholds = {total} cells\n")
        print(f"{'clear_mm':>8} {'thresh':>7} {'stop_reason':>16} "
              f"{'final_eff':>9} {'plateau':>8} {'width_mm':>8} {'d_width':>8}")
        print("-" * 72)

        for clearance in clearances:
            for threshold in thresholds:
                cell += 1
                target = max(0.0, true_width - clearance)

                backend.gripper.open()
                time.sleep(POST_OPEN_SEC)

                effort_log.phase(cell, "close")
                try:
                    res = _call_guarded_move(
                        gripper_node,
                        action_name=action_name,
                        target_width_m=target,
                        force_threshold=threshold,
                        mode="torque",
                    )
                    stop_reason = res.stop_reason
                    final_effort = float(res.final_effort)
                    final_width = float(res.final_width_m)
                except Exception as exc:  # noqa: BLE001
                    print(f"{clearance*1000:8.1f} {threshold:7.2f} "
                          f"{'EXC':>16}  {type(exc).__name__}: {exc}")
                    rows.append({
                        "clearance_m": clearance, "force_threshold": threshold,
                        "target_width_m": target, "stop_reason": "exception",
                        "final_effort": "", "plateau_mean": "", "plateau_max": "",
                        "final_width_m": "", "width_error_m": "", "error": str(exc),
                    })
                    continue

                # Settled plateau: the gripper holds its commanded angle after
                # the action returns, so this window measures steady state.
                effort_log.phase(cell, "hold")
                time.sleep(HOLD_SEC)
                stats = effort_log.window_stats(cell, "hold")

                plateau_mean = stats["mean"] if stats["n"] else float("nan")
                plateau_max = stats["max"] if stats["n"] else float("nan")
                width_err = final_width - true_width

                print(f"{clearance*1000:8.1f} {threshold:7.2f} {stop_reason:>16} "
                      f"{final_effort:9.3f} {plateau_mean:8.3f} "
                      f"{final_width*1000:8.2f} {width_err*1000:+8.2f}")

                rows.append({
                    "clearance_m": clearance,
                    "force_threshold": threshold,
                    "target_width_m": target,
                    "stop_reason": stop_reason,
                    "final_effort": final_effort,
                    "plateau_mean": plateau_mean,
                    "plateau_max": plateau_max,
                    "final_width_m": final_width,
                    "width_error_m": width_err,
                    "error": "",
                })

        backend.gripper.open()
        time.sleep(POST_OPEN_SEC)
        move_to_named_safe_joint_pose_skill(backend, SAFE_POSE_NAME)

        # --- analysis ---
        print("\n" + "=" * 72)
        print("CEILING LAW CHECK — settled plateau per clearance, across thresholds")
        print(f"{'clear_mm':>8} {'n':>3} {'plateau_min':>11} {'plateau_max':>11} "
              f"{'spread':>8}   stop_reasons")
        for clearance in clearances:
            cells = [r for r in rows
                      if r["clearance_m"] == clearance and r["error"] == ""]
            vals = [r["plateau_mean"] for r in cells
                     if isinstance(r["plateau_mean"], float)
                     and r["plateau_mean"] == r["plateau_mean"]]
            if not vals:
                continue
            reasons = {}
            for r in cells:
                reasons[r["stop_reason"]] = reasons.get(r["stop_reason"], 0) + 1
            spread = max(vals) - min(vals)
            reason_str = ", ".join(f"{k}x{v}" for k, v in sorted(reasons.items()))
            print(f"{clearance*1000:8.1f} {len(vals):3d} {min(vals):11.3f} "
                  f"{max(vals):11.3f} {spread:8.3f}   {reason_str}")

        print("\nHow to read this:")
        print("  - If plateau is roughly FLAT across thresholds within each")
        print("    clearance row (small spread) but RISES down the clearance")
        print("    column, the ceiling law holds: clearance sets the effort")
        print("    ceiling, force_threshold only stops the close early.")
        print("  - Per row, the highest threshold that still reports")
        print("    stop_reason='force_threshold' is the reachable ceiling at")
        print("    that clearance. Above it you should see 'effort_plateau'")
        print("    (the close stalled without reaching the threshold).")
        print("  - width_error should stay near zero in EVERY cell. That is the")
        print("    executor's blind spot: a rigid object's width is identical")
        print("    whether it was grasped gently or crushed.")

        with open(args.csv, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nwrote {len(rows)} cells to {args.csv}")

        if args.trace_csv:
            n = effort_log.dump_csv(args.trace_csv)
            print(f"wrote {n} effort samples to {args.trace_csv}")

        return 0
    finally:
        try:
            backend.gripper.open()
        except Exception:
            pass
        effort_log.shutdown()
        tf.shutdown()
        try:
            exec_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
