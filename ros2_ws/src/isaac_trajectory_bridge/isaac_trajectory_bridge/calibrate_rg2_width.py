#!/usr/bin/env python3
"""RG2 unified calibration sweep.

Staircases the master joint from OPEN_POSITION to CLOSE_POSITION (and back),
recording at each step:
  - true_width_m   : lateral separation of contact frames in the gripper base frame
  - tcp_to_gc_{x,y,z}_m : grasp-center position in the TCP frame (z is the key offset)

Writes a single CSV used by the feedforward controller.

Run:
    ros2 run isaac_trajectory_bridge calibrate_rg2_width
"""

import csv
import math
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

# ---------------------------------------------------------------------------
# Gripper geometry constants (must match rg2_controller.py)
# ---------------------------------------------------------------------------

OPEN_POSITION = -0.558505
CLOSE_POSITION = 0.7731809

GRIPPER_JOINT_NAMES = [
    "finger_joint",
    "left_inner_knuckle_joint",
    "right_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "left_inner_finger_joint",
    "right_inner_finger_joint",
]
GRIPPER_JOINT_MULTIPLIERS = [1.0, -1.0, -1.0, -1.0, 1.0, 1.0]

JOINT_COMMAND_TOPIC = "/joint_command"

# ---------------------------------------------------------------------------
# TF frame names — edit here if frames are renamed
# ---------------------------------------------------------------------------

LEFT_CONTACT_FRAME = "left_contact_frame"
RIGHT_CONTACT_FRAME = "right_contact_frame"
TCP_FRAME = "tcp"
GRIPPER_BASE_FRAME = "onrobot_rg2_base_link"


def _default_output_path() -> str:
    try:
        share = get_package_share_directory("isaac_trajectory_bridge")
        return str(Path(share) / "config" / "rg2_calibration_table.csv")
    except Exception:
        return str(Path(__file__).resolve().parent.parent / "config" / "rg2_calibration_table.csv")


class CalibrationSweep(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_rg2_width")

        self.declare_parameter("step_rad", 0.005)
        self.declare_parameter("settle_sec", 0.4)
        self.declare_parameter("samples_per_step", 5)
        self.declare_parameter("sample_interval_sec", 0.02)
        self.declare_parameter("output_path", _default_output_path())
        self.declare_parameter("bidirectional", True)

        self._step_rad = float(self.get_parameter("step_rad").value)
        self._settle_sec = float(self.get_parameter("settle_sec").value)
        self._samples_per_step = int(self.get_parameter("samples_per_step").value)
        self._sample_interval_sec = float(self.get_parameter("sample_interval_sec").value)
        self._output_path = str(self.get_parameter("output_path").value)
        self._bidirectional = bool(self.get_parameter("bidirectional").value)

        self._joint_pub = self.create_publisher(JointState, JOINT_COMMAND_TOPIC, 10)

        # Buffer with its own executor thread so TF callbacks keep firing even
        # while the sweep loop calls time.sleep().
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            f"Calibration sweep — step={self._step_rad:.4f} rad  "
            f"settle={self._settle_sec:.2f}s  output={self._output_path}"
        )

    # -------------------------------------------------------------------------

    def _command_angle(self, angle_rad: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(GRIPPER_JOINT_NAMES)
        msg.position = [m * angle_rad for m in GRIPPER_JOINT_MULTIPLIERS]
        self._joint_pub.publish(msg)

    def _read_tf_width(self) -> Optional[float]:
        """Width = absolute y-separation of contact frames in gripper base frame."""
        try:
            left_tf = self._tf_buffer.lookup_transform(
                GRIPPER_BASE_FRAME, LEFT_CONTACT_FRAME, Time(), timeout=Duration(seconds=0.1)
            )
            right_tf = self._tf_buffer.lookup_transform(
                GRIPPER_BASE_FRAME, RIGHT_CONTACT_FRAME, Time(), timeout=Duration(seconds=0.1)
            )
            left_y = left_tf.transform.translation.y
            right_y = right_tf.transform.translation.y
            return max(0.0, abs(right_y - left_y))
        except Exception:
            return None

    def _read_tf_grasp_center(self) -> Optional[Tuple[float, float, float]]:
        """Grasp center in TCP frame = midpoint of contact frames expressed in tcp."""
        try:
            left_tf = self._tf_buffer.lookup_transform(
                TCP_FRAME, LEFT_CONTACT_FRAME, Time(), timeout=Duration(seconds=0.1)
            )
            right_tf = self._tf_buffer.lookup_transform(
                TCP_FRAME, RIGHT_CONTACT_FRAME, Time(), timeout=Duration(seconds=0.1)
            )
            lt = left_tf.transform.translation
            rt = right_tf.transform.translation
            gc_x = (lt.x + rt.x) / 2.0
            gc_y = (lt.y + rt.y) / 2.0
            gc_z = (lt.z + rt.z) / 2.0
            return gc_x, gc_y, gc_z
        except Exception:
            return None

    def _sample_step(self) -> Tuple[float, float, float, float, int]:
        """Collect samples_per_step readings and return (mean_width, mean_gc_x, mean_gc_y, mean_gc_z, n_valid)."""
        widths: List[float] = []
        gc_xs: List[float] = []
        gc_ys: List[float] = []
        gc_zs: List[float] = []

        for _ in range(self._samples_per_step):
            w = self._read_tf_width()
            gc = self._read_tf_grasp_center()
            if (
                w is not None
                and math.isfinite(w)
                and gc is not None
                and all(math.isfinite(v) for v in gc)
            ):
                widths.append(w)
                gc_xs.append(gc[0])
                gc_ys.append(gc[1])
                gc_zs.append(gc[2])
            time.sleep(self._sample_interval_sec)

        n = len(widths)
        if n == 0:
            return float("nan"), float("nan"), float("nan"), float("nan"), 0
        return (
            float(np.mean(widths)),
            float(np.mean(gc_xs)),
            float(np.mean(gc_ys)),
            float(np.mean(gc_zs)),
            n,
        )

    def _sweep(
        self, angles: np.ndarray, direction: str
    ) -> List[Tuple[float, float, float, float, float, int, str]]:
        rows: List[Tuple[float, float, float, float, float, int, str]] = []
        log_every = 20

        for idx, angle in enumerate(angles):
            self._command_angle(float(angle))
            time.sleep(self._settle_sec)

            mean_w, mean_gc_x, mean_gc_y, mean_gc_z, n_valid = self._sample_step()
            rows.append((float(angle), mean_w, mean_gc_x, mean_gc_y, mean_gc_z, n_valid, direction))

            if idx % log_every == 0:
                self.get_logger().info(
                    f"[{direction}] step {idx+1}/{len(angles)}: "
                    f"angle={angle:.4f} rad  width={mean_w*1000:.1f} mm  "
                    f"gc_z={mean_gc_z*1000:.1f} mm  n_valid={n_valid}"
                )

        return rows

    # -------------------------------------------------------------------------

    def run(self) -> None:
        # Wait for TF to become available.
        self.get_logger().info("Waiting for TF …")
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if self._read_tf_width() is not None and self._read_tf_grasp_center() is not None:
                break
            time.sleep(0.1)
        else:
            self.get_logger().error("TF not available after 15 s — is Isaac Sim running?")
            return

        self.get_logger().info("TF ready. Starting sweep open → close …")

        angles_fwd = np.arange(OPEN_POSITION, CLOSE_POSITION + self._step_rad, self._step_rad)
        angles_fwd = np.clip(angles_fwd, OPEN_POSITION, CLOSE_POSITION)
        all_rows = self._sweep(angles_fwd, "open_to_close")

        if self._bidirectional:
            self.get_logger().info("Starting sweep close → open …")
            all_rows += self._sweep(angles_fwd[::-1], "close_to_open")

        all_rows.sort(key=lambda r: (r[0], r[6]))

        out_path = Path(self._output_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["master_angle_rad", "true_width_m", "tcp_to_gc_x_m", "tcp_to_gc_y_m", "tcp_to_gc_z_m", "n_samples", "direction"]
            )
            for angle, width, gc_x, gc_y, gc_z, n, direction in all_rows:
                writer.writerow([
                    f"{angle:.6f}",
                    f"{width:.6f}",
                    f"{gc_x:.6f}",
                    f"{gc_y:.6f}",
                    f"{gc_z:.6f}",
                    n,
                    direction,
                ])

        valid_rows = [(w, gc_x, gc_y, gc_z) for _, w, gc_x, gc_y, gc_z, n, _ in all_rows if n > 0 and math.isfinite(w)]
        if valid_rows:
            widths = [r[0] for r in valid_rows]
            max_abs_x = max(abs(r[1]) for r in valid_rows)
            max_abs_y = max(abs(r[2]) for r in valid_rows)
            self.get_logger().info(
                f"Calibration complete — {len(all_rows)} rows written to {out_path}\n"
                f"  width range: {min(widths)*1000:.1f} mm … {max(widths)*1000:.1f} mm\n"
                f"  symmetry check — max |gc_x|={max_abs_x*1000:.2f} mm  max |gc_y|={max_abs_y*1000:.2f} mm"
            )
            if max_abs_x > 0.005 or max_abs_y > 0.005:
                self.get_logger().warn(
                    "SYMMETRY WARNING: grasp-center x or y exceeds 5 mm — "
                    "check contact frame placement or TF tree."
                )
        else:
            self.get_logger().error("No valid samples collected — check TF and gripper.")


def main() -> None:
    rclpy.init()
    node = CalibrationSweep()

    # Run the node in a MultiThreadedExecutor on a background thread so that
    # TF subscription callbacks keep firing while the sweep loop sleeps.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
