#!/usr/bin/env python3
"""RG2 width calibration sweep.

Staircases the master joint from OPEN_POSITION to CLOSE_POSITION (and back),
recording (master_angle_rad, true_width_m) at each step from live TF lookups.
Writes a dense CSV table used by the feedforward controller.

Run:
    ros2 run isaac_trajectory_bridge calibrate_rg2_width

The calibration script publishes directly to /joint_command and reads TF
itself — it does NOT require the rg2_controller to be running.
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

LEFT_FINGER_FRAME = "onrobot_rg2_left_inner_finger"
RIGHT_FINGER_FRAME = "onrobot_rg2_right_inner_finger"
GRIPPER_BASE_FRAME = "onrobot_rg2_base_link"
FINGER_LINK_OFFSET_M = 0.021


def _default_output_path() -> str:
    try:
        share = get_package_share_directory("isaac_trajectory_bridge")
        return str(Path(share) / "config" / "rg2_width_table.csv")
    except Exception:
        return str(Path(__file__).resolve().parent.parent / "config" / "rg2_width_table.csv")


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
        try:
            left_tf = self._tf_buffer.lookup_transform(
                GRIPPER_BASE_FRAME, LEFT_FINGER_FRAME, Time(), timeout=Duration(seconds=0.1)
            )
            right_tf = self._tf_buffer.lookup_transform(
                GRIPPER_BASE_FRAME, RIGHT_FINGER_FRAME, Time(), timeout=Duration(seconds=0.1)
            )
            left_y = left_tf.transform.translation.y
            right_y = right_tf.transform.translation.y
            return max(0.0, abs(right_y - left_y) - 2.0 * FINGER_LINK_OFFSET_M)
        except Exception:
            return None

    def _sample_width(self) -> Tuple[float, int]:
        """Collect samples_per_step readings and return (mean, n_valid)."""
        readings: List[float] = []
        for _ in range(self._samples_per_step):
            w = self._read_tf_width()
            if w is not None and math.isfinite(w):
                readings.append(w)
            time.sleep(self._sample_interval_sec)
        if not readings:
            return float("nan"), 0
        return float(np.mean(readings)), len(readings)

    def _sweep(
        self, angles: np.ndarray, direction: str
    ) -> List[Tuple[float, float, int, str]]:
        rows: List[Tuple[float, float, int, str]] = []
        log_every = 20

        for idx, angle in enumerate(angles):
            self._command_angle(float(angle))
            time.sleep(self._settle_sec)

            mean_w, n_valid = self._sample_width()
            rows.append((float(angle), mean_w, n_valid, direction))

            if idx % log_every == 0:
                self.get_logger().info(
                    f"[{direction}] step {idx+1}/{len(angles)}: "
                    f"angle={angle:.4f} rad  width={mean_w*1000:.1f} mm  "
                    f"n_valid={n_valid}"
                )

        return rows

    # -------------------------------------------------------------------------

    def run(self) -> None:
        # Wait for TF to become available.  The executor thread is already
        # spinning so we can just poll the buffer directly.
        self.get_logger().info("Waiting for TF …")
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if self._read_tf_width() is not None:
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

        all_rows.sort(key=lambda r: (r[0], r[3]))

        out_path = Path(self._output_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["master_angle_rad", "true_width_m", "n_samples", "direction"])
            for angle, width, n, direction in all_rows:
                writer.writerow([f"{angle:.6f}", f"{width:.6f}", n, direction])

        valid_widths = [w for _, w, n, _ in all_rows if n > 0 and math.isfinite(w)]
        if valid_widths:
            self.get_logger().info(
                f"Calibration complete — {len(all_rows)} rows written to {out_path}\n"
                f"  width range: {min(valid_widths)*1000:.1f} mm … {max(valid_widths)*1000:.1f} mm"
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
