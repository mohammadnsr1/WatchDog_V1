#!/usr/bin/env python3
"""Log RG2 finger width (from TF) and joint effort (from /joint_states) during a grasp.

Run this BEFORE starting a grasp, then perform the grasp manually or via the
existing grasp_skill. The script logs at ~50 Hz and writes a timestamped CSV
so you can inspect the effort plateau and the true width at contact.

Usage:
    ros2 run robot_backend log_gripper_grasp  (if installed as a node)
    OR
    python3 log_gripper_grasp.py

Output:
    ~/gripper_grasp_log_<timestamp>.csv
    Columns: sim_time_sec, width_m, left_y_m, right_y_m, asymmetry_m,
              effort_finger, effort_lif, effort_rif, effort_rok, effort_lik, effort_rik

Press Ctrl+C to stop and write the file.
"""

from __future__ import annotations

import csv
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, TransformException


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# TF frames for the two inner fingertip contact surfaces
LEFT_FINGER_FRAME = "onrobot_rg2_left_inner_finger"
RIGHT_FINGER_FRAME = "onrobot_rg2_right_inner_finger"
GRIPPER_BASE_FRAME = "onrobot_rg2_base_link"

# Joint names we want to log effort for
EFFORT_JOINTS = [
    "finger_joint",
    "left_inner_finger_joint",
    "right_inner_finger_joint",
    "right_outer_knuckle_joint",
    "left_inner_knuckle_joint",
    "right_inner_knuckle_joint",
]

LOG_RATE_HZ = 50.0
OUTPUT_DIR = Path.home()


class GripperGraspLogger(Node):

    def __init__(self) -> None:
        super().__init__("gripper_grasp_logger")

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        # Joint states cache
        self._joint_effort: dict[str, float] = {}
        self._joint_lock = threading.Lock()
        self._js_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._js_callback,
            50,
        )

        # Log storage
        self._rows: list[dict[str, Any]] = []
        self._running = True

        # Timer at LOG_RATE_HZ
        period = 1.0 / LOG_RATE_HZ
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Logging gripper width (TF) and effort (/joint_states) at {LOG_RATE_HZ} Hz. "
            "Press Ctrl+C to stop and save."
        )

    def _js_callback(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        efforts: dict[str, float] = {}
        for jname in EFFORT_JOINTS:
            idx = name_to_idx.get(jname)
            if idx is not None and msg.effort and idx < len(msg.effort):
                efforts[jname] = float(msg.effort[idx])
        with self._joint_lock:
            self._joint_effort = efforts

    def _tick(self) -> None:
        if not self._running:
            return

        now = self.get_clock().now()
        sim_sec = now.nanoseconds / 1e9

        # --- TF width measurement ---
        left_y: float | None = None
        right_y: float | None = None
        width_m: float | None = None
        asymmetry_m: float | None = None

        try:
            tf_left = self._tf_buffer.lookup_transform(
                GRIPPER_BASE_FRAME,
                LEFT_FINGER_FRAME,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            tf_right = self._tf_buffer.lookup_transform(
                GRIPPER_BASE_FRAME,
                RIGHT_FINGER_FRAME,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            # In the gripper base frame, the fingers move along Y.
            # Left finger is at negative Y, right finger at positive Y.
            left_y = float(tf_left.transform.translation.y)
            right_y = float(tf_right.transform.translation.y)
            # Width = distance between inner contact faces along Y
            width_m = abs(right_y - left_y)
            # Asymmetry = how far each finger deviates from the centerline symmetrically
            # (non-zero means one finger has moved more than the other)
            center = (left_y + right_y) / 2.0
            asymmetry_m = abs(abs(left_y - center) - abs(right_y - center))
        except TransformException as exc:
            self.get_logger().warning(f"TF lookup failed: {exc}", throttle_duration_sec=2.0)

        # --- Joint efforts ---
        with self._joint_lock:
            efforts = dict(self._joint_effort)

        row = {
            "sim_time_sec": round(sim_sec, 4),
            "width_m": round(width_m, 6) if width_m is not None else "",
            "left_y_m": round(left_y, 6) if left_y is not None else "",
            "right_y_m": round(right_y, 6) if right_y is not None else "",
            "asymmetry_m": round(asymmetry_m, 6) if asymmetry_m is not None else "",
        }
        for jname in EFFORT_JOINTS:
            short = jname.replace("_joint", "").replace("onrobot_rg2_", "")
            row[f"effort_{short}"] = round(efforts.get(jname, 0.0), 6)

        self._rows.append(row)

        # Live print every 10 ticks (~5 Hz to console)
        if len(self._rows) % 10 == 0:
            w_str = f"{width_m*100:.2f} cm" if width_m is not None else "TF unavail"
            asym_str = f"{asymmetry_m*1000:.1f} mm" if asymmetry_m is not None else ""
            eff = efforts.get("finger_joint", 0.0)
            lif = efforts.get("left_inner_finger_joint", 0.0)
            rif = efforts.get("right_inner_finger_joint", 0.0)
            print(
                f"t={sim_sec:7.2f}s  width={w_str}  asym={asym_str}  "
                f"effort finger={eff:+.4f}  lif={lif:+.4f}  rif={rif:+.4f}"
            )

    def save_and_stop(self) -> None:
        self._running = False
        if not self._rows:
            self.get_logger().info("No rows collected.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = OUTPUT_DIR / f"gripper_grasp_log_{ts}.csv"
        fieldnames = list(self._rows[0].keys())
        with out_path.open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

        self.get_logger().info(f"Saved {len(self._rows)} rows to {out_path}")
        print(f"\nSaved: {out_path}")

        # Print a quick summary of effort range during the run
        print("\n--- Effort summary ---")
        for jname in EFFORT_JOINTS:
            short = jname.replace("_joint", "")
            key = f"effort_{short.replace('onrobot_rg2_', '')}"
            vals = [r[key] for r in self._rows if isinstance(r.get(key), float)]
            if vals:
                print(f"  {jname:40s}  min={min(vals):+.4f}  max={max(vals):+.4f}")

        print("\n--- Width summary ---")
        widths = [r["width_m"] for r in self._rows if isinstance(r.get("width_m"), float)]
        if widths:
            print(f"  min={min(widths):.4f} m  max={max(widths):.4f} m  "
                  f"range={max(widths)-min(widths):.4f} m")


def main() -> None:
    rclpy.init()
    node = GripperGraspLogger()

    def _sigint(sig, frame):
        print("\nStopping logger...")
        node.save_and_stop()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_stop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
