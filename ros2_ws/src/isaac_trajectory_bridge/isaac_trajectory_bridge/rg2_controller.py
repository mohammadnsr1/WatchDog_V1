#!/usr/bin/env python3
"""Feedforward RG2 gripper controller with calibration-table width lookup.

Command path for open / close / set_position:
  1. target_true_width_m  →  angle_for_width() (numpy.interp on the CSV table)
  2. Publish the calibrated joint angle ONCE — Isaac's inner drive holds it
     (or optionally ramp that setpoint open-loop for command shaping).
  3. Verify loop: wait until the live published width reaches the target band,
     then remains there for consecutive ticks.  Report success / miss; do NOT
     nudge on miss.

Command path for guarded_move (force/guarded mode):
  Incremental probe-close at probe_step_rad per tick until effort crosses the
  contact threshold OR measured width reaches target.  No width outer-loop.

Exposes the same three services and one action as the previous controller:
  /rg2_controller/open
  /rg2_controller/close
  /rg2_controller/set_position
  /rg2_controller/guarded_move (GripperGuardedMove action)
"""

import csv
import math
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from isaac_trajectory_bridge_interfaces.srv import SetGripperPosition
from isaac_trajectory_bridge_interfaces.action import GripperGuardedMove


# ---------------------------------------------------------------------------
# Geometry / hardware constants
# ---------------------------------------------------------------------------

DEFAULT_GRIPPER_JOINT_NAMES = [
    "finger_joint",
    "left_inner_knuckle_joint",
    "right_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "left_inner_finger_joint",
    "right_inner_finger_joint",
]
DEFAULT_GRIPPER_JOINT_MULTIPLIERS = [1.0, -1.0, -1.0, -1.0, 1.0, 1.0]
DEFAULT_GRIPPER_JOINT_OFFSETS = [0.0] * 6

DEFAULT_MASTER_JOINT_NAME = "finger_joint"
DEFAULT_OPEN_POSITION = -0.558505
DEFAULT_CLOSE_POSITION = 0.7731809

DEFAULT_LEFT_FINGER_FRAME = "onrobot_rg2_left_inner_finger"
DEFAULT_RIGHT_FINGER_FRAME = "onrobot_rg2_right_inner_finger"
DEFAULT_GRIPPER_BASE_FRAME = "onrobot_rg2_base_link"
DEFAULT_FINGER_LINK_OFFSET_M = 0.021  # kept for launch compatibility; unused in live width path

# Contact frames rigidly fixed to the true contact surface of each finger.
# Width is the raw lateral separation — no inset subtraction needed.
DEFAULT_LEFT_CONTACT_FRAME = "left_contact_frame"
DEFAULT_RIGHT_CONTACT_FRAME = "right_contact_frame"

# ---------------------------------------------------------------------------
# Calibration table
# ---------------------------------------------------------------------------

def _default_table_path() -> str:
    try:
        share = get_package_share_directory("isaac_trajectory_bridge")
        return str(Path(share) / "config" / "rg2_calibration_table.csv")
    except Exception:
        return str(Path(__file__).resolve().parent.parent / "config" / "rg2_calibration_table.csv")


DEFAULT_WIDTH_TABLE_PATH = _default_table_path()

# ---------------------------------------------------------------------------
# Verify-after-command parameters
# ---------------------------------------------------------------------------

DEFAULT_VERIFY_TOLERANCE_M = 0.0005     # 3 mm — acceptable final error
DEFAULT_VERIFY_STABLE_DELTA = 0.0005   # kept for launch compatibility; verify uses target band
DEFAULT_VERIFY_STABLE_TICKS = 4        # consecutive in-band ticks required
DEFAULT_VERIFY_SETTLE_EPSILON_M = 0.001  # max tick-to-tick change to be considered settled
DEFAULT_VERIFY_TIMEOUT_SEC = 3.0
DEFAULT_NO_MOTION_EPSILON_M = 0.002
DEFAULT_RAMP_TIME_SEC = 1.5

# ---------------------------------------------------------------------------
# Force / effort parameters (guarded move only)
# ---------------------------------------------------------------------------

DEFAULT_CONTROL_HZ = 50.0
DEFAULT_PROBE_STEP_RAD = 0.010         # angle increment per tick during contact search (width mode)
DEFAULT_TORQUE_PROBE_STEP_RAD = 0.005  # uniform step size for the entire torque-mode close
# Measured idle magnitude peaks at ~0.027 across orientations; floor adds a margin.
DEFAULT_EFFORT_NOISE_FLOOR = 0.03
# Contact threshold: above worst-case idle (~0.027) with margin, below fast-rising
# hard-contact effort (~0.05–0.1).  Comparisons use abs(effort) — idle ranges
# from about -0.026 to +0.027 depending on gripper orientation/gravity loading.
DEFAULT_EFFORT_CONTACT_THRESHOLD = 0.18
DEFAULT_EFFORT_PLATEAU_DERIV = 0.05
DEFAULT_EFFORT_PLATEAU_TICKS = 2
DEFAULT_ASYMMETRY_THRESHOLD_M = 0.010
DEFAULT_ENABLE_ASYMMETRY_FAULT = False
DEFAULT_MAX_CONTROL_ITERS = 400        # used by width mode; torque mode uses wall-clock timeout
# Wall-clock timeout for torque mode: 15 s >> ~5.3 s worst-case full traverse at 50 Hz.
DEFAULT_TORQUE_MAX_DURATION_SEC = 30.0
# Max wall-clock gap since the last fresh joint_state before the probe holds.
DEFAULT_TORQUE_STALE_HOLD_SEC = 0.060  # 60 ms ≈ 3× the 50 Hz period


class RG2Controller(Node):
    def __init__(self) -> None:
        super().__init__("rg2_controller")

        # ---- parameters --------------------------------------------------------
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("service_namespace", "/rg2_controller")
        self.declare_parameter("master_joint_name", DEFAULT_MASTER_JOINT_NAME)
        self.declare_parameter("open_position", DEFAULT_OPEN_POSITION)
        self.declare_parameter("close_position", DEFAULT_CLOSE_POSITION)
        self.declare_parameter("command_rate_hz", DEFAULT_CONTROL_HZ)
        self.declare_parameter("control_rate_hz", DEFAULT_CONTROL_HZ)
        self.declare_parameter("command_duration_sec", 1.0)   # kept for launch compat; unused
        self.declare_parameter("gripper_joint_names", DEFAULT_GRIPPER_JOINT_NAMES)
        self.declare_parameter("gripper_joint_multipliers", DEFAULT_GRIPPER_JOINT_MULTIPLIERS)
        self.declare_parameter("gripper_joint_offsets", DEFAULT_GRIPPER_JOINT_OFFSETS)

        self.declare_parameter("left_finger_frame", DEFAULT_LEFT_FINGER_FRAME)
        self.declare_parameter("right_finger_frame", DEFAULT_RIGHT_FINGER_FRAME)
        self.declare_parameter("gripper_base_frame", DEFAULT_GRIPPER_BASE_FRAME)
        self.declare_parameter("finger_link_offset_m", DEFAULT_FINGER_LINK_OFFSET_M)  # accepted but unused
        self.declare_parameter("left_contact_frame", DEFAULT_LEFT_CONTACT_FRAME)
        self.declare_parameter("right_contact_frame", DEFAULT_RIGHT_CONTACT_FRAME)

        self.declare_parameter("width_table_path", DEFAULT_WIDTH_TABLE_PATH)

        self.declare_parameter("verify_tolerance_m", DEFAULT_VERIFY_TOLERANCE_M)
        self.declare_parameter("verify_stable_delta", DEFAULT_VERIFY_STABLE_DELTA)
        self.declare_parameter("verify_stable_ticks", DEFAULT_VERIFY_STABLE_TICKS)
        self.declare_parameter("verify_settle_epsilon_m", DEFAULT_VERIFY_SETTLE_EPSILON_M)
        self.declare_parameter("verify_timeout_sec", DEFAULT_VERIFY_TIMEOUT_SEC)
        self.declare_parameter("no_motion_epsilon_m", DEFAULT_NO_MOTION_EPSILON_M)
        self.declare_parameter("ramp_time_sec", DEFAULT_RAMP_TIME_SEC)

        self.declare_parameter("probe_step_rad", DEFAULT_PROBE_STEP_RAD)
        self.declare_parameter("torque_probe_step_rad", DEFAULT_TORQUE_PROBE_STEP_RAD)
        self.declare_parameter("effort_noise_floor", DEFAULT_EFFORT_NOISE_FLOOR)
        self.declare_parameter("effort_contact_threshold", DEFAULT_EFFORT_CONTACT_THRESHOLD)
        self.declare_parameter("effort_plateau_deriv", DEFAULT_EFFORT_PLATEAU_DERIV)
        self.declare_parameter("effort_plateau_ticks", DEFAULT_EFFORT_PLATEAU_TICKS)
        self.declare_parameter("asymmetry_threshold_m", DEFAULT_ASYMMETRY_THRESHOLD_M)
        self.declare_parameter("enable_asymmetry_fault", DEFAULT_ENABLE_ASYMMETRY_FAULT)
        self.declare_parameter("max_control_iters", DEFAULT_MAX_CONTROL_ITERS)
        self.declare_parameter("torque_max_duration_sec", DEFAULT_TORQUE_MAX_DURATION_SEC)
        self.declare_parameter("torque_stale_hold_sec", DEFAULT_TORQUE_STALE_HOLD_SEC)

        # ---- read --------------------------------------------------------------
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self._service_ns = self._normalize_ns(str(self.get_parameter("service_namespace").value))
        self._master_joint_name = str(self.get_parameter("master_joint_name").value)
        self._open_position = float(self.get_parameter("open_position").value)
        self._close_position = float(self.get_parameter("close_position").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        if control_rate_hz == DEFAULT_CONTROL_HZ and command_rate_hz != DEFAULT_CONTROL_HZ:
            control_rate_hz = command_rate_hz
        self._control_hz = max(1.0, control_rate_hz)
        self._control_period_sec = 1.0 / self._control_hz
        self._gripper_joint_names = list(self.get_parameter("gripper_joint_names").value)
        self._gripper_joint_multipliers = [
            float(v) for v in self.get_parameter("gripper_joint_multipliers").value
        ]
        self._gripper_joint_offsets = [
            float(v) for v in self.get_parameter("gripper_joint_offsets").value
        ]

        self._left_finger_frame = str(self.get_parameter("left_finger_frame").value)
        self._right_finger_frame = str(self.get_parameter("right_finger_frame").value)
        self._gripper_base_frame = str(self.get_parameter("gripper_base_frame").value)
        self._finger_link_offset_m = float(self.get_parameter("finger_link_offset_m").value)  # unused in width path
        self._left_contact_frame = str(self.get_parameter("left_contact_frame").value)
        self._right_contact_frame = str(self.get_parameter("right_contact_frame").value)

        self._width_table_path = str(self.get_parameter("width_table_path").value)

        self._verify_tolerance_m = float(self.get_parameter("verify_tolerance_m").value)
        self._verify_stable_delta = float(self.get_parameter("verify_stable_delta").value)
        self._verify_stable_ticks = int(self.get_parameter("verify_stable_ticks").value)
        self._verify_settle_epsilon_m = float(self.get_parameter("verify_settle_epsilon_m").value)
        self._verify_timeout_sec = float(self.get_parameter("verify_timeout_sec").value)
        self._no_motion_epsilon_m = float(self.get_parameter("no_motion_epsilon_m").value)
        self._ramp_time_sec = max(0.0, float(self.get_parameter("ramp_time_sec").value))

        self._probe_step_rad = float(self.get_parameter("probe_step_rad").value)
        self._torque_probe_step_rad = float(self.get_parameter("torque_probe_step_rad").value)
        self._effort_noise_floor = float(self.get_parameter("effort_noise_floor").value)
        self._effort_contact_threshold = float(self.get_parameter("effort_contact_threshold").value)
        self._effort_plateau_deriv = float(self.get_parameter("effort_plateau_deriv").value)
        self._effort_plateau_ticks = int(self.get_parameter("effort_plateau_ticks").value)
        self._asymmetry_threshold_m = float(self.get_parameter("asymmetry_threshold_m").value)
        self._enable_asymmetry_fault = bool(self.get_parameter("enable_asymmetry_fault").value)
        self._max_control_iters = int(self.get_parameter("max_control_iters").value)
        self._torque_max_duration_sec = float(self.get_parameter("torque_max_duration_sec").value)
        self._torque_stale_hold_sec = float(self.get_parameter("torque_stale_hold_sec").value)

        # ---- validate ----------------------------------------------------------
        if len(self._gripper_joint_names) != 6:
            raise ValueError("gripper_joint_names must contain exactly 6 joints.")
        if self._master_joint_name not in self._gripper_joint_names:
            raise ValueError("master_joint_name must be in gripper_joint_names.")

        if self._effort_contact_threshold <= self._effort_noise_floor:
            clamped = self._effort_noise_floor * 1.5
            self.get_logger().error(
                f"effort_contact_threshold ({self._effort_contact_threshold:.4f}) is not above "
                f"effort_noise_floor ({self._effort_noise_floor:.4f}) — every torque-mode close "
                f"will false-trigger on sensor noise.  Clamping contact threshold to "
                f"{clamped:.4f} (noise_floor × 1.5).  Set effort_contact_threshold > "
                f"effort_noise_floor in your launch config."
            )
            self._effort_contact_threshold = clamped

        # ---- load calibration table -------------------------------------------
        self._table_angles: np.ndarray
        self._table_widths: np.ndarray
        self._table_loaded = False
        self._load_width_table()

        # ---- state -------------------------------------------------------------
        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._busy = False
        self._latest_joint_positions: Dict[str, float] = {}
        self._latest_joint_efforts: Dict[str, float] = {}
        self._latest_joint_state_stamp: Optional[Tuple[int, int]] = None  # (sec, nanosec)
        self._last_known_width: Optional[float] = None
        self._latest_published_width: Optional[float] = None

        # ---- TF ----------------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- publishers / subscribers ------------------------------------------
        self._joint_pub = self.create_publisher(JointState, self._joint_command_topic, 10)
        self._width_pub = self.create_publisher(Float64, "/gripper_width_true", 10)

        self.create_subscription(JointState, self._joint_state_topic, self._joint_state_cb, 50)
        self.create_timer(self._control_period_sec, self._width_timer_cb)

        # ---- services ----------------------------------------------------------
        svc_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, f"{self._service_ns}/open", self._open_cb, callback_group=svc_group
        )
        self.create_service(
            Trigger, f"{self._service_ns}/close", self._close_cb, callback_group=svc_group
        )
        self.create_service(
            SetGripperPosition,
            f"{self._service_ns}/set_position",
            self._set_position_cb,
            callback_group=svc_group,
        )

        # ---- action server -----------------------------------------------------
        ActionServer(
            self,
            GripperGuardedMove,
            f"{self._service_ns}/guarded_move",
            self._execute_guarded_move,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"RG2 feedforward controller ready — "
            f"services: {self._service_ns}/{{open,close,set_position}}  "
            f"action: {self._service_ns}/guarded_move  "
            f"table_loaded={self._table_loaded}  "
            f"verify_tol={self._verify_tolerance_m*1000:.0f} mm"
        )

    # =========================================================================
    # Calibration table
    # =========================================================================

    def _load_width_table(self) -> None:
        """Load rg2_calibration_table.csv and build monotonic angle↔width interpolators.

        Uses only the open_to_close direction rows (or all rows if no direction
        column) to guarantee the angle→width mapping is monotonic decreasing.
        Rows with invalid/NaN width are dropped.  If the file is missing or
        has fewer than 2 valid rows the node falls back to a linear map and
        logs a clear instruction to run calibrate_rg2_width first.
        """
        path = Path(self._width_table_path)
        if not path.exists():
            self.get_logger().error(
                f"Width table not found: {path}\n"
                "  Run:  ros2 run isaac_trajectory_bridge calibrate_rg2_width\n"
                "  then restart this controller.  Falling back to linear map."
            )
            self._build_linear_fallback()
            return

        rows: List[Tuple[float, float]] = []
        try:
            with open(path, newline="") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    direction = row.get("direction", "open_to_close")
                    if direction not in ("open_to_close", ""):
                        continue
                    angle = float(row["master_angle_rad"])
                    width = float(row["true_width_m"])
                    if math.isfinite(angle) and math.isfinite(width) and width >= 0.0:
                        rows.append((angle, width))
        except Exception as exc:
            self.get_logger().error(f"Failed to read width table: {exc} — using linear fallback.")
            self._build_linear_fallback()
            return

        if len(rows) < 2:
            self.get_logger().error(
                "Width table has fewer than 2 valid rows — using linear fallback."
            )
            self._build_linear_fallback()
            return

        rows.sort(key=lambda r: r[0])
        angles = np.array([r[0] for r in rows])
        widths = np.array([r[1] for r in rows])

        # Smooth out any local non-monotonicities from noise by enforcing a
        # monotone-decreasing width with respect to increasing angle.
        for i in range(1, len(widths)):
            if widths[i] > widths[i - 1]:
                widths[i] = widths[i - 1]

        self._table_angles = angles
        self._table_widths = widths
        self._table_loaded = True

        self.get_logger().info(
            f"Width table loaded: {len(rows)} rows  "
            f"angle [{angles[0]:.4f}, {angles[-1]:.4f}] rad  "
            f"width [{widths[-1]*1000:.1f}, {widths[0]*1000:.1f}] mm"
        )

    def _build_linear_fallback(self) -> None:
        """Fallback: linear angle→width based on hardware limits only."""
        n = 200
        self._table_angles = np.linspace(DEFAULT_OPEN_POSITION, DEFAULT_CLOSE_POSITION, n)
        # At open angle: ~100 mm; at close angle: ~0 mm (linear approximation)
        self._table_widths = np.linspace(0.100, 0.0, n)
        self._table_loaded = False

    def angle_for_width(self, width_m: float) -> float:
        """Return master joint angle [rad] for the given true width [m].

        Clamps to the table's angle range so extreme requests still produce a
        valid command (open/close stops).
        """
        # _table_widths is monotone decreasing vs _table_angles increasing.
        # np.interp requires xp increasing, so flip the arrays.
        angle = float(np.interp(width_m, self._table_widths[::-1], self._table_angles[::-1]))
        return float(np.clip(angle, self._open_position, self._close_position))

    def width_for_angle(self, angle_rad: float) -> float:
        """Return predicted true width [m] for a given master joint angle [rad]."""
        return float(np.interp(angle_rad, self._table_angles, self._table_widths))

    # =========================================================================
    # ROS callbacks
    # =========================================================================

    def _joint_state_cb(self, msg: JointState) -> None:
        idx_map = {name: i for i, name in enumerate(msg.name)}
        pos, eff = {}, {}
        for jname in self._gripper_joint_names:
            i = idx_map.get(jname)
            if i is None:
                continue
            if i < len(msg.position):
                pos[jname] = msg.position[i]
            if msg.effort and i < len(msg.effort):
                eff[jname] = msg.effort[i]
        if pos:
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            with self._state_lock:
                self._latest_joint_positions = pos
                self._latest_joint_efforts = eff
                self._latest_joint_state_stamp = stamp

    def _width_timer_cb(self) -> None:
        result = self._read_tf_width()
        if result is not None:
            width = result[0]
            with self._state_lock:
                self._last_known_width = width
                self._latest_published_width = width
            msg = Float64()
            msg.data = width
            self._width_pub.publish(msg)

    # =========================================================================
    # TF helpers
    # =========================================================================

    def _read_tf_width(self) -> Optional[Tuple[float, float, float]]:
        """Return (true_width_m, left_y, right_y) or None on TF failure.

        Looks up left_contact_frame and right_contact_frame (fixed to the true
        contact surface) relative to the gripper base frame. Width is the raw
        lateral separation — no inset subtraction. left_y + right_y is kept for
        the asymmetry diagnostic; the contact frames are symmetric about y=0.
        """
        try:
            left_tf = self._tf_buffer.lookup_transform(
                self._gripper_base_frame,
                self._left_contact_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            right_tf = self._tf_buffer.lookup_transform(
                self._gripper_base_frame,
                self._right_contact_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            left_y = left_tf.transform.translation.y
            right_y = right_tf.transform.translation.y
            true_width = max(0.0, abs(right_y - left_y))
            return true_width, left_y, right_y
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}", throttle_duration_sec=2.0)
            return None

    def _get_true_width(self) -> Optional[float]:
        result = self._read_tf_width()
        if result is not None:
            with self._state_lock:
                self._last_known_width = result[0]
            return result[0]
        with self._state_lock:
            return self._last_known_width

    def _get_latest_published_width(self) -> Optional[float]:
        """Return the latest width value maintained by /gripper_width_true publishing."""
        with self._state_lock:
            return self._latest_published_width

    # =========================================================================
    # Joint state helpers
    # =========================================================================

    def _get_master_position(self) -> Optional[float]:
        with self._state_lock:
            return self._latest_joint_positions.get(self._master_joint_name)

    def _get_master_effort(self) -> float:
        with self._state_lock:
            return abs(self._latest_joint_efforts.get(self._master_joint_name, 0.0))

    # =========================================================================
    # Feedforward command + verify
    # =========================================================================

    def _command_angle(self, angle_rad: float) -> None:
        """Publish a single joint command for the given master angle (all joints)."""
        positions = [
            off + mult * angle_rad
            for mult, off in zip(self._gripper_joint_multipliers, self._gripper_joint_offsets)
        ]
        self._publish_command(positions)

    def _command_angle_shaped(self, target_angle_rad: float) -> None:
        """Publish the target angle, optionally ramping the command setpoint."""
        if self._ramp_time_sec <= 0.0:
            self._command_angle(target_angle_rad)
            return

        current_angle = self._get_master_position()
        if current_angle is None:
            self._command_angle(target_angle_rad)
            return

        steps = max(1, int(math.ceil(self._ramp_time_sec * self._control_hz)))
        for step in range(1, steps + 1):
            fraction = step / steps
            angle = current_angle + fraction * (target_angle_rad - current_angle)
            if step == steps:
                angle = target_angle_rad
            self._command_angle(angle)
            time.sleep(self._control_period_sec)

    def _verify_width_target(
        self,
        target_width_m: float,
        label: str = "",
        tolerance_m: Optional[float] = None,
    ) -> Tuple[bool, float, Optional[float]]:
        """Verify the live published width reaches and settles in target band.

        stable_count increments only when BOTH conditions hold on the same tick:
          1. abs(width - target) <= tol           (in-band)
          2. abs(width - prev_width) <= settle_epsilon_m  (no longer moving)
        Either failure resets stable_count to 0, so convergence requires the
        gripper to be simultaneously on-target and motionless.
        """
        tol = self._verify_tolerance_m if tolerance_m is None else float(tolerance_m)
        start_time = time.time()
        deadline = start_time + self._verify_timeout_sec

        start_width = self._get_latest_published_width()
        final_width = start_width if start_width is not None else target_width_m
        max_motion = 0.0
        warned_no_motion = False
        stable_count = 0
        prev_width: Optional[float] = start_width

        if label and start_width is not None:
            self.get_logger().info(
                f"{label}: verify_start={start_width*1000:.1f} mm  "
                f"target={target_width_m*1000:.1f} mm"
            )

        while time.time() < deadline:
            width = self._get_latest_published_width()
            if width is None:
                time.sleep(self._control_period_sec)
                continue

            final_width = width
            delta = abs(width - prev_width) if prev_width is not None else float("inf")

            if start_width is not None:
                max_motion = max(max_motion, abs(width - start_width))

            elapsed = time.time() - start_time
            target_requires_motion = (
                start_width is not None
                and abs(target_width_m - start_width) > tol
            )
            if (
                target_requires_motion
                and not warned_no_motion
                and elapsed >= 0.5 * self._verify_timeout_sec
                and max_motion < self._no_motion_epsilon_m
            ):
                self.get_logger().warn(
                    f"{label}: commanded but no motion detected  "
                    f"start={start_width*1000:.1f} mm  "
                    f"target={target_width_m*1000:.1f} mm"
                )
                warned_no_motion = True

            in_band = abs(width - target_width_m) <= tol
            settled = delta <= self._verify_settle_epsilon_m
            if in_band and settled:
                stable_count += 1
                if stable_count >= self._verify_stable_ticks:
                    return True, final_width, start_width
            else:
                stable_count = 0

            prev_width = width
            time.sleep(self._control_period_sec)

        return False, final_width, start_width

    def _feedforward_move(
        self,
        target_width_m: float,
        label: str = "",
        tolerance_m: Optional[float] = None,
    ) -> Tuple[bool, float]:
        """Command the calibrated angle for target_width_m, then verify live width.

        Verification is intentionally target-driven: first wait for the latest
        published /gripper_width_true value to enter the target band, then
        require consecutive in-band reads. Stability outside the target band
        never terminates verification.
        """
        target_angle = self.angle_for_width(target_width_m)

        if label:
            self.get_logger().info(
                f"{label}: target_width={target_width_m*1000:.1f} mm  "
                f"→ angle={target_angle:.4f} rad"
            )

        self._command_angle_shaped(target_angle)
        converged, final_width, start_width = self._verify_width_target(
            target_width_m,
            label=label,
            tolerance_m=tolerance_m,
        )

        if not converged:
            start_msg = (
                f"start={start_width*1000:.1f} mm  "
                if start_width is not None
                else "start=unknown  "
            )
            self.get_logger().warn(
                f"{label}: target={target_width_m*1000:.1f} mm  "
                f"{start_msg}"
                f"final={final_width*1000:.1f} mm  "
                f"error={abs(final_width-target_width_m)*1000:.1f} mm  "
                f"(table may need recalibration or motion did not complete)"
            )

        return converged, final_width

    # =========================================================================
    # Service callbacks
    # =========================================================================

    def _open_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        return self._handle_svc(self._table_widths[0], "open", resp)

    def _close_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        return self._handle_svc(self._table_widths[-1], "close", resp)

    def _set_position_cb(
        self,
        req: SetGripperPosition.Request,
        resp: SetGripperPosition.Response,
    ) -> SetGripperPosition.Response:
        """Accept a master joint angle from ROSGripperBackend and command it directly.

        ROSGripperBackend sends a joint angle (not a width) via set_position.
        We command the angle as-is and report the table-predicted width for it.
        """
        angle = float(req.position)
        angle = float(np.clip(angle, self._open_position, self._close_position))
        predicted_width = self.width_for_angle(angle)

        if not self._acquire():
            resp.success = False
            resp.message = "Busy."
            return resp
        try:
            self._command_angle_shaped(angle)
            converged, final_width, start_width = self._verify_width_target(
                predicted_width,
                label="set_position",
            )

            start_msg = (
                f"start_width={start_width*1000:.1f} mm  "
                if start_width is not None
                else "start_width=unknown  "
            )
            resp.success = True
            resp.message = (
                f"set_position: angle={angle:.4f} rad  "
                f"predicted_width={predicted_width*1000:.1f} mm  "
                f"{start_msg}"
                f"final_width={final_width*1000:.1f} mm  converged={converged}"
            )
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(f"set_position failed: {exc}")
        finally:
            self._release()
        return resp

    def _handle_svc(
        self,
        target_width_m: float,
        label: str,
        resp,
    ) -> object:
        if not self._acquire():
            resp.success = False
            resp.message = "Busy."
            return resp
        try:
            converged, final_width = self._feedforward_move(target_width_m, label=label)
            resp.success = True
            resp.message = (
                f"{label}: target={target_width_m*1000:.1f} mm  "
                f"final={final_width*1000:.1f} mm  converged={converged}"
            )
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(f"{label} failed: {exc}")
        finally:
            self._release()
        return resp

    # =========================================================================
    # GripperGuardedMove action
    # =========================================================================

    def _execute_guarded_move(self, goal_handle) -> GripperGuardedMove.Result:
        """Guarded move action — two mutually isolated modes.

        mode "width":
            Pure feedforward.  Command angle_for_width(target_width_m) once,
            then verify settle exactly as the width path does.  No effort code
            path executes.  stop_reason = "width_reached" / "timeout".

        mode "torque":
            Pure effort-based probe-close, independent of width as a stop
            condition.  Starting from the current joint angle, increment
            commanded_angle by torque_probe_step_rad per tick toward
            CLOSE_POSITION.

            On every iteration the order is strictly:
              1. FRESHNESS GATE — require a joint_state with a newer
                 header.stamp than the one consumed on the prior iteration.
                 If the wall-clock gap since the last fresh sample exceeds
                 torque_stale_hold_sec, hold the current commanded angle and
                 skip all further checks for this iteration.  A stale sample
                 is never treated as safe-to-continue.
              2. HARD GUARD — effort >= goal.force_threshold → stop immediately.
                 stop_reason = "force_threshold".
              3. PRIMARY STOP — effort >= effort_contact_threshold → HOLD the
                 current commanded_angle and stop immediately (no further step).
                 stop_reason = "force_threshold".
              4. PLATEAU CONFIRM (secondary) — |d_effort/dt| stable for
                 effort_plateau_ticks while effort is non-negligible.  Catches
                 slow contact build-up below the primary threshold.
                 stop_reason = "force_threshold".
              5. PROBE STEP — advance commanded_angle by torque_probe_step_rad
                 toward CLOSE_POSITION, clamped to floor_angle (safety floor,
                 never a stop condition).

            target_width_m is a safety floor ONLY: commanded_angle is capped at
            angle_for_width(target_width_m).  It does NOT produce a
            "width_reached" stop.

            Asymmetry (left_y + right_y in base frame) is a feedback diagnostic
            only; never a stop condition unless enable_asymmetry_fault.
            Timeout after torque_max_duration_sec wall-clock seconds →
            stop_reason = "timeout".
        """
        goal: GripperGuardedMove.Goal = goal_handle.request

        target_width = float(goal.target_width_m)
        force_threshold = float(goal.force_threshold) if goal.force_threshold > 0.0 else 0.18
        mode = str(goal.mode) if goal.mode else "torque"
        tol = (
            float(goal.width_tolerance_m)
            if goal.width_tolerance_m > 0.0
            else self._verify_tolerance_m
        )

        self.get_logger().info(
            f"GuardedMove: target={target_width*1000:.1f} mm  "
            f"force_threshold={force_threshold:.3f}  mode={mode}  tol={tol*1000:.1f} mm"
        )

        fb = GripperGuardedMove.Feedback()
        result = GripperGuardedMove.Result()
        stop_reason = "timeout"

        asymmetry_ever_exceeded = False
        final_width: float = self._get_true_width() or target_width
        final_effort: float = self._get_master_effort()

        # ------------------------------------------------------------------
        # mode "width" — pure feedforward, no effort involvement whatsoever
        # ------------------------------------------------------------------
        if mode == "width":
            converged, final_width = self._feedforward_move(
                target_width,
                label="guarded_width",
                tolerance_m=tol,
            )
            stop_reason = "width_reached" if converged else "timeout"

            result.success = converged
            result.stop_reason = stop_reason
            result.final_width_m = final_width
            result.final_effort = self._get_master_effort()
            result.asymmetry_detected = False
            result.message = f"stop={stop_reason}  final_width={final_width*1000:.1f} mm"
            self.get_logger().info(result.message)
            goal_handle.succeed()
            return result

        # ------------------------------------------------------------------
        # mode "torque" — pure effort-based probe-close
        # ------------------------------------------------------------------
        if mode == "torque":
            current_angle = self._get_master_position()
            if current_angle is None:
                current_angle = self._open_position
            commanded_angle = current_angle

            # Safety floor: cap commanded_angle here, never a stop condition.
            floor_angle = self.angle_for_width(target_width)

            prev_effort: float = 0.0
            plateau_count = 0
            dt = self._control_period_sec

            # Snapshot the current stamp as "already used" so the first
            # iteration must wait for a genuinely new joint_state.
            with self._state_lock:
                last_used_stamp = self._latest_joint_state_stamp
            # Wall-clock references: stale-hold window and overall timeout both
            # measure elapsed real time, consistent with torque_stale_hold_sec.
            start_wall_time: float = time.time()
            last_fresh_wall_time: float = start_wall_time

            while time.time() - start_wall_time < self._torque_max_duration_sec:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.stop_reason = "cancelled"
                    result.final_width_m = final_width
                    result.final_effort = final_effort
                    result.asymmetry_detected = asymmetry_ever_exceeded
                    result.message = "Cancelled."
                    return result

                # ── 1. FRESHNESS GATE ─────────────────────────────────────
                # /joint_states averages 50 Hz but can stall ~286 ms.  Read
                # the latest stamp non-blocking; if it is the same as the
                # one already consumed, no new data has arrived.
                with self._state_lock:
                    current_stamp = self._latest_joint_state_stamp
                    candidate_effort = abs(
                        self._latest_joint_efforts.get(self._master_joint_name, 0.0)
                    )

                got_fresh = (
                    current_stamp is not None and current_stamp != last_used_stamp
                )

                if not got_fresh:
                    # No new sample — check how long we have been stale.
                    stale_sec = time.time() - last_fresh_wall_time
                    if stale_sec >= self._torque_stale_hold_sec:
                        self.get_logger().warn(
                            f"torque mode: no fresh joint_state for "
                            f"{stale_sec*1000:.0f} ms — holding position",
                            throttle_duration_sec=1.0,
                        )
                    fb.current_width_m = final_width
                    fb.current_effort = final_effort
                    fb.asymmetry_m = 0.0
                    fb.phase = "hold_stale"
                    goal_handle.publish_feedback(fb)
                    time.sleep(dt)
                    continue

                # Fresh sample confirmed — consume it.
                last_used_stamp = current_stamp
                last_fresh_wall_time = time.time()
                effort = candidate_effort

                d_effort = (effort - prev_effort) / dt
                prev_effort = effort
                final_effort = effort

                # ── Asymmetry diagnostic (TF-based, feedback only) ────────
                asymmetry_m = 0.0
                tf_result = self._read_tf_width()
                if tf_result is not None:
                    width, left_y, right_y = tf_result
                    final_width = width
                    with self._state_lock:
                        self._last_known_width = width
                    asymmetry_m = abs(left_y + right_y)
                    if asymmetry_m > self._asymmetry_threshold_m:
                        asymmetry_ever_exceeded = True

                fb.current_width_m = final_width
                fb.current_effort = effort
                fb.asymmetry_m = asymmetry_m
                fb.phase = "closing"
                goal_handle.publish_feedback(fb)

                # ── 2. HARD GUARD: over-force ─────────────────────────────
                # abs(): idle effort ranges -0.026..+0.027 across orientations.
                if abs(effort) >= force_threshold:
                    stop_reason = "force_threshold"
                    break

                # ── 3. PRIMARY STOP: instant contact ──────────────────────
                if abs(effort) >= self._effort_contact_threshold:
                    self._command_angle(commanded_angle)  # hold
                    stop_reason = "force_threshold"
                    break

                # ── 4. PLATEAU CONFIRM (secondary) ────────────────────────
                # Catches slow effort build-up that stalls just below the
                # primary threshold without ever crossing it.  Gate requires
                # abs(effort) clearly above noise AND close to the primary
                # threshold.  d_effort keeps its sign for the derivative check.
                if abs(effort) > max(self._effort_noise_floor, self._effort_contact_threshold * 0.8):
                    if abs(d_effort) < self._effort_plateau_deriv:
                        plateau_count += 1
                        if plateau_count >= self._effort_plateau_ticks:
                            self._command_angle(commanded_angle)  # hold
                            stop_reason = "force_threshold"
                            break
                    else:
                        plateau_count = 0
                else:
                    plateau_count = 0

                # Asymmetry fault (diagnostic only by default)
                if self._enable_asymmetry_fault and asymmetry_m > self._asymmetry_threshold_m:
                    stop_reason = "error"
                    break

                # ── 5. PROBE STEP ─────────────────────────────────────────
                # Uniform fine step for the entire close — no speed switching.
                # Consistent granularity avoids overshoot right as effort rises.
                next_angle = min(
                    commanded_angle + self._torque_probe_step_rad,
                    self._close_position,
                    floor_angle,
                )
                if next_angle > commanded_angle:
                    commanded_angle = next_angle
                    self._command_angle(commanded_angle)

                time.sleep(dt)

            fb.phase = "contact" if stop_reason == "force_threshold" else stop_reason
            goal_handle.publish_feedback(fb)

            result.success = stop_reason == "force_threshold"
            result.stop_reason = stop_reason
            result.final_width_m = final_width
            result.final_effort = final_effort
            result.asymmetry_detected = asymmetry_ever_exceeded
            result.message = (
                f"stop={stop_reason}  final_width={final_width*1000:.1f} mm  "
                f"effort={final_effort:.3f}  asymmetry={asymmetry_ever_exceeded}"
            )
            self.get_logger().info(result.message)
            goal_handle.succeed()
            return result

        # Unknown mode
        result.success = False
        result.stop_reason = "error"
        result.final_width_m = final_width
        result.final_effort = final_effort
        result.asymmetry_detected = False
        result.message = f"Unknown mode '{mode}'. Supported modes: 'width', 'torque'."
        self.get_logger().error(result.message)
        goal_handle.succeed()
        return result

    # =========================================================================
    # Helpers
    # =========================================================================

    def _publish_command(self, positions: Sequence[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._gripper_joint_names)
        msg.position = list(positions)
        self._joint_pub.publish(msg)

    def _acquire(self) -> bool:
        with self._command_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _release(self) -> None:
        with self._command_lock:
            self._busy = False

    @staticmethod
    def _normalize_ns(value: str) -> str:
        s = value.strip().rstrip("/")
        return s if s.startswith("/") else f"/{s}" if s else "/rg2_controller"


def main() -> None:
    rclpy.init()
    node = RG2Controller()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
