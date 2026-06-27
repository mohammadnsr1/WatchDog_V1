#!/usr/bin/env python3
"""Feedforward RG2 gripper controller with calibration-table width lookup.

Command path for open / close / set_position:
  1. target_true_width_m  →  angle_for_width() (numpy.interp on the CSV table)
  2. Publish the calibrated joint angle ONCE — Isaac's inner drive holds it.
  3. Verify loop: wait until TF width is stable (or timeout) without issuing
     any further commands.  Report success / miss; do NOT nudge on miss.

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
DEFAULT_FINGER_LINK_OFFSET_M = 0.021

# ---------------------------------------------------------------------------
# Calibration table
# ---------------------------------------------------------------------------

def _default_table_path() -> str:
    try:
        share = get_package_share_directory("isaac_trajectory_bridge")
        return str(Path(share) / "config" / "rg2_width_table.csv")
    except Exception:
        return str(Path(__file__).resolve().parent.parent / "config" / "rg2_width_table.csv")


DEFAULT_WIDTH_TABLE_PATH = _default_table_path()

# ---------------------------------------------------------------------------
# Verify-after-command parameters
# ---------------------------------------------------------------------------

DEFAULT_VERIFY_TOLERANCE_M = 0.001     # 3 mm — acceptable final error
DEFAULT_VERIFY_STABLE_DELTA = 0.0005   # 0.5 mm max change between ticks to call "stable"
DEFAULT_VERIFY_STABLE_TICKS = 4        # consecutive stable ticks required
DEFAULT_VERIFY_TIMEOUT_SEC = 2.0

# ---------------------------------------------------------------------------
# Force / effort parameters (guarded move only)
# ---------------------------------------------------------------------------

DEFAULT_CONTROL_HZ = 50.0
DEFAULT_PROBE_STEP_RAD = 0.010         # angle increment per tick during contact search
DEFAULT_EFFORT_CONTACT_THRESHOLD = 0.10
DEFAULT_EFFORT_PLATEAU_DERIV = 0.05
DEFAULT_EFFORT_PLATEAU_TICKS = 2
DEFAULT_ASYMMETRY_THRESHOLD_M = 0.010
DEFAULT_ENABLE_ASYMMETRY_FAULT = False
DEFAULT_MAX_CONTROL_ITERS = 200        # 200 × 20 ms = 4 s hard timeout


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
        self.declare_parameter("command_duration_sec", 1.0)   # kept for launch compat; unused
        self.declare_parameter("gripper_joint_names", DEFAULT_GRIPPER_JOINT_NAMES)
        self.declare_parameter("gripper_joint_multipliers", DEFAULT_GRIPPER_JOINT_MULTIPLIERS)
        self.declare_parameter("gripper_joint_offsets", DEFAULT_GRIPPER_JOINT_OFFSETS)

        self.declare_parameter("left_finger_frame", DEFAULT_LEFT_FINGER_FRAME)
        self.declare_parameter("right_finger_frame", DEFAULT_RIGHT_FINGER_FRAME)
        self.declare_parameter("gripper_base_frame", DEFAULT_GRIPPER_BASE_FRAME)
        self.declare_parameter("finger_link_offset_m", DEFAULT_FINGER_LINK_OFFSET_M)

        self.declare_parameter("width_table_path", DEFAULT_WIDTH_TABLE_PATH)

        self.declare_parameter("verify_tolerance_m", DEFAULT_VERIFY_TOLERANCE_M)
        self.declare_parameter("verify_stable_delta", DEFAULT_VERIFY_STABLE_DELTA)
        self.declare_parameter("verify_stable_ticks", DEFAULT_VERIFY_STABLE_TICKS)
        self.declare_parameter("verify_timeout_sec", DEFAULT_VERIFY_TIMEOUT_SEC)

        self.declare_parameter("probe_step_rad", DEFAULT_PROBE_STEP_RAD)
        self.declare_parameter("effort_contact_threshold", DEFAULT_EFFORT_CONTACT_THRESHOLD)
        self.declare_parameter("effort_plateau_deriv", DEFAULT_EFFORT_PLATEAU_DERIV)
        self.declare_parameter("effort_plateau_ticks", DEFAULT_EFFORT_PLATEAU_TICKS)
        self.declare_parameter("asymmetry_threshold_m", DEFAULT_ASYMMETRY_THRESHOLD_M)
        self.declare_parameter("enable_asymmetry_fault", DEFAULT_ENABLE_ASYMMETRY_FAULT)
        self.declare_parameter("max_control_iters", DEFAULT_MAX_CONTROL_ITERS)

        # ---- read --------------------------------------------------------------
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self._service_ns = self._normalize_ns(str(self.get_parameter("service_namespace").value))
        self._master_joint_name = str(self.get_parameter("master_joint_name").value)
        self._open_position = float(self.get_parameter("open_position").value)
        self._close_position = float(self.get_parameter("close_position").value)
        self._control_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
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
        self._finger_link_offset_m = float(self.get_parameter("finger_link_offset_m").value)

        self._width_table_path = str(self.get_parameter("width_table_path").value)

        self._verify_tolerance_m = float(self.get_parameter("verify_tolerance_m").value)
        self._verify_stable_delta = float(self.get_parameter("verify_stable_delta").value)
        self._verify_stable_ticks = int(self.get_parameter("verify_stable_ticks").value)
        self._verify_timeout_sec = float(self.get_parameter("verify_timeout_sec").value)

        self._probe_step_rad = float(self.get_parameter("probe_step_rad").value)
        self._effort_contact_threshold = float(self.get_parameter("effort_contact_threshold").value)
        self._effort_plateau_deriv = float(self.get_parameter("effort_plateau_deriv").value)
        self._effort_plateau_ticks = int(self.get_parameter("effort_plateau_ticks").value)
        self._asymmetry_threshold_m = float(self.get_parameter("asymmetry_threshold_m").value)
        self._enable_asymmetry_fault = bool(self.get_parameter("enable_asymmetry_fault").value)
        self._max_control_iters = int(self.get_parameter("max_control_iters").value)

        # ---- validate ----------------------------------------------------------
        if len(self._gripper_joint_names) != 6:
            raise ValueError("gripper_joint_names must contain exactly 6 joints.")
        if self._master_joint_name not in self._gripper_joint_names:
            raise ValueError("master_joint_name must be in gripper_joint_names.")

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
        self._last_known_width: Optional[float] = None

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
        """Load rg2_width_table.csv and build monotonic angle↔width interpolators.

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
            with self._state_lock:
                self._latest_joint_positions = pos
                self._latest_joint_efforts = eff

    def _width_timer_cb(self) -> None:
        result = self._read_tf_width()
        if result is not None:
            width = result[0]
            with self._state_lock:
                self._last_known_width = width
            msg = Float64()
            msg.data = width
            self._width_pub.publish(msg)

    # =========================================================================
    # TF helpers
    # =========================================================================

    def _read_tf_width(self) -> Optional[Tuple[float, float, float]]:
        """Return (true_width_m, left_y, right_y) or None on TF failure."""
        try:
            left_tf = self._tf_buffer.lookup_transform(
                self._gripper_base_frame,
                self._left_finger_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            right_tf = self._tf_buffer.lookup_transform(
                self._gripper_base_frame,
                self._right_finger_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            left_y = left_tf.transform.translation.y
            right_y = right_tf.transform.translation.y
            true_width = max(0.0, abs(right_y - left_y) - 2.0 * self._finger_link_offset_m)
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

    def _feedforward_move(
        self,
        target_width_m: float,
        label: str = "",
    ) -> Tuple[bool, float]:
        """Command the calibrated angle for target_width_m, then verify via TF.

        Steps:
          1. Look up target_angle from the width table.
          2. Publish ONE joint command.
          3. Wait until TF width is stable (change < verify_stable_delta over
             verify_stable_ticks consecutive ticks) or verify_timeout_sec elapses.
          4. Return (converged, final_width_m).
             converged = abs(final_width - target_width_m) <= verify_tolerance_m.

        Does NOT retry or nudge on failure.  A persistent miss means the table
        needs to be recalibrated.
        """
        target_angle = self.angle_for_width(target_width_m)

        if label:
            self.get_logger().info(
                f"{label}: target_width={target_width_m*1000:.1f} mm  "
                f"→ angle={target_angle:.4f} rad"
            )

        self._command_angle(target_angle)

        # Verify loop — read TF until stable or timeout
        deadline = time.time() + self._verify_timeout_sec
        prev_width: Optional[float] = None
        stable_count = 0
        final_width: float = self._get_true_width() or target_width_m

        while time.time() < deadline:
            width = self._get_true_width()
            if width is None:
                time.sleep(self._control_period_sec)
                continue

            final_width = width

            if prev_width is not None and abs(width - prev_width) < self._verify_stable_delta:
                stable_count += 1
                if stable_count >= self._verify_stable_ticks:
                    break
            else:
                stable_count = 0

            prev_width = width
            time.sleep(self._control_period_sec)

        converged = abs(final_width - target_width_m) <= self._verify_tolerance_m

        if not converged:
            self.get_logger().warn(
                f"{label}: target={target_width_m*1000:.1f} mm  "
                f"final={final_width*1000:.1f} mm  "
                f"error={abs(final_width-target_width_m)*1000:.1f} mm  "
                f"(table may need recalibration)"
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
            self._command_angle(angle)

            # Verify
            deadline = time.time() + self._verify_timeout_sec
            prev_width: Optional[float] = None
            stable_count = 0
            final_width: float = predicted_width

            while time.time() < deadline:
                width = self._get_true_width()
                if width is None:
                    time.sleep(self._control_period_sec)
                    continue
                final_width = width
                if prev_width is not None and abs(width - prev_width) < self._verify_stable_delta:
                    stable_count += 1
                    if stable_count >= self._verify_stable_ticks:
                        break
                else:
                    stable_count = 0
                prev_width = width
                time.sleep(self._control_period_sec)

            converged = abs(final_width - predicted_width) <= self._verify_tolerance_m
            resp.success = True
            resp.message = (
                f"set_position: angle={angle:.4f} rad  "
                f"predicted_width={predicted_width*1000:.1f} mm  "
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
        """Guarded move action.

        mode "width":
            Command angle_for_width(target_width_m) once, then verify.
            No effort involvement.  stop_reason = "width_reached" / "timeout".

        mode "force" / "guarded":
            Probe-close: increment commanded_angle by probe_step_rad per tick
            from the current joint angle toward CLOSE_POSITION.  After each
            step read effort_finger from /joint_states.

            PRIMARY STOP — effort > effort_contact_threshold:
                Hold current angle.  stop_reason = "force_threshold".

            For "guarded": ALSO stop if measured TF width reaches
                target_width_m ± verify_tolerance_m before contact.
                stop_reason = "width_reached".

            Over-force guard: effort > goal.force_threshold → stop immediately.

            Plateau confirm (optional, after contact detected):
                require |d_effort/dt| < effort_plateau_deriv for
                effort_plateau_ticks consecutive ticks before declaring success.

        Asymmetry (left_y + right_y in base frame) is published in feedback as
        a diagnostic only; it never triggers a stop unless enable_asymmetry_fault.
        """
        goal: GripperGuardedMove.Goal = goal_handle.request

        target_width = float(goal.target_width_m)
        force_threshold = float(goal.force_threshold) if goal.force_threshold > 0.0 else 0.45
        mode = str(goal.mode) if goal.mode else "guarded"
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
        # mode "width" — pure feedforward, no effort loop
        # ------------------------------------------------------------------
        if mode == "width":
            converged, final_width = self._feedforward_move(target_width, label="guarded_width")
            stop_reason = "width_reached" if converged else "timeout"

            result.success = converged
            result.stop_reason = stop_reason
            result.final_width_m = final_width
            result.final_effort = self._get_master_effort()
            result.asymmetry_detected = False
            result.message = (
                f"stop={stop_reason}  final_width={final_width*1000:.1f} mm"
            )
            self.get_logger().info(result.message)
            goal_handle.succeed()
            return result

        # ------------------------------------------------------------------
        # mode "force" / "guarded" — incremental probe toward contact
        # ------------------------------------------------------------------

        # Start from current joint angle, probe toward CLOSE_POSITION.
        current_angle = self._get_master_position()
        if current_angle is None:
            current_angle = self._open_position
        commanded_angle = current_angle

        prev_effort: float = 0.0
        contact_detected = False
        plateau_count = 0
        dt = self._control_period_sec

        for _ in range(self._max_control_iters):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.stop_reason = "cancelled"
                result.final_width_m = final_width
                result.final_effort = final_effort
                result.asymmetry_detected = asymmetry_ever_exceeded
                result.message = "Cancelled."
                return result

            # --- Read sensors ---
            tf_result = self._read_tf_width()
            if tf_result is None:
                time.sleep(dt)
                continue

            width, left_y, right_y = tf_result
            with self._state_lock:
                self._last_known_width = width

            effort = self._get_master_effort()
            d_effort = (effort - prev_effort) / dt
            prev_effort = effort

            asymmetry_m = abs(left_y + right_y)
            if asymmetry_m > self._asymmetry_threshold_m:
                asymmetry_ever_exceeded = True

            final_width = width
            final_effort = effort

            # Feedback
            phase = "contact" if contact_detected else "closing"
            fb.current_width_m = width
            fb.current_effort = effort
            fb.asymmetry_m = asymmetry_m
            fb.phase = phase
            goal_handle.publish_feedback(fb)

            # Over-force guard (immediate)
            if effort > force_threshold:
                stop_reason = "force_threshold"
                break

            # Contact detection + plateau confirmation
            if not contact_detected and effort > self._effort_contact_threshold:
                contact_detected = True
                plateau_count = 0

            if contact_detected:
                if abs(d_effort) < self._effort_plateau_deriv:
                    plateau_count += 1
                    if plateau_count >= self._effort_plateau_ticks:
                        stop_reason = "force_threshold"
                        break
                else:
                    plateau_count = 0

            # "guarded" mode: also stop when width target reached before contact
            if mode == "guarded" and abs(width - target_width) <= tol:
                stop_reason = "width_reached"
                break

            # Asymmetry fault (disabled by default)
            if self._enable_asymmetry_fault and asymmetry_m > self._asymmetry_threshold_m:
                stop_reason = "error"
                break

            # Probe one step toward CLOSE_POSITION — only while no contact yet.
            # Once contact is detected, hold the current angle while the plateau
            # confirmation ticks accumulate.
            if not contact_detected:
                commanded_angle = min(commanded_angle + self._probe_step_rad, self._close_position)
                self._command_angle(commanded_angle)

            time.sleep(dt)

        fb.phase = "contact" if stop_reason == "force_threshold" else stop_reason
        goal_handle.publish_feedback(fb)

        result.success = stop_reason in ("width_reached", "force_threshold")
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
