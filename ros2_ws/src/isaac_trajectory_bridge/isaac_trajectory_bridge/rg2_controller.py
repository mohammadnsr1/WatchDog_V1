#!/usr/bin/env python3
"""Closed-loop RG2 gripper controller with TF-measured finger width feedback.

Exposes the same three services as the previous open-loop controller
(/rg2_controller/open, /rg2_controller/close, /rg2_controller/set_position)
so ROSGripperBackend requires no changes.

Adds:
- /rg2_controller/guarded_move action (GripperGuardedMove) for force-guarded grasps
- /gripper_width_true topic (Float64, 50 Hz) for external width monitoring
"""

import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
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
# Calibration / geometry
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
DEFAULT_TF_WIDTH_OPEN_M = 0.1420
DEFAULT_TF_WIDTH_CLOSED_M = 0.0355
DEFAULT_TRUE_WIDTH_OPEN_M = DEFAULT_TF_WIDTH_OPEN_M - 2.0 * DEFAULT_FINGER_LINK_OFFSET_M  # 0.100

# ---------------------------------------------------------------------------
# PD control gains
#
# error     = target_width_m - current_true_width_m   (+ → need to open)
# d_error   = (error - prev_error) / dt               (m/s)
# delta_rad = -(Kp * error + Kd * d_error)
#
# Sign check (open_position < 0, close_position > 0):
#   Need to open  → error > 0 → delta_rad < 0 → master decreases → fingers open  ✓
#   Need to close → error < 0 → delta_rad > 0 → master increases → fingers close ✓
#   Approaching target while closing → d_error > 0 → Kd term reduces delta_rad → damping ✓
# ---------------------------------------------------------------------------

DEFAULT_KP_RAD_PER_M = 12.5      # proportional gain [rad/m]
DEFAULT_KD_RAD_PER_M = 2.5       # derivative gain [rad·s/m]; tune down if oscillation persists
DEFAULT_MAX_STEP_RAD = 0.020     # max |Δmaster| per 20 ms tick ≈ 1.0 rad/s
DEFAULT_N_STABLE = 3              # consecutive in-band ticks to declare convergence
DEFAULT_MAX_CONTROL_ITERS = 150  # 150 × 20 ms = 3 s timeout
DEFAULT_CONTROL_HZ = 50.0
DEFAULT_WIDTH_TOLERANCE_M = 0.003

# ---------------------------------------------------------------------------
# Force / effort parameters
# ---------------------------------------------------------------------------

DEFAULT_EFFORT_CONTACT_THRESHOLD = 0.10   # effort level that triggers contact detection
DEFAULT_EFFORT_PLATEAU_DERIV = 0.05       # |d_effort/dt| must be below this to confirm plateau
DEFAULT_EFFORT_PLATEAU_TICKS = 2          # consecutive stable ticks to confirm steady contact
DEFAULT_ASYMMETRY_THRESHOLD_M = 0.010
DEFAULT_ENABLE_ASYMMETRY_FAULT = False


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
        self.declare_parameter("command_duration_sec", 1.0)  # accepted from launch; unused
        self.declare_parameter("gripper_joint_names", DEFAULT_GRIPPER_JOINT_NAMES)
        self.declare_parameter("gripper_joint_multipliers", DEFAULT_GRIPPER_JOINT_MULTIPLIERS)
        self.declare_parameter("gripper_joint_offsets", DEFAULT_GRIPPER_JOINT_OFFSETS)

        self.declare_parameter("left_finger_frame", DEFAULT_LEFT_FINGER_FRAME)
        self.declare_parameter("right_finger_frame", DEFAULT_RIGHT_FINGER_FRAME)
        self.declare_parameter("gripper_base_frame", DEFAULT_GRIPPER_BASE_FRAME)
        self.declare_parameter("finger_link_offset_m", DEFAULT_FINGER_LINK_OFFSET_M)
        self.declare_parameter("tf_width_open_m", DEFAULT_TF_WIDTH_OPEN_M)
        self.declare_parameter("tf_width_closed_m", DEFAULT_TF_WIDTH_CLOSED_M)
        self.declare_parameter("true_width_open_m", DEFAULT_TRUE_WIDTH_OPEN_M)

        self.declare_parameter("kp_rad_per_m", DEFAULT_KP_RAD_PER_M)
        self.declare_parameter("kd_rad_per_m", DEFAULT_KD_RAD_PER_M)
        self.declare_parameter("max_step_rad", DEFAULT_MAX_STEP_RAD)
        self.declare_parameter("n_stable", DEFAULT_N_STABLE)
        self.declare_parameter("max_control_iters", DEFAULT_MAX_CONTROL_ITERS)
        self.declare_parameter("width_tolerance_m", DEFAULT_WIDTH_TOLERANCE_M)

        self.declare_parameter("effort_contact_threshold", DEFAULT_EFFORT_CONTACT_THRESHOLD)
        self.declare_parameter("effort_plateau_deriv", DEFAULT_EFFORT_PLATEAU_DERIV)
        self.declare_parameter("effort_plateau_ticks", DEFAULT_EFFORT_PLATEAU_TICKS)
        self.declare_parameter("asymmetry_threshold_m", DEFAULT_ASYMMETRY_THRESHOLD_M)
        self.declare_parameter("enable_asymmetry_fault", DEFAULT_ENABLE_ASYMMETRY_FAULT)

        # ---- read --------------------------------------------------------------
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self._service_ns = self._normalize_ns(str(self.get_parameter("service_namespace").value))
        self._master_joint_name = str(self.get_parameter("master_joint_name").value)
        self._open_position = float(self.get_parameter("open_position").value)
        self._close_position = float(self.get_parameter("close_position").value)
        self._control_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
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
        self._tf_width_open_m = float(self.get_parameter("tf_width_open_m").value)
        self._tf_width_closed_m = float(self.get_parameter("tf_width_closed_m").value)
        self._true_width_open_m = float(self.get_parameter("true_width_open_m").value)

        self._kp_rad_per_m = float(self.get_parameter("kp_rad_per_m").value)
        self._kd_rad_per_m = float(self.get_parameter("kd_rad_per_m").value)
        self._max_step_rad = float(self.get_parameter("max_step_rad").value)
        self._n_stable = int(self.get_parameter("n_stable").value)
        self._max_control_iters = int(self.get_parameter("max_control_iters").value)
        self._width_tolerance_m = float(self.get_parameter("width_tolerance_m").value)
        self._control_period_sec = 1.0 / self._control_hz

        self._effort_contact_threshold = float(self.get_parameter("effort_contact_threshold").value)
        self._effort_plateau_deriv = float(self.get_parameter("effort_plateau_deriv").value)
        self._effort_plateau_ticks = int(self.get_parameter("effort_plateau_ticks").value)
        self._asymmetry_threshold_m = float(self.get_parameter("asymmetry_threshold_m").value)
        self._enable_asymmetry_fault = bool(self.get_parameter("enable_asymmetry_fault").value)

        # ---- validate ----------------------------------------------------------
        if len(self._gripper_joint_names) != 6:
            raise ValueError("gripper_joint_names must contain exactly 6 joints.")
        if self._master_joint_name not in self._gripper_joint_names:
            raise ValueError("master_joint_name must be in gripper_joint_names.")

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
            f"RG2 closed-loop controller ready (PD) — "
            f"services: {self._service_ns}/{{open,close,set_position}}  "
            f"action: {self._service_ns}/guarded_move  "
            f"Kp={self._kp_rad_per_m:.1f} Kd={self._kd_rad_per_m:.2f} "
            f"max_step={self._max_step_rad:.3f} n_stable={self._n_stable}"
        )

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
    # PD control kernel
    # =========================================================================

    def _pd_step(
        self,
        commanded_master: float,
        error: float,
        prev_error: float,
    ) -> Tuple[float, float]:
        """One PD tick. Returns (new_commanded_master, error_as_new_prev_error).

        error = target_width - current_width (positive → need to open)
        delta_rad = -(Kp * error + Kd * d_error/dt)
        Clamped to ±max_step_rad then to [open_position, close_position].
        """
        d_error = (error - prev_error) / self._control_period_sec
        delta_rad = -(self._kp_rad_per_m * error + self._kd_rad_per_m * d_error)
        delta_rad = max(-self._max_step_rad, min(self._max_step_rad, delta_rad))
        new_master = max(
            self._open_position,
            min(self._close_position, commanded_master + delta_rad),
        )
        return new_master, error

    # =========================================================================
    # Core closed-loop width controller (used by services)
    # =========================================================================

    def _closed_loop_control(
        self,
        target_true_width_m: float,
        width_tolerance: Optional[float] = None,
        max_iters: Optional[int] = None,
    ) -> Tuple[bool, float]:
        """Drive the gripper to target_true_width_m with PD width control.

        Declares convergence after N_STABLE consecutive ticks inside tolerance.
        Returns (converged, final_true_width_m).
        """
        tol = width_tolerance if width_tolerance is not None else self._width_tolerance_m
        iters = max_iters if max_iters is not None else self._max_control_iters

        commanded_master = self._get_master_position()
        if commanded_master is None:
            commanded_master = (self._open_position + self._close_position) / 2.0

        prev_error: Optional[float] = None
        stable_count = 0
        final_width: float = self._get_true_width() or 0.0

        for _ in range(iters):
            width = self._get_true_width()
            if width is None:
                time.sleep(self._control_period_sec)
                continue

            final_width = width
            error = target_true_width_m - width

            if abs(error) < tol:
                stable_count += 1
                if stable_count >= self._n_stable:
                    return True, width
            else:
                stable_count = 0

            if prev_error is None:
                prev_error = error  # first tick: d_error = 0

            commanded_master, prev_error = self._pd_step(commanded_master, error, prev_error)
            self._publish_command(self._master_to_positions(commanded_master))
            time.sleep(self._control_period_sec)

        width = self._get_true_width()
        if width is not None:
            final_width = width
        return abs(target_true_width_m - final_width) < tol, final_width

    # =========================================================================
    # Service callbacks
    # =========================================================================

    def _open_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        return self._handle_svc(self._true_width_open_m, "open", resp)

    def _close_cb(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        return self._handle_svc(0.0, "close", resp)

    def _set_position_cb(
        self,
        req: SetGripperPosition.Request,
        resp: SetGripperPosition.Response,
    ) -> SetGripperPosition.Response:
        # Convert master joint radians → true width via linear calibration
        joint_span = self._close_position - self._open_position
        fraction = (req.position - self._open_position) / joint_span
        target_width = self._true_width_open_m * (1.0 - max(0.0, min(1.0, fraction)))
        return self._handle_svc(target_width, "set_position", resp)

    def _handle_svc(self, target_width_m: float, label: str, resp) -> object:
        if not self._acquire():
            resp.success = False
            resp.message = "Busy."
            return resp
        try:
            converged, final_width = self._closed_loop_control(target_width_m)
            resp.success = True
            resp.message = (
                f"{label}: target={target_width_m:.4f}m "
                f"final={final_width:.4f}m converged={converged}"
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
        """Guarded move with PD width control and plateau-confirmed force stop.

        Force termination has two layers:
          1. Over-force guard (immediate): effort exceeds goal.force_threshold → stop now.
          2. Contact + plateau: effort crosses EFFORT_CONTACT_THRESHOLD, then
             abs(d_effort/dt) < EFFORT_PLATEAU_DERIV for EFFORT_PLATEAU_TICKS
             consecutive ticks → declares stable contact and stops.
        """
        goal: GripperGuardedMove.Goal = goal_handle.request

        target_width = float(goal.target_width_m)
        force_threshold = float(goal.force_threshold) if goal.force_threshold > 0.0 else 0.45
        mode = str(goal.mode) if goal.mode else "guarded"
        tol = float(goal.width_tolerance_m) if goal.width_tolerance_m > 0.0 else self._width_tolerance_m

        commanded_master = self._get_master_position()
        if commanded_master is None:
            commanded_master = self._open_position

        self.get_logger().info(
            f"GuardedMove: target={target_width:.4f}m force_threshold={force_threshold:.3f} "
            f"mode={mode} tol={tol:.4f}m"
        )

        fb = GripperGuardedMove.Feedback()
        result = GripperGuardedMove.Result()
        stop_reason = "timeout"

        # PD state
        prev_error: Optional[float] = None
        stable_count = 0

        # Force state
        prev_effort: float = 0.0
        contact_detected = False
        plateau_count = 0

        # Asymmetry tracking (diagnostic only by default)
        asymmetry_ever_exceeded = False

        final_width: float = self._get_true_width() or 0.0
        final_effort: float = self._get_master_effort()
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

            # Asymmetry: symmetric gripper → left_y + right_y ≈ 0 in base frame
            asymmetry_m = abs(left_y + right_y)
            if asymmetry_m > self._asymmetry_threshold_m:
                asymmetry_ever_exceeded = True

            final_width = width
            final_effort = effort

            # --- Feedback ---
            phase = (
                "contact" if contact_detected
                else ("closing" if target_width < width else "opening")
            )
            fb.current_width_m = width
            fb.current_effort = effort
            fb.asymmetry_m = asymmetry_m
            fb.phase = phase
            goal_handle.publish_feedback(fb)

            # --- Force: over-force guard (immediate stop, any effort spike) ---
            if mode in ("force", "guarded") and effort > force_threshold:
                stop_reason = "force_threshold"
                break

            # --- Force: contact detection + plateau confirmation ---
            if mode in ("force", "guarded"):
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
                        plateau_count = 0  # effort still rising; wait for it to settle

            # --- Asymmetry fault (only when explicitly enabled) ---
            if self._enable_asymmetry_fault and asymmetry_m > self._asymmetry_threshold_m:
                stop_reason = "error"
                break

            # --- Width convergence: N_STABLE consecutive in-band ticks ---
            error = target_width - width

            if mode in ("width", "guarded"):
                if abs(error) < tol:
                    stable_count += 1
                    if stable_count >= self._n_stable:
                        stop_reason = "width_reached"
                        break
                else:
                    stable_count = 0

            # --- PD step (always, even while inside band — small corrections are fine) ---
            if prev_error is None:
                prev_error = error  # first tick: derivative kick = 0

            commanded_master, prev_error = self._pd_step(commanded_master, error, prev_error)
            self._publish_command(self._master_to_positions(commanded_master))
            time.sleep(dt)

        fb.phase = "contact" if stop_reason == "force_threshold" else stop_reason
        goal_handle.publish_feedback(fb)

        result.success = stop_reason in ("width_reached", "force_threshold")
        result.stop_reason = stop_reason
        result.final_width_m = final_width
        result.final_effort = final_effort
        result.asymmetry_detected = asymmetry_ever_exceeded
        result.message = (
            f"stop={stop_reason} final_width={final_width:.4f}m "
            f"effort={final_effort:.3f} asymmetry_detected={asymmetry_ever_exceeded}"
        )
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    # =========================================================================
    # Helpers
    # =========================================================================

    def _master_to_positions(self, master: float) -> List[float]:
        return [
            off + mult * master
            for mult, off in zip(self._gripper_joint_multipliers, self._gripper_joint_offsets)
        ]

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
