#!/usr/bin/env python3
"""Closed-loop RG2 gripper controller with TF-measured finger width feedback.

Exposes the same three services as the previous open-loop controller
(/rg2_controller/open, /rg2_controller/close, /rg2_controller/set_position)
so ROSGripperBackend requires no changes.

Adds:
- /rg2_controller/guarded_move action (GripperGuardedMove) for force-guarded grasps
- /gripper_width_true topic (Float64, 50 Hz) for external width monitoring
"""

import math
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
from robot_backend_interfaces.action import GripperGuardedMove


# ---------------------------------------------------------------------------
# Default joint configuration (unchanged from original controller)
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
DEFAULT_MASTER_JOINT_MIN_POSITION = -0.558505
DEFAULT_MASTER_JOINT_MAX_POSITION = 0.785398
DEFAULT_OPEN_POSITION = -0.558505
DEFAULT_CLOSE_POSITION = 0.7731809

# ---------------------------------------------------------------------------
# Calibration defaults (all overridable as ROS parameters)
# ---------------------------------------------------------------------------

DEFAULT_LEFT_FINGER_FRAME = "onrobot_rg2_left_inner_finger"
DEFAULT_RIGHT_FINGER_FRAME = "onrobot_rg2_right_inner_finger"
DEFAULT_GRIPPER_BASE_FRAME = "onrobot_rg2_base_link"
DEFAULT_FINGER_LINK_OFFSET_M = 0.021
DEFAULT_TF_WIDTH_OPEN_M = 0.1420
DEFAULT_TF_WIDTH_CLOSED_M = 0.0355
DEFAULT_TRUE_WIDTH_OPEN_M = DEFAULT_TF_WIDTH_OPEN_M - 2.0 * DEFAULT_FINGER_LINK_OFFSET_M   # 0.100
DEFAULT_EFFORT_FREE_SPACE_MAX = 0.05
DEFAULT_EFFORT_CONTACT_THRESHOLD = 0.10
DEFAULT_EFFORT_GRASP_PLATEAU = 0.55
DEFAULT_ASYMMETRY_THRESHOLD_M = 0.010
DEFAULT_WIDTH_TOLERANCE_M = 0.003
DEFAULT_MAX_CONTROL_ITERS = 50
DEFAULT_CONTROL_HZ = 50.0


class RG2Controller(Node):
    def __init__(self) -> None:
        super().__init__("rg2_controller")

        # ---- legacy joint parameters ----------------------------------------
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("service_namespace", "/rg2_controller")
        self.declare_parameter("master_joint_name", DEFAULT_MASTER_JOINT_NAME)
        self.declare_parameter("master_joint_min_position", DEFAULT_MASTER_JOINT_MIN_POSITION)
        self.declare_parameter("master_joint_max_position", DEFAULT_MASTER_JOINT_MAX_POSITION)
        self.declare_parameter("open_position", DEFAULT_OPEN_POSITION)
        self.declare_parameter("close_position", DEFAULT_CLOSE_POSITION)
        self.declare_parameter("command_rate_hz", DEFAULT_CONTROL_HZ)
        self.declare_parameter("command_duration_sec", 1.0)  # kept for launch-file compat; unused by closed-loop control
        self.declare_parameter("wait_for_joint_state_timeout_sec", 2.0)
        self.declare_parameter("gripper_joint_names", DEFAULT_GRIPPER_JOINT_NAMES)
        self.declare_parameter("gripper_joint_multipliers", DEFAULT_GRIPPER_JOINT_MULTIPLIERS)
        self.declare_parameter("gripper_joint_offsets", DEFAULT_GRIPPER_JOINT_OFFSETS)

        # ---- calibration parameters -----------------------------------------
        self.declare_parameter("left_finger_frame", DEFAULT_LEFT_FINGER_FRAME)
        self.declare_parameter("right_finger_frame", DEFAULT_RIGHT_FINGER_FRAME)
        self.declare_parameter("gripper_base_frame", DEFAULT_GRIPPER_BASE_FRAME)
        self.declare_parameter("finger_link_offset_m", DEFAULT_FINGER_LINK_OFFSET_M)
        self.declare_parameter("tf_width_open_m", DEFAULT_TF_WIDTH_OPEN_M)
        self.declare_parameter("tf_width_closed_m", DEFAULT_TF_WIDTH_CLOSED_M)
        self.declare_parameter("true_width_open_m", DEFAULT_TRUE_WIDTH_OPEN_M)
        self.declare_parameter("effort_free_space_max", DEFAULT_EFFORT_FREE_SPACE_MAX)
        self.declare_parameter("effort_contact_threshold", DEFAULT_EFFORT_CONTACT_THRESHOLD)
        self.declare_parameter("effort_grasp_plateau", DEFAULT_EFFORT_GRASP_PLATEAU)
        self.declare_parameter("asymmetry_threshold_m", DEFAULT_ASYMMETRY_THRESHOLD_M)
        self.declare_parameter("width_tolerance_m", DEFAULT_WIDTH_TOLERANCE_M)
        self.declare_parameter("max_control_iters", DEFAULT_MAX_CONTROL_ITERS)
        # use_sim_time is declared automatically by the rclpy Node base class;
        # redeclaring it raises ParameterAlreadyDeclaredException.

        # ---- read joint parameters ------------------------------------------
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self.service_namespace = self._normalize_service_namespace(
            str(self.get_parameter("service_namespace").value)
        )
        self.master_joint_name = str(self.get_parameter("master_joint_name").value)
        self.master_joint_min_position = float(self.get_parameter("master_joint_min_position").value)
        self.master_joint_max_position = float(self.get_parameter("master_joint_max_position").value)
        self.open_position = float(self.get_parameter("open_position").value)
        self.close_position = float(self.get_parameter("close_position").value)
        self.command_rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.wait_for_joint_state_timeout_sec = max(
            0.0, float(self.get_parameter("wait_for_joint_state_timeout_sec").value)
        )
        self.gripper_joint_names = list(self.get_parameter("gripper_joint_names").value)
        self.gripper_joint_multipliers = [
            float(v) for v in self.get_parameter("gripper_joint_multipliers").value
        ]
        self.gripper_joint_offsets = [
            float(v) for v in self.get_parameter("gripper_joint_offsets").value
        ]

        # ---- read calibration parameters ------------------------------------
        self._left_finger_frame = str(self.get_parameter("left_finger_frame").value)
        self._right_finger_frame = str(self.get_parameter("right_finger_frame").value)
        self._gripper_base_frame = str(self.get_parameter("gripper_base_frame").value)
        self._finger_link_offset_m = float(self.get_parameter("finger_link_offset_m").value)
        self._true_width_open_m = float(self.get_parameter("true_width_open_m").value)
        self._effort_contact_threshold = float(self.get_parameter("effort_contact_threshold").value)
        self._asymmetry_threshold_m = float(self.get_parameter("asymmetry_threshold_m").value)
        self._width_tolerance_m = float(self.get_parameter("width_tolerance_m").value)
        self._max_control_iters = int(self.get_parameter("max_control_iters").value)

        # Closed-loop gain: rad of master joint per meter of true width
        # Increasing master → closing → decreasing width, so gain maps |Δwidth| → |Δmaster|
        joint_range = self.close_position - self.open_position
        tf_width_range = (
            float(self.get_parameter("tf_width_open_m").value)
            - float(self.get_parameter("tf_width_closed_m").value)
        )
        self._gain_rad_per_m = joint_range / tf_width_range  # ≈ 12.56

        self._control_period_sec = 1.0 / self.command_rate_hz

        # ---- validate -------------------------------------------------------
        if len(self.gripper_joint_names) != 6:
            raise ValueError("gripper_joint_names must contain exactly 6 RG2 joints.")
        if len(self.gripper_joint_multipliers) != len(self.gripper_joint_names):
            raise ValueError("gripper_joint_multipliers must match gripper_joint_names length.")
        if len(self.gripper_joint_offsets) != len(self.gripper_joint_names):
            raise ValueError("gripper_joint_offsets must match gripper_joint_names length.")
        if self.master_joint_name not in self.gripper_joint_names:
            raise ValueError("master_joint_name must be present in gripper_joint_names.")

        # ---- state ----------------------------------------------------------
        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._busy = False
        self._latest_joint_positions: Dict[str, float] = {}
        self._latest_joint_efforts: Dict[str, float] = {}
        self._last_known_width: Optional[float] = None
        self._width_lock = threading.Lock()

        # ---- TF -------------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- publishers / subscribers ---------------------------------------
        self._publisher = self.create_publisher(JointState, self.joint_command_topic, 10)
        self._width_publisher = self.create_publisher(Float64, "/gripper_width_true", 10)

        self._subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            50,
        )

        self._width_timer = self.create_timer(
            self._control_period_sec,
            self._publish_width_callback,
        )

        # ---- services (mutually exclusive — only one runs at a time) --------
        svc_group = MutuallyExclusiveCallbackGroup()
        self._open_service = self.create_service(
            Trigger,
            f"{self.service_namespace}/open",
            self._open_callback,
            callback_group=svc_group,
        )
        self._close_service = self.create_service(
            Trigger,
            f"{self.service_namespace}/close",
            self._close_callback,
            callback_group=svc_group,
        )
        self._set_position_service = self.create_service(
            SetGripperPosition,
            f"{self.service_namespace}/set_position",
            self._set_position_callback,
            callback_group=svc_group,
        )

        # ---- action server --------------------------------------------------
        action_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            GripperGuardedMove,
            f"{self.service_namespace}/guarded_move",
            self._execute_guarded_move,
            callback_group=action_group,
        )

        self.get_logger().info(
            f"RG2 closed-loop controller ready. "
            f"Services: {self.service_namespace}/open|close|set_position  "
            f"Action: {self.service_namespace}/guarded_move  "
            f"Gain: {self._gain_rad_per_m:.2f} rad/m"
        )

    # =========================================================================
    # Joint state callback
    # =========================================================================

    def _joint_state_callback(self, msg: JointState) -> None:
        joint_map = {name: idx for idx, name in enumerate(msg.name)}
        positions: Dict[str, float] = {}
        efforts: Dict[str, float] = {}

        for joint_name in self.gripper_joint_names:
            idx = joint_map.get(joint_name)
            if idx is None:
                continue
            if idx < len(msg.position):
                positions[joint_name] = msg.position[idx]
            if msg.effort and idx < len(msg.effort):
                efforts[joint_name] = msg.effort[idx]

        if not positions:
            return

        with self._state_lock:
            self._latest_joint_positions = positions
            self._latest_joint_efforts = efforts

    # =========================================================================
    # Width publisher timer (50 Hz)
    # =========================================================================

    def _publish_width_callback(self) -> None:
        result = self._read_tf_width()
        if result is not None:
            width, _, _ = result
            with self._width_lock:
                self._last_known_width = width
            msg = Float64()
            msg.data = width
            self._width_publisher.publish(msg)

    # =========================================================================
    # TF width reading
    # =========================================================================

    def _read_tf_width(self) -> Optional[Tuple[float, float, float]]:
        """Return (true_width_m, left_y, right_y) from TF, or None on failure."""
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
        """Get true width from TF, falling back to last known on failure."""
        result = self._read_tf_width()
        if result is not None:
            width = result[0]
            with self._width_lock:
                self._last_known_width = width
            return width
        with self._width_lock:
            return self._last_known_width

    def _get_asymmetry(self, left_y: float, right_y: float) -> float:
        """Absolute deviation of one finger from symmetric closure midpoint."""
        mid = (left_y + right_y) / 2.0
        return abs(left_y - mid)

    # =========================================================================
    # Joint state helpers
    # =========================================================================

    def _get_master_position(self) -> Optional[float]:
        with self._state_lock:
            return self._latest_joint_positions.get(self.master_joint_name)

    def _get_master_effort(self) -> float:
        with self._state_lock:
            return self._latest_joint_efforts.get(self.master_joint_name, 0.0)

    def _snapshot_gripper_positions(self) -> Optional[List[float]]:
        with self._state_lock:
            if not all(name in self._latest_joint_positions for name in self.gripper_joint_names):
                return None
            return [self._latest_joint_positions[name] for name in self.gripper_joint_names]

    # =========================================================================
    # Closed-loop width controller (used by services)
    # =========================================================================

    def _closed_loop_control(
        self,
        target_true_width_m: float,
        width_tolerance: float = DEFAULT_WIDTH_TOLERANCE_M,
        max_iters: int = DEFAULT_MAX_CONTROL_ITERS,
    ) -> Tuple[bool, float]:
        """Step the master joint incrementally until TF width reaches target.

        Returns (tolerance_met, final_true_width_m).
        Blocks for up to max_iters * control_period_sec seconds.
        Requires the node to be spinning in another thread (MultiThreadedExecutor)
        so TF and joint_state callbacks keep firing while this sleeps.
        """
        final_width = self._get_true_width() or 0.0

        for _ in range(max_iters):
            width = self._get_true_width()
            if width is None:
                time.sleep(self._control_period_sec)
                continue

            final_width = width
            error = target_true_width_m - width

            if abs(error) < width_tolerance:
                return True, width

            master_pos = self._get_master_position()
            if master_pos is None:
                time.sleep(self._control_period_sec)
                continue

            # Increasing master closes the gripper (decreases width), so:
            # to reduce width (error < 0): nudge master positive
            # to increase width (error > 0): nudge master negative
            nudge = -self._gain_rad_per_m * error * 0.3
            new_master = max(
                self.open_position,
                min(self.close_position, master_pos + nudge),
            )

            self._publish_joint_command(self._compute_target_positions(new_master))
            time.sleep(self._control_period_sec)

        # Final measurement after loop
        width = self._get_true_width()
        if width is not None:
            final_width = width

        return abs(target_true_width_m - final_width) < width_tolerance, final_width

    # =========================================================================
    # Service callbacks
    # =========================================================================

    def _open_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._handle_width_command(self._true_width_open_m, "open", response)

    def _close_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._handle_width_command(0.0, "close", response)

    def _set_position_callback(
        self,
        request: SetGripperPosition.Request,
        response: SetGripperPosition.Response,
    ) -> SetGripperPosition.Response:
        # Convert master joint position (rad) → true width (m)
        joint_span = self.close_position - self.open_position
        fraction = (request.position - self.open_position) / joint_span
        fraction = max(0.0, min(1.0, fraction))
        target_width = self._true_width_open_m * (1.0 - fraction)
        return self._handle_width_command(target_width, "set_position", response)

    def _handle_width_command(
        self, target_width_m: float, label: str, response
    ):
        if not self._begin_command():
            response.success = False
            response.message = "RG2 controller is busy with another command."
            return response

        try:
            success, final_width = self._closed_loop_control(
                target_width_m,
                width_tolerance=self._width_tolerance_m,
                max_iters=self._max_control_iters,
            )
            response.success = True
            response.message = (
                f"RG2 {label}: target={target_width_m:.4f}m "
                f"final={final_width:.4f}m converged={success}"
            )
            self.get_logger().info(response.message)
            return response
        except Exception as exc:
            self.get_logger().error(f"RG2 {label} failed: {exc}")
            response.success = False
            response.message = f"RG2 {label} failed: {exc}"
            return response
        finally:
            self._end_command()

    # =========================================================================
    # GripperGuardedMove action server
    # =========================================================================

    def _execute_guarded_move(self, goal_handle) -> GripperGuardedMove.Result:
        """Guarded-move state machine: FREE_SPACE → WIDTH_REACHED | CONTACT | FAULT."""
        goal: GripperGuardedMove.Goal = goal_handle.request

        target_width = float(goal.target_width_m)
        force_threshold = float(goal.force_threshold) if goal.force_threshold > 0.0 else 0.45
        mode = str(goal.mode) if goal.mode else "guarded"
        width_tolerance = float(goal.width_tolerance_m) if goal.width_tolerance_m > 0.0 else self._width_tolerance_m

        self.get_logger().info(
            f"GuardedMove: target={target_width:.4f}m force_threshold={force_threshold:.3f} "
            f"mode={mode} tolerance={width_tolerance:.4f}m"
        )

        result = GripperGuardedMove.Result()
        feedback = GripperGuardedMove.Feedback()

        stop_reason = "timeout"
        asymmetry_detected = False
        final_width = self._get_true_width() or 0.0
        final_effort = self._get_master_effort()

        for _ in range(self._max_control_iters):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.stop_reason = "cancelled"
                result.final_width_m = final_width
                result.final_effort = final_effort
                result.asymmetry_detected = asymmetry_detected
                result.message = "Goal cancelled by client."
                return result

            # --- Read sensors ---
            tf_result = self._read_tf_width()
            if tf_result is not None:
                current_width, left_y, right_y = tf_result
                with self._width_lock:
                    self._last_known_width = current_width
                asymmetry = self._get_asymmetry(left_y, right_y)
            else:
                current_width = self._get_true_width() or final_width
                asymmetry = 0.0

            current_effort = abs(self._get_master_effort())
            final_width = current_width
            final_effort = current_effort

            # --- Feedback ---
            error = target_width - current_width
            feedback.current_width_m = current_width
            feedback.current_effort = current_effort
            feedback.asymmetry_m = asymmetry
            feedback.phase = "closing"
            goal_handle.publish_feedback(feedback)

            # --- Termination checks ---
            width_reached = abs(error) < width_tolerance
            force_hit = current_effort > force_threshold
            asymmetry_fault = asymmetry > self._asymmetry_threshold_m

            if asymmetry_fault:
                asymmetry_detected = True
                stop_reason = "asymmetry_detected"
                feedback.phase = "contact"
                goal_handle.publish_feedback(feedback)
                break

            if mode == "width":
                if width_reached:
                    stop_reason = "width_reached"
                    feedback.phase = "width_reached"
                    goal_handle.publish_feedback(feedback)
                    break
            elif mode == "force":
                if force_hit:
                    stop_reason = "force_threshold"
                    feedback.phase = "contact"
                    goal_handle.publish_feedback(feedback)
                    break
            else:  # "guarded"
                if width_reached:
                    stop_reason = "width_reached"
                    feedback.phase = "width_reached"
                    goal_handle.publish_feedback(feedback)
                    break
                if force_hit:
                    stop_reason = "force_threshold"
                    feedback.phase = "contact"
                    goal_handle.publish_feedback(feedback)
                    break

            # --- Control nudge ---
            master_pos = self._get_master_position()
            if master_pos is None:
                time.sleep(self._control_period_sec)
                continue

            nudge = -self._gain_rad_per_m * error * 0.3
            new_master = max(
                self.open_position,
                min(self.close_position, master_pos + nudge),
            )
            self._publish_joint_command(self._compute_target_positions(new_master))
            time.sleep(self._control_period_sec)

        success = stop_reason in ("width_reached", "force_threshold")

        result.success = success
        result.stop_reason = stop_reason
        result.final_width_m = final_width
        result.final_effort = final_effort
        result.asymmetry_detected = asymmetry_detected
        result.message = (
            f"GuardedMove done: stop_reason={stop_reason} "
            f"final_width={final_width:.4f}m effort={final_effort:.3f}"
        )

        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    # =========================================================================
    # Command bus helpers (identical to original controller)
    # =========================================================================

    def _begin_command(self) -> bool:
        with self._command_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _end_command(self) -> None:
        with self._command_lock:
            self._busy = False

    def _compute_target_positions(self, master_position: float) -> List[float]:
        return [
            offset + multiplier * master_position
            for multiplier, offset in zip(
                self.gripper_joint_multipliers,
                self.gripper_joint_offsets,
            )
        ]

    def _clamp_master_position(self, requested_position: float) -> float:
        return min(
            max(requested_position, self.master_joint_min_position),
            self.master_joint_max_position,
        )

    def _publish_joint_command(self, positions: Sequence[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.gripper_joint_names)
        msg.position = list(positions)
        self._publisher.publish(msg)

    @staticmethod
    def _normalize_service_namespace(value: str) -> str:
        stripped = value.strip()
        if not stripped:
            return "/rg2_controller"
        if not stripped.startswith("/"):
            stripped = "/" + stripped
        return stripped.rstrip("/")


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
