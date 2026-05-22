"""ROS-backed MoveIt motion backend."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any
from typing import Mapping

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import RobotState
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

from robot_backend.interfaces.motion_backend import MotionBackend
from robot_backend.interfaces.motion_backend import PoseLike


ARM_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
DEFAULT_PLANNING_GROUP = "ur_manipulator"
DEFAULT_TCP_FRAME = "tcp"
DEFAULT_MOVE_GROUP_ACTION = "/move_action"
DEFAULT_IK_SERVICE_NAME = "/compute_ik"
DEFAULT_JOINT_STATE_TOPIC = "/joint_states"
_DEFAULT_ALLOWED_PLANNING_TIME = 5.0
_DEFAULT_ACTION_TIMEOUT = 30.0
_DEFAULT_GOAL_POSITION_TOLERANCE_M = 0.005
_DEFAULT_GOAL_ORIENTATION_TOLERANCE_RAD = 0.01
_DEFAULT_IK_TIMEOUT = 5.0
_INITIAL_JOINT_STATE_TIMEOUT = 1.0

_ERROR_CODE_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}


class ROSMoveItMotionBackend(MotionBackend):
    """Motion backend wrapper for MoveIt planning and execution."""

    def __init__(
        self,
        node: Node | None = None,
        *,
        move_group_action_name: str = DEFAULT_MOVE_GROUP_ACTION,
        ik_service_name: str = DEFAULT_IK_SERVICE_NAME,
        joint_state_topic: str = DEFAULT_JOINT_STATE_TOPIC,
    ) -> None:
        self._owns_node = node is None
        self._node = node or self._create_node()
        self._move_group_action_name = move_group_action_name
        self._ik_service_name = ik_service_name
        self._joint_state_topic = joint_state_topic
        self._latest_joint_state: JointState | None = None
        self._joint_state_received = False
        self._joint_state_subscription = self._node.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_callback,
            10,
        )
        self._move_group_client = ActionClient(
            self._node,
            MoveGroup,
            self._move_group_action_name,
        )
        self._ik_client = self._node.create_client(
            GetPositionIK,
            self._ik_service_name,
        )

    @property
    def node(self) -> Node:
        """Return the shared ROS node used for motion operations."""

        return self._node

    def move_to_pose(
        self,
        target_pose: PoseLike,
        planning_group: str = DEFAULT_PLANNING_GROUP,
        tcp_frame: str = DEFAULT_TCP_FRAME,
        **kwargs: Any,
    ) -> dict[str, Any]:
        """Resolve a pose target through MoveIt IK, then plan as a joint goal."""

        pose_stamped = self._coerce_pose_stamped(target_pose)
        ik_joint_positions = self._compute_ik_joint_positions(
            pose_stamped=pose_stamped,
            planning_group=planning_group,
            tcp_frame=tcp_frame,
            timeout_sec=float(kwargs.get("ik_timeout_sec", _DEFAULT_IK_TIMEOUT)),
            avoid_collisions=bool(kwargs.get("avoid_collisions", True)),
        )
        joint_result = self.move_to_joint_positions(
            ik_joint_positions,
            planning_group=planning_group,
            **kwargs,
        )
        return {
            "ok": True,
            "command": "move_to_pose",
            "planning_group": planning_group,
            "tcp_frame": tcp_frame,
            "target_pose": self._pose_stamped_to_dict(pose_stamped),
            "ik_solution": ik_joint_positions,
            "response": joint_result["response"],
        }

    def move_to_joint_positions(
        self,
        joint_positions: Mapping[str, float] | list[float] | tuple[float, ...],
        planning_group: str = DEFAULT_PLANNING_GROUP,
        **kwargs: Any,
    ) -> dict[str, Any]:
        """Plan and execute a motion to explicit joint targets."""

        normalized_joint_positions = self._normalize_joint_positions(joint_positions)
        allowed_planning_time = float(
            kwargs.get("allowed_planning_time", _DEFAULT_ALLOWED_PLANNING_TIME)
        )

        goal = MoveGroup.Goal()
        goal.request = self._build_joint_motion_plan_request(
            joint_positions=normalized_joint_positions,
            planning_group=planning_group,
            allowed_planning_time=allowed_planning_time,
            num_planning_attempts=int(kwargs.get("num_planning_attempts", 1)),
            max_velocity_scaling_factor=float(
                kwargs.get("max_velocity_scaling_factor", 1.0)
            ),
            max_acceleration_scaling_factor=float(
                kwargs.get("max_acceleration_scaling_factor", 1.0)
            ),
            joint_tolerance=float(kwargs.get("joint_tolerance", 1e-3)),
        )
        goal.planning_options = self._build_planning_options(kwargs)

        result_response = self._execute_move_group_goal(
            goal,
            timeout_sec=float(kwargs.get("timeout_sec", _DEFAULT_ACTION_TIMEOUT)),
        )

        return {
            "ok": True,
            "command": "move_to_joint_positions",
            "planning_group": planning_group,
            "joint_positions": normalized_joint_positions,
            "response": self._summarize_move_group_result(result_response),
        }

    def get_robot_state(self) -> dict[str, Any]:
        """Return backend metadata and the latest observed arm joint state."""

        state: dict[str, Any] = {
            "planning_group_default": DEFAULT_PLANNING_GROUP,
            "tcp_frame_default": DEFAULT_TCP_FRAME,
            "move_group_action_name": self._move_group_action_name,
            "ik_service_name": self._ik_service_name,
            "joint_state_topic": self._joint_state_topic,
            "known_arm_joint_names": list(ARM_JOINT_NAMES),
            "known_arm_joint_order": {
                joint_name: index for index, joint_name in enumerate(ARM_JOINT_NAMES)
            },
        }

        joint_state = self._latest_joint_state
        if joint_state is None:
            try:
                joint_state = self._wait_for_joint_state(
                    timeout_sec=_INITIAL_JOINT_STATE_TIMEOUT
                )
            except TimeoutError:
                state["current_joint_positions"] = None
                state["todo"] = (
                    "No /joint_states message has been observed yet. When the ROS graph "
                    "is active, this backend will report the latest arm joint positions."
                )
                return state

        joint_map = {
            name: joint_state.position[index]
            for index, name in enumerate(joint_state.name)
            if index < len(joint_state.position)
        }
        state["current_joint_positions"] = {
            joint_name: joint_map.get(joint_name) for joint_name in ARM_JOINT_NAMES
        }
        state["joint_state_header_frame_id"] = joint_state.header.frame_id
        return state

    def _create_node(self) -> Node:
        """Create an internal helper node if one was not supplied."""

        if not rclpy.ok():
            rclpy.init(args=None)
        return Node("robot_backend_moveit_motion_client")

    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.name:
            return
        self._latest_joint_state = msg
        self._joint_state_received = True

    def _wait_for_joint_state(self, timeout_sec: float) -> JointState:
        if self._latest_joint_state is not None:
            return self._latest_joint_state

        deadline_ns = self._node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and not self._joint_state_received:
            if self._node.get_clock().now().nanoseconds >= deadline_ns:
                raise TimeoutError(
                    f"Timed out waiting for {self._joint_state_topic}."
                )
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if self._latest_joint_state is None:
            raise RuntimeError(
                f"{self._joint_state_topic} callback completed without joint data."
            )
        return self._latest_joint_state

    def _normalize_joint_positions(
        self,
        joint_positions: Mapping[str, float] | Sequence[float],
    ) -> dict[str, float]:
        if isinstance(joint_positions, Mapping):
            missing = [name for name in ARM_JOINT_NAMES if name not in joint_positions]
            unexpected = sorted(
                name for name in joint_positions.keys() if name not in ARM_JOINT_NAMES
            )
            if missing or unexpected:
                details = []
                if missing:
                    details.append(f"missing joints: {missing}")
                if unexpected:
                    details.append(f"unexpected joints: {unexpected}")
                raise ValueError("Invalid joint mapping; " + ", ".join(details) + ".")
            return {
                joint_name: float(joint_positions[joint_name])
                for joint_name in ARM_JOINT_NAMES
            }

        if isinstance(joint_positions, Sequence) and not isinstance(
            joint_positions,
            (str, bytes),
        ):
            if len(joint_positions) != len(ARM_JOINT_NAMES):
                raise ValueError(
                    f"Expected {len(ARM_JOINT_NAMES)} joint values in UR5e order "
                    f"{ARM_JOINT_NAMES}, got {len(joint_positions)}."
                )
            return {
                joint_name: float(value)
                for joint_name, value in zip(ARM_JOINT_NAMES, joint_positions)
            }

        raise TypeError(
            "joint_positions must be either a mapping of joint names to positions or "
            "a 6-element list/tuple in UR5e joint order."
        )

    def _build_joint_motion_plan_request(
        self,
        *,
        joint_positions: Mapping[str, float],
        planning_group: str,
        allowed_planning_time: float,
        num_planning_attempts: int,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
        joint_tolerance: float,
    ) -> MotionPlanRequest:
        request = self._base_motion_plan_request(
            planning_group=planning_group,
            allowed_planning_time=allowed_planning_time,
            num_planning_attempts=num_planning_attempts,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
        )

        constraints = Constraints()
        constraints.name = "joint_goal"
        constraints.joint_constraints = []
        for joint_name in ARM_JOINT_NAMES:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_positions[joint_name])
            joint_constraint.tolerance_above = joint_tolerance
            joint_constraint.tolerance_below = joint_tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        request.goal_constraints = [constraints]
        return request


    def _compute_ik_joint_positions(
        self,
        *,
        pose_stamped: PoseStamped,
        planning_group: str,
        tcp_frame: str,
        timeout_sec: float,
        avoid_collisions: bool,
    ) -> dict[str, float]:
        current_joint_state = self._wait_for_joint_state(
            timeout_sec=_INITIAL_JOINT_STATE_TIMEOUT
        )
        request = GetPositionIK.Request()
        request.ik_request.group_name = planning_group
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = current_joint_state
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = tcp_frame
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout = self._duration_from_seconds(timeout_sec)

        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(
                f"MoveIt IK service {self._ik_service_name} is not available within 2.0 seconds."
            )

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=max(timeout_sec, 2.0),
        )
        if not future.done():
            raise TimeoutError(
                f"Timed out waiting for MoveIt IK response from {self._ik_service_name}."
            )

        exception = future.exception()
        if exception is not None:
            raise RuntimeError(
                f"MoveIt IK request failed: {exception}"
            ) from exception

        response = future.result()
        if response is None:
            raise RuntimeError(
                f"MoveIt IK service {self._ik_service_name} returned no response."
            )

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(self._format_moveit_error(response.error_code))

        solution_map = {
            name: response.solution.joint_state.position[index]
            for index, name in enumerate(response.solution.joint_state.name)
            if index < len(response.solution.joint_state.position)
        }
        missing = [name for name in ARM_JOINT_NAMES if name not in solution_map]
        if missing:
            raise RuntimeError(
                "MoveIt IK response did not include all UR5e arm joints: "
                f"missing {missing}."
            )
        return {joint_name: float(solution_map[joint_name]) for joint_name in ARM_JOINT_NAMES}


    def _base_motion_plan_request(
        self,
        *,
        planning_group: str,
        allowed_planning_time: float,
        num_planning_attempts: int,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
    ) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = planning_group
        request.num_planning_attempts = max(num_planning_attempts, 1)
        request.allowed_planning_time = max(allowed_planning_time, 0.1)
        request.max_velocity_scaling_factor = self._clamp_scaling_factor(
            max_velocity_scaling_factor
        )
        request.max_acceleration_scaling_factor = self._clamp_scaling_factor(
            max_acceleration_scaling_factor
        )
        return request

    def _build_planning_options(self, kwargs: Mapping[str, Any]) -> PlanningOptions:
        options = PlanningOptions()
        options.plan_only = bool(kwargs.get("plan_only", False))
        options.look_around = bool(kwargs.get("look_around", False))
        options.look_around_attempts = int(kwargs.get("look_around_attempts", 0))
        options.max_safe_execution_cost = float(kwargs.get("max_safe_execution_cost", 0.0))
        options.replan = bool(kwargs.get("replan", False))
        options.replan_attempts = int(kwargs.get("replan_attempts", 0))
        options.replan_delay = float(kwargs.get("replan_delay", 0.0))
        return options

    def _execute_move_group_goal(
        self,
        goal: MoveGroup.Goal,
        *,
        timeout_sec: float,
    ) -> Any:
        if not self._move_group_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(
                f"MoveIt action server {self._move_group_action_name} is not available "
                "within 2.0 seconds."
            )

        send_goal_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self._node,
            send_goal_future,
            timeout_sec=timeout_sec,
        )
        if not send_goal_future.done():
            raise TimeoutError(
                f"Timed out sending goal to {self._move_group_action_name}."
            )

        send_goal_exception = send_goal_future.exception()
        if send_goal_exception is not None:
            raise RuntimeError(
                f"Failed to send goal to {self._move_group_action_name}: "
                f"{send_goal_exception}"
            ) from send_goal_exception

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(
                f"MoveIt action server {self._move_group_action_name} rejected the goal."
            )

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node,
            result_future,
            timeout_sec=timeout_sec,
        )
        if not result_future.done():
            raise TimeoutError(
                f"Timed out waiting for result from {self._move_group_action_name}."
            )

        result_exception = result_future.exception()
        if result_exception is not None:
            raise RuntimeError(
                f"MoveIt execution failed: {result_exception}"
            ) from result_exception

        result_response = result_future.result()
        if result_response is None or result_response.result is None:
            raise RuntimeError(
                f"MoveIt action {self._move_group_action_name} returned no result."
            )

        move_group_result = result_response.result
        error_code = move_group_result.error_code
        if error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(self._format_moveit_error(error_code))

        return result_response

    def _coerce_pose_stamped(self, target_pose: PoseLike) -> PoseStamped:
        if isinstance(target_pose, PoseStamped):
            return target_pose

        if not isinstance(target_pose, Mapping):
            raise TypeError(
                "target_pose must be a geometry_msgs.msg.PoseStamped or a dictionary "
                "with frame_id, position, and orientation fields."
            )

        frame_id = str(target_pose.get("frame_id", "")).strip()
        if not frame_id:
            raise ValueError("target_pose dictionary must include a non-empty frame_id.")

        position = target_pose.get("position")
        orientation = target_pose.get("orientation")
        if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
            raise ValueError(
                "target_pose dictionary must include mapping values for position and orientation."
            )

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position.x = float(position["x"])
        pose_stamped.pose.position.y = float(position["y"])
        pose_stamped.pose.position.z = float(position["z"])
        pose_stamped.pose.orientation.x = float(orientation["x"])
        pose_stamped.pose.orientation.y = float(orientation["y"])
        pose_stamped.pose.orientation.z = float(orientation["z"])
        pose_stamped.pose.orientation.w = float(orientation["w"])
        return pose_stamped

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        safe_seconds = max(float(seconds), 0.0)
        whole_seconds = int(safe_seconds)
        nanoseconds = int(round((safe_seconds - whole_seconds) * 1e9))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000

        duration = Duration()
        duration.sec = whole_seconds
        duration.nanosec = nanoseconds
        return duration


    def _pose_stamped_to_dict(self, pose_stamped: PoseStamped) -> dict[str, Any]:
        return {
            "frame_id": pose_stamped.header.frame_id,
            "position": {
                "x": pose_stamped.pose.position.x,
                "y": pose_stamped.pose.position.y,
                "z": pose_stamped.pose.position.z,
            },
            "orientation": {
                "x": pose_stamped.pose.orientation.x,
                "y": pose_stamped.pose.orientation.y,
                "z": pose_stamped.pose.orientation.z,
                "w": pose_stamped.pose.orientation.w,
            },
        }

    def _summarize_move_group_result(self, result_response: Any) -> dict[str, Any]:
        move_group_result = result_response.result
        error_code = move_group_result.error_code
        planned_trajectory = move_group_result.planned_trajectory.joint_trajectory
        executed_trajectory = move_group_result.executed_trajectory.joint_trajectory
        return {
            "status": int(result_response.status),
            "error_code": {
                "val": int(error_code.val),
                "name": _ERROR_CODE_NAMES.get(int(error_code.val), "UNKNOWN"),
                "message": error_code.message,
                "source": error_code.source,
            },
            "planning_time": float(move_group_result.planning_time),
            "trajectory_start_joint_names": list(
                move_group_result.trajectory_start.joint_state.name
            ),
            "planned_trajectory": {
                "joint_names": list(planned_trajectory.joint_names),
                "point_count": len(planned_trajectory.points),
            },
            "executed_trajectory": {
                "joint_names": list(executed_trajectory.joint_names),
                "point_count": len(executed_trajectory.points),
            },
        }

    def _format_moveit_error(self, error_code: MoveItErrorCodes) -> str:
        error_name = _ERROR_CODE_NAMES.get(int(error_code.val), "UNKNOWN")
        message = error_code.message or "no additional message"
        source = error_code.source or "unknown source"
        return (
            f"MoveIt request failed with {error_name} ({error_code.val}): "
            f"{message} [source: {source}]"
        )

    @staticmethod
    def _clamp_scaling_factor(value: float) -> float:
        if value <= 0.0:
            return 1.0
        return min(value, 1.0)
