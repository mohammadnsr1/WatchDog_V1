"""ROS-backed MoveIt motion backend."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any
from typing import Mapping

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from robot_backend.interfaces.motion_backend import MotionBackend
from robot_backend.interfaces.motion_backend import PoseLike
from robot_backend.utils.geometry import PoseDict
from robot_backend.utils.geometry import make_pose
from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center


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

# The cell's goal-precision spec, defined here ONCE so that every entry point
# -- MCP tools, plan_assembly_step.py plans, direct backend calls -- lands on
# the same numbers. motion_mcp_server.py imports these rather than restating
# them; callers may pass tighter values, but nothing should silently widen
# them. 0.1 mm of position, and orientation tighter than the 0.1 deg spec.
DEFAULT_POSITION_TOLERANCE_M = 0.0001
DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.001  # 0.057 deg, inside the 0.1 deg spec
# Joint-space counterpart, used on the solve_ik_first path and by
# move_to_joint_positions. Kept an order of magnitude under the position spec
# in angle terms: at the UR5e's ~0.85 m reach, 1e-4 rad at one joint is ~0.085
# mm at the TCP, so a six-joint worst case stays in the same ballpark as
# DEFAULT_POSITION_TOLERANCE_M. A looser value here (the previous 1e-3) lets a
# joint-space plan be "reached" almost a millimetre off the pose it was
# solved from, which defeats the pose tolerance above.
DEFAULT_JOINT_TOLERANCE_RAD = 0.0001

_DEFAULT_ACTION_TIMEOUT = 30.0
_DEFAULT_IK_TIMEOUT = 5.0
_INITIAL_JOINT_STATE_TIMEOUT = 1.0

_ERROR_CODE_NAMES = {
    getattr(MoveItErrorCodes, name): name
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance(getattr(MoveItErrorCodes, name), int)
}

# Codes where retrying with a DIFFERENT sampling algorithm is a plausible fix:
# the planner searched and came up empty on a problem another algorithm may
# solve, and it failed before any trajectory was sent to the controller.
#
# Deliberately narrow. Excluded on purpose:
#   START_STATE_IN_COLLISION / GOAL_IN_COLLISION -- rejected before (or
#     regardless of) the planner; the geometry is the problem, not the search.
#   CONTROL_FAILED / PREEMPTED -- execution stage: the trajectory already ran
#     in part, so the arm is somewhere unmodelled and a blind replan would
#     stack a second motion on top of an unknown state.
#   MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE / TIMED_OUT -- ambiguous as
#     to whether execution had begun, so they get the same treatment.
#   The invalid-request and NO_IK_SOLUTION codes -- no planner change helps.
_PLANNER_RETRYABLE_ERROR_CODES = frozenset(
    {
        MoveItErrorCodes.PLANNING_FAILED,       # -1
        MoveItErrorCodes.INVALID_MOTION_PLAN,   # -2
        MoveItErrorCodes.FAILURE,               # 99999, OMPL's generic "no path found"
    }
)

# Tried in order after the requested planner fails with a retryable code.
# Mixed by algorithm family on purpose -- a bidirectional tree, a cost-aware
# tree that pushes through narrow passages, then two roadmap/optimizing
# planners. Reseeding the same planner is already covered by
# num_planning_attempts, so a fallback list of RRT variants would add nothing.
# Every name here must exist in ur5e_moveit_config/config/ompl_planning.yaml.
# Deliberately SHORT. Each entry costs a full allowed_planning_time on failure,
# and observed runs showed that when RRTConnect cannot solve one of these, the
# optimizing planners do not either -- they just spend their whole budget. Two
# entries keep a genuinely different algorithm family in reserve without
# turning a hopeless plan into a minute of latency.
DEFAULT_PLANNER_FALLBACKS = (
    "RRTConnectkConfigDefault",
    "BiTRRTkConfigDefault",
)


class MoveGroupPlanningError(RuntimeError):
    """Raised when a MoveGroup goal completes with a non-SUCCESS error code.

    Carries the structured MoveIt error code so callers can branch on it
    (e.g. to detect START_STATE_IN_COLLISION) without string-matching the
    formatted message.
    """

    def __init__(self, error_code_val: int, message: str) -> None:
        super().__init__(message)
        self.error_code_val = error_code_val


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
        # Created lazily: only used to explain a START_STATE_IN_COLLISION.
        self._state_validity_client = None

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
        """Plan and execute a motion to a TCP pose through MoveIt.

        By default, `target_pose` is interpreted as the desired fixed TCP pose.
        If `gripper_width_m` or `target_is_grasp_center=True` is supplied,
        `target_pose` is interpreted as the desired logical grasp-center pose.

        In grasp-center mode, the backend computes the fixed TCP target pose as:

            T_world_tcp = T_world_grasp_center * inverse(T_tcp_grasp_center)

        using the RG2 calibration utilities. MoveIt then receives a pose
        constraint for the fixed `tcp` link and performs IK + collision-aware
        planning internally. The URDF TCP remains fixed.

        `solve_ik_first=True` resolves the target pose to one exact joint
        solution through `/compute_ik` first, then plans in joint space
        against that solution with joint_tolerance (default
        DEFAULT_JOINT_TOLERANCE_RAD). Use
        this when a call needs the end-effector to land at (deterministically
        close to) the literal target instead of anywhere inside the default
        pose-goal's tolerance sphere/cone -- the default Cartesian pose-goal
        path only requires the sampled plan to land inside that region, not
        at its center. Leave disabled for ordinary moves: it is a single IK
        seed (from the current joint state) with no retry across multiple
        seeds, so it can fail on poses the sampling-based path would still
        solve.
        """

        requested_pose_stamped = self._coerce_pose_stamped(target_pose)
        requested_pose_dict = self._coerce_pose_dict(target_pose)

        gripper_width_m = kwargs.get("gripper_width_m")
        target_is_grasp_center = bool(kwargs.get("target_is_grasp_center", False))
        target_semantics = (
            "grasp_center"
            if target_is_grasp_center or gripper_width_m is not None
            else "tcp"
        )

        if target_semantics == "grasp_center":
            if gripper_width_m is None:
                raise ValueError(
                    "gripper_width_m is required when target_is_grasp_center=True "
                    "or when using grasp-center target semantics."
                )
            computed_tcp_target_pose = tcp_target_from_grasp_center(
                requested_pose_dict,
                width_m=float(gripper_width_m),
            )
            tcp_target_pose_stamped = self._coerce_pose_stamped(
                computed_tcp_target_pose
            )
        else:
            computed_tcp_target_pose = None
            tcp_target_pose_stamped = requested_pose_stamped

        if bool(kwargs.get("solve_ik_first", False)):
            ik_joint_positions = self._compute_ik_joint_positions(
                pose_stamped=tcp_target_pose_stamped,
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
                "target_semantics": target_semantics,
                "requested_pose": requested_pose_dict,
                "computed_tcp_target_pose": computed_tcp_target_pose,
                "gripper_width_m": (
                    float(gripper_width_m) if gripper_width_m is not None else None
                ),
                "tcp_frame": tcp_frame,
                "planning_group": planning_group,
                "solve_ik_first": True,
                "ik_solution": ik_joint_positions,
                "response": joint_result["response"],
            }

        # Policy defaults for these tunables live in motion_mcp_server.py, which
        # always passes them explicitly; the literals below are only a
        # last-resort mechanism default for direct backend callers.
        allowed_planning_time = float(kwargs.get("allowed_planning_time", 15.0))
        num_planning_attempts = int(kwargs.get("num_planning_attempts", 30))
        max_velocity_scaling_factor = float(
            kwargs.get("max_velocity_scaling_factor", 1.0)
        )
        max_acceleration_scaling_factor = float(
            kwargs.get("max_acceleration_scaling_factor", 0.3)
        )
        # Goal precision is NOT a per-call tuning knob with a local literal:
        # both defaults are the module-level spec above, which the MCP server
        # imports rather than restating.
        position_tolerance_m = float(
            kwargs.get("position_tolerance_m", DEFAULT_POSITION_TOLERANCE_M)
        )
        orientation_tolerance_rad = float(
            kwargs.get("orientation_tolerance_rad", DEFAULT_ORIENTATION_TOLERANCE_RAD)
        )
        action_timeout_sec = float(kwargs.get("timeout_sec", _DEFAULT_ACTION_TIMEOUT))

        def _build_goal(pose_stamped: PoseStamped, planner_id: str) -> MoveGroup.Goal:
            goal = MoveGroup.Goal()
            goal.request = self._build_pose_motion_plan_request(
                pose_stamped=pose_stamped,
                planning_group=planning_group,
                tcp_frame=tcp_frame,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=num_planning_attempts,
                max_velocity_scaling_factor=max_velocity_scaling_factor,
                max_acceleration_scaling_factor=max_acceleration_scaling_factor,
                position_tolerance_m=position_tolerance_m,
                orientation_tolerance_rad=orientation_tolerance_rad,
                planner_id=planner_id,
            )
            goal.planning_options = self._build_planning_options(kwargs)
            return goal

        result_response = self._run_goal_with_planner_fallbacks(
            build_goal=lambda planner: _build_goal(tcp_target_pose_stamped, planner),
            planner_id=str(kwargs.get("planner_id", "")),
            planner_fallbacks=self._resolve_planner_fallbacks(kwargs),
            timeout_sec=action_timeout_sec,
        )

        return {
            "ok": True,
            "command": "move_to_pose",
            "target_semantics": target_semantics,
            "requested_pose": requested_pose_dict,
            "computed_tcp_target_pose": computed_tcp_target_pose,
            "gripper_width_m": (
                float(gripper_width_m) if gripper_width_m is not None else None
            ),
            "tcp_frame": tcp_frame,
            "planning_group": planning_group,
            "solve_ik_first": False,
            "response": self._summarize_move_group_result(result_response),
        }

    def move_to_joint_positions(
        self,
        joint_positions: Mapping[str, float] | list[float] | tuple[float, ...],
        planning_group: str = DEFAULT_PLANNING_GROUP,
        **kwargs: Any,
    ) -> dict[str, Any]:
        """Plan and execute a motion to explicit joint targets."""

        normalized_joint_positions = self._normalize_joint_positions(joint_positions)
        # Policy defaults for these tunables live in motion_mcp_server.py, which
        # always passes them explicitly; the literals below are only a
        # last-resort mechanism default for direct backend callers.
        allowed_planning_time = float(kwargs.get("allowed_planning_time", 10.0))

        def _build_goal(planner_id: str) -> MoveGroup.Goal:
            goal = MoveGroup.Goal()
            goal.request = self._build_joint_motion_plan_request(
                joint_positions=normalized_joint_positions,
                planning_group=planning_group,
                allowed_planning_time=allowed_planning_time,
                num_planning_attempts=int(kwargs.get("num_planning_attempts", 10)),
                max_velocity_scaling_factor=float(
                    kwargs.get("max_velocity_scaling_factor", 0.5)
                ),
                max_acceleration_scaling_factor=float(
                    kwargs.get("max_acceleration_scaling_factor", 0.5)
                ),
                joint_tolerance=float(
                    kwargs.get("joint_tolerance", DEFAULT_JOINT_TOLERANCE_RAD)
                ),
                planner_id=planner_id,
            )
            goal.planning_options = self._build_planning_options(kwargs)
            return goal

        result_response = self._run_goal_with_planner_fallbacks(
            build_goal=_build_goal,
            planner_id=str(kwargs.get("planner_id", "")),
            planner_fallbacks=self._resolve_planner_fallbacks(kwargs),
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
        planner_id: str = "",
    ) -> MotionPlanRequest:
        request = self._base_motion_plan_request(
            planning_group=planning_group,
            allowed_planning_time=allowed_planning_time,
            num_planning_attempts=num_planning_attempts,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            planner_id=planner_id,
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

    def _build_pose_motion_plan_request(
        self,
        *,
        pose_stamped: PoseStamped,
        planning_group: str,
        tcp_frame: str,
        allowed_planning_time: float,
        num_planning_attempts: int,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        planner_id: str = "",
    ) -> MotionPlanRequest:
        request = self._base_motion_plan_request(
            planning_group=planning_group,
            allowed_planning_time=allowed_planning_time,
            num_planning_attempts=num_planning_attempts,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            planner_id=planner_id,
        )
        request.start_state = self._build_current_robot_state(
            timeout_sec=_INITIAL_JOINT_STATE_TIMEOUT
        )

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [max(position_tolerance_m, 1e-6)]

        region_pose = Pose()
        region_pose.position.x = pose_stamped.pose.position.x
        region_pose.position.y = pose_stamped.pose.position.y
        region_pose.position.z = pose_stamped.pose.position.z
        region_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives = [primitive]
        bounding_volume.primitive_poses = [region_pose]

        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = tcp_frame
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = tcp_frame
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance_rad
        orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "pose_goal"
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
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
        """Resolve one exact IK solution through /compute_ik, seeded from the current joint state."""

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
            raise RuntimeError(f"MoveIt IK request failed: {exception}") from exception

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
        return {
            joint_name: float(solution_map[joint_name]) for joint_name in ARM_JOINT_NAMES
        }

    def _base_motion_plan_request(
        self,
        *,
        planning_group: str,
        allowed_planning_time: float,
        num_planning_attempts: int,
        max_velocity_scaling_factor: float,
        max_acceleration_scaling_factor: float,
        planner_id: str = "",
    ) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = planning_group
        request.planner_id = planner_id
        request.num_planning_attempts = max(num_planning_attempts, 1)
        request.allowed_planning_time = max(allowed_planning_time, 0.1)
        request.max_velocity_scaling_factor = self._clamp_scaling_factor(
            max_velocity_scaling_factor
        )
        request.max_acceleration_scaling_factor = self._clamp_scaling_factor(
            max_acceleration_scaling_factor
        )
        return request

    def _build_current_robot_state(self, timeout_sec: float) -> RobotState:
        joint_state = self._wait_for_joint_state(timeout_sec=timeout_sec)
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        robot_state.is_diff = True
        return robot_state

    def _build_planning_options(self, kwargs: Mapping[str, Any]) -> PlanningOptions:
        options = PlanningOptions()
        options.plan_only = bool(kwargs.get("plan_only", False))
        options.look_around = bool(kwargs.get("look_around", False))
        options.look_around_attempts = int(kwargs.get("look_around_attempts", 0))
        options.max_safe_execution_cost = float(
            kwargs.get("max_safe_execution_cost", 0.0)
        )
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
            raise RuntimeError(f"MoveIt execution failed: {result_exception}") from result_exception

        result_response = result_future.result()
        if result_response is None or result_response.result is None:
            raise RuntimeError(
                f"MoveIt action {self._move_group_action_name} returned no result."
            )

        move_group_result = result_response.result
        error_code = move_group_result.error_code
        if error_code.val != MoveItErrorCodes.SUCCESS:
            message = self._format_moveit_error(error_code)
            if error_code.val == MoveItErrorCodes.START_STATE_IN_COLLISION:
                # This code says only that the start state is invalid, never
                # which bodies touch -- and the goal geometry is irrelevant to
                # it, so the usual reflex of re-checking the target wastes a
                # cycle. Ask MoveIt what is actually in contact right now.
                group_name = getattr(
                    getattr(goal, "request", None), "group_name", ""
                ) or DEFAULT_PLANNING_GROUP
                detail = self._describe_current_state_collisions(group_name)
                if detail:
                    message = f"{message} | current-state contacts: {detail}"
            raise MoveGroupPlanningError(int(error_code.val), message)

        return result_response

    @staticmethod
    def _resolve_planner_fallbacks(kwargs: Mapping[str, Any]) -> tuple[str, ...]:
        """Alternate planners to try after the requested one fails.

        Pass planner_fallbacks=[] to opt out and get single-planner behavior.
        """

        if "planner_fallbacks" not in kwargs:
            return DEFAULT_PLANNER_FALLBACKS
        fallbacks = kwargs.get("planner_fallbacks") or ()
        return tuple(str(name) for name in fallbacks if str(name).strip())

    def _run_goal_with_planner_fallbacks(
        self,
        *,
        build_goal: Any,
        planner_id: str,
        planner_fallbacks: Sequence[str],
        timeout_sec: float,
    ) -> Any:
        """Run a MoveGroup goal, retrying with alternate planners on planning failure.

        Only codes in _PLANNER_RETRYABLE_ERROR_CODES are retried; everything
        else re-raises immediately, because a different sampling algorithm
        cannot address it and retrying would only burn the time budget (or,
        for an execution-stage failure, command a second motion from an
        unmodelled state). The goal is rebuilt per attempt, so each retry
        re-reads the current joint state as its start state.
        """

        attempts: list[str] = [planner_id]
        for candidate in planner_fallbacks:
            if candidate and candidate not in attempts:
                attempts.append(candidate)

        last_error: MoveGroupPlanningError | None = None
        for index, candidate in enumerate(attempts):
            try:
                return self._execute_move_group_goal(
                    build_goal(candidate), timeout_sec=timeout_sec
                )
            except TimeoutError as error:
                # A planner that overruns the action timeout escapes as
                # TimeoutError, not MoveGroupPlanningError, so without this it
                # bypassed the fallback chain entirely and surfaced as what
                # looked like a transport fault. Treat it as this planner
                # failing and move to the next one.
                last_error = MoveGroupPlanningError(
                    int(MoveItErrorCodes.TIMED_OUT), str(error)
                )
                remaining = attempts[index + 1 :]
                if not remaining:
                    break
                self._node.get_logger().warning(
                    f"Planner {candidate or '(group default)'} timed out; "
                    f"retrying with {remaining[0]}."
                )
                continue
            except MoveGroupPlanningError as error:
                if error.error_code_val not in _PLANNER_RETRYABLE_ERROR_CODES:
                    raise
                last_error = error
                remaining = attempts[index + 1 :]
                if not remaining:
                    break
                self._node.get_logger().warning(
                    f"Planner {candidate or '(group default)'} failed with "
                    f"{self._format_moveit_error_code_name(error.error_code_val)}; "
                    f"retrying with {remaining[0]}."
                )

        tried = ", ".join(name or "(group default)" for name in attempts)
        raise MoveGroupPlanningError(
            last_error.error_code_val,
            f"{last_error}. Tried {len(attempts)} planner(s) with no motion "
            f"plan found: {tried}.",
        )

    @staticmethod
    def _format_moveit_error_code_name(error_code_val: int) -> str:
        return _ERROR_CODE_NAMES.get(error_code_val, str(error_code_val))


    _BODY_TYPE_NAMES = {0: "ROBOT_LINK", 1: "WORLD_OBJECT", 2: "ROBOT_ATTACHED"}

    def _describe_current_state_collisions(self, planning_group: str) -> str:
        """Name the bodies currently in collision, for a -10 error message.

        Best effort: any failure here returns "" so it can never mask the
        planning error it is annotating.
        """

        try:
            if self._state_validity_client is None:
                self._state_validity_client = self._node.create_client(
                    GetStateValidity, "/check_state_validity"
                )
            client = self._state_validity_client
            if not client.wait_for_service(timeout_sec=2.0):
                return ""
            request = GetStateValidity.Request()
            request.group_name = planning_group
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            if not future.done():
                return ""
            response = future.result()
            if response is None or not response.contacts:
                return ""
            parts = []
            for contact in response.contacts[:8]:
                t1 = self._BODY_TYPE_NAMES.get(contact.body_type_1, contact.body_type_1)
                t2 = self._BODY_TYPE_NAMES.get(contact.body_type_2, contact.body_type_2)
                parts.append(
                    f"{contact.contact_body_1}[{t1}] <-> {contact.contact_body_2}[{t2}] "
                    f"depth={contact.depth:.4f}m"
                )
            return "; ".join(parts)
        except Exception:
            return ""

    def _coerce_pose_stamped(self, target_pose: PoseLike) -> PoseStamped:
        pose_dict = self._coerce_pose_dict(target_pose)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = str(pose_dict["frame_id"])
        pose_stamped.pose.position.x = float(pose_dict["position"]["x"])
        pose_stamped.pose.position.y = float(pose_dict["position"]["y"])
        pose_stamped.pose.position.z = float(pose_dict["position"]["z"])
        pose_stamped.pose.orientation.x = float(pose_dict["orientation"]["x"])
        pose_stamped.pose.orientation.y = float(pose_dict["orientation"]["y"])
        pose_stamped.pose.orientation.z = float(pose_dict["orientation"]["z"])
        pose_stamped.pose.orientation.w = float(pose_dict["orientation"]["w"])
        return pose_stamped

    def _coerce_pose_dict(self, target_pose: PoseLike) -> PoseDict:
        if isinstance(target_pose, PoseStamped):
            return self._pose_stamped_to_dict(target_pose)

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

        return make_pose(
            position={
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position["z"]),
            },
            orientation={
                "x": float(orientation["x"]),
                "y": float(orientation["y"]),
                "z": float(orientation["z"]),
                "w": float(orientation["w"]),
            },
            frame_id=frame_id,
        )

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

    def _pose_stamped_to_dict(self, pose_stamped: PoseStamped) -> PoseDict:
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
