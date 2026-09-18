#!/usr/bin/env python3
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class FollowJointTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("isaac_follow_joint_trajectory_bridge")

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "action_name", "/arm_controller/follow_joint_trajectory"
        )
        # Match the simulator's physics rate (physxScene:timeStepsPerSecond).
        # At 50 Hz into a 60 Hz physics loop the command stream and the solver
        # beat against each other at 10 Hz: periodically a step reuses the
        # previous target and then jumps, which reads as jitter.
        self.declare_parameter("publish_rate_hz", 60.0)
        self.declare_parameter("clock_stall_timeout_sec", 5.0)
        # Per-joint acceptance band for the final trajectory point. This is a
        # gross-failure detector ("the arm never got there"), NOT the precision
        # spec -- a per-joint band cannot express a Cartesian requirement, since
        # joint error reaches the TCP through the Jacobian weighted by moment
        # arm. Do not re-tighten it to a TCP figure; that mistake has been made
        # twice (0.03 -> 1e-4 -> 5e-4) and both times it aborted valid motions.
        #
        # Measured 2026-09-17 on the fmb2 set, sweeping Isaac drive stiffness
        # over 4x on two joints, fitting residual = A/k + B:
        #   wrist_2       A~11.4  floor B ~ 4.1e-4 rad   <-- 82% of a 5e-4 gate
        #   shoulder_pan  A~11.1  floor B ~ 2.2e-4 rad
        # The floor is stiffness-INDEPENDENT: 4x stiffness bought only ~45%,
        # not the 75% that pure tau/k predicts, and wrist_2 cannot reach 5e-4
        # with margin at any gain. Separately, Isaac publishes /joint_states
        # quantized to 1e-4 rad, so a 5e-4 gate is five quanta wide.
        #
        # For scale: wrist_2's residual at 5.5e-4 rad is ~0.05 mm at the TCP,
        # while the registry's own placement tolerance for these parts is
        # spec.tolerances.position_m = 0.02 (20 mm). 2e-3 sits 3.6x above the
        # measured floor and still 10x under the task tolerance. Cartesian
        # precision is enforced by MoveIt's goal constraints instead
        # (DEFAULT_POSITION_TOLERANCE_M / DEFAULT_ORIENTATION_TOLERANCE_RAD).
        self.declare_parameter("goal_tolerance", 0.002)
        self.declare_parameter("goal_time_tolerance_sec", 2.0)
        self.declare_parameter("require_joint_state_before_execute", True)
        # Default False, matching both launch files (trajectory_bridge.launch.py
        # declares "false"; sim_moveit_bringup passes "false"). It defaulted to
        # True here, so running this node bare -- a test, a hand-launch --
        # silently selected the full-articulation path, which is NOT
        # equivalent: see _publish_full_joint_command for why it drops clamp
        # force on the gripper.
        self.declare_parameter("publish_full_joint_command", False)
        self.declare_parameter("log_extra_joints_once", True)

        self.joint_names = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.joint_command_topic = (
            self.get_parameter("joint_command_topic").get_parameter_value().string_value
        )
        self.joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self.action_name = (
            self.get_parameter("action_name").get_parameter_value().string_value
        )
        self.publish_rate_hz = max(
            1.0,
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value,
        )
        self.clock_stall_timeout_sec = max(
            0.5,
            self.get_parameter("clock_stall_timeout_sec")
            .get_parameter_value()
            .double_value,
        )
        self.goal_tolerance = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )
        self.goal_time_tolerance_sec = (
            self.get_parameter("goal_time_tolerance_sec")
            .get_parameter_value()
            .double_value
        )
        self.require_joint_state_before_execute = (
            self.get_parameter("require_joint_state_before_execute")
            .get_parameter_value()
            .bool_value
        )
        self.publish_full_joint_command = (
            self.get_parameter("publish_full_joint_command")
            .get_parameter_value()
            .bool_value
        )
        self.log_extra_joints_once = (
            self.get_parameter("log_extra_joints_once").get_parameter_value().bool_value
        )

        if self.publish_full_joint_command:
            self.get_logger().warn(
                "Bridge configured for FULL joint command mode. Non-arm joints "
                "are echoed from measured /joint_states as commands, which "
                "cancels gripper clamp force during every arm move — see "
                "_publish_full_joint_command. Both launch files ship this off; "
                "only leave it on if Isaac requires full-articulation commands."
            )
        else:
            self.get_logger().info("Bridge configured for arm-only command mode.")

        if len(self.joint_names) != 6:
            raise ValueError("Parameter 'joint_names' must contain exactly 6 joints.")
        if self.goal_tolerance <= 0.0:
            raise ValueError("Parameter 'goal_tolerance' must be greater than zero.")
        if self.goal_time_tolerance_sec < 0.0:
            raise ValueError(
                "Parameter 'goal_time_tolerance_sec' must be zero or greater."
            )

        self._state_lock = threading.Lock()
        self._latest_joint_state: Optional[Tuple[List[float], List[float]]] = None
        self._latest_full_joint_names: Optional[List[str]] = None
        self._latest_full_joint_positions: Optional[List[float]] = None
        self._latest_full_joint_velocities: Optional[List[float]] = None
        self._logged_extra_joints = False
        self._logged_full_joint_state_fallback_warning = False

        self._publisher = self.create_publisher(JointState, self.joint_command_topic, 10)
        self._subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            50,
        )

        action_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=action_callback_group,
        )

        self.get_logger().info(
            f"Bridge ready on action '{self.action_name}', publishing commands to "
            f"'{self.joint_command_topic}' and reading state from "
            f"'{self.joint_state_topic}'."
        )

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()

    def _joint_state_callback(self, msg: JointState) -> None:
        joint_map = {name: idx for idx, name in enumerate(msg.name)}
        full_joint_names = list(msg.name)
        full_joint_positions = list(msg.position)
        full_joint_velocities = list(msg.velocity) if msg.velocity else []
        extra_joints = [name for name in msg.name if name not in self.joint_names]

        with self._state_lock:
            self._latest_full_joint_names = full_joint_names
            self._latest_full_joint_positions = full_joint_positions
            self._latest_full_joint_velocities = full_joint_velocities

            if (
                extra_joints
                and self.log_extra_joints_once
                and not self._logged_extra_joints
            ):
                self.get_logger().info(
                    f"Ignoring/preserving extra joints from {self.joint_state_topic}: "
                    f"{extra_joints}"
                )
                self._logged_extra_joints = True

        if not all(name in joint_map for name in self.joint_names):
            return

        positions: List[float] = []
        velocities: List[float] = []
        for name in self.joint_names:
            idx = joint_map[name]
            positions.append(msg.position[idx] if idx < len(msg.position) else 0.0)
            velocities.append(msg.velocity[idx] if idx < len(msg.velocity) else 0.0)

        with self._state_lock:
            self._latest_joint_state = (positions, velocities)

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        trajectory = goal_request.trajectory
        if not trajectory.points:
            self.get_logger().error("Rejected goal: received an empty trajectory.")
            return GoalResponse.REJECT

        valid, message = self._validate_joint_names(trajectory.joint_names)
        if not valid:
            self.get_logger().error(f"Rejected goal: {message}")
            return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepted trajectory goal with {len(trajectory.points)} points."
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel request received for active trajectory goal.")
        return CancelResponse.ACCEPT

    def _validate_joint_names(self, goal_joint_names: Sequence[str]) -> Tuple[bool, str]:
        if len(goal_joint_names) != len(self.joint_names):
            return (
                False,
                "trajectory joint count does not match the expected 6-joint arm",
            )

        expected_set = set(self.joint_names)
        goal_set = set(goal_joint_names)
        if goal_set != expected_set:
            missing = sorted(expected_set - goal_set)
            unexpected = sorted(goal_set - expected_set)
            details = []
            if missing:
                details.append(f"missing joints: {missing}")
            if unexpected:
                details.append(f"unexpected joints: {unexpected}")
            return False, ", ".join(details)

        return True, ""

    def _execute_callback(
        self, goal_handle
    ) -> FollowJointTrajectory.Result:
        trajectory = goal_handle.request.trajectory
        reordered_points = self._reorder_trajectory_points(
            trajectory.joint_names, trajectory.points
        )

        if not reordered_points:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                "Trajectory became empty after joint reordering.",
            )

        if self.require_joint_state_before_execute:
            if not self._wait_for_joint_state(timeout_sec=1.0):
                goal_handle.abort()
                message = (
                    "No matching /joint_states received before execution; aborting goal."
                )
                self.get_logger().error(message)
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    message,
                )

        final_time_sec = self._point_time_sec(reordered_points[-1])
        timeout_margin = self._goal_time_tolerance_for_goal(goal_handle.request)
        # Execution is paced by the ROS clock, which under use_sim_time is
        # Isaac's /clock. MoveIt profiles time_from_start against that same
        # clock, so interpolating against wall time replays the trajectory at
        # the simulator's real-time factor (measured 0.80 under load: a 6 s
        # plan ran in 4.8 s of sim time, i.e. every velocity and acceleration
        # 1.24x what was planned).
        start_time = self.get_clock().now()
        period_sec = 1.0 / self.publish_rate_hz
        next_deadline_sec = 0.0

        self.get_logger().info(
            f"Executing trajectory with {len(reordered_points)} points over "
            f"{final_time_sec:.3f} s."
        )

        while rclpy.ok():
            elapsed_sec = self._elapsed_sec(start_time)

            if goal_handle.is_cancel_requested:
                self.get_logger().info("Trajectory execution canceled.")
                goal_handle.canceled()
                return self._result(
                    FollowJointTrajectory.Result.SUCCESSFUL,
                    "Trajectory goal canceled.",
                )

            desired = self._interpolate_positions(reordered_points, elapsed_sec)
            desired_velocities = self._interpolate_velocities(reordered_points, elapsed_sec)

            self._publish_joint_command(desired, desired_velocities)
            self._publish_feedback(goal_handle, desired, desired_velocities)

            if elapsed_sec >= final_time_sec:
                actual = self._current_positions()
                if actual is not None and self._within_tolerance(
                    desired=self._final_positions(reordered_points),
                    actual=actual,
                    tolerance=self.goal_tolerance,
                ):
                    self.get_logger().info("Trajectory execution completed successfully.")
                    goal_handle.succeed()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        "Trajectory reached final point within tolerance.",
                    )

                if actual is None and not self.require_joint_state_before_execute:
                    self.get_logger().warn(
                        "No joint state available at completion; treating final command as success."
                    )
                    goal_handle.succeed()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        "Trajectory command stream completed without state feedback.",
                    )

                if elapsed_sec > final_time_sec + timeout_margin:
                    # Report the residual PER JOINT at full precision, not the
                    # actual positions rounded to 3 decimals: at a goal_tolerance
                    # of 1e-4 rad the rounded form cannot distinguish a drive
                    # that missed by a micro-radian from one that missed by a
                    # centimetre, which is exactly what you need to know to tell
                    # "tolerance set below what the drive can hold" apart from
                    # "the arm never got there".
                    actual_text = "unknown"
                    residual_text = "unknown"
                    if actual is not None:
                        goal_positions = self._final_positions(reordered_points)
                        residuals = [
                            abs(goal_value - actual_value)
                            for goal_value, actual_value in zip(goal_positions, actual)
                        ]
                        actual_text = ", ".join(f"{value:.6f}" for value in actual)
                        residual_text = ", ".join(
                            f"{name}={value:.3e}"
                            for name, value in zip(self.joint_names, residuals)
                        )
                        residual_text += " | max=%.3e rad (tolerance %.3e)" % (
                            max(residuals),
                            self.goal_tolerance,
                        )
                    message = (
                        "Timed out waiting for final joint tolerance after %.3f s. "
                        "Per-joint residual: [%s]. Last actual positions: [%s]"
                    ) % (elapsed_sec, residual_text, actual_text)
                    self.get_logger().error(message)
                    goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                        message,
                    )

            # The ROS clock under Isaac only advances one physics step at a
            # time, so a deadline accumulator drifts inside a step and then
            # fires twice in a row to catch up -- two commands stamped in the
            # same step, the first of which the drive never sees. Resync the
            # deadline to the clock instead of bursting; at publish_rate_hz ==
            # the physics rate this makes the loop tick-locked, one command per
            # step, which is the most the simulator can act on.
            next_deadline_sec += period_sec
            elapsed_now = self._elapsed_sec(start_time)
            if elapsed_now > next_deadline_sec:
                next_deadline_sec = elapsed_now + period_sec * 0.5
            if not self._wait_until_elapsed(start_time, next_deadline_sec):
                message = (
                    "Simulation clock stalled for %.1f s of wall time during "
                    "execution (sim time %.3f s of %.3f s). Is the simulator "
                    "paused?"
                ) % (self.clock_stall_timeout_sec, elapsed_sec, final_time_sec)
                self.get_logger().error(message)
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                    message,
                )

        goal_handle.abort()
        return self._result(
            FollowJointTrajectory.Result.INVALID_GOAL,
            "ROS shutdown interrupted trajectory execution.",
        )

    def _reorder_trajectory_points(
        self,
        goal_joint_names: Sequence[str],
        points: Sequence[JointTrajectoryPoint],
    ) -> List[JointTrajectoryPoint]:
        joint_index_map = {name: idx for idx, name in enumerate(goal_joint_names)}
        reordered_points: List[JointTrajectoryPoint] = []

        # MoveIt may send the right six joints in a different order, so remap every
        # point into the canonical Isaac command order before execution begins.
        for point in points:
            reordered = JointTrajectoryPoint()
            reordered.positions = [
                point.positions[joint_index_map[name]] for name in self.joint_names
            ]
            if point.velocities:
                reordered.velocities = [
                    point.velocities[joint_index_map[name]] for name in self.joint_names
                ]
            if point.accelerations:
                reordered.accelerations = [
                    point.accelerations[joint_index_map[name]]
                    for name in self.joint_names
                ]
            if point.effort:
                reordered.effort = [
                    point.effort[joint_index_map[name]] for name in self.joint_names
                ]
            reordered.time_from_start = point.time_from_start
            reordered_points.append(reordered)

        return reordered_points

    def _publish_joint_command(
        self, positions: Sequence[float], velocities: Sequence[float]
    ) -> None:
        if not self.publish_full_joint_command:
            self._publish_arm_only_joint_command(positions, velocities)
            return

        if self._publish_full_joint_command(positions, velocities):
            return

        if not self._logged_full_joint_state_fallback_warning:
            self.get_logger().warn(
                f"No full joint state available from {self.joint_state_topic}; "
                "falling back to arm-only /joint_command publishing."
            )
            self._logged_full_joint_state_fallback_warning = True

        self._publish_arm_only_joint_command(positions, velocities)

    def _publish_arm_only_joint_command(
        self, positions: Sequence[float], velocities: Sequence[float]
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(positions)
        if velocities:
            msg.velocity = list(velocities)
        self._publisher.publish(msg)

    def _publish_full_joint_command(
        self, positions: Sequence[float], velocities: Sequence[float]
    ) -> bool:
        """Publish every joint, overwriting only the arm's six.

        WARNING -- this mode fights the gripper controller, which is why it is
        off in both launch files. The non-arm entries are copied from the
        latest MEASURED /joint_states and published as COMMANDS. While the
        gripper is clamping an object, its commanded angle is deliberately
        past its measured angle (that tracking error IS the clamp force), so
        republishing the measured angle as the target at publish_rate_hz
        cancels the squeeze for the duration of every arm move, and nothing
        restores the RG2 controller's setpoint afterwards. Measured velocities
        get republished as velocity targets the same way.

        Only enable this if Isaac genuinely requires full-articulation
        commands, and then fix the preservation source first: the non-arm
        entries must come from the last COMMANDED setpoint, not from
        /joint_states.
        """
        latest_full_joint_state = self._get_latest_full_joint_state_locked()
        if latest_full_joint_state is None:
            return False

        joint_names, joint_positions, joint_velocities = latest_full_joint_state
        joint_map = {name: idx for idx, name in enumerate(joint_names)}

        for arm_index, joint_name in enumerate(self.joint_names):
            joint_positions[joint_map[joint_name]] = positions[arm_index]

        preserved_velocities: List[float] = []
        if joint_velocities is not None:
            preserved_velocities = list(joint_velocities)
            if velocities:
                for arm_index, joint_name in enumerate(self.joint_names):
                    preserved_velocities[joint_map[joint_name]] = velocities[arm_index]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = joint_positions
        if preserved_velocities:
            msg.velocity = preserved_velocities
        self._publisher.publish(msg)
        return True

    def _publish_feedback(
        self,
        goal_handle,
        desired_positions: Sequence[float],
        desired_velocities: Sequence[float],
    ) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(self.joint_names)

        desired = JointTrajectoryPoint()
        desired.positions = list(desired_positions)
        if desired_velocities:
            desired.velocities = list(desired_velocities)
        feedback.desired = desired

        actual_positions, actual_velocities = self._current_state_or_default(
            desired_positions
        )
        actual = JointTrajectoryPoint()
        actual.positions = actual_positions
        actual.velocities = actual_velocities
        feedback.actual = actual

        # Feedback error is reported as desired minus actual for each joint so
        # MoveIt can reason about tracking quality in the usual controller format.
        error = JointTrajectoryPoint()
        error.positions = [
            desired_value - actual_value
            for desired_value, actual_value in zip(desired_positions, actual_positions)
        ]
        if desired_velocities and actual_velocities:
            error.velocities = [
                desired_value - actual_value
                for desired_value, actual_value in zip(
                    desired_velocities, actual_velocities
                )
            ]
        feedback.error = error

        goal_handle.publish_feedback(feedback)

    def _current_positions(self) -> Optional[List[float]]:
        with self._state_lock:
            if self._latest_joint_state is None:
                return None
            return list(self._latest_joint_state[0])

    def _current_state_or_default(
        self, default_positions: Sequence[float]
    ) -> Tuple[List[float], List[float]]:
        with self._state_lock:
            if self._latest_joint_state is None:
                return list(default_positions), [0.0] * len(default_positions)
            positions, velocities = self._latest_joint_state
            return list(positions), list(velocities)

    def _get_latest_full_joint_state_locked(
        self,
    ) -> Optional[Tuple[List[str], List[float], Optional[List[float]]]]:
        with self._state_lock:
            if (
                self._latest_full_joint_names is None
                or self._latest_full_joint_positions is None
            ):
                return None

            joint_names = list(self._latest_full_joint_names)
            joint_positions = list(self._latest_full_joint_positions)
            if len(joint_positions) != len(joint_names):
                return None
            if not all(name in joint_names for name in self.joint_names):
                return None

            joint_velocities: Optional[List[float]] = None
            if (
                self._latest_full_joint_velocities is not None
                and len(self._latest_full_joint_velocities) == len(joint_names)
            ):
                joint_velocities = list(self._latest_full_joint_velocities)

            return joint_names, joint_positions, joint_velocities

    def _wait_for_joint_state(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            with self._state_lock:
                if self._latest_joint_state is not None:
                    return True
            time.sleep(0.02)
        return False

    def _interpolate_positions(
        self, points: Sequence[JointTrajectoryPoint], elapsed_sec: float
    ) -> List[float]:
        if elapsed_sec <= self._point_time_sec(points[0]):
            return list(points[0].positions)

        for previous_point, next_point in zip(points[:-1], points[1:]):
            previous_time = self._point_time_sec(previous_point)
            next_time = self._point_time_sec(next_point)
            if elapsed_sec <= next_time:
                # Execution is time-based: interpolate between the surrounding
                # trajectory timestamps so Isaac sees a smooth stream of targets.
                span = max(next_time - previous_time, 1e-9)
                ratio = (elapsed_sec - previous_time) / span
                return [
                    previous + ratio * (next_value - previous)
                    for previous, next_value in zip(
                        previous_point.positions, next_point.positions
                    )
                ]

        return list(points[-1].positions)

    def _interpolate_velocities(
        self, points: Sequence[JointTrajectoryPoint], elapsed_sec: float
    ) -> List[float]:
        available = all(point.velocities for point in points)
        if not available:
            return []

        if elapsed_sec <= self._point_time_sec(points[0]):
            return list(points[0].velocities)

        for previous_point, next_point in zip(points[:-1], points[1:]):
            previous_time = self._point_time_sec(previous_point)
            next_time = self._point_time_sec(next_point)
            if elapsed_sec <= next_time:
                span = max(next_time - previous_time, 1e-9)
                ratio = (elapsed_sec - previous_time) / span
                return [
                    previous + ratio * (next_value - previous)
                    for previous, next_value in zip(
                        previous_point.velocities, next_point.velocities
                    )
                ]

        return list(points[-1].velocities)

    def _within_tolerance(
        self, desired: Sequence[float], actual: Sequence[float], tolerance: float
    ) -> bool:
        return all(
            abs(desired_value - actual_value) <= tolerance
            for desired_value, actual_value in zip(desired, actual)
        )

    def _final_positions(self, points: Sequence[JointTrajectoryPoint]) -> List[float]:
        return list(points[-1].positions)

    def _goal_time_tolerance_for_goal(
        self, goal: FollowJointTrajectory.Goal
    ) -> float:
        requested = self._duration_to_sec(goal.goal_time_tolerance)
        return requested if requested > 0.0 else self.goal_time_tolerance_sec

    def _point_time_sec(self, point: JointTrajectoryPoint) -> float:
        return self._duration_to_sec(point.time_from_start)

    @staticmethod
    def _duration_to_sec(duration_msg) -> float:
        return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9

    def _elapsed_sec(self, start_time) -> float:
        """Seconds of ROS-clock time since ``start_time`` (sim time when
        use_sim_time is set)."""
        return (self.get_clock().now() - start_time).nanoseconds * 1e-9

    def _wait_until_elapsed(self, start_time, deadline_sec: float) -> bool:
        """Block until the ROS clock reaches ``deadline_sec`` past start.

        Sleeps on wall time in short slices so a paused or stalled simulator
        cannot hang the goal forever. Returns False if the ROS clock failed to
        advance for ``clock_stall_timeout_sec`` of wall time.
        """
        last_elapsed = self._elapsed_sec(start_time)
        last_progress_wall = time.monotonic()
        while rclpy.ok():
            elapsed = self._elapsed_sec(start_time)
            if elapsed >= deadline_sec:
                return True
            if elapsed > last_elapsed:
                last_elapsed = elapsed
                last_progress_wall = time.monotonic()
            elif time.monotonic() - last_progress_wall > self.clock_stall_timeout_sec:
                return False
            # Slice well under one physics step so we do not overshoot the
            # deadline, but not so fine that we burn a core spinning.
            time.sleep(min(0.002, max(0.0005, (deadline_sec - elapsed) * 0.5)))
        return True

    @staticmethod
    def _result(error_code: int, error_string: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()

    try:
        node = FollowJointTrajectoryBridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
