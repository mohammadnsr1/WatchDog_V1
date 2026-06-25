#!/usr/bin/env python3
import math
import threading
import time
from typing import Dict, List, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from isaac_trajectory_bridge_interfaces.srv import SetGripperPosition


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


class RG2Controller(Node):
    def __init__(self) -> None:
        super().__init__("rg2_controller")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("service_namespace", "/rg2_controller")
        self.declare_parameter("master_joint_name", DEFAULT_MASTER_JOINT_NAME)
        self.declare_parameter("master_joint_min_position", DEFAULT_MASTER_JOINT_MIN_POSITION)
        self.declare_parameter("master_joint_max_position", DEFAULT_MASTER_JOINT_MAX_POSITION)
        self.declare_parameter("open_position", DEFAULT_OPEN_POSITION)
        self.declare_parameter("close_position", DEFAULT_CLOSE_POSITION)
        self.declare_parameter("command_duration_sec", 1.0)
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("wait_for_joint_state_timeout_sec", 2.0)
        self.declare_parameter("gripper_joint_names", DEFAULT_GRIPPER_JOINT_NAMES)
        self.declare_parameter("gripper_joint_multipliers", DEFAULT_GRIPPER_JOINT_MULTIPLIERS)
        self.declare_parameter("gripper_joint_offsets", DEFAULT_GRIPPER_JOINT_OFFSETS)

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
        self.command_duration_sec = max(0.0, float(self.get_parameter("command_duration_sec").value))
        self.command_rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.wait_for_joint_state_timeout_sec = max(
            0.0, float(self.get_parameter("wait_for_joint_state_timeout_sec").value)
        )
        self.gripper_joint_names = list(self.get_parameter("gripper_joint_names").value)
        self.gripper_joint_multipliers = [
            float(value) for value in self.get_parameter("gripper_joint_multipliers").value
        ]
        self.gripper_joint_offsets = [
            float(value) for value in self.get_parameter("gripper_joint_offsets").value
        ]

        if len(self.gripper_joint_names) != 6:
            raise ValueError("gripper_joint_names must contain exactly 6 RG2 joints.")
        if len(self.gripper_joint_multipliers) != len(self.gripper_joint_names):
            raise ValueError("gripper_joint_multipliers must match gripper_joint_names length.")
        if len(self.gripper_joint_offsets) != len(self.gripper_joint_names):
            raise ValueError("gripper_joint_offsets must match gripper_joint_names length.")
        if self.master_joint_name not in self.gripper_joint_names:
            raise ValueError("master_joint_name must be present in gripper_joint_names.")
        if self.master_joint_min_position > self.master_joint_max_position:
            raise ValueError("master_joint_min_position must be <= master_joint_max_position.")

        self._state_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._busy = False
        self._latest_gripper_positions: Dict[str, float] = {}

        self._publisher = self.create_publisher(JointState, self.joint_command_topic, 10)
        self._subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            50,
        )

        self._open_service = self.create_service(
            Trigger,
            f"{self.service_namespace}/open",
            self._open_callback,
        )
        self._close_service = self.create_service(
            Trigger,
            f"{self.service_namespace}/close",
            self._close_callback,
        )
        self._set_position_service = self.create_service(
            SetGripperPosition,
            f"{self.service_namespace}/set_position",
            self._set_position_callback,
        )

        self.get_logger().info(
            "RG2 controller ready. Services: "
            f"{self.service_namespace}/open, "
            f"{self.service_namespace}/close, "
            f"{self.service_namespace}/set_position"
        )
        self.get_logger().info(
            "RG2 controller commands gripper joints only: "
            f"{self.gripper_joint_names}"
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        joint_map = {name: idx for idx, name in enumerate(msg.name)}
        positions: Dict[str, float] = {}
        for joint_name in self.gripper_joint_names:
            idx = joint_map.get(joint_name)
            if idx is None or idx >= len(msg.position):
                continue
            positions[joint_name] = msg.position[idx]

        if not positions:
            return

        with self._state_lock:
            self._latest_gripper_positions = positions

    def _open_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._handle_command(self.open_position, "open", response)

    def _close_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._handle_command(self.close_position, "close", response)

    def _set_position_callback(
        self,
        request: SetGripperPosition.Request,
        response: SetGripperPosition.Response,
    ) -> SetGripperPosition.Response:
        return self._handle_command(request.position, "set_position", response)

    def _handle_command(self, requested_master_position: float, label: str, response):
        if not self._begin_command():
            response.success = False
            response.message = "RG2 controller is busy with another command."
            return response

        try:
            clamped_master_position = self._clamp_master_position(requested_master_position)
            target_positions = self._compute_target_positions(clamped_master_position)
            start_positions, has_current_state = self._get_start_positions(target_positions)

            if has_current_state:
                publish_count = self._publish_interpolated_command(start_positions, target_positions)
                state_message = "Interpolated from latest gripper joint state."
            else:
                publish_count = self._publish_immediate_command(target_positions)
                state_message = (
                    "Latest RG2 joint state unavailable or incomplete; published target immediately."
                )

            targets_text = ", ".join(
                f"{name}={position:.6f}"
                for name, position in zip(self.gripper_joint_names, target_positions)
            )
            self.get_logger().info(
                f"RG2 {label} command published on {self.joint_command_topic}: {targets_text}"
            )

            response.success = True
            if clamped_master_position != requested_master_position:
                response.message = (
                    f"Requested master position {requested_master_position:.6f} rad was clamped to "
                    f"{clamped_master_position:.6f} rad. Published {publish_count} gripper-only command "
                    f"messages. {state_message}"
                )
            else:
                response.message = (
                    f"Published {publish_count} gripper-only command messages for {label}. "
                    f"{state_message}"
                )
            return response
        except Exception as exc:
            self.get_logger().error(f"RG2 {label} command failed: {exc}")
            response.success = False
            response.message = f"RG2 {label} command failed: {exc}"
            return response
        finally:
            self._end_command()

    def _begin_command(self) -> bool:
        with self._command_lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def _end_command(self) -> None:
        with self._command_lock:
            self._busy = False

    def _get_start_positions(self, target_positions: Sequence[float]) -> tuple[List[float], bool]:
        deadline = time.monotonic() + self.wait_for_joint_state_timeout_sec
        while time.monotonic() < deadline:
            positions = self._snapshot_gripper_positions()
            if positions is not None:
                return positions, True
            time.sleep(0.05)

        positions = self._snapshot_gripper_positions()
        if positions is not None:
            return positions, True
        return list(target_positions), False

    def _snapshot_gripper_positions(self) -> Optional[List[float]]:
        with self._state_lock:
            if not all(name in self._latest_gripper_positions for name in self.gripper_joint_names):
                return None
            return [self._latest_gripper_positions[name] for name in self.gripper_joint_names]

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

    def _publish_interpolated_command(
        self,
        start_positions: Sequence[float],
        target_positions: Sequence[float],
    ) -> int:
        steps = max(1, int(math.ceil(self.command_duration_sec * self.command_rate_hz)))
        period_sec = 1.0 / self.command_rate_hz

        for step in range(1, steps + 1):
            alpha = step / steps
            interpolated = [
                start + alpha * (target - start)
                for start, target in zip(start_positions, target_positions)
            ]
            self._publish_joint_command(interpolated)
            time.sleep(period_sec)

        return steps

    def _publish_immediate_command(self, target_positions: Sequence[float]) -> int:
        self._publish_joint_command(target_positions)
        return 1

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
