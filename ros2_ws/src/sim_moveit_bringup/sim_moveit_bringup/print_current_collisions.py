#!/usr/bin/env python3
import sys
from typing import Optional

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CurrentCollisionPrinter(Node):
    def __init__(self) -> None:
        super().__init__("print_current_collisions")
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_received = False
        self._subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
        self._client = self.create_client(GetStateValidity, "/check_state_validity")

    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.name:
            return
        self._latest_joint_state = msg
        self._joint_state_received = True

    def wait_for_joint_state(self, timeout_sec: float = 10.0) -> JointState:
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and not self._joint_state_received:
            if self.get_clock().now().nanoseconds / 1e9 >= deadline:
                raise TimeoutError("Timed out waiting for /joint_states.")
            rclpy.spin_once(self, timeout_sec=0.1)
        assert self._latest_joint_state is not None
        return self._latest_joint_state

    def wait_for_service(self, timeout_sec: float = 10.0) -> None:
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("Timed out waiting for /check_state_validity.")

    def check_current_state(self) -> GetStateValidity.Response:
        joint_state = self.wait_for_joint_state()
        self.wait_for_service()

        robot_state = RobotState()
        robot_state.joint_state = joint_state

        request = GetStateValidity.Request()
        request.robot_state = robot_state
        request.group_name = "ur_manipulator"

        future = self._client.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        response = future.result()
        if response is None:
            raise RuntimeError("/check_state_validity returned no response.")
        return response


def main() -> None:
    rclpy.init()
    node = CurrentCollisionPrinter()
    exit_code = 0
    try:
        response = node.check_current_state()
        print(f"valid: {response.valid}")
        if not response.contacts:
            print("contacts: none")
            return

        print("contacts:")
        for contact in response.contacts:
            depth = getattr(contact, "depth", None)
            if depth is None:
                print(f"- {contact.contact_body_1} <-> {contact.contact_body_2}")
            else:
                print(
                    f"- {contact.contact_body_1} <-> {contact.contact_body_2} depth={depth}"
                )
    except Exception as exc:  # pragma: no cover - diagnostic script
        print(f"error: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
