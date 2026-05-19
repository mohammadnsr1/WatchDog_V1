#!/usr/bin/env python3
import sys
from collections import Counter
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

        joint_state = node._latest_joint_state
        rg2_joint_names = [
            "onrobot_rg2_finger_1_origin_to_moment_arm_joint",
            "onrobot_rg2_finger_1_origin_to_truss_arm_joint",
            "onrobot_rg2_finger_1_truss_arm_to_finger_tip_joint",
            "onrobot_rg2_finger_2_origin_to_moment_arm_joint",
            "onrobot_rg2_finger_2_origin_to_truss_arm_joint",
            "onrobot_rg2_finger_2_truss_arm_to_finger_tip_joint",
        ]
        print("rg2_joints:")
        if joint_state is None:
            print("- unavailable")
        else:
            joint_map = dict(zip(joint_state.name, joint_state.position))
            for joint_name in rg2_joint_names:
                value = joint_map.get(joint_name)
                if value is None:
                    print(f"- {joint_name}: <missing>")
                else:
                    print(f"- {joint_name}: {value:.6f}")

        if not response.contacts:
            print("contacts: none")
            return

        pair_counts = Counter()
        for contact in response.contacts:
            pair = tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
            pair_counts[pair] += 1

        print("contact_pair_counts:")
        for (body_1, body_2), count in sorted(pair_counts.items()):
            print(f"- {body_1} <-> {body_2}: {count}")

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
