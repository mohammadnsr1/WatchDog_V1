"""Manual smoke test for the RG2 gripper backend.

This helper is intentionally not exposed as a console script yet.
Run it manually later in a ROS environment after the relevant workspace is
built and sourced.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import close_gripper_skill
from robot_backend.skills.gripper_skills import open_gripper_skill
from robot_backend.skills.gripper_skills import set_gripper_width_skill


def main() -> None:
    """Create the backend and exercise the gripper skill wrappers."""

    rclpy.init(args=None)
    node = Node("gripper_backend_smoke_test")
    try:
        backend = create_ros_moveit_backend(node=node)

        print("Calling open_gripper_skill()...")
        print(open_gripper_skill(backend))

        print("Calling set_gripper_width_skill(backend, 0.3)...")
        print(set_gripper_width_skill(backend, 0.3))

        print("Calling close_gripper_skill()...")
        print(close_gripper_skill(backend))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
