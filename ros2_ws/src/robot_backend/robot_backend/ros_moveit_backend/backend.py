"""Factory for constructing the ROS + MoveIt backend bundle."""

from __future__ import annotations

from rclpy.node import Node

from robot_backend.interfaces.robot_backend import RobotBackend
from robot_backend.ros_moveit_backend.gripper import ROSGripperBackend
from robot_backend.ros_moveit_backend.motion import ROSMoveItMotionBackend
from robot_backend.ros_moveit_backend.scene import ROSMoveItSceneBackend


def create_ros_moveit_backend(node: Node | None = None) -> RobotBackend:
    """Create a backend bundle that shares one optional ROS node."""

    motion = ROSMoveItMotionBackend(node=node)
    gripper = ROSGripperBackend(node=node)
    scene = ROSMoveItSceneBackend(node=node)
    return RobotBackend(
        motion=motion,
        gripper=gripper,
        scene=scene,
    )
