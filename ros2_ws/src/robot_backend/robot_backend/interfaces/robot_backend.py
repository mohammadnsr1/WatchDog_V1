"""Container that bundles the motion, gripper, and scene backends."""

from __future__ import annotations

from dataclasses import dataclass

from .gripper_backend import GripperBackend
from .motion_backend import MotionBackend
from .scene_backend import SceneBackend


@dataclass(frozen=True)
class RobotBackend:
    """Backend bundle used by skills.

    This class intentionally stays free of ROS-specific logic.
    """

    motion: MotionBackend
    gripper: GripperBackend
    scene: SceneBackend
