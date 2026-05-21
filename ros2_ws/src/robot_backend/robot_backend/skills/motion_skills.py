"""Thin motion skills that only depend on the backend contract."""

from __future__ import annotations

from typing import Any
from typing import Mapping

from robot_backend.interfaces.robot_backend import RobotBackend


def move_to_pose_skill(
    backend: RobotBackend,
    target_pose: Any,
    **kwargs: Any,
) -> Any:
    """Move the robot to a target pose via the motion backend only."""

    return backend.motion.move_to_pose(target_pose, **kwargs)


def move_to_joint_positions_skill(
    backend: RobotBackend,
    joint_positions: Mapping[str, float] | list[float] | tuple[float, ...],
    **kwargs: Any,
) -> Any:
    """Move the robot to explicit joint positions via the motion backend only."""

    return backend.motion.move_to_joint_positions(joint_positions, **kwargs)
