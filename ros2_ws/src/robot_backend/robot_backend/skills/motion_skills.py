"""Thin motion skills that only depend on the backend contract."""

from __future__ import annotations

from typing import Any
from typing import Mapping

from robot_backend.interfaces.robot_backend import RobotBackend

SAFE_NAMED_JOINT_POSES: dict[str, list[float]] = {
    "ready": [0.0, -1.2, 1.2, -1.57, -1.57, 0.0],
    "upright": [0.0, -1.2, 1.2, 0.0, 0.0, 0.0],
}


def get_robot_state_skill(backend: RobotBackend) -> Any:
    """Return backend motion state and planning metadata."""

    return backend.motion.get_robot_state()


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


def move_to_named_safe_joint_pose_skill(
    backend: RobotBackend,
    name: str,
    **kwargs: Any,
) -> Any:
    """Move to a small whitelist of known-safe named joint poses."""

    normalized_name = name.strip().lower()
    if normalized_name not in SAFE_NAMED_JOINT_POSES:
        allowed_names = ", ".join(sorted(SAFE_NAMED_JOINT_POSES))
        raise ValueError(
            f"Unknown safe joint pose '{name}'. Allowed names: {allowed_names}."
        )

    return backend.motion.move_to_joint_positions(
        SAFE_NAMED_JOINT_POSES[normalized_name],
        **kwargs,
    )


def move_tcp_to_pose_skill(
    backend: RobotBackend,
    target_pose: Any,
    **kwargs: Any,
) -> Any:
    """Move the fixed TCP frame to a target pose via the motion backend only."""

    return backend.motion.move_to_pose(
        target_pose,
        target_is_grasp_center=False,
        **kwargs,
    )


def move_grasp_center_to_pose_skill(
    backend: RobotBackend,
    target_pose: Any,
    gripper_width_m: float,
    **kwargs: Any,
) -> Any:
    """Move the logical grasp center to a pose via the motion backend only."""

    return backend.motion.move_to_pose(
        target_pose,
        target_is_grasp_center=True,
        gripper_width_m=gripper_width_m,
        **kwargs,
    )
