"""Thin gripper skills that only depend on the backend contract."""

from __future__ import annotations

from typing import Any

from robot_backend.interfaces.robot_backend import RobotBackend


def open_gripper_skill(backend: RobotBackend) -> Any:
    """Open the gripper via the backend only."""

    return backend.gripper.open()


def close_gripper_skill(backend: RobotBackend) -> Any:
    """Close the gripper via the backend only."""

    return backend.gripper.close()


def set_gripper_width_skill(backend: RobotBackend, width_m: float) -> Any:
    """Set the target gripper width in meters via the backend only."""

    return backend.gripper.set_width(width_m)
