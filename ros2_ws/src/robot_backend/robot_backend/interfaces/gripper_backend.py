"""Abstract gripper backend contract."""

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from typing import Any


class GripperBackend(ABC):
    """Interface for gripper control.

    `width_m` values are expressed in meters.
    """

    @abstractmethod
    def open(self) -> Any:
        """Command the gripper to open."""

    @abstractmethod
    def close(self) -> Any:
        """Command the gripper to close."""

    @abstractmethod
    def set_width(self, width_m: float) -> Any:
        """Command the gripper width in meters."""

    @abstractmethod
    def get_state(self) -> Any:
        """Return the current gripper state in a backend-defined format."""

    @abstractmethod
    def read_joint_state(
        self,
        joint_name: str,
        timeout_sec: float = 1.0,
    ) -> dict[str, Any]:
        """Return effort and position for one named joint from /joint_states.

        Parameters
        ----------
        joint_name:
            Exact URDF joint name to look up (e.g. "finger_joint").
        timeout_sec:
            How long to wait for a message that contains the named joint.

        Returns a dict with keys:
            ok          — True if the joint was found within timeout.
            joint_name  — echoed back.
            effort      — joint effort in backend-defined units.
            position    — joint position in radians.
        On failure: ok=False, error contains the reason.
        """
