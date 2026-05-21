"""Abstract gripper backend contract."""

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from typing import Any


class GripperBackend(ABC):
    """Interface for gripper control."""

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
