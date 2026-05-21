"""Abstract planning scene backend contract."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Sequence


class SceneBackend(ABC):
    """Interface for collision scene and attachment management.

    Conceptual behavior:
    - `add_collision_object` adds a known object into the planning scene
    - `allow_touch` temporarily permits contact between an object and gripper
      links
    - `attach_object` makes the planner treat an object as attached to the
      robot
    - `detach_object` releases the object from the robot
    - `clear_octomap` clears perception-based occupancy when needed
    """

    @abstractmethod
    def add_collision_object(
        self,
        object_id: str,
        pose: Any,
        shape: Any,
        **kwargs: Any,
    ) -> Any:
        """Add or update a collision object in the planning scene."""

    @abstractmethod
    def remove_collision_object(self, object_id: str) -> Any:
        """Remove a collision object from the planning scene."""

    @abstractmethod
    def allow_touch(self, object_id: str, link_names: Sequence[str]) -> Any:
        """Temporarily allow contact between an object and selected links."""

    @abstractmethod
    def attach_object(
        self,
        object_id: str,
        link_name: str,
        touch_links: Sequence[str] | None = None,
    ) -> Any:
        """Attach an object to a robot link."""

    @abstractmethod
    def detach_object(self, object_id: str) -> Any:
        """Detach an attached object from the robot."""

    @abstractmethod
    def clear_octomap(self) -> Any:
        """Clear perception-derived occupancy data when needed."""

    @abstractmethod
    def get_objects(self) -> Any:
        """Return known scene objects in a backend-defined format."""
