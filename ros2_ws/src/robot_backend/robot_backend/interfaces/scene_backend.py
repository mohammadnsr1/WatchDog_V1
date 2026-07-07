"""Abstract planning scene backend contract."""

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from typing import Any
from typing import Sequence


class SceneBackend(ABC):
    """Interface for collision scene and attachment management.

    Conceptual behavior:
    - `add_collision_object` adds a known object into the planning scene
    - `attach_object` makes the planner treat the object as attached to the
      robot
    - `detach_object` releases the object from the robot
    - `clear_octomap` clears perception-derived occupancy when needed
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

    @abstractmethod
    def sync_collision_objects(
        self,
        object_ids: Sequence[str] | None = None,
    ) -> Any:
        """Sync collision objects from TF + model_registry into the planning scene.

        Args:
            object_ids: If provided, sync only these objects. If None or empty,
                        sync all enabled objects in the model registry.
        """
