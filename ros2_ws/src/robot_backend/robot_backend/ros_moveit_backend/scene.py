"""ROS-backed MoveIt planning scene backend."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node

from robot_backend.interfaces.scene_backend import SceneBackend


class ROSMoveItSceneBackend(SceneBackend):
    """Planning scene backend wrapper for MoveIt scene operations.

    This skeleton keeps scene manipulation behind a stable Python interface.
    It intentionally does not introduce timestamp workarounds for perception or
    octomap data; input point-cloud stamps and the reset Isaac sim-time base
    should remain the normal timing path.
    """

    def __init__(self, node: Node | None = None) -> None:
        self._owns_node = node is None
        self._node = node or self._create_node()

    @property
    def node(self) -> Node:
        """Return the shared ROS node used for scene operations."""

        return self._node

    def add_collision_object(
        self,
        object_id: str,
        pose: Any,
        shape: Any,
        **kwargs: Any,
    ) -> Any:
        """Add or update a collision object in the planning scene.

        TODO: Connect this to the repo's existing planning scene API pattern.
        """

        raise NotImplementedError(
            "Planning scene object insertion is not implemented yet. "
            "Wire this to the existing MoveIt scene service/topic/API used in "
            "the workspace."
        )

    def remove_collision_object(self, object_id: str) -> Any:
        """Remove a collision object from the planning scene."""

        raise NotImplementedError(
            "Planning scene object removal is not implemented yet."
        )

    def allow_touch(
        self,
        object_id: str,
        link_names: Sequence[str],
    ) -> dict[str, Any]:
        """Temporarily allow contact between an object and selected links.

        TODO: Depending on the repo's existing MoveIt pattern, this may map to
        touch links on attached objects or an allowed collision matrix update.
        """

        return {
            "object_id": object_id,
            "touch_links": list(link_names),
        }

    def attach_object(
        self,
        object_id: str,
        link_name: str,
        touch_links: Sequence[str] | None = None,
    ) -> Any:
        """Attach an object to a robot link."""

        raise NotImplementedError(
            "Planning scene attach operation is not implemented yet."
        )

    def detach_object(self, object_id: str) -> Any:
        """Detach an attached object from the robot."""

        raise NotImplementedError(
            "Planning scene detach operation is not implemented yet."
        )

    def clear_octomap(self) -> Any:
        """Clear perception-derived occupancy data when needed.

        TODO: Connect this to the MoveIt/octomap clearing API already used in
        the workspace without introducing a fake current-time workaround.
        """

        raise NotImplementedError("Octomap clearing is not implemented yet.")

    def get_objects(self) -> dict[str, Any]:
        """Return known objects in a backend-defined format.

        TODO: Replace with a real planning scene query once the existing repo
        pattern is confirmed.
        """

        return {"objects": []}

    def _create_node(self) -> Node:
        """Create an internal helper node if one was not supplied."""

        if not rclpy.ok():
            rclpy.init(args=None)
        return Node("robot_backend_moveit_scene_client")
