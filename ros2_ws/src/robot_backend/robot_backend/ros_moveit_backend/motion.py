"""ROS-backed MoveIt motion backend skeleton."""

from __future__ import annotations

from typing import Any
from typing import Mapping

import rclpy
from rclpy.node import Node

from robot_backend.interfaces.motion_backend import MotionBackend
from robot_backend.interfaces.motion_backend import PoseLike


class ROSMoveItMotionBackend(MotionBackend):
    """Motion backend wrapper for MoveIt planning and execution.

    This first pass provides the backend boundary and a reusable class
    structure. The actual MoveIt API bindings should be wired to existing repo
    helpers once those interfaces are confirmed.
    """

    def __init__(self, node: Node | None = None) -> None:
        self._owns_node = node is None
        self._node = node or self._create_node()

    @property
    def node(self) -> Node:
        """Return the shared ROS node used for motion operations."""

        return self._node

    def move_to_pose(
        self,
        target_pose: PoseLike,
        planning_group: str = "ur_manipulator",
        tcp_frame: str = "tcp",
        **kwargs: Any,
    ) -> Any:
        """Plan and execute a motion to a target pose.

        TODO: Wrap the existing MoveIt planning/execution client or helper from
        this repo once its Python API is confirmed.
        """

        raise NotImplementedError(
            "MoveIt pose execution wiring is not implemented yet. "
            "Connect this method to the existing MoveIt client/helper already "
            "used in the workspace."
        )

    def move_to_joint_positions(
        self,
        joint_positions: Mapping[str, float] | list[float] | tuple[float, ...],
        planning_group: str = "ur_manipulator",
        **kwargs: Any,
    ) -> Any:
        """Plan and execute a motion to explicit joint targets.

        TODO: Wrap the existing MoveIt planning/execution client or helper from
        this repo once its Python API is confirmed.
        """

        raise NotImplementedError(
            "MoveIt joint execution wiring is not implemented yet. "
            "Connect this method to the existing MoveIt client/helper already "
            "used in the workspace."
        )

    def get_robot_state(self) -> dict[str, Any]:
        """Return a lightweight state placeholder for now.

        TODO: Replace with a real state query once the existing MoveIt state
        access pattern in the repo is confirmed.
        """

        return {
            "planning_group_default": "ur_manipulator",
            "tcp_frame_default": "tcp",
        }

    def _create_node(self) -> Node:
        """Create an internal helper node if one was not supplied."""

        if not rclpy.ok():
            rclpy.init(args=None)
        return Node("robot_backend_moveit_motion_client")
