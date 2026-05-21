"""Abstract motion backend contract."""

from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from typing import Any
from typing import Mapping


PoseLike = Any


class MotionBackend(ABC):
    """Interface for arm motion planning and execution.

    `target_pose` should support either:
    - `geometry_msgs.msg.PoseStamped`
    - a documented dictionary-like representation with `frame_id`, `position`,
      and `orientation` fields for future non-ROS backends
    """

    @abstractmethod
    def move_to_pose(
        self,
        target_pose: PoseLike,
        planning_group: str = "ur_manipulator",
        tcp_frame: str = "tcp",
        **kwargs: Any,
    ) -> Any:
        """Plan and execute a move to a target pose."""

    @abstractmethod
    def move_to_joint_positions(
        self,
        joint_positions: Mapping[str, float] | list[float] | tuple[float, ...],
        planning_group: str = "ur_manipulator",
        **kwargs: Any,
    ) -> Any:
        """Plan and execute a move to explicit joint targets."""

    @abstractmethod
    def get_robot_state(self) -> Any:
        """Return the current robot state in a backend-defined format."""
