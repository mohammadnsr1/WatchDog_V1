"""Helpers for converting logical grasp-center poses to fixed-TCP poses.

MoveIt uses a fixed tcp frame. Width-dependent grasp geometry should be handled
here by converting a logical grasp-center pose into a fixed tcp target pose.
The URDF tcp should not be dynamically changed based on gripper width.
"""

from __future__ import annotations

from .geometry import PoseDict, compose_poses, invert_pose


def compute_tcp_pose_from_grasp_center(
    grasp_center_pose: PoseDict,
    tcp_offset: PoseDict,
) -> PoseDict:
    """Convert a logical grasp-center pose into a fixed-TCP target pose.

    `tcp_offset` represents the transform from the fixed tcp frame to the
    logical grasp center.
    """

    return compose_poses(grasp_center_pose, invert_pose(tcp_offset))


def compute_grasp_center_from_tcp_pose(
    tcp_pose: PoseDict,
    tcp_offset: PoseDict,
) -> PoseDict:
    """Convert a fixed-TCP pose into a logical grasp-center pose."""

    return compose_poses(tcp_pose, tcp_offset)
