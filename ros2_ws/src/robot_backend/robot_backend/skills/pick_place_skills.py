"""Backend-agnostic pick/place skill skeletons.

These skills intentionally avoid direct ROS, topic, service, action, or MoveIt
API calls. They only coordinate behavior through the backend interfaces.
"""

from __future__ import annotations

from typing import Any, Sequence

from robot_backend.interfaces.robot_backend import RobotBackend
from robot_backend.utils.grasp_offsets import compute_tcp_pose_from_grasp_center


def pick_object(
    backend: RobotBackend,
    *,
    object_id: str,
    object_pose: Any | None = None,
    object_shape: Any | None = None,
    grasp_center_pose: Any | None = None,
    tcp_offset: Any | None = None,
    pregrasp_pose: Any | None = None,
    grasp_pose: Any | None = None,
    gripper_width_open_m: float | None = None,
    attach_link_name: str = "tcp",
    touch_links: Sequence[str] | None = None,
    retreat_pose: Any | None = None,
) -> dict[str, Any]:
    """Skeleton pick sequence using backend methods only."""

    if object_pose is not None and object_shape is not None:
        backend.scene.add_collision_object(object_id, object_pose, object_shape)

    effective_grasp_pose = grasp_pose
    if effective_grasp_pose is None and grasp_center_pose is not None and tcp_offset is not None:
        effective_grasp_pose = compute_tcp_pose_from_grasp_center(
            grasp_center_pose,
            tcp_offset,
        )

    if pregrasp_pose is not None:
        backend.motion.move_to_pose(pregrasp_pose)

    if touch_links:
        backend.scene.allow_touch(object_id, touch_links)

    if gripper_width_open_m is not None:
        backend.gripper.set_width(gripper_width_open_m)
    else:
        backend.gripper.open()

    if effective_grasp_pose is not None:
        backend.motion.move_to_pose(effective_grasp_pose)

    backend.gripper.close()
    backend.scene.attach_object(object_id, attach_link_name, touch_links=touch_links)

    if retreat_pose is not None:
        backend.motion.move_to_pose(retreat_pose)

    return {
        "status": "pick_sequence_requested",
        "object_id": object_id,
        "attach_link_name": attach_link_name,
    }


def place_object(
    backend: RobotBackend,
    *,
    object_id: str,
    place_object_pose: Any | None = None,
    object_shape: Any | None = None,
    place_grasp_center_pose: Any | None = None,
    tcp_offset: Any | None = None,
    preplace_pose: Any | None = None,
    place_pose: Any | None = None,
    retreat_pose: Any | None = None,
) -> dict[str, Any]:
    """Skeleton place sequence using backend methods only."""

    effective_place_pose = place_pose
    if effective_place_pose is None and place_grasp_center_pose is not None and tcp_offset is not None:
        effective_place_pose = compute_tcp_pose_from_grasp_center(
            place_grasp_center_pose,
            tcp_offset,
        )

    if preplace_pose is not None:
        backend.motion.move_to_pose(preplace_pose)

    if effective_place_pose is not None:
        backend.motion.move_to_pose(effective_place_pose)

    backend.scene.detach_object(object_id)
    backend.gripper.open()

    if place_object_pose is not None and object_shape is not None:
        backend.scene.add_collision_object(object_id, place_object_pose, object_shape)

    if retreat_pose is not None:
        backend.motion.move_to_pose(retreat_pose)

    return {"status": "place_sequence_requested", "object_id": object_id}


def pick_and_place(
    backend: RobotBackend,
    *,
    pick_kwargs: dict[str, Any],
    place_kwargs: dict[str, Any],
) -> dict[str, Any]:
    """Run pick then place using the backend-only skill skeletons."""

    pick_result = pick_object(backend, **pick_kwargs)
    place_result = place_object(backend, **place_kwargs)
    return {"pick": pick_result, "place": place_result}
