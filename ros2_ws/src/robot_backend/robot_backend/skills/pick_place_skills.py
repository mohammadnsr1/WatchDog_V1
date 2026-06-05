"""Backend-agnostic pick/place skill implementations.

These skills intentionally avoid direct ROS, topic, service, action,
or MoveIt API calls. They only coordinate behavior through the backend
interfaces.

Ownership model:
- Objects start as WORLD_TRACKED (in MoveIt's world scene, pose from TF).
- pick_object transitions an object to ATTACHED (attached to the robot link).
- place_object transitions it back to WORLD_TRACKED.
- sync_collision_objects is called automatically at the start of pick
  (to ensure fresh TF pose before planning) and at the end of place
  (to restore the object to the world scene via TF).
"""

from __future__ import annotations

from typing import Any
from typing import Sequence

from robot_backend.interfaces.robot_backend import RobotBackend
from robot_backend.utils.grasp_offsets import compute_tcp_pose_from_grasp_center
from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center


# All RG2 links that physically contact or surround a grasped object.
# These are passed as touch_links to attach_object so MoveIt does not treat
# gripper-object contact as a collision during planning.
RG2_TOUCH_LINKS: list[str] = [
    "onrobot_rg2_base_link",
    "onrobot_rg2_left_outer_knuckle",
    "onrobot_rg2_left_inner_knuckle",
    "onrobot_rg2_left_inner_finger",
    "onrobot_rg2_right_outer_knuckle",
    "onrobot_rg2_right_inner_knuckle",
    "onrobot_rg2_right_inner_finger",
    "tcp",
]


def pick_object(
    backend: RobotBackend,
    *,
    object_id: str,
    object_pose: Any | None = None,
    object_shape: Any | None = None,
    grasp_center_pose: Any | None = None,
    grasp_width_m: float | None = None,
    tcp_offset: Any | None = None,
    pregrasp_pose: Any | None = None,
    grasp_pose: Any | None = None,
    gripper_width_open_m: float | None = None,
    attach_link_name: str = "tcp",
    touch_links: Sequence[str] | None = None,
    retreat_pose: Any | None = None,
) -> dict[str, Any]:
    """Run the full pick sequence: scene setup → approach → open → grasp → attach → retreat.

    Scene setup:
    - If `object_pose` and `object_shape` are provided the object is added to
      MoveIt explicitly (useful when TF is not available).
    - Otherwise `sync_collision_objects` is called to pull the current TF pose
      from the sync node, ensuring MoveIt has the freshest geometry before
      planning.

    Grasp pose resolution (in order of priority):
    1. `grasp_pose` — explicit TCP target, used as-is.
    2. `grasp_center_pose` + `grasp_width_m` — converted to TCP via RG2 calibration.
    3. `grasp_center_pose` + `tcp_offset` — converted to TCP via explicit offset.

    Touch links default to `RG2_TOUCH_LINKS` when not provided.
    """
    steps: list[dict[str, Any]] = []

    # --- Scene setup ---
    if object_pose is not None and object_shape is not None:
        scene_result = backend.scene.add_collision_object(object_id, object_pose, object_shape)
        steps.append({"step": "add_collision_object", "result": scene_result})
    else:
        scene_result = backend.scene.sync_collision_objects([object_id])
        steps.append({"step": "sync_collision_objects", "result": scene_result})

    # --- Resolve grasp TCP pose ---
    effective_grasp_pose = grasp_pose
    if effective_grasp_pose is None and grasp_center_pose is not None:
        if grasp_width_m is not None:
            effective_grasp_pose = tcp_target_from_grasp_center(
                grasp_center_pose, grasp_width_m
            )
        elif tcp_offset is not None:
            effective_grasp_pose = compute_tcp_pose_from_grasp_center(
                grasp_center_pose, tcp_offset
            )

    # --- Motion + gripper sequence ---
    if pregrasp_pose is not None:
        result = backend.motion.move_to_pose(pregrasp_pose)
        steps.append({"step": "move_to_pregrasp", "result": result})

    if gripper_width_open_m is not None:
        result = backend.gripper.set_width(gripper_width_open_m)
    else:
        result = backend.gripper.open()
    steps.append({"step": "open_gripper", "result": result})

    if effective_grasp_pose is not None:
        result = backend.motion.move_to_pose(effective_grasp_pose)
        steps.append({"step": "move_to_grasp", "result": result})

    result = backend.gripper.close()
    steps.append({"step": "close_gripper", "result": result})

    # --- Attach: transition object from WORLD_TRACKED to ATTACHED ---
    resolved_touch_links = list(touch_links) if touch_links is not None else list(RG2_TOUCH_LINKS)
    attach_result = backend.scene.attach_object(
        object_id,
        attach_link_name,
        touch_links=resolved_touch_links,
    )
    steps.append({"step": "attach_object", "result": attach_result})

    if retreat_pose is not None:
        result = backend.motion.move_to_pose(retreat_pose)
        steps.append({"step": "move_to_retreat", "result": result})

    return {
        "ok": attach_result.get("ok", False),
        "command": "pick_object",
        "object_id": object_id,
        "attach_link_name": attach_link_name,
        "touch_links": resolved_touch_links,
        "steps": steps,
    }


def place_object(
    backend: RobotBackend,
    *,
    object_id: str,
    place_object_pose: Any | None = None,
    object_shape: Any | None = None,
    place_grasp_center_pose: Any | None = None,
    place_width_m: float | None = None,
    tcp_offset: Any | None = None,
    preplace_pose: Any | None = None,
    place_pose: Any | None = None,
    retreat_pose: Any | None = None,
) -> dict[str, Any]:
    """Run the full place sequence: approach → place → open → detach → restore scene → retreat.

    Scene restoration after detach:
    - If `place_object_pose` and `object_shape` are provided the object is
      re-added at the explicit pose (useful when TF will not update).
    - Otherwise `sync_collision_objects` is called so MoveIt gets the new
      world pose from TF (the expected path after physically placing the object).

    Place pose resolution (same priority as pick):
    1. `place_pose` — explicit TCP target.
    2. `place_grasp_center_pose` + `place_width_m` — converted via RG2 calibration.
    3. `place_grasp_center_pose` + `tcp_offset` — converted via explicit offset.
    """
    steps: list[dict[str, Any]] = []

    # --- Resolve place TCP pose ---
    effective_place_pose = place_pose
    if effective_place_pose is None and place_grasp_center_pose is not None:
        if place_width_m is not None:
            effective_place_pose = tcp_target_from_grasp_center(
                place_grasp_center_pose, place_width_m
            )
        elif tcp_offset is not None:
            effective_place_pose = compute_tcp_pose_from_grasp_center(
                place_grasp_center_pose, tcp_offset
            )

    # --- Motion to place position ---
    if preplace_pose is not None:
        result = backend.motion.move_to_pose(preplace_pose)
        steps.append({"step": "move_to_preplace", "result": result})

    if effective_place_pose is not None:
        result = backend.motion.move_to_pose(effective_place_pose)
        steps.append({"step": "move_to_place", "result": result})

    # --- Release: open gripper then detach ---
    result = backend.gripper.open()
    steps.append({"step": "open_gripper", "result": result})

    detach_result = backend.scene.detach_object(object_id)
    steps.append({"step": "detach_object", "result": detach_result})

    # --- Restore object to WORLD_TRACKED state ---
    if place_object_pose is not None and object_shape is not None:
        scene_result = backend.scene.add_collision_object(
            object_id, place_object_pose, object_shape
        )
        steps.append({"step": "add_collision_object", "result": scene_result})
    else:
        scene_result = backend.scene.sync_collision_objects([object_id])
        steps.append({"step": "sync_collision_objects", "result": scene_result})

    if retreat_pose is not None:
        result = backend.motion.move_to_pose(retreat_pose)
        steps.append({"step": "move_to_retreat", "result": result})

    return {
        "ok": detach_result.get("ok", False),
        "command": "place_object",
        "object_id": object_id,
        "steps": steps,
    }


def pick_and_place(
    backend: RobotBackend,
    *,
    pick_kwargs: dict[str, Any],
    place_kwargs: dict[str, Any],
) -> dict[str, Any]:
    """Run pick then place using the backend-only skill implementations."""

    pick_result = pick_object(backend, **pick_kwargs)
    place_result = place_object(backend, **place_kwargs)
    return {
        "ok": pick_result.get("ok", False) and place_result.get("ok", False),
        "command": "pick_and_place",
        "pick": pick_result,
        "place": place_result,
    }
