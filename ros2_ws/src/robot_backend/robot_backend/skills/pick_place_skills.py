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

import copy
import math
from typing import Any
from typing import Sequence

from robot_backend.events.decorator import emit_skill_events
from robot_backend.interfaces.robot_backend import RobotBackend
from robot_backend.skills.pose_query_skills import get_object_pose as _get_object_tf_pose
from robot_backend.utils.grasp_offsets import compute_tcp_pose_from_grasp_center
from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center

try:
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import TransformException
except ImportError:
    # Stubs for pure-Python test environments (no ROS shared libraries).
    class Duration:  # type: ignore[no-redef]
        def __init__(self, *, seconds: float = 0.0, nanoseconds: int = 0) -> None:
            pass
    class Time:  # type: ignore[no-redef]
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            pass
    class TransformException(Exception):  # type: ignore[no-redef]
        pass


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


# ---------------------------------------------------------------------------
# Grasp verification helpers
# ---------------------------------------------------------------------------


def _get_grasp_spec(
    registry: dict[str, Any] | None,
    object_id: str,
) -> dict[str, Any] | None:
    """Return the 'spec' block for object_id from the registry, or None."""
    if not registry:
        return None
    entry = registry.get("objects", {}).get(object_id)
    if not isinstance(entry, dict):
        return None
    return entry.get("spec")


def _lookup_frame_pose(
    tf_buffer: Any,
    frame: str,
    lookup_frame: str = "world",
    *,
    timeout_sec: float = 0.5,
) -> dict[str, Any] | None:
    """Return the pose of `frame` in `lookup_frame` coordinates, or None on error."""
    try:
        transform = tf_buffer.lookup_transform(
            lookup_frame,
            frame,
            Time(),
            timeout=Duration(seconds=timeout_sec),
        )
        t = transform.transform.translation
        r = transform.transform.rotation
        return {
            "position": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
            "orientation": {"x": float(r.x), "y": float(r.y), "z": float(r.z), "w": float(r.w)},
        }
    except Exception:
        return None


def _make_verification_lift_pose(
    grasp_pose: dict[str, Any],
    lift_m: float,
) -> dict[str, Any]:
    """Return a deep copy of grasp_pose with the Z position raised by lift_m.

    Assumes the pose is expressed in a world-aligned frame where +Z is up
    (standard table-top manipulation with "world" frame_id).
    """
    lifted = copy.deepcopy(grasp_pose)
    lifted["position"]["z"] += lift_m
    return lifted


def _compute_distance_3d(
    pose1: dict[str, Any],
    pose2: dict[str, Any],
) -> float:
    """Euclidean distance between the position fields of two pose dicts."""
    p1 = pose1["position"]
    p2 = pose2["position"]
    return math.sqrt(
        (p1["x"] - p2["x"]) ** 2
        + (p1["y"] - p2["y"]) ** 2
        + (p1["z"] - p2["z"]) ** 2
    )


def _verify_grasp_by_pose(
    *,
    tf_buffer: Any,
    registry: dict[str, Any] | None,
    object_id: str,
    reference_pose: dict[str, Any],
    tcp_frame: str = "tcp",
    grasp_width_m: float | None = None,
    tf_timeout_sec: float = 0.5,
) -> dict[str, Any]:
    """Verify that the object followed the gripper after a lift.

    Primary check  — Z displacement from resting pose ≥ spec.tolerances.lift_height_min_m.
    Secondary check — 3-D object-to-TCP distance ≤ grasp_width_m + spec.tolerances.position_m.

    Returns dict with: ok, reason, measured_displacement_m, required_displacement_m,
    object_pose_after_lift, tcp_pose_after_lift, tcp_offset_3d_m, tcp_proximity_bound_m.
    """
    spec = _get_grasp_spec(registry, object_id)
    tolerances = spec.get("tolerances", {}) if spec else {}
    lift_height_min_m = float(tolerances.get("lift_height_min_m", 0.01))
    position_m = float(tolerances.get("position_m", 0.05))

    effective_grasp_width = grasp_width_m if grasp_width_m is not None else 0.0
    tcp_proximity_bound_m = effective_grasp_width + position_m

    # Current object pose (post-lift)
    obj_result = _get_object_tf_pose(
        tf_buffer, registry or {}, object_id, tf_timeout_sec=tf_timeout_sec
    )
    if not obj_result["ok"]:
        return {
            "ok": False,
            "reason": (
                f"Cannot verify: TF lookup failed for {object_id!r}: "
                f"{obj_result.get('error')}"
            ),
            "measured_displacement_m": None,
            "required_displacement_m": lift_height_min_m,
            "object_pose_after_lift": None,
            "tcp_pose_after_lift": None,
            "tcp_offset_3d_m": None,
            "tcp_proximity_bound_m": tcp_proximity_bound_m,
        }

    object_pose_after = obj_result["pose"]
    displacement_z = (
        object_pose_after["position"]["z"] - reference_pose["position"]["z"]
    )
    displacement_ok = displacement_z >= lift_height_min_m

    # Secondary: object center should be within gripper reach of the TCP
    tcp_pose = _lookup_frame_pose(tf_buffer, tcp_frame, timeout_sec=tf_timeout_sec)
    tcp_offset_3d: float | None = None
    tcp_ok = True
    if tcp_pose is not None:
        tcp_offset_3d = _compute_distance_3d(object_pose_after, tcp_pose)
        tcp_ok = tcp_offset_3d <= tcp_proximity_bound_m

    if not displacement_ok:
        reason = (
            f"Object did not follow gripper: displacement "
            f"{displacement_z:.4f} m < required {lift_height_min_m:.4f} m."
        )
    elif not tcp_ok:
        reason = (
            f"Object too far from TCP: 3D offset "
            f"{tcp_offset_3d:.4f} m > bound {tcp_proximity_bound_m:.4f} m."
        )
    else:
        reason = "Grasp verified: object followed gripper."

    return {
        "ok": displacement_ok and tcp_ok,
        "reason": reason,
        "measured_displacement_m": displacement_z,
        "required_displacement_m": lift_height_min_m,
        "object_pose_after_lift": object_pose_after,
        "tcp_pose_after_lift": tcp_pose,
        "tcp_offset_3d_m": tcp_offset_3d,
        "tcp_proximity_bound_m": tcp_proximity_bound_m,
    }


# ---------------------------------------------------------------------------
# Skills
# ---------------------------------------------------------------------------


@emit_skill_events
def pick_object(
    backend: RobotBackend,
    *,
    object_id: str,
    object_pose: Any | None = None,
    object_shape: Any | None = None,
    grasp_center_pose: Any | None = None,
    grasp_width_m: float | None = None,
    grasp_width_source: str = "agent_estimate",
    tcp_offset: Any | None = None,
    pregrasp_pose: Any | None = None,
    grasp_pose: Any | None = None,
    gripper_width_open_m: float | None = None,
    attach_link_name: str = "tcp",
    touch_links: Sequence[str] | None = None,
    retreat_pose: Any | None = None,
    # Grasp verification parameters:
    tf_buffer: Any | None = None,
    registry: dict[str, Any] | None = None,
    tcp_frame: str = "tcp",
    verification_lift_m: float = 0.05,
) -> dict[str, Any]:
    """Run the full pick sequence: scene setup → approach → open → grasp → attach → verify → retreat.

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

    Grasp verification (when tf_buffer and registry are both provided):
    The object's resting TF pose is captured before the approach. After the
    gripper closes the object is attached and the robot lifts to the retreat
    position (or a minimal `verification_lift_m` if no retreat is provided).
    Then the object's current TF pose is compared against the reference:

    - SUCCESS: Z displacement ≥ spec.tolerances.lift_height_min_m  AND
               object-to-TCP distance ≤ grasp_width_m + spec.tolerances.position_m.
               The object stays attached; the skill returns ok=True.
    - FAILURE: The attachment is reversed (detach + scene sync) and the skill
               returns ok=False with grasp_failed=True plus the measurements.

    When tf_buffer is None the skill attaches unconditionally (original behavior).

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

    # --- Capture reference TF pose (before approach) ---
    reference_pose: dict[str, Any] | None = None
    if tf_buffer is not None and registry is not None:
        ref_result = _get_object_tf_pose(tf_buffer, registry, object_id)
        if ref_result["ok"]:
            reference_pose = ref_result["pose"]
            steps.append({"step": "capture_reference_pose", "pose": reference_pose})
        else:
            steps.append({
                "step": "capture_reference_pose_failed",
                "error": ref_result.get("error"),
            })

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
    # Attach before lifting so MoveIt plans the lift without the world collision
    # object obstructing the path. Verification may reverse this attachment on failure.
    resolved_touch_links = list(touch_links) if touch_links is not None else list(RG2_TOUCH_LINKS)
    attach_result = backend.scene.attach_object(
        object_id,
        attach_link_name,
        touch_links=resolved_touch_links,
    )
    steps.append({"step": "attach_object", "result": attach_result})

    # --- Lift and verify ---
    do_verify = tf_buffer is not None and reference_pose is not None
    _lift_used_retreat = False

    if do_verify:
        # Use the provided retreat as the lift if available, otherwise synthesise one.
        if retreat_pose is not None:
            verify_lift_pose = retreat_pose
            _lift_used_retreat = True
        elif effective_grasp_pose is not None:
            verify_lift_pose = _make_verification_lift_pose(effective_grasp_pose, verification_lift_m)
        else:
            verify_lift_pose = None

        if verify_lift_pose is not None:
            result = backend.motion.move_to_pose(verify_lift_pose)
            steps.append({"step": "move_to_verification_lift", "result": result})
        else:
            # No lift achievable; verification cannot be conclusive — skip it.
            do_verify = False
            steps.append({
                "step": "grasp_verification_skipped",
                "reason": "no lift pose available (no retreat_pose and no grasp_pose)",
            })

    else:
        # No verification: replicate original retreat behavior.
        if retreat_pose is not None:
            result = backend.motion.move_to_pose(retreat_pose)
            steps.append({"step": "move_to_retreat", "result": result})

    # --- Grasp verification check ---
    if do_verify:
        verify_result = _verify_grasp_by_pose(
            tf_buffer=tf_buffer,
            registry=registry,
            object_id=object_id,
            reference_pose=reference_pose,
            tcp_frame=tcp_frame,
            grasp_width_m=grasp_width_m,
        )
        steps.append({"step": "verify_grasp", "result": verify_result})

        if not verify_result["ok"]:
            # Grasp failed: undo the attachment and restore the world collision
            # object so the planning scene reflects the actual state.
            detach_result = backend.scene.detach_object(object_id)
            steps.append({"step": "detach_on_verify_failure", "result": detach_result})
            sync_result = backend.scene.sync_collision_objects([object_id])
            steps.append({"step": "sync_on_verify_failure", "result": sync_result})
            return {
                "ok": False,
                "command": "pick_object",
                "object_id": object_id,
                "grasp_failed": True,
                "reason": verify_result["reason"],
                "measured_displacement_m": verify_result["measured_displacement_m"],
                "required_displacement_m": verify_result["required_displacement_m"],
                "tcp_offset_3d_m": verify_result["tcp_offset_3d_m"],
                "object_pose_after_lift": verify_result["object_pose_after_lift"],
                "tcp_pose_after_lift": verify_result["tcp_pose_after_lift"],
                "steps": steps,
            }

        # Verification passed. If we did a minimal lift (not the retreat), move
        # to the actual retreat position now.
        if not _lift_used_retreat and retreat_pose is not None:
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


@emit_skill_events
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
