"""Dependency-light geometry helpers for pose composition and inversion."""

from __future__ import annotations

from math import sqrt
from typing import Any


PoseDict = dict[str, Any]
Vector3 = dict[str, float]
Quaternion = dict[str, float]


def make_pose(
    position: Vector3,
    orientation: Quaternion,
    frame_id: str | None = None,
) -> PoseDict:
    """Create a lightweight dictionary pose representation."""

    pose: PoseDict = {
        "position": dict(position),
        "orientation": normalize_quaternion(dict(orientation)),
    }
    if frame_id is not None:
        pose["frame_id"] = frame_id
    return pose


def invert_pose(pose: PoseDict) -> PoseDict:
    """Invert a dictionary pose representation."""

    q = normalize_quaternion(dict(pose["orientation"]))
    q_inv = invert_quaternion(q)
    p = pose["position"]
    rotated = rotate_vector(
        {"x": -p["x"], "y": -p["y"], "z": -p["z"]},
        q_inv,
    )
    return make_pose(rotated, q_inv, frame_id=pose.get("frame_id"))


def compose_poses(base_pose: PoseDict, offset_pose: PoseDict) -> PoseDict:
    """Compose two dictionary poses as `base_pose * offset_pose`."""

    base_q = normalize_quaternion(dict(base_pose["orientation"]))
    offset_q = normalize_quaternion(dict(offset_pose["orientation"]))
    rotated_offset = rotate_vector(offset_pose["position"], base_q)

    base_p = base_pose["position"]
    position = {
        "x": base_p["x"] + rotated_offset["x"],
        "y": base_p["y"] + rotated_offset["y"],
        "z": base_p["z"] + rotated_offset["z"],
    }
    orientation = multiply_quaternions(base_q, offset_q)
    return make_pose(position, orientation, frame_id=base_pose.get("frame_id"))


def grasp_center_from_object_and_offset(
    object_pose: PoseDict, offset_pose: PoseDict
) -> PoseDict:
    """Resolve a grasp_center_offset against an object pose.

    Both halves are OBJECT-RELATIVE, and both are composed through the
    object's pose:

        position    = object.position + R(object.orientation) * offset.position
        orientation = object.orientation (x) offset.orientation

    POSITION names a physical point on the object, so rotating it through the
    object's orientation is what keeps one value naming the same point however
    the object is lying.

    ORIENTATION is the grasp centre's rotation RELATIVE TO the object, so the
    grasp inherits the part's actual roll, pitch and yaw. That is what lets a
    part re-picked after it has settled off-square be gripped square to
    itself, instead of at a fixed world orientation that no longer matches it.

    Read the authored value against the object's RESTING orientation. Where a
    part rests at identity -- which is most of them at the start of a task --
    object-relative and world agree, so {1,0,0,0} is top-down and stays
    readable as "top-down". Where a part rests at some other orientation (one
    standing at a reorient frame, say), the authored value is that rotation
    composed with the part's own, so it will NOT look like a world
    orientation; derive it as
        offset.orientation = inverse(object.orientation) (x) desired_world
    for the pose the part is expected to be in when the step runs.

    Corollary the caller owns: position and orientation are chosen
    independently here, so nothing checks that the named point is reachable
    at the requested orientation. Dry-run the geometry first.

    Single source of truth for this convention -- motion_mcp_server and
    test/plan_assembly_step.py both call this, having previously each
    implemented it (differently, and divergently).
    """

    position_only_offset = make_pose(
        position=offset_pose["position"],
        orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    composed_position = compose_poses(object_pose, position_only_offset)["position"]
    orientation = normalize_quaternion(
        multiply_quaternions(
            normalize_quaternion(dict(object_pose["orientation"])),
            normalize_quaternion(dict(offset_pose["orientation"])),
        )
    )
    return make_pose(
        position=composed_position,
        orientation=orientation,
        frame_id=object_pose.get("frame_id"),
    )


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    """Return a normalized quaternion."""

    norm = sqrt(
        quaternion["x"] ** 2
        + quaternion["y"] ** 2
        + quaternion["z"] ** 2
        + quaternion["w"] ** 2
    )
    if norm == 0.0:
        raise ValueError("Quaternion norm must be non-zero.")
    return {
        "x": quaternion["x"] / norm,
        "y": quaternion["y"] / norm,
        "z": quaternion["z"] / norm,
        "w": quaternion["w"] / norm,
    }


def invert_quaternion(quaternion: Quaternion) -> Quaternion:
    """Return the inverse of a normalized quaternion."""

    q = normalize_quaternion(quaternion)
    return {
        "x": -q["x"],
        "y": -q["y"],
        "z": -q["z"],
        "w": q["w"],
    }


def multiply_quaternions(left: Quaternion, right: Quaternion) -> Quaternion:
    """Multiply two quaternions."""

    return normalize_quaternion(_multiply_quaternions_raw(left, right))


def rotate_vector(vector: Vector3, quaternion: Quaternion) -> Vector3:
    """Rotate a vector by a quaternion."""

    q = normalize_quaternion(quaternion)
    v_quat = {
        "x": vector["x"],
        "y": vector["y"],
        "z": vector["z"],
        "w": 0.0,
    }
    rotated = _multiply_quaternions_raw(
        _multiply_quaternions_raw(q, v_quat),
        invert_quaternion(q),
    )
    return {
        "x": rotated["x"],
        "y": rotated["y"],
        "z": rotated["z"],
    }


def _multiply_quaternions_raw(
    left: dict[str, float],
    right: dict[str, float],
) -> dict[str, float]:
    """Multiply two quaternions without normalization."""

    x1, y1, z1, w1 = left["x"], left["y"], left["z"], left["w"]
    x2, y2, z2, w2 = right["x"], right["y"], right["z"], right["w"]
    return {
        "x": w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        "y": w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        "z": w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        "w": w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    }
