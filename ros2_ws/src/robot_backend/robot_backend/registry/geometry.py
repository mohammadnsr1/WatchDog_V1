"""ROS-free collision geometry resolution for registry objects."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any

from robot_backend.utils.geometry import PoseDict, normalize_quaternion

from .model import MeshGeometry, Triangle, Vector3Tuple


def load_mesh_geometry(
    mesh_path: Path,
    scale: Vector3Tuple,
    *,
    object_id: str,
) -> MeshGeometry:
    """Load one triangle mesh, apply component scale, and compute its AABB."""
    try:
        import trimesh

        loaded: Any = trimesh.load_mesh(str(mesh_path), process=False)
        if isinstance(loaded, trimesh.Scene):
            loaded = loaded.to_mesh()
        if not isinstance(loaded, trimesh.Trimesh):
            raise TypeError(f"loader returned {type(loaded).__name__}, not a mesh")

        vertices = tuple(
            tuple(float(vertex[index]) * scale[index] for index in range(3))
            for vertex in loaded.vertices
        )
        faces = tuple(tuple(int(index) for index in face) for face in loaded.faces)
    except Exception as exc:
        raise ValueError(
            f"Object {object_id!r}: failed to load mesh {mesh_path}: {exc}"
        ) from exc

    if not vertices:
        raise ValueError(f"Object {object_id!r}: mesh {mesh_path} has no vertices.")
    if not faces:
        raise ValueError(f"Object {object_id!r}: mesh {mesh_path} has no faces.")
    if any(len(face) != 3 for face in faces):
        raise ValueError(
            f"Object {object_id!r}: mesh {mesh_path} contains non-triangle faces."
        )
    if any(not math.isfinite(component) for vertex in vertices for component in vertex):
        raise ValueError(
            f"Object {object_id!r}: mesh {mesh_path} contains non-finite vertices."
        )

    minimum = tuple(min(vertex[axis] for vertex in vertices) for axis in range(3))
    maximum = tuple(max(vertex[axis] for vertex in vertices) for axis in range(3))
    center = tuple((minimum[axis] + maximum[axis]) / 2.0 for axis in range(3))
    half_extents = tuple(
        (maximum[axis] - minimum[axis]) / 2.0 for axis in range(3)
    )
    return MeshGeometry(
        vertices=vertices,
        faces=faces,
        aabb_center=center,
        aabb_half_extents=half_extents,
        file_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
    )


def resolve_box_aabb(
    dimensions: Vector3Tuple,
    pose_offset: PoseDict,
) -> tuple[dict[str, float], dict[str, float]]:
    """Resolve an offset/oriented box to an object-frame axis-aligned AABB."""
    local_half = tuple(value / 2.0 for value in dimensions)
    return transform_aabb((0.0, 0.0, 0.0), local_half, pose_offset)


def transform_aabb(
    center: Vector3Tuple,
    half_extents: Vector3Tuple,
    pose_offset: PoseDict,
) -> tuple[dict[str, float], dict[str, float]]:
    """Transform a collision-frame AABB into an object-frame AABB."""
    rotation = _rotation_matrix(pose_offset)
    rotated_center = tuple(
        sum(rotation[row][column] * center[column] for column in range(3))
        for row in range(3)
    )
    resolved_half = tuple(
        sum(abs(rotation[row][column]) * half_extents[column] for column in range(3))
        for row in range(3)
    )
    position = pose_offset["position"]
    resolved_center = {
        axis: float(position[axis]) + rotated_center[index]
        for index, axis in enumerate(("x", "y", "z"))
    }
    return (
        resolved_center,
        dict(zip(("x", "y", "z"), resolved_half, strict=True)),
    )


def _rotation_matrix(
    pose_offset: PoseDict,
) -> tuple[tuple[float, float, float], ...]:
    q = normalize_quaternion(dict(pose_offset["orientation"]))
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )
