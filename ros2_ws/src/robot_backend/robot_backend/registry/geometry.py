"""ROS-free collision geometry resolution for registry objects."""

from __future__ import annotations

from robot_backend.utils.geometry import PoseDict, normalize_quaternion


def resolve_box_aabb(
    dimensions: tuple[float, float, float],
    pose_offset: PoseDict,
) -> tuple[dict[str, float], dict[str, float]]:
    """Resolve an offset/oriented box to an object-frame axis-aligned AABB."""
    q = normalize_quaternion(dict(pose_offset["orientation"]))
    x, y, z, w = q["x"], q["y"], q["z"], q["w"]
    rotation = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )
    local_half = tuple(value / 2.0 for value in dimensions)
    resolved_half = tuple(
        sum(abs(rotation[row][column]) * local_half[column] for column in range(3))
        for row in range(3)
    )
    position = pose_offset["position"]
    return (
        {axis: float(position[axis]) for axis in ("x", "y", "z")},
        dict(zip(("x", "y", "z"), resolved_half, strict=True)),
    )
