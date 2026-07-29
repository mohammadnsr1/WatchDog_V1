"""Deterministic hashing helpers for ROS CDR regression fixtures."""

from __future__ import annotations

from typing import Any

from rclpy.serialization import serialize_message


def canonical_collision_bytes(message: Any) -> bytes:
    """Serialize a box CollisionObject and zero non-semantic CDR padding.

    Fast-CDR leaves alignment padding uninitialized, so hashing its raw output
    directly is nondeterministic across processes. This retains every encoded
    field byte and canonicalizes only alignment gaps for the current one-box,
    no-mesh CollisionObject layout used by the Phase 0.5 golden gate.
    """
    data = bytearray(serialize_message(message))
    cursor = 4

    def align(size: int) -> None:
        nonlocal cursor
        padding = (size - ((cursor - 4) % size)) % size
        if padding:
            data[cursor:cursor + padding] = b"\0" * padding
            cursor += padding

    def uint32() -> int:
        nonlocal cursor
        align(4)
        value = int.from_bytes(data[cursor:cursor + 4], "little")
        cursor += 4
        return value

    def string() -> None:
        nonlocal cursor
        length = uint32()
        cursor += length

    cursor += 8
    string()
    align(8)
    cursor += 7 * 8
    string()
    string()
    string()

    primitive_count = uint32()
    for _ in range(primitive_count):
        cursor += 1
        dimension_count = uint32()
        align(8)
        cursor += dimension_count * 8
        point_count = uint32()
        align(4)
        cursor += point_count * 3 * 4

    primitive_pose_count = uint32()
    align(8)
    cursor += primitive_pose_count * 7 * 8

    for _ in range(6):
        count = uint32()
        if count != 0:
            raise ValueError("Golden canonicalizer only supports empty trailing arrays.")
    cursor += 1
    if cursor != len(data):
        raise ValueError(
            f"Unexpected CollisionObject CDR layout: parsed {cursor} of {len(data)} bytes."
        )
    return bytes(data)
