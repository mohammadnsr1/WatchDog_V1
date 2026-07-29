#!/usr/bin/env python3
"""One-shot pre-refactor golden capture for registry box collision objects."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from robot_backend.utils.geometry import compose_poses, make_pose
from robot_backend.watchdog.session import sha256_file
from registry_golden_utils import canonical_collision_bytes


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REGISTRY_PATH = PACKAGE_ROOT / "config" / "model_registry.yaml"

FIXED_WORLD_POSE = {
    "frame_id": "world",
    "position": {"x": 0.31, "y": -0.27, "z": 0.42},
    "orientation": {
        "x": 0.10259783520851541,
        "y": -0.20519567041703082,
        "z": 0.3077935056255462,
        "w": 0.9233805168766387,
    },
}


class _FixedNow:
    def to_msg(self) -> Time:
        return Time(sec=123, nanosec=456_789)


class _FixedClock:
    def now(self) -> _FixedNow:
        return _FixedNow()


def _pre_refactor_build(object_id: str, entry: dict) -> CollisionObject:
    """Frozen copy of the pre-refactor box path used for the Step A capture."""
    lookup_frame = str(entry.get("lookup_frame", "world")).strip() or "world"
    collision = entry["collision"]
    offset = collision.get("pose_offset")
    if offset is None:
        pose_offset = make_pose(
            {"x": 0.0, "y": 0.0, "z": 0.0},
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            frame_id=lookup_frame,
        )
    else:
        position = offset.get("position", {})
        orientation = offset.get("orientation", {})
        pose_offset = make_pose(
            {axis: float(position.get(axis, 0.0)) for axis in ("x", "y", "z")},
            {
                "x": float(orientation.get("x", 0.0)),
                "y": float(orientation.get("y", 0.0)),
                "z": float(orientation.get("z", 0.0)),
                "w": float(orientation.get("w", 1.0)),
            },
            frame_id=lookup_frame,
        )
    collision_pose = compose_poses(FIXED_WORLD_POSE, pose_offset)

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in collision["dimensions"]]

    pose = Pose()
    pose.position.x = collision_pose["position"]["x"]
    pose.position.y = collision_pose["position"]["y"]
    pose.position.z = collision_pose["position"]["z"]
    pose.orientation.x = collision_pose["orientation"]["x"]
    pose.orientation.y = collision_pose["orientation"]["y"]
    pose.orientation.z = collision_pose["orientation"]["z"]
    pose.orientation.w = collision_pose["orientation"]["w"]

    message = CollisionObject()
    message.header.frame_id = lookup_frame
    message.header.stamp = _FixedClock().now().to_msg()
    message.id = object_id
    message.operation = CollisionObject.ADD
    message.primitives = [primitive]
    message.primitive_poses = [pose]
    return message


def capture() -> dict:
    raw = yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8")) or {}
    hashes: dict[str, str] = {}

    for registry_key, entry in raw.get("objects", {}).items():
        if not entry.get("enabled", True):
            continue
        object_id = str(entry.get("object_id", registry_key)).strip() or registry_key
        message = _pre_refactor_build(object_id, entry)
        hashes[object_id] = hashlib.sha256(
            canonical_collision_bytes(message)
        ).hexdigest()

    return {
        "fixed_world_pose": FIXED_WORLD_POSE,
        "fixed_stamp": {"sec": 123, "nanosec": 456_789},
        "collision_object_sha256": hashes,
        "registry_file_sha256": sha256_file(REGISTRY_PATH),
    }


if __name__ == "__main__":
    print(json.dumps(capture(), indent=2, sort_keys=True))
