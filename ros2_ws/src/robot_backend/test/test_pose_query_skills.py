"""Tests for pose_query_skills — no ROS required.

The TF buffer is replaced with a lightweight mock so the tests can run
without a running ROS context. We verify:
  - happy-path single and batch lookup
  - failure paths (missing object, missing tf_frame, TF exception)
  - enabled_only / graspable_only filtering

Run with: pytest test/test_pose_query_skills.py -v
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import pytest

from robot_backend.skills.pose_query_skills import get_object_pose, get_object_poses

# Use whichever TransformException is available: the real tf2_ros one when the
# full ROS environment is loaded, or the stub defined inside pose_query_skills
# when ROS shared libraries are absent. Both are caught by the skill's except clause
# because the skill imports the same class from the same path.
try:
    from tf2_ros import TransformException
except ImportError:
    from robot_backend.skills.pose_query_skills import TransformException  # type: ignore[no-redef,attr-defined]


# ---------------------------------------------------------------------------
# Test fixtures
# ---------------------------------------------------------------------------

def _make_transform(x: float, y: float, z: float) -> MagicMock:
    """Build a mock TransformStamped with a simple translation."""
    t = MagicMock()
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    return t


def _make_buffer(transforms: dict[tuple[str, str], MagicMock] | None = None) -> MagicMock:
    """Return a mock tf2_ros.Buffer.

    transforms: maps (lookup_frame, tf_frame) -> transform mock.
    Missing keys raise TransformException (same class the skill catches).
    """
    buf = MagicMock()

    def _lookup(target_frame: str, source_frame: str, time: Any, timeout: Any = None) -> Any:
        key = (target_frame, source_frame)
        if transforms and key in transforms:
            return transforms[key]
        raise TransformException(f"No transform {source_frame} -> {target_frame}")

    buf.lookup_transform.side_effect = _lookup
    return buf


FULL_REGISTRY: dict[str, Any] = {
    "objects": {
        "red_cube": {
            "enabled": True,
            "object_id": "red_cube",
            "tf_frame": "red_cube",
            "lookup_frame": "world",
            "graspable": True,
        },
        "green_cube": {
            "enabled": True,
            "object_id": "green_cube",
            "tf_frame": "green_cube",
            "lookup_frame": "world",
            "graspable": True,
        },
        "obstacle_box": {
            "enabled": True,
            "object_id": "obstacle_box",
            "tf_frame": "obstacle_box",
            "lookup_frame": "world",
            "graspable": False,
        },
        "disabled_thing": {
            "enabled": False,
            "object_id": "disabled_thing",
            "tf_frame": "disabled_thing",
            "lookup_frame": "world",
            "graspable": True,
        },
        "no_frame_entry": {
            "enabled": True,
            "tf_frame": "",  # intentionally empty
            "lookup_frame": "world",
            "graspable": True,
        },
    }
}

_TRANSFORMS = {
    ("world", "red_cube"): _make_transform(0.3, 0.1, 0.05),
    ("world", "green_cube"): _make_transform(0.4, -0.1, 0.05),
    ("world", "obstacle_box"): _make_transform(0.5, 0.0, 0.05),
}


# ---------------------------------------------------------------------------
# get_object_pose tests
# ---------------------------------------------------------------------------

class TestGetObjectPose:

    def test_happy_path(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_pose(buf, FULL_REGISTRY, "red_cube")
        assert result["ok"] is True
        assert result["object_id"] == "red_cube"
        assert result["frame_id"] == "world"
        assert result["tf_frame"] == "red_cube"
        assert result["pose"]["position"]["x"] == pytest.approx(0.3)
        assert result["pose"]["position"]["y"] == pytest.approx(0.1)
        assert result["pose"]["position"]["z"] == pytest.approx(0.05)
        assert result["pose"]["orientation"]["w"] == pytest.approx(1.0)

    def test_unknown_object_id(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_pose(buf, FULL_REGISTRY, "purple_cube")
        assert result["ok"] is False
        assert "not found in registry" in result["error"]

    def test_empty_tf_frame(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_pose(buf, FULL_REGISTRY, "no_frame_entry")
        assert result["ok"] is False
        assert "missing tf_frame" in result["error"]

    def test_tf_lookup_failure(self) -> None:
        # obstacle_box has no TF available in this buffer variant
        buf = _make_buffer({"world", "red_cube"})  # wrong type — triggers side_effect fallback
        buf = _make_buffer({("world", "red_cube"): _TRANSFORMS[("world", "red_cube")]})
        # green_cube not in buffer
        result = get_object_pose(buf, FULL_REGISTRY, "green_cube")
        assert result["ok"] is False
        assert "TF lookup failed" in result["error"]

    def test_command_field(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_pose(buf, FULL_REGISTRY, "red_cube")
        assert result["command"] == "get_object_pose"

    def test_custom_timeout_passed_through(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        get_object_pose(buf, FULL_REGISTRY, "red_cube", tf_timeout_sec=2.0)
        # lookup_transform(target, source, time, timeout=Duration(...))
        # — 3 positional + "timeout" keyword
        call_args, call_kwargs = buf.lookup_transform.call_args
        assert len(call_args) == 3
        assert "timeout" in call_kwargs


# ---------------------------------------------------------------------------
# get_object_poses tests
# ---------------------------------------------------------------------------

class TestGetObjectPoses:

    def test_all_enabled_returned(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_poses(buf, FULL_REGISTRY)
        assert result["command"] == "get_object_poses"
        object_ids = {p["object_id"] for p in result["poses"]}
        # disabled_thing is skipped, no_frame_entry fails, so not in poses
        assert "disabled_thing" not in object_ids
        assert "red_cube" in object_ids
        assert "green_cube" in object_ids
        assert "obstacle_box" in object_ids

    def test_disabled_objects_excluded(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_poses(buf, FULL_REGISTRY, enabled_only=True)
        object_ids = {p["object_id"] for p in result["poses"]}
        assert "disabled_thing" not in object_ids

    def test_graspable_only_filter(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_poses(buf, FULL_REGISTRY, graspable_only=True)
        object_ids = {p["object_id"] for p in result["poses"]}
        assert "obstacle_box" not in object_ids
        assert "red_cube" in object_ids
        assert "green_cube" in object_ids

    def test_failed_entries_collected(self) -> None:
        # no_frame_entry has empty tf_frame → will appear in failed list
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_poses(buf, FULL_REGISTRY)
        failed_ids = {f["object_id"] for f in result["failed"]}
        assert "no_frame_entry" in failed_ids

    def test_ok_false_when_any_failure(self) -> None:
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_poses(buf, FULL_REGISTRY)
        # no_frame_entry will fail, so ok should be False
        assert result["ok"] is False

    def test_ok_true_when_all_succeed(self) -> None:
        # Registry with only objects that have valid TF frames
        clean_registry: dict[str, Any] = {
            "objects": {
                "red_cube": FULL_REGISTRY["objects"]["red_cube"],
                "green_cube": FULL_REGISTRY["objects"]["green_cube"],
            }
        }
        buf = _make_buffer(_TRANSFORMS)
        result = get_object_poses(buf, clean_registry)
        assert result["ok"] is True
        assert len(result["failed"]) == 0

    def test_empty_registry(self) -> None:
        buf = _make_buffer({})
        result = get_object_poses(buf, {"objects": {}})
        assert result["ok"] is True
        assert result["poses"] == []
        assert result["failed"] == []
