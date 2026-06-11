"""Tests for maintenance/recovery skills — no ROS required.

Covers:
  - reset_object_state:
    - object is WORLD_TRACKED → skips detach, calls sync
    - object is ATTACHED → detaches then syncs
    - normalised ID comparison (mixed case, whitespace)
    - sync failure propagated in ok field
    - steps audit trail contents
  - recover_scene:
    - empty registry → no resets, only clear_octomap
    - one world-tracked object → reset + clear_octomap
    - one attached + one world-tracked → correct detach calls
    - disabled object skipped
    - partial failure (one reset fails) → overall ok=False
    - returns counts and per-object results
  - get_scene_summary:
    - empty scene
    - world objects only
    - attached objects only
    - mixed world + attached
    - always returns octomap_voxel_count=None
    - propagates ok=False from backend

Run with: pytest test/test_maintenance_skills.py -v
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock, call

import pytest

from robot_backend.skills.maintenance_skills import (
    get_scene_summary,
    recover_scene,
    reset_object_state,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_backend(
    world_objects: list[dict[str, Any]] | None = None,
    attached_objects: list[dict[str, Any]] | None = None,
    *,
    get_objects_ok: bool = True,
    sync_ok: bool = True,
    detach_ok: bool = True,
    clear_octomap_ok: bool = True,
) -> MagicMock:
    """Return a mock RobotBackend whose scene returns the supplied objects."""
    backend = MagicMock()
    backend.scene.get_objects.return_value = {
        "ok": get_objects_ok,
        "command": "get_objects",
        "world_collision_objects": world_objects or [],
        "attached_objects": attached_objects or [],
    }
    backend.scene.detach_object.return_value = {"ok": detach_ok}
    backend.scene.sync_collision_objects.return_value = {
        "ok": sync_ok,
        "command": "sync_collision_objects",
        "synced_object_ids": [],
    }
    backend.scene.clear_octomap.return_value = {"ok": clear_octomap_ok}
    return backend


def _world_obj(object_id: str) -> dict[str, Any]:
    return {"id": object_id, "frame_id": "world", "geometry": []}


def _attached_obj(object_id: str, link: str = "tcp") -> dict[str, Any]:
    return {"id": object_id, "link_name": link, "touch_links": [], "geometry": []}


_REGISTRY_TWO_OBJECTS = {
    "objects": {
        "red_cube": {"tf_frame": "red_cube", "lookup_frame": "world", "enabled": True},
        "green_cube": {"tf_frame": "green_cube", "lookup_frame": "world", "enabled": True},
    }
}

_REGISTRY_ONE_DISABLED = {
    "objects": {
        "red_cube": {"tf_frame": "red_cube", "lookup_frame": "world", "enabled": True},
        "green_cube": {"tf_frame": "green_cube", "lookup_frame": "world", "enabled": False},
    }
}


# ---------------------------------------------------------------------------
# reset_object_state
# ---------------------------------------------------------------------------

class TestResetObjectState:

    def test_world_tracked_object_skips_detach(self) -> None:
        backend = _make_backend(world_objects=[_world_obj("red_cube")])
        result = reset_object_state(backend, object_id="red_cube")
        assert result["ok"] is True
        assert result["was_attached"] is False
        backend.scene.detach_object.assert_not_called()

    def test_world_tracked_calls_sync(self) -> None:
        backend = _make_backend(world_objects=[_world_obj("red_cube")])
        reset_object_state(backend, object_id="red_cube")
        backend.scene.sync_collision_objects.assert_called_once_with(["red_cube"])

    def test_attached_object_detaches_then_syncs(self) -> None:
        backend = _make_backend(attached_objects=[_attached_obj("red_cube")])
        result = reset_object_state(backend, object_id="red_cube")
        assert result["was_attached"] is True
        backend.scene.detach_object.assert_called_once_with("red_cube")
        backend.scene.sync_collision_objects.assert_called_once_with(["red_cube"])

    def test_attached_object_detach_before_sync_ordering(self) -> None:
        """detach_object must be called before sync_collision_objects."""
        call_order: list[str] = []
        backend = _make_backend(attached_objects=[_attached_obj("red_cube")])
        backend.scene.detach_object.side_effect = lambda *a, **kw: call_order.append("detach") or {"ok": True}
        backend.scene.sync_collision_objects.side_effect = lambda *a, **kw: call_order.append("sync") or {"ok": True}
        reset_object_state(backend, object_id="red_cube")
        assert call_order == ["detach", "sync"]

    def test_id_comparison_is_case_insensitive(self) -> None:
        # Backend stores IDs lowercase; caller may pass mixed case.
        backend = _make_backend(attached_objects=[_attached_obj("RED_CUBE")])
        result = reset_object_state(backend, object_id="red_cube")
        assert result["was_attached"] is True

    def test_id_comparison_strips_whitespace(self) -> None:
        backend = _make_backend(attached_objects=[_attached_obj("red_cube")])
        result = reset_object_state(backend, object_id="  red_cube  ")
        assert result["was_attached"] is True

    def test_sync_failure_propagates_to_ok(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("red_cube")],
            sync_ok=False,
        )
        result = reset_object_state(backend, object_id="red_cube")
        assert result["ok"] is False

    def test_absent_object_still_calls_sync(self) -> None:
        # Object not in world scene and not attached → sync anyway (TF-based restore).
        backend = _make_backend()
        result = reset_object_state(backend, object_id="red_cube")
        assert result["was_attached"] is False
        backend.scene.detach_object.assert_not_called()
        backend.scene.sync_collision_objects.assert_called_once()

    def test_steps_contain_get_objects_and_sync(self) -> None:
        backend = _make_backend(world_objects=[_world_obj("red_cube")])
        result = reset_object_state(backend, object_id="red_cube")
        step_names = [s["step"] for s in result["steps"]]
        assert "get_objects" in step_names
        assert "sync_collision_objects" in step_names

    def test_steps_contain_detach_when_attached(self) -> None:
        backend = _make_backend(attached_objects=[_attached_obj("red_cube")])
        result = reset_object_state(backend, object_id="red_cube")
        step_names = [s["step"] for s in result["steps"]]
        assert "detach_object" in step_names

    def test_steps_no_detach_step_when_not_attached(self) -> None:
        backend = _make_backend(world_objects=[_world_obj("red_cube")])
        result = reset_object_state(backend, object_id="red_cube")
        step_names = [s["step"] for s in result["steps"]]
        assert "detach_object" not in step_names

    def test_result_contains_object_id(self) -> None:
        backend = _make_backend()
        result = reset_object_state(backend, object_id="red_cube")
        assert result["object_id"] == "red_cube"


# ---------------------------------------------------------------------------
# recover_scene
# ---------------------------------------------------------------------------

class TestRecoverScene:

    def test_empty_registry_skips_resets_calls_clear_octomap(self) -> None:
        backend = _make_backend()
        result = recover_scene(backend, registry={})
        assert result["objects_reset"] == 0
        backend.scene.detach_object.assert_not_called()
        backend.scene.clear_octomap.assert_called_once()

    def test_none_registry_skips_resets_calls_clear_octomap(self) -> None:
        backend = _make_backend()
        result = recover_scene(backend, registry=None)
        assert result["objects_reset"] == 0
        backend.scene.clear_octomap.assert_called_once()

    def test_resets_all_enabled_objects(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("red_cube"), _world_obj("green_cube")]
        )
        result = recover_scene(backend, registry=_REGISTRY_TWO_OBJECTS)
        assert result["objects_reset"] == 2
        assert len(result["reset_results"]) == 2

    def test_disabled_object_skipped(self) -> None:
        backend = _make_backend(world_objects=[_world_obj("red_cube")])
        result = recover_scene(backend, registry=_REGISTRY_ONE_DISABLED)
        # Only red_cube (enabled); green_cube (disabled) skipped.
        assert result["objects_reset"] == 1
        synced_ids = [
            c.args[0][0]
            for c in backend.scene.sync_collision_objects.call_args_list
        ]
        assert "red_cube" in synced_ids
        assert "green_cube" not in synced_ids

    def test_attached_objects_detached_during_recovery(self) -> None:
        backend = _make_backend(
            attached_objects=[_attached_obj("red_cube")]
        )
        recover_scene(backend, registry={"objects": {"red_cube": {"enabled": True}}})
        backend.scene.detach_object.assert_called_once_with("red_cube")

    def test_clear_octomap_always_called_even_with_objects(self) -> None:
        backend = _make_backend(world_objects=[_world_obj("red_cube")])
        recover_scene(backend, registry=_REGISTRY_TWO_OBJECTS)
        backend.scene.clear_octomap.assert_called_once()

    def test_ok_false_when_sync_fails(self) -> None:
        backend = _make_backend(sync_ok=False)
        result = recover_scene(backend, registry=_REGISTRY_TWO_OBJECTS)
        assert result["ok"] is False

    def test_ok_false_when_clear_octomap_fails(self) -> None:
        backend = _make_backend(clear_octomap_ok=False)
        result = recover_scene(backend, registry=_REGISTRY_TWO_OBJECTS)
        assert result["ok"] is False

    def test_ok_true_when_all_succeed(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("red_cube"), _world_obj("green_cube")]
        )
        result = recover_scene(backend, registry=_REGISTRY_TWO_OBJECTS)
        assert result["ok"] is True

    def test_result_contains_clear_octomap_result(self) -> None:
        backend = _make_backend()
        result = recover_scene(backend, registry={})
        assert "clear_octomap_result" in result
        assert result["clear_octomap_result"]["ok"] is True

    def test_reset_results_list_has_one_entry_per_object(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("red_cube"), _world_obj("green_cube")]
        )
        result = recover_scene(backend, registry=_REGISTRY_TWO_OBJECTS)
        assert len(result["reset_results"]) == 2
        commands = [r["command"] for r in result["reset_results"]]
        assert all(c == "reset_object_state" for c in commands)

    def test_steps_include_per_object_and_clear_octomap(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("red_cube"), _world_obj("green_cube")]
        )
        result = recover_scene(backend, registry=_REGISTRY_TWO_OBJECTS)
        step_names = [s["step"] for s in result["steps"]]
        assert "clear_octomap" in step_names
        assert any("reset_object_state" in name for name in step_names)


# ---------------------------------------------------------------------------
# get_scene_summary
# ---------------------------------------------------------------------------

class TestGetSceneSummary:

    def test_empty_scene_returns_zero_counts(self) -> None:
        backend = _make_backend()
        result = get_scene_summary(backend)
        assert result["world_object_count"] == 0
        assert result["attached_object_count"] == 0
        assert result["world_collision_objects"] == []
        assert result["attached_objects"] == []

    def test_world_objects_only(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("red_cube"), _world_obj("green_cube")]
        )
        result = get_scene_summary(backend)
        assert result["world_object_count"] == 2
        assert result["attached_object_count"] == 0

    def test_attached_objects_only(self) -> None:
        backend = _make_backend(attached_objects=[_attached_obj("red_cube")])
        result = get_scene_summary(backend)
        assert result["world_object_count"] == 0
        assert result["attached_object_count"] == 1

    def test_mixed_world_and_attached(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("green_cube")],
            attached_objects=[_attached_obj("red_cube")],
        )
        result = get_scene_summary(backend)
        assert result["world_object_count"] == 1
        assert result["attached_object_count"] == 1

    def test_octomap_voxel_count_always_none(self) -> None:
        backend = _make_backend(
            world_objects=[_world_obj("red_cube")],
            attached_objects=[_attached_obj("green_cube")],
        )
        result = get_scene_summary(backend)
        assert result["octomap_voxel_count"] is None

    def test_ok_false_when_backend_reports_failure(self) -> None:
        backend = _make_backend(get_objects_ok=False)
        result = get_scene_summary(backend)
        assert result["ok"] is False

    def test_ok_true_when_backend_succeeds(self) -> None:
        backend = _make_backend()
        result = get_scene_summary(backend)
        assert result["ok"] is True

    def test_does_not_modify_scene(self) -> None:
        backend = _make_backend(world_objects=[_world_obj("red_cube")])
        get_scene_summary(backend)
        backend.scene.detach_object.assert_not_called()
        backend.scene.sync_collision_objects.assert_not_called()
        backend.scene.clear_octomap.assert_not_called()
        backend.scene.attach_object.assert_not_called()

    def test_world_objects_contents_preserved(self) -> None:
        obj = _world_obj("red_cube")
        backend = _make_backend(world_objects=[obj])
        result = get_scene_summary(backend)
        assert result["world_collision_objects"][0]["id"] == "red_cube"

    def test_attached_objects_contents_preserved(self) -> None:
        obj = _attached_obj("red_cube", link="wrist_3_link")
        backend = _make_backend(attached_objects=[obj])
        result = get_scene_summary(backend)
        assert result["attached_objects"][0]["link_name"] == "wrist_3_link"
