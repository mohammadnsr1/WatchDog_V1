"""MCP server for scene maintenance and recovery tools.

Exposes three tools:
  reset_object_state   — idempotent single-object recovery to WORLD_TRACKED.
  recover_scene        — reset all objects + clear octomap (nuclear option).
  get_scene_summary    — read-only audit of MoveIt's current scene beliefs.

Process pattern (same as pick_place_mcp_server):
  - Lazy rclpy.init on first tool call.
  - Single shared _NODE + _BACKEND for the process lifetime.
  - atexit-registered shutdown closes the node cleanly.
  - No TF buffer needed: maintenance ops are purely MoveIt scene operations.
  - Event bus configured on startup (ROS publisher + JSONL log).
"""

from __future__ import annotations

import atexit
from pathlib import Path
from typing import Any

import yaml

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.maintenance_mcp_server")

import rclpy
from rclpy.node import Node

from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.maintenance_skills import get_scene_summary as get_scene_summary_skill
from robot_backend.skills.maintenance_skills import recover_scene as recover_scene_skill
from robot_backend.skills.maintenance_skills import reset_object_state as reset_object_state_skill


try:
    from mcp.server.fastmcp import FastMCP
except ImportError as exc:
    FastMCP = None  # type: ignore[assignment]
    _MCP_IMPORT_ERROR = exc
else:
    _MCP_IMPORT_ERROR = None


_DEFAULT_REGISTRY_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "model_registry.yaml"
)

mcp = FastMCP("robot_backend_maintenance") if FastMCP is not None else None
_NODE: Node | None = None
_BACKEND = None
_REGISTRY: dict[str, Any] | None = None


def _load_registry() -> dict[str, Any]:
    """Load model_registry.yaml, trying ament share dir then source-relative path."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        path = _DEFAULT_REGISTRY_PATH
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _ensure_backend():
    global _NODE, _BACKEND, _REGISTRY
    if _BACKEND is not None:
        return _BACKEND
    if not rclpy.ok():
        rclpy.init(args=None)
    _NODE = Node("robot_backend_maintenance_mcp_server")
    _BACKEND = create_ros_moveit_backend(node=_NODE)
    import os
    log_path = os.environ.get(
        "WATCHDOG_EVENT_LOG",
        str(Path.home() / ".watchdog" / "events.jsonl"),
    )
    configure_bus(node=_NODE, log_path=log_path)
    if _REGISTRY is None:
        _REGISTRY = _load_registry()
    return _BACKEND


def _shutdown_backend() -> None:
    global _NODE, _BACKEND, _REGISTRY
    shutdown_bus()
    if _NODE is not None:
        _NODE.destroy_node()
        _NODE = None
    _BACKEND = None
    _REGISTRY = None
    if rclpy.ok():
        rclpy.shutdown()


atexit.register(_shutdown_backend)


def _ok(command: str, **payload: Any) -> dict[str, Any]:
    return {"ok": True, "command": command, **payload}


def _error(command: str, error: Exception | str, **payload: Any) -> dict[str, Any]:
    return {"ok": False, "command": command, "error": str(error), **payload}


def _validate_object_id(object_id: Any) -> str:
    if not isinstance(object_id, str) or not object_id.strip():
        raise ValueError("object_id must be a non-empty string.")
    return object_id.strip()


if mcp is not None:

    @mcp.tool()
    def reset_object_state(object_id: str) -> dict[str, Any]:
        """Reset one object to WORLD_TRACKED state, idempotently.

        Queries MoveIt's current scene to determine whether the object is
        attached to the robot. If it is, the object is detached first.
        Then sync_collision_objects is called to restore the object's
        TF-based pose in the world collision scene.

        Call this after a failed pick (grasp_failed=True in pick_object result)
        or any time the scene state for a single object may be inconsistent.

        Parameters
        ----------
        object_id:
            Registry object ID to reset (e.g. "red_cube").
        """
        command = "reset_object_state"
        try:
            oid = _validate_object_id(object_id)
            backend = _ensure_backend()
            result = reset_object_state_skill(backend, object_id=oid)
            return _ok(command, object_id=oid, result=result)
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    def recover_scene() -> dict[str, Any]:
        """Full scene recovery: reset all tracked objects and clear the octomap.

        Iterates every enabled object in the model registry and calls
        reset_object_state for each — detaching any attached objects and
        syncing them back to their current TF poses. After all objects are
        reset, clears the octomap to flush stale perception voxels.

        Use this after a catastrophic failure (agent crash mid-pick, emergency
        stop, or a series of back-to-back grasp failures) where the scene state
        is completely unknown.
        """
        command = "recover_scene"
        try:
            backend = _ensure_backend()
            result = recover_scene_skill(backend, registry=_REGISTRY)
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def get_scene_summary() -> dict[str, Any]:
        """Return a read-only audit of MoveIt's current planning scene.

        Returns the list of world collision objects (TF-tracked objects in the
        world scene) and attached objects (objects held by the robot), along
        with convenience counts. Does not modify any state.

        Use this to verify the scene is in the expected state before planning,
        to cross-check against WatchDog observations, or to diagnose why a
        subsequent pick or place is failing.
        """
        command = "get_scene_summary"
        try:
            backend = _ensure_backend()
            result = get_scene_summary_skill(backend)
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)


def main() -> None:
    """Run the maintenance MCP server."""
    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run maintenance_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
