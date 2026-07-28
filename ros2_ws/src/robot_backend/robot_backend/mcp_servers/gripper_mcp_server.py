"""Minimal MCP server for high-level RG2 gripper skills.

Flow:
Agent or MCP client
    -> MCP gripper tool
    -> gripper skill
    -> RobotBackend
    -> ROSGripperBackend
    -> RG2 ROS service

This is the first minimal closed loop for validating the
agent-to-skill-to-backend architecture. The Python environment used to run
this server must have the `mcp` package installed.
"""

from __future__ import annotations

import atexit
import os
import sys
from pathlib import Path
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.gripper_mcp_server")

import yaml

import rclpy
from rclpy.node import Node

from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.events.decorator import emit_skill_events
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.watchdog import session as session_helper
from robot_backend.skills.gripper_skills import close_gripper_skill
from robot_backend.skills.gripper_skills import grasp_skill
from robot_backend.skills.gripper_skills import open_gripper_skill
from robot_backend.skills.gripper_skills import release_skill
from robot_backend.skills.gripper_skills import set_gripper_width_skill
from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION


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

mcp = FastMCP("robot_backend_gripper") if FastMCP is not None else None
_NODE: Node | None = None
_BACKEND = None
_REGISTRY: dict[str, Any] | None = None


def _load_registry() -> dict[str, Any]:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        path = _DEFAULT_REGISTRY_PATH
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _ensure_backend():
    """Create and cache one shared ROS node and backend for MCP tools."""

    global _NODE, _BACKEND, _REGISTRY

    if _BACKEND is not None:
        return _BACKEND

    if not rclpy.ok():
        rclpy.init(args=None)

    _NODE = Node("robot_backend_gripper_mcp_server")
    # /watchdog/events (ROS topic) is the unified cross-process timeline; this
    # JSONL file is a per-process crash-survival artifact, not the merge point.
    log_path = os.environ.get(
        "WATCHDOG_EVENT_LOG",
        session_helper.event_log_default("gripper"),
    )
    configure_bus(node=_NODE, log_path=log_path)
    _BACKEND = create_ros_moveit_backend(node=_NODE)
    if _REGISTRY is None:
        _REGISTRY = _load_registry()
    return _BACKEND


def _shutdown_backend() -> None:
    """Destroy the shared ROS node and shut down rclpy if this server owns it."""

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


def _serialize_response(response: Any) -> Any:
    """Convert ROS responses into MCP-friendly dictionaries when possible."""

    if response is None:
        return None

    success = getattr(response, "success", None)
    message = getattr(response, "message", None)
    if success is not None or message is not None:
        return {
            "success": bool(success) if success is not None else None,
            "message": message,
        }

    field_types = getattr(response.__class__, "get_fields_and_field_types", None)
    if callable(field_types):
        return {
            field_name: getattr(response, field_name)
            for field_name in field_types().keys()
        }

    return str(response)


def _ok(command: str, **payload: Any) -> dict[str, Any]:
    """Build a success response payload."""

    return {
        "ok": True,
        "command": command,
        **payload,
    }


def _error(command: str, error: Exception | str, **payload: Any) -> dict[str, Any]:
    """Build an error response payload."""

    return {
        "ok": False,
        "command": command,
        "error": str(error),
        **payload,
    }


def _validate_object_id(object_id: Any) -> str:
    if not isinstance(object_id, str) or not object_id.strip():
        raise ValueError("object_id must be a non-empty string.")
    return object_id.strip()


def _validate_width(width_m: float) -> float:
    """Strictly validate physical RG2 width input for agent-facing MCP use."""

    width = float(width_m)
    if width < 0.0:
        raise ValueError("width_m must be non-negative.")
    if width > DEFAULT_RG2_CALIBRATION.width_open_m:
        raise ValueError(
            "width_m exceeds the calibrated RG2 maximum opening of "
            f"{DEFAULT_RG2_CALIBRATION.width_open_m} m."
        )
    return width


if mcp is not None:

    @mcp.tool()
    def open_gripper() -> dict[str, Any]:
        """Open the RG2 gripper through the backend skill API."""

        command = "open_gripper"
        try:
            backend = _ensure_backend()
            response = open_gripper_skill(backend)
            return _ok(command, response=_serialize_response(response))
        except Exception as exc:
            return _error(command, exc)


    @mcp.tool()
    def close_gripper() -> dict[str, Any]:
        """Close the RG2 gripper through the backend skill API."""

        command = "close_gripper"
        try:
            backend = _ensure_backend()
            response = close_gripper_skill(backend)
            return _ok(command, response=_serialize_response(response))
        except Exception as exc:
            return _error(command, exc)


    @mcp.tool()
    def set_gripper_width(width_m: float) -> dict[str, Any]:
        """Set physical RG2 finger gap in meters via the backend skill API.

        The backend converts width to the raw RG2 joint target using the same
        calibration-table semantics as the controller when that CSV is available.
        """

        command = "set_gripper_width"
        try:
            width = _validate_width(width_m)
            backend = _ensure_backend()
            response = set_gripper_width_skill(backend, width)
            return _ok(
                command,
                width_m=width,
                response=_serialize_response(response),
            )
        except Exception as exc:
            return _error(command, exc, width_m=width_m)


    @mcp.tool()
    @emit_skill_events
    def get_gripper_state() -> dict[str, Any]:
        """Return backend gripper bindings, width-mapping mode, and calibration metadata."""

        command = "get_gripper_state"
        try:
            backend = _ensure_backend()
            state = backend.gripper.get_state()
            return _ok(command, state=state)
        except Exception as exc:
            return _error(command, exc)


    @mcp.tool()
    def grasp(object_id: str) -> dict[str, Any]:
        """Run the RG2 guarded-move action and attach object_id on confirmed contact.

        The backend opens the gripper, then sends a torque-mode
        `GripperGuardedMove` goal that closes the fingers until measured
        effort crosses the contact threshold; only the `force_threshold` stop
        reason counts as confirmed contact. The resulting width is then
        checked against the registry spec as a sanity bound before the object
        is attached. Returns structured provenance on both success and
        failure paths and does not verify that the object remains grasped
        through later motion.

        Parameters
        ----------
        object_id:
            Registry object ID to grasp (e.g. "red_cube").
        """
        command = "grasp"
        try:
            oid = _validate_object_id(object_id)
            backend = _ensure_backend()
            result = grasp_skill(backend, object_id=oid, registry=_REGISTRY or {})
            return result
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    def release(object_id: str) -> dict[str, Any]:
        """Open the gripper, detach object_id, and sync its TF pose into the world scene.

        Parameters
        ----------
        object_id:
            Registry object ID to release (e.g. "red_cube").
        """
        command = "release"
        try:
            oid = _validate_object_id(object_id)
            backend = _ensure_backend()
            result = release_skill(backend, object_id=oid)
            return result
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    @emit_skill_events
    def set_gripper_joint_position(position_rad: float) -> dict[str, Any]:
        """Send a raw RG2 master joint target in radians.

        This debug-oriented tool bypasses physical width semantics and talks to
        the backend's raw RG2 joint-position command path directly.
        """

        command = "set_gripper_joint_position"
        try:
            backend = _ensure_backend()
            setter = getattr(backend.gripper, "set_joint_position", None)
            if setter is None or not callable(setter):
                raise RuntimeError(
                    "The active gripper backend does not provide "
                    "set_joint_position(position_rad)."
                )
            response = setter(float(position_rad))
            return _ok(
                command,
                position_rad=float(position_rad),
                response=_serialize_response(response),
            )
        except Exception as exc:
            return _error(command, exc, position_rad=position_rad)


def main() -> None:
    """Run the gripper MCP server.

    Eagerly constructs the backend/node/event bus before mcp.run() -- see
    scene_mcp_server.main()'s docstring for why (DDS discovery during the
    ~40s the runner spends on the first LLM turn, before any tool call
    lands). Non-fatal: a failure here logs a warning and falls through to
    mcp.run() unchanged, so --dry-run keeps working with no simulator up and
    the lazy _ensure_backend() path inside each tool remains the fallback.
    """

    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run gripper_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        _ensure_backend()
    except Exception as exc:
        print(
            f"WARNING: gripper_mcp_server eager init failed ({exc}); "
            "continuing -- the first tool call will retry initialization.",
            file=sys.stderr,
        )

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
