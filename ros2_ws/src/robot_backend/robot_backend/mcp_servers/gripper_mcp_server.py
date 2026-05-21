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
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.gripper_mcp_server")

import rclpy
from rclpy.node import Node

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import close_gripper_skill
from robot_backend.skills.gripper_skills import open_gripper_skill
from robot_backend.skills.gripper_skills import set_gripper_width_skill
from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION


try:
    from mcp.server.fastmcp import FastMCP
except ImportError as exc:
    FastMCP = None  # type: ignore[assignment]
    _MCP_IMPORT_ERROR = exc
else:
    _MCP_IMPORT_ERROR = None


mcp = FastMCP("robot_backend_gripper") if FastMCP is not None else None
_NODE: Node | None = None
_BACKEND = None


def _ensure_backend():
    """Create and cache one shared ROS node and backend for MCP tools."""

    global _NODE, _BACKEND

    if _BACKEND is not None:
        return _BACKEND

    if not rclpy.ok():
        rclpy.init(args=None)

    _NODE = Node("robot_backend_gripper_mcp_server")
    _BACKEND = create_ros_moveit_backend(node=_NODE)
    return _BACKEND


def _shutdown_backend() -> None:
    """Destroy the shared ROS node and shut down rclpy if this server owns it."""

    global _NODE, _BACKEND

    if _NODE is not None:
        _NODE.destroy_node()
        _NODE = None
    _BACKEND = None
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
        """Set physical RG2 finger gap in meters via the backend skill API."""

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
    def get_gripper_state() -> dict[str, Any]:
        """Return backend gripper state and active RG2 calibration metadata."""

        command = "get_gripper_state"
        try:
            backend = _ensure_backend()
            state = backend.gripper.get_state()
            return _ok(command, state=state)
        except Exception as exc:
            return _error(command, exc)


    @mcp.tool()
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
    """Run the gripper MCP server."""

    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run gripper_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
