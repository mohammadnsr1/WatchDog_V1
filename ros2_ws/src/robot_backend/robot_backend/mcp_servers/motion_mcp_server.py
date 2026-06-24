"""Minimal MCP server for high-level robot motion tools.

Flow:
Agent / MCP client
    -> motion MCP tool
    -> RobotBackend motion method
    -> ROSMoveItMotionBackend
    -> MoveIt MoveGroup action
    -> arm_controller / Isaac trajectory bridge

`move_tcp_to_pose` targets the fixed TCP frame directly.

`move_grasp_center_to_pose` targets the logical gripper grasp center and
computes the fixed TCP target internally from `gripper_width_m` before the
backend sends the final goal to MoveIt.
"""

from __future__ import annotations

import atexit
import math
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.motion_mcp_server")

import rclpy
from rclpy.node import Node

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION
from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center


try:
    from mcp.server.fastmcp import FastMCP
except ImportError as exc:
    FastMCP = None  # type: ignore[assignment]
    _MCP_IMPORT_ERROR = exc
else:
    _MCP_IMPORT_ERROR = None


_SAFE_NAMED_JOINT_POSES: dict[str, list[float]] = {
    "ready": [0.0, -1.2, 1.2, -1.57, -1.57, 0.0],
    "upright": [0.0, -1.2, 1.2, 0.0, 0.0, 0.0],
}
_DEFAULT_ALLOWED_PLANNING_TIME = 5.0
_DEFAULT_APPROACH_DISTANCE_M = 0.05
_DEFAULT_NUM_PLANNING_ATTEMPTS = 3
_DEFAULT_MAX_VELOCITY_SCALING_FACTOR = 0.3
_DEFAULT_MAX_ACCELERATION_SCALING_FACTOR = 0.3
_DEFAULT_POSITION_TOLERANCE_M = 0.005
_DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.01
_MIN_QUATERNION_NORM = 1e-8

mcp = FastMCP("robot_backend_motion") if FastMCP is not None else None
_NODE: Node | None = None
_BACKEND = None


def _ensure_backend():
    """Create and cache one shared ROS node and backend for MCP tools."""

    global _NODE, _BACKEND

    if _BACKEND is not None:
        return _BACKEND

    if not rclpy.ok():
        rclpy.init(args=None)

    _NODE = Node("robot_backend_motion_mcp_server")
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


def _validate_float(name: str, value: Any) -> float:
    """Validate that an MCP input is numeric and finite."""

    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric, not boolean.")

    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric.") from exc

    if not math.isfinite(numeric):
        raise ValueError(f"{name} must be finite.")

    return numeric


def _validate_joint_positions(joint_positions: Any) -> list[float]:
    """Validate MCP-facing UR5e joint targets in fixed joint order."""

    if not isinstance(joint_positions, (list, tuple)):
        raise ValueError("joint_positions must be a list or tuple of 6 numeric values.")

    if len(joint_positions) != 6:
        raise ValueError(
            "joint_positions must contain exactly 6 values in UR5e order."
        )

    return [
        _validate_float(f"joint_positions[{index}]", value)
        for index, value in enumerate(joint_positions)
    ]


def _validate_width(width_m: Any) -> float:
    """Strictly validate RG2 grasp width for grasp-center targeting."""

    width = _validate_float("gripper_width_m", width_m)
    if width < 0.0:
        raise ValueError("gripper_width_m must be non-negative.")
    if width > DEFAULT_RG2_CALIBRATION.width_open_m:
        raise ValueError(
            "gripper_width_m exceeds the calibrated RG2 maximum opening of "
            f"{DEFAULT_RG2_CALIBRATION.width_open_m} m."
        )
    return width


def _build_pose_dict(
    *,
    frame_id: str,
    x: Any,
    y: Any,
    z: Any,
    qx: Any,
    qy: Any,
    qz: Any,
    qw: Any,
) -> dict[str, Any]:
    """Validate pose inputs and build the backend pose dictionary shape."""

    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a non-empty string.")

    px = _validate_float("x", x)
    py = _validate_float("y", y)
    pz = _validate_float("z", z)
    ox = _validate_float("qx", qx)
    oy = _validate_float("qy", qy)
    oz = _validate_float("qz", qz)
    ow = _validate_float("qw", qw)

    quaternion_norm = math.sqrt((ox * ox) + (oy * oy) + (oz * oz) + (ow * ow))
    if quaternion_norm <= _MIN_QUATERNION_NORM:
        raise ValueError("Quaternion norm is too close to zero.")

    return {
        "frame_id": frame_id.strip(),
        "position": {"x": px, "y": py, "z": pz},
        "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
    }


def _resolve_approach_defaults(
    gripper_width_m: Any,
    approach_distance_m: Any,
) -> tuple[Any, Any, list[str]]:
    """Apply fallback defaults when parameters are None and track which were substituted."""

    defaults_used: list[str] = []

    if gripper_width_m is None:
        gripper_width_m = DEFAULT_RG2_CALIBRATION.width_open_m
        defaults_used.append("gripper_width_m")
        if _NODE is not None:
            _NODE.get_logger().warning(
                "gripper_width_m not supplied; "
                f"using fallback {DEFAULT_RG2_CALIBRATION.width_open_m} m"
            )

    if approach_distance_m is None:
        approach_distance_m = _DEFAULT_APPROACH_DISTANCE_M
        defaults_used.append("approach_distance_m")
        if _NODE is not None:
            _NODE.get_logger().warning(
                "approach_distance_m not supplied; "
                f"using fallback {_DEFAULT_APPROACH_DISTANCE_M} m"
            )

    return gripper_width_m, approach_distance_m, defaults_used


def _compute_approach_geometry(
    *,
    frame_id: str,
    x: Any,
    y: Any,
    z: Any,
    qx: Any,
    qy: Any,
    qz: Any,
    qw: Any,
    gripper_width_m: Any,
    approach_distance_m: Any,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], float, float, list[str]]:
    """Validate inputs and compute both TCP approach poses.

    Returns (grasp_center_pose, pre_grasp_tcp_pose, grasp_tcp_pose,
             effective_width_m, effective_dist_m, defaults_used).
    """

    gripper_width_m, approach_distance_m, defaults_used = _resolve_approach_defaults(
        gripper_width_m, approach_distance_m
    )

    grasp_center_pose = _build_pose_dict(
        frame_id=frame_id,
        x=x, y=y, z=z,
        qx=qx, qy=qy, qz=qz, qw=qw,
    )
    width = _validate_width(gripper_width_m)
    dist = _validate_float("approach_distance_m", approach_distance_m)
    if dist < 0.0:
        raise ValueError("approach_distance_m must be non-negative.")

    # Rotate (0, 0, dist) by grasp_center orientation to get the approach offset
    # in the world frame.  Formula: v' = q*(0,0,dist)*q_conj, expanded as
    #   t = 2*cross(q_xyz, v);  v' = v + qw*t + cross(q_xyz, t)
    # with v = (0, 0, dist) the cross products simplify to:
    #   t = (2*qy*dist, -2*qx*dist, 0)
    ox = grasp_center_pose["orientation"]["x"]
    oy = grasp_center_pose["orientation"]["y"]
    oz = grasp_center_pose["orientation"]["z"]
    ow = grasp_center_pose["orientation"]["w"]
    tx = 2.0 * oy * dist
    ty = -2.0 * ox * dist
    vpx = ow * tx - oz * ty
    vpy = ow * ty + oz * tx
    vpz = dist + ox * ty - oy * tx

    gcp = grasp_center_pose["position"]
    pre_grasp_center_pose: dict[str, Any] = {
        "frame_id": grasp_center_pose["frame_id"],
        "position": {
            "x": gcp["x"] - vpx,
            "y": gcp["y"] - vpy,
            "z": gcp["z"] - vpz,
        },
        "orientation": dict(grasp_center_pose["orientation"]),
    }

    grasp_tcp_pose = tcp_target_from_grasp_center(grasp_center_pose, width)
    pre_grasp_tcp_pose = tcp_target_from_grasp_center(pre_grasp_center_pose, width)

    return grasp_center_pose, pre_grasp_tcp_pose, grasp_tcp_pose, width, dist, defaults_used


if mcp is not None:

    @mcp.tool()
    def get_robot_state() -> dict[str, Any]:
        """Return planning metadata and the latest observed arm joint state."""

        command = "get_robot_state"
        try:
            backend = _ensure_backend()
            state = backend.motion.get_robot_state()
            return _ok(command, state=state)
        except Exception as exc:
            return _error(command, exc)


    @mcp.tool()
    def move_to_joint_positions(joint_positions: list[float]) -> dict[str, Any]:
        """Move to explicit UR5e joint targets in fixed joint order."""

        command = "move_to_joint_positions"
        try:
            validated_joint_positions = _validate_joint_positions(joint_positions)
            backend = _ensure_backend()
            result = backend.motion.move_to_joint_positions(
                validated_joint_positions,
                allowed_planning_time=_DEFAULT_ALLOWED_PLANNING_TIME,
                num_planning_attempts=_DEFAULT_NUM_PLANNING_ATTEMPTS,
                max_velocity_scaling_factor=_DEFAULT_MAX_VELOCITY_SCALING_FACTOR,
                max_acceleration_scaling_factor=(
                    _DEFAULT_MAX_ACCELERATION_SCALING_FACTOR
                ),
            )
            return _ok(
                command,
                joint_positions=validated_joint_positions,
                result=result,
            )
        except Exception as exc:
            return _error(command, exc, joint_positions=joint_positions)


    @mcp.tool()
    def move_to_named_safe_joint_pose(name: str) -> dict[str, Any]:
        """Move to a small whitelist of agent-safe named joint poses."""

        command = "move_to_named_safe_joint_pose"
        try:
            if not isinstance(name, str) or not name.strip():
                raise ValueError("name must be a non-empty string.")

            normalized_name = name.strip().lower()
            if normalized_name not in _SAFE_NAMED_JOINT_POSES:
                allowed_names = ", ".join(sorted(_SAFE_NAMED_JOINT_POSES))
                raise ValueError(
                    f"Unknown safe joint pose '{name}'. Allowed names: {allowed_names}."
                )

            joint_positions = list(_SAFE_NAMED_JOINT_POSES[normalized_name])
            backend = _ensure_backend()
            result = backend.motion.move_to_joint_positions(
                joint_positions,
                allowed_planning_time=_DEFAULT_ALLOWED_PLANNING_TIME,
                num_planning_attempts=_DEFAULT_NUM_PLANNING_ATTEMPTS,
                max_velocity_scaling_factor=_DEFAULT_MAX_VELOCITY_SCALING_FACTOR,
                max_acceleration_scaling_factor=(
                    _DEFAULT_MAX_ACCELERATION_SCALING_FACTOR
                ),
            )
            return _ok(
                command,
                name=normalized_name,
                joint_positions=joint_positions,
                result=result,
            )
        except Exception as exc:
            return _error(command, exc, name=name)


    @mcp.tool()
    def move_tcp_to_pose(
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> dict[str, Any]:
        """Move the fixed TCP frame to a target pose."""

        command = "move_tcp_to_pose"
        try:
            pose = _build_pose_dict(
                frame_id=frame_id,
                x=x,
                y=y,
                z=z,
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
            )
            backend = _ensure_backend()
            result = backend.motion.move_to_pose(
                pose,
                target_is_grasp_center=False,
                position_tolerance_m=_DEFAULT_POSITION_TOLERANCE_M,
                orientation_tolerance_rad=_DEFAULT_ORIENTATION_TOLERANCE_RAD,
                allowed_planning_time=_DEFAULT_ALLOWED_PLANNING_TIME,
                num_planning_attempts=_DEFAULT_NUM_PLANNING_ATTEMPTS,
                max_velocity_scaling_factor=_DEFAULT_MAX_VELOCITY_SCALING_FACTOR,
                max_acceleration_scaling_factor=(
                    _DEFAULT_MAX_ACCELERATION_SCALING_FACTOR
                ),
            )
            return _ok(command, requested_tcp_pose=pose, result=result)
        except Exception as exc:
            return _error(
                command,
                exc,
                requested_tcp_pose={
                    "frame_id": frame_id,
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                },
            )


    @mcp.tool()
    def move_grasp_center_to_pose(
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        gripper_width_m: float,
    ) -> dict[str, Any]:
        """Move the logical gripper grasp center to a target pose."""

        command = "move_grasp_center_to_pose"
        try:
            pose = _build_pose_dict(
                frame_id=frame_id,
                x=x,
                y=y,
                z=z,
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
            )
            width = _validate_width(gripper_width_m)
            backend = _ensure_backend()
            result = backend.motion.move_to_pose(
                pose,
                target_is_grasp_center=True,
                gripper_width_m=width,
                position_tolerance_m=_DEFAULT_POSITION_TOLERANCE_M,
                orientation_tolerance_rad=_DEFAULT_ORIENTATION_TOLERANCE_RAD,
                allowed_planning_time=_DEFAULT_ALLOWED_PLANNING_TIME,
                num_planning_attempts=_DEFAULT_NUM_PLANNING_ATTEMPTS,
                max_velocity_scaling_factor=_DEFAULT_MAX_VELOCITY_SCALING_FACTOR,
                max_acceleration_scaling_factor=(
                    _DEFAULT_MAX_ACCELERATION_SCALING_FACTOR
                ),
            )
            return _ok(
                command,
                requested_grasp_center_pose=pose,
                gripper_width_m=width,
                result=result,
            )
        except Exception as exc:
            return _error(
                command,
                exc,
                requested_grasp_center_pose={
                    "frame_id": frame_id,
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                },
                gripper_width_m=gripper_width_m,
            )


    @mcp.tool()
    def compute_grasp_approach_poses(
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        gripper_width_m: float | None = None,
        approach_distance_m: float | None = None,
    ) -> dict[str, Any]:
        """Compute pre-grasp and grasp TCP poses from a desired grasp-center pose.

        Pure geometry — no motion is commanded.  Both TCP poses are returned so
        the caller can inspect them before deciding to execute.

        `gripper_width_m` defaults to the fully-open RG2 width when omitted.
        `approach_distance_m` defaults to 0.05 m when omitted.
        `defaults_used` in the response lists any parameter that fell back to its
        default so the caller can detect when an explicit value was not supplied.
        """

        command = "compute_grasp_approach_poses"
        try:
            (
                grasp_center_pose,
                pre_grasp_tcp_pose,
                grasp_tcp_pose,
                effective_width,
                effective_dist,
                defaults_used,
            ) = _compute_approach_geometry(
                frame_id=frame_id,
                x=x, y=y, z=z,
                qx=qx, qy=qy, qz=qz, qw=qw,
                gripper_width_m=gripper_width_m,
                approach_distance_m=approach_distance_m,
            )
            return _ok(
                command,
                grasp_center_pose=grasp_center_pose,
                pre_grasp_tcp_pose=pre_grasp_tcp_pose,
                grasp_tcp_pose=grasp_tcp_pose,
                gripper_width_m=effective_width,
                approach_distance_m=effective_dist,
                defaults_used=defaults_used,
            )
        except Exception as exc:
            return _error(command, exc)


    @mcp.tool()
    def execute_approach_and_preposition(
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        gripper_width_m: float | None = None,
        approach_distance_m: float | None = None,
        stop_at_pregrasp: bool = False,
    ) -> dict[str, Any]:
        """Move the arm through the pre-grasp pose and optionally into the grasp pose.

        Geometry is identical to `compute_grasp_approach_poses`.  Both computed
        TCP poses are included in the response even when motion fails so the
        caller always sees what geometry was attempted.

        When `stop_at_pregrasp` is True the arm stops at the pre-grasp TCP pose
        and does not continue to the grasp TCP pose.
        """

        command = "execute_approach_and_preposition"
        grasp_center_pose: dict[str, Any] | None = None
        pre_grasp_tcp_pose: dict[str, Any] | None = None
        grasp_tcp_pose: dict[str, Any] | None = None
        effective_width: float = 0.0
        effective_dist: float = 0.0
        defaults_used: list[str] = []

        try:
            (
                grasp_center_pose,
                pre_grasp_tcp_pose,
                grasp_tcp_pose,
                effective_width,
                effective_dist,
                defaults_used,
            ) = _compute_approach_geometry(
                frame_id=frame_id,
                x=x, y=y, z=z,
                qx=qx, qy=qy, qz=qz, qw=qw,
                gripper_width_m=gripper_width_m,
                approach_distance_m=approach_distance_m,
            )
        except Exception as exc:
            return _error(command, exc, defaults_used=defaults_used)

        def _motion_kwargs() -> dict[str, Any]:
            return dict(
                target_is_grasp_center=False,
                position_tolerance_m=_DEFAULT_POSITION_TOLERANCE_M,
                orientation_tolerance_rad=_DEFAULT_ORIENTATION_TOLERANCE_RAD,
                allowed_planning_time=_DEFAULT_ALLOWED_PLANNING_TIME,
                num_planning_attempts=_DEFAULT_NUM_PLANNING_ATTEMPTS,
                max_velocity_scaling_factor=_DEFAULT_MAX_VELOCITY_SCALING_FACTOR,
                max_acceleration_scaling_factor=_DEFAULT_MAX_ACCELERATION_SCALING_FACTOR,
            )

        payload: dict[str, Any] = dict(
            grasp_center_pose=grasp_center_pose,
            pre_grasp_tcp_pose=pre_grasp_tcp_pose,
            grasp_tcp_pose=grasp_tcp_pose,
            gripper_width_m=effective_width,
            approach_distance_m=effective_dist,
            defaults_used=defaults_used,
            stop_at_pregrasp=stop_at_pregrasp,
        )

        try:
            backend = _ensure_backend()
            pregrasp_result = backend.motion.move_to_pose(
                pre_grasp_tcp_pose, **_motion_kwargs()
            )
            payload["pregrasp_motion_result"] = pregrasp_result

            if stop_at_pregrasp:
                return _ok(command, **payload)

            grasp_result = backend.motion.move_to_pose(
                grasp_tcp_pose, **_motion_kwargs()
            )
            payload["grasp_motion_result"] = grasp_result
            return _ok(command, **payload)

        except Exception as exc:
            return _error(command, exc, **payload)


def main() -> None:
    """Run the motion MCP server."""

    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run motion_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
