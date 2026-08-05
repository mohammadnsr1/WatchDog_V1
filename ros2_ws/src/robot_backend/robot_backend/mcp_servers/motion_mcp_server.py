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
import os
import sys
from pathlib import Path
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.motion_mcp_server")

import rclpy
from rclpy.node import Node

from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.events.decorator import emit_skill_events
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.watchdog import session as session_helper
from robot_backend.utils.geometry import compose_poses, make_pose
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
_DEFAULT_ALLOWED_PLANNING_TIME = 10.0
_DEFAULT_APPROACH_DISTANCE_M = 0.05
_DEFAULT_NUM_PLANNING_ATTEMPTS = 10
_DEFAULT_MAX_VELOCITY_SCALING_FACTOR = 1.0
_DEFAULT_MAX_ACCELERATION_SCALING_FACTOR = 1.0
_DEFAULT_POSITION_TOLERANCE_M = 0.001
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
    # /watchdog/events (ROS topic) is the unified cross-process timeline; this
    # JSONL file is a per-process crash-survival artifact, not the merge point.
    log_path = os.environ.get(
        "WATCHDOG_EVENT_LOG",
        session_helper.event_log_default("motion"),
    )
    configure_bus(node=_NODE, log_path=log_path)
    _BACKEND = create_ros_moveit_backend(node=_NODE)
    return _BACKEND


def _shutdown_backend() -> None:
    """Destroy the shared ROS node and shut down rclpy if this server owns it."""

    global _NODE, _BACKEND

    shutdown_bus()
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
    """Require gripper_width_m; apply fallback defaults for the rest.

    gripper_width_m has no fallback: the TCP target is computed FROM it
    (jaw-opening-dependent TCP<->grasp-center offset), so a silently
    defaulted width computes the offset for the wrong opening and places the
    TCP at the wrong height -- exactly the failure mode that drove a top-down
    descent into the table on a real run. Fail loud at the tool boundary,
    before any motion, instead.
    """

    defaults_used: list[str] = []

    if gripper_width_m is None:
        raise ValueError(
            "gripper_width_m is required; the TCP target is computed from "
            "it. Source it from the step's grasp_width_m."
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


def _compose_grasp_center_from_object(
    object_pose: dict[str, Any],
    ox: Any,
    oy: Any,
    oz: Any,
    oqx: Any,
    oqy: Any,
    oqz: Any,
    oqw: Any,
) -> dict[str, Any]:
    """Compose T_frame_graspcentre = T_frame_object * T_object_graspcentre.

    object_pose is the object's own pose in frame_id; the o* arguments are the
    task instruction step's grasp_center_offset (T_object_grasp_center) --
    see agents/tasks/*.yaml's steps: block, not the model registry. Any
    component left None defaults to identity (position 0, quaternion 0,0,0,1),
    so a caller can supply only the axes that differ from identity.
    """
    offset_pose = make_pose(
        position={
            "x": _validate_float("grasp_center_offset_x", 0.0 if ox is None else ox),
            "y": _validate_float("grasp_center_offset_y", 0.0 if oy is None else oy),
            "z": _validate_float("grasp_center_offset_z", 0.0 if oz is None else oz),
        },
        orientation={
            "x": _validate_float("grasp_center_offset_qx", 0.0 if oqx is None else oqx),
            "y": _validate_float("grasp_center_offset_qy", 0.0 if oqy is None else oqy),
            "z": _validate_float("grasp_center_offset_qz", 0.0 if oqz is None else oqz),
            "w": _validate_float("grasp_center_offset_qw", 1.0 if oqw is None else oqw),
        },
    )
    return compose_poses(object_pose, offset_pose)


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
    approach_vec_x: Any,
    approach_vec_y: Any,
    approach_vec_z: Any,
    grasp_center_offset_x: Any = None,
    grasp_center_offset_y: Any = None,
    grasp_center_offset_z: Any = None,
    grasp_center_offset_qx: Any = None,
    grasp_center_offset_qy: Any = None,
    grasp_center_offset_qz: Any = None,
    grasp_center_offset_qw: Any = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], float, float, dict[str, float], list[str]]:
    """Validate inputs and compute both TCP approach poses.

    Returns (grasp_center_pose, pre_grasp_tcp_pose, grasp_tcp_pose,
             effective_width_m, effective_dist_m, approach_vector, defaults_used).

    approach_vector is the normalized world-frame direction used for the offset.

    When any grasp_center_offset_* is supplied, x..qw are interpreted as the
    OBJECT pose in frame_id and the grasp centre is composed from the object
    pose and the offset supplied by the caller (sourced from the task
    instruction step, not the model registry); "grasp_center_offset" is
    appended to defaults_used to record that interpretation. When all are
    None, x..qw are the grasp-centre pose directly (behaviour unchanged).
    """

    gripper_width_m, approach_distance_m, defaults_used = _resolve_approach_defaults(
        gripper_width_m, approach_distance_m
    )

    # Detect whether the caller supplied an approach vector at all.  All three
    # components must be None for the [0, 0, 1] default to apply.
    if approach_vec_x is None and approach_vec_y is None and approach_vec_z is None:
        approach_vec_x = 0.0
        approach_vec_y = 0.0
        approach_vec_z = 1.0
        defaults_used.append("approach_vector")
        if _NODE is not None:
            _NODE.get_logger().warning(
                "approach_vec not supplied; using fallback [0, 0, 1] (world +Z)"
            )

    raw_x = _validate_float("approach_vec_x", 0.0 if approach_vec_x is None else approach_vec_x)
    raw_y = _validate_float("approach_vec_y", 0.0 if approach_vec_y is None else approach_vec_y)
    raw_z = _validate_float("approach_vec_z", 1.0 if approach_vec_z is None else approach_vec_z)
    vec_norm = math.sqrt(raw_x ** 2 + raw_y ** 2 + raw_z ** 2)
    if vec_norm < 1e-6:
        raise ValueError(
            "approach vector norm is too small; supply a non-zero direction."
        )
    ux, uy, uz = raw_x / vec_norm, raw_y / vec_norm, raw_z / vec_norm
    approach_vector: dict[str, float] = {"x": ux, "y": uy, "z": uz}

    supplied_pose = _build_pose_dict(
        frame_id=frame_id,
        x=x, y=y, z=z,
        qx=qx, qy=qy, qz=qz, qw=qw,
    )
    offset_supplied = any(
        component is not None
        for component in (
            grasp_center_offset_x,
            grasp_center_offset_y,
            grasp_center_offset_z,
            grasp_center_offset_qx,
            grasp_center_offset_qy,
            grasp_center_offset_qz,
            grasp_center_offset_qw,
        )
    )
    if offset_supplied:
        # x..qw were the object pose; compose the grasp centre from it.
        grasp_center_pose = _compose_grasp_center_from_object(
            supplied_pose,
            grasp_center_offset_x,
            grasp_center_offset_y,
            grasp_center_offset_z,
            grasp_center_offset_qx,
            grasp_center_offset_qy,
            grasp_center_offset_qz,
            grasp_center_offset_qw,
        )
        defaults_used.append("grasp_center_offset")
    else:
        # x..qw were the grasp centre directly (behaviour unchanged).
        grasp_center_pose = supplied_pose
    width = _validate_width(gripper_width_m)
    dist = _validate_float("approach_distance_m", approach_distance_m)
    if dist < 0.0:
        raise ValueError("approach_distance_m must be non-negative.")

    gcp = grasp_center_pose["position"]
    pre_grasp_center_pose = make_pose(
        position={
            "x": gcp["x"] + ux * dist,
            "y": gcp["y"] + uy * dist,
            "z": gcp["z"] + uz * dist,
        },
        orientation=grasp_center_pose["orientation"],
        frame_id=grasp_center_pose.get("frame_id"),
    )

    grasp_tcp_pose = tcp_target_from_grasp_center(grasp_center_pose, width)
    pre_grasp_tcp_pose = tcp_target_from_grasp_center(pre_grasp_center_pose, width)

    return (
        grasp_center_pose,
        pre_grasp_tcp_pose,
        grasp_tcp_pose,
        width,
        dist,
        approach_vector,
        defaults_used,
    )


if mcp is not None:

    @mcp.tool()
    @emit_skill_events
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
    @emit_skill_events
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
    @emit_skill_events
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
    @emit_skill_events
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
    @emit_skill_events
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
        gripper_width_m: float,
        approach_distance_m: float | None = None,
        approach_vec_x: float | None = None,
        approach_vec_y: float | None = None,
        approach_vec_z: float | None = None,
        grasp_center_offset_x: float | None = None,
        grasp_center_offset_y: float | None = None,
        grasp_center_offset_z: float | None = None,
        grasp_center_offset_qx: float | None = None,
        grasp_center_offset_qy: float | None = None,
        grasp_center_offset_qz: float | None = None,
        grasp_center_offset_qw: float | None = None,
    ) -> dict[str, Any]:
        """Return the two TCP poses needed to approach and grasp an object,
        without commanding any motion.

        Use this to inspect the planned pre-grasp and grasp positions before
        committing to motion. Pass the results to execute_approach_and_preposition,
        or drive the poses individually via move_tcp_to_pose.

        Parameters
        ----------
        frame_id : str
            The coordinate frame in which x, y, z are expressed. Use "world".
        x, y, z : float
            Position of the desired grasp center in frame_id. The grasp center
            is the logical midpoint between the RG2 fingertips at the given
            gripper width. For an object whose TF pose reports its geometric
            center, pass that center here if you want the fingers to close
            around it. The tool computes where the fixed TCP frame must be so
            that the grasp center lands exactly on this point at the given width.
        qx, qy, qz, qw : float
            Orientation of the grasp center frame in frame_id. For a standard
            top-down grasp use (qx=1, qy=0, qz=0, qw=0), which points the
            gripper straight down. Identity orientation (qx=0, qy=0, qz=0,
            qw=1) points the gripper upward and is not a valid top-down grasp.
        gripper_width_m : float
            Required. Expected physical finger gap in meters at the moment of
            contact. This determines where the grasp center sits relative to
            the fixed TCP frame -- the TCP target is computed FROM this
            value, so there is no fallback: an omitted or wrong width places
            the TCP at the wrong height for the intended grasp. Obtain this
            value from the target step's grasp_width_m in the task
            instruction's steps: block.
        approach_distance_m : float, optional
            Distance in meters between the grasp center target and the
            pre-grasp staging position, measured along the approach vector.
            Defaults to 0.05 m when omitted.
        approach_vec_x, approach_vec_y, approach_vec_z : float, optional
            World-frame direction the robot backs off along to place the
            pre-grasp pose. Default [0, 0, 1] positions the pre-grasp directly
            above the grasp center in world Z, which is correct for top-down
            grasps. For a side grasp approaching from world +X, supply
            approach_vec_x=1 and leave the others at 0. The vector is
            normalized internally so its magnitude does not matter. When all
            three are omitted the [0, 0, 1] default is used and
            "approach_vector" is added to defaults_used.
        grasp_center_offset_x, grasp_center_offset_y, grasp_center_offset_z,
        grasp_center_offset_qx, grasp_center_offset_qy, grasp_center_offset_qz,
        grasp_center_offset_qw : float, optional
            T_object_grasp_center from the task instruction's step
            (grasp_center_offset in the steps: block, see agents/tasks/*.yaml
            -- NOT the model registry): where the grasp centre sits relative
            to the object's own frame. Identity for a symmetric object
            grasped at its centre. Supply this and pass the OBJECT pose in
            x..qw; omit it and pass the grasp-centre pose directly. When any
            component is supplied, "grasp_center_offset" is added to
            defaults_used to record that x..qw were read as the object pose.

            This is a property of the object and is unrelated to the
            gripper-side TCP-to-grasp-centre offset, which is width-dependent,
            comes from the RG2 calibration table, and is applied automatically
            from gripper_width_m.

        Returns
        -------
        dict with keys:
            ok : bool
                True on success.
            grasp_center_pose : dict
                The requested grasp center expressed as a pose dict with
                frame_id, position, and orientation.
            pre_grasp_tcp_pose : dict
                Fixed TCP target for the staging position, offset from the
                grasp center by approach_distance_m along the approach vector.
            grasp_tcp_pose : dict
                Fixed TCP target that places the grasp center on the requested
                position.
            gripper_width_m : float
                Effective width used for the TCP offset calculation.
            approach_distance_m : float
                Effective approach distance used.
            approach_vector : dict
                Normalized approach direction used, with keys x, y, z.
            defaults_used : list of str
                Names of parameters for which fallback values were applied.

        After inspecting the poses, call execute_approach_and_preposition to
        command the motion, or move to grasp_tcp_pose individually and then
        call grasp() on the gripper server to close the fingers.
        """

        command = "compute_grasp_approach_poses"
        try:
            (
                grasp_center_pose,
                pre_grasp_tcp_pose,
                grasp_tcp_pose,
                effective_width,
                effective_dist,
                approach_vector,
                defaults_used,
            ) = _compute_approach_geometry(
                frame_id=frame_id,
                x=x, y=y, z=z,
                qx=qx, qy=qy, qz=qz, qw=qw,
                gripper_width_m=gripper_width_m,
                approach_distance_m=approach_distance_m,
                approach_vec_x=approach_vec_x,
                approach_vec_y=approach_vec_y,
                approach_vec_z=approach_vec_z,
                grasp_center_offset_x=grasp_center_offset_x,
                grasp_center_offset_y=grasp_center_offset_y,
                grasp_center_offset_z=grasp_center_offset_z,
                grasp_center_offset_qx=grasp_center_offset_qx,
                grasp_center_offset_qy=grasp_center_offset_qy,
                grasp_center_offset_qz=grasp_center_offset_qz,
                grasp_center_offset_qw=grasp_center_offset_qw,
            )
            return _ok(
                command,
                grasp_center_pose=grasp_center_pose,
                pre_grasp_tcp_pose=pre_grasp_tcp_pose,
                grasp_tcp_pose=grasp_tcp_pose,
                gripper_width_m=effective_width,
                approach_distance_m=effective_dist,
                approach_vector=approach_vector,
                defaults_used=defaults_used,
            )
        except Exception as exc:
            return _error(command, exc)


    @mcp.tool()
    @emit_skill_events
    def execute_approach_and_preposition(
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        gripper_width_m: float,
        approach_distance_m: float | None = None,
        approach_vec_x: float | None = None,
        approach_vec_y: float | None = None,
        approach_vec_z: float | None = None,
        grasp_center_offset_x: float | None = None,
        grasp_center_offset_y: float | None = None,
        grasp_center_offset_z: float | None = None,
        grasp_center_offset_qx: float | None = None,
        grasp_center_offset_qy: float | None = None,
        grasp_center_offset_qz: float | None = None,
        grasp_center_offset_qw: float | None = None,
        stop_at_pregrasp: bool = False,
    ) -> dict[str, Any]:
        """Move the arm to the pre-grasp staging position and optionally
        continue to the grasp position, then return the planned poses and
        motion outcomes.

        The pose geometry is identical to compute_grasp_approach_poses. Both
        computed TCP poses are included in the response even when motion fails
        so the caller always has the planned geometry for diagnostics.

        Parameters
        ----------
        frame_id : str
            The coordinate frame in which x, y, z are expressed. Use "world".
        x, y, z : float
            Position of the desired grasp center in frame_id. See
            compute_grasp_approach_poses for the physical definition of the
            grasp center.
        qx, qy, qz, qw : float
            Orientation of the grasp center frame in frame_id. For a standard
            top-down grasp use (qx=1, qy=0, qz=0, qw=0).
        gripper_width_m : float
            Required. Expected physical finger gap in meters at the moment of
            contact -- the TCP target is computed FROM this value, so there
            is no fallback: an omitted or wrong width places the TCP at the
            wrong height and drives the approach into whatever is there.
            Obtain from the target step's grasp_width_m in the task
            instruction's steps: block.
        approach_distance_m : float, optional
            Distance in meters between the grasp center target and the
            pre-grasp staging position. Defaults to 0.05 m when omitted.
        approach_vec_x, approach_vec_y, approach_vec_z : float, optional
            World-frame direction along which the pre-grasp position is offset
            from the grasp center. Default [0, 0, 1] gives a position directly
            above the grasp center in world Z, correct for top-down grasps.
            For a side grasp approaching from world +X, supply approach_vec_x=1
            and leave the others at 0. The vector is normalized internally.
            Omit all three for the [0, 0, 1] default; recorded in defaults_used.
        grasp_center_offset_x, grasp_center_offset_y, grasp_center_offset_z,
        grasp_center_offset_qx, grasp_center_offset_qy, grasp_center_offset_qz,
        grasp_center_offset_qw : float, optional
            T_object_grasp_center from the task instruction's step
            (grasp_center_offset in the steps: block, see agents/tasks/*.yaml
            -- NOT the model registry): where the grasp centre sits relative
            to the object's own frame. Identity for a symmetric object
            grasped at its centre. Supply this and pass the OBJECT pose in
            x..qw; omit it and pass the grasp-centre pose directly. When any
            component is supplied, "grasp_center_offset" is added to
            defaults_used to record that x..qw were read as the object pose.

            This is a property of the object and is unrelated to the
            gripper-side TCP-to-grasp-centre offset, which is width-dependent,
            comes from the RG2 calibration table, and is applied automatically
            from gripper_width_m.
        stop_at_pregrasp : bool, optional
            When True the arm stops at the pre-grasp TCP pose and the tool
            returns without proceeding to the grasp position. Defaults to False.

        Sequence
        --------
        1. Robot moves the grasp center to approach_distance_m along the
           approach vector away from the target, with the gripper open
           (pre-grasp TCP pose).
        2. If stop_at_pregrasp is True, returns here.
        3. Robot moves the grasp center to the exact requested position
           (grasp TCP pose). Gripper remains open.
        4. Returns. Call grasp() on the gripper server to close the fingers.

        Returns
        -------
        dict with keys:
            ok : bool
                True when all commanded motion steps completed successfully.
            grasp_center_pose : dict
                The requested grasp center expressed as a pose dict.
            pre_grasp_tcp_pose : dict
                Fixed TCP target for the staging position.
            grasp_tcp_pose : dict
                Fixed TCP target for the grasp position.
            gripper_width_m : float
                Effective width used for the TCP offset calculation.
            approach_distance_m : float
                Effective approach distance used.
            approach_vector : dict
                Normalized approach direction used, with keys x, y, z.
            defaults_used : list of str
                Names of parameters for which fallback values were applied.
            pregrasp_motion_result : dict
                Motion planner outcome for the move to the pre-grasp pose.
            grasp_motion_result : dict
                Motion planner outcome for the move to the grasp pose. Present
                only when stop_at_pregrasp is False and the pre-grasp move
                succeeded.
        """

        command = "execute_approach_and_preposition"
        grasp_center_pose: dict[str, Any] | None = None
        pre_grasp_tcp_pose: dict[str, Any] | None = None
        grasp_tcp_pose: dict[str, Any] | None = None
        effective_width: float = 0.0
        effective_dist: float = 0.0
        approach_vector: dict[str, float] = {"x": 0.0, "y": 0.0, "z": 1.0}
        defaults_used: list[str] = []

        try:
            (
                grasp_center_pose,
                pre_grasp_tcp_pose,
                grasp_tcp_pose,
                effective_width,
                effective_dist,
                approach_vector,
                defaults_used,
            ) = _compute_approach_geometry(
                frame_id=frame_id,
                x=x, y=y, z=z,
                qx=qx, qy=qy, qz=qz, qw=qw,
                gripper_width_m=gripper_width_m,
                approach_distance_m=approach_distance_m,
                approach_vec_x=approach_vec_x,
                approach_vec_y=approach_vec_y,
                approach_vec_z=approach_vec_z,
                grasp_center_offset_x=grasp_center_offset_x,
                grasp_center_offset_y=grasp_center_offset_y,
                grasp_center_offset_z=grasp_center_offset_z,
                grasp_center_offset_qx=grasp_center_offset_qx,
                grasp_center_offset_qy=grasp_center_offset_qy,
                grasp_center_offset_qz=grasp_center_offset_qz,
                grasp_center_offset_qw=grasp_center_offset_qw,
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
            approach_vector=approach_vector,
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
    """Run the motion MCP server.

    Eagerly constructs the backend/node/event bus before mcp.run() -- see
    scene_mcp_server.main()'s docstring for why (DDS discovery during the
    ~40s the runner spends on the first LLM turn, before any tool call
    lands). Non-fatal: a failure here logs a warning and falls through to
    mcp.run() unchanged, so --dry-run keeps working with no simulator up and
    the lazy _ensure_backend() path inside each tool remains the fallback.
    """

    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run motion_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        _ensure_backend()
    except Exception as exc:
        print(
            f"WARNING: motion_mcp_server eager init failed ({exc}); "
            "continuing -- the first tool call will retry initialization.",
            file=sys.stderr,
        )

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
