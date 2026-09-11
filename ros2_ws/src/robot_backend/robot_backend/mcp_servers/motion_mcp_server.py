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
from robot_backend.utils.geometry import compose_poses
from robot_backend.utils.geometry import grasp_center_from_object_and_offset, make_pose, rotate_vector
from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION
from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center
from robot_backend.mcp_servers import role_context


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
# 5s, not 10s: with a fallback chain each failure costs this in full, and the
# observed 50s stalls were mostly budget burned on plans that were never going
# to solve. Raise per-call via the tool argument for a genuinely hard move.
_DEFAULT_ALLOWED_PLANNING_TIME = 5.0
# NEGATIVE since approach_distance_m became signed and is applied exactly as
# given: for the top-down grasps in this registry local +Z points INTO the
# object, so backing off is -Z. Leaving this at +0.05 after the sign change
# would have staged the pre-grasp 5cm inside the part.
_DEFAULT_APPROACH_DISTANCE_M = -0.05
_DEFAULT_NUM_PLANNING_ATTEMPTS = 4
_DEFAULT_MAX_VELOCITY_SCALING_FACTOR = 0.7
_DEFAULT_MAX_ACCELERATION_SCALING_FACTOR = 0.5
_DEFAULT_POSITION_TOLERANCE_M = 0.0001
_DEFAULT_ORIENTATION_TOLERANCE_RAD = 0.001
_MIN_QUATERNION_NORM = 1e-8

mcp = FastMCP("robot_backend_motion") if FastMCP is not None else None
# Must precede the tool definitions below: it patches mcp.tool() so every tool
# registered afterwards records its call to mcp_calls_<role>.jsonl.
role_context.install_call_logging(mcp, "motion")
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


def _validate_scaling(name: str, value: Any) -> float:
    """Validate a MoveIt scaling factor: a fraction of the robot's limits."""

    scaling = _validate_float(name, value)
    if not (0.0 < scaling <= 1.0):
        raise ValueError(
            f"{name} must be in (0, 1] -- it is a FRACTION of the robot's "
            f"configured limit, not a speed in m/s. Got {scaling}."
        )
    return scaling


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
    """Resolve the grasp centre from an object pose and a grasp_center_offset.

    object_pose is the object's own pose in frame_id; the o* arguments are the
    task instruction step's grasp_center_offset -- see agents/tasks/*.yaml's
    steps: block, not the model registry. Any component left None defaults to
    identity, so a caller can supply only the axes that differ.

    Delegates to geometry.grasp_center_from_object_and_offset, the single
    implementation this and test/plan_assembly_step.py share, so an offset
    proven there means the same thing here. BOTH halves are object-relative
    and composed through the object's pose, so the grasp inherits the part's
    actual roll/pitch/yaw -- which is what lets a part re-picked after
    settling off-square be gripped square to itself.
    """
    # Orientation has no safe default. Identity here means "grasp centre
    # aligned with the object's own frame", which for a part resting square
    # points the gripper UP -- inverting the approach axis and sending the
    # pre-grasp BELOW the object instead of above it. Nothing intends that,
    # and it fails silently rather than erroring, so require it.
    if all(component is None for component in (oqx, oqy, oqz, oqw)):
        raise ValueError(
            "grasp_center_offset orientation (grasp_center_offset_qx/qy/qz/qw) is "
            "required when a grasp_center_offset position is supplied. It is the "
            "grasp centre's rotation RELATIVE TO the object and is composed "
            "through the object's pose, so there is no meaningful default: "
            "identity would align the grasp with the object's own frame and, "
            "for a part resting square, point the gripper upward. For a part "
            "resting square, qx=1, qy=0, qz=0, qw=0 is top-down. Source it from "
            "this step's grasp_center_offset in the task instruction's steps: block."
        )

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
    return grasp_center_from_object_and_offset(object_pose, offset_pose)


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

    The pre-grasp staging position is the grasp center backed off by
    approach_distance_m along the grasp center's OWN local Z axis (sign taken
    as given), composed
    via compose_poses (T_world_pregrasp = T_world_graspcenter *
    T_graspcenter_pregrasp), not a fixed world-frame direction. This is what
    makes the geometry correct for ANY grasp orientation, not just top-down:
    the grasp-center frame's own +Z axis is, by convention, the approach
    axis pointing INTO the object, so retreating along -Z always backs away
    from the object regardless of which way the gripper is actually pointed
    in world coordinates. For the top-down orientation every object in this
    registry currently uses (qx=1, qy=0, qz=0, qw=0, a 180-degree flip about
    X), a NEGATIVE distance resolves to world +Z, matching the old behavior
    exactly -- so this is a strict generalization, not a behavior change for
    any grasp defined today. approach_vector in the return value is that
    same direction expressed in world/frame_id coordinates, for diagnostics.

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

    # Offset along the grasp center's own local Z: a pure local-frame
    # translation composed onto the grasp center, not a world-frame addition.
    # Generalizes to any grasp orientation, side approaches included.
    #
    # SIGN CONVENTION CHANGE: dist is now applied EXACTLY as given rather than
    # negated, and negative values are no longer rejected. Forcing the sign
    # assumed local +Z always points into the object, which is only true for
    # the top-down orientation; it breaks for any orientation where local +Z
    # points away from the surface. Callers choose the sign, matching
    # test/plan_assembly_step.py's pregrasp_center_from_grasp_center. For the
    # top-down grasps in the registry today, pass -0.10 where you previously
    # passed +0.10.
    local_backoff = make_pose(
        position={"x": 0.0, "y": 0.0, "z": dist},
        orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    )
    pre_grasp_center_pose = compose_poses(grasp_center_pose, local_backoff)

    approach_vector = rotate_vector(
        {"x": 0.0, "y": 0.0, "z": 1.0 if dist >= 0 else -1.0},
        grasp_center_pose["orientation"],
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
    def get_rendered_task() -> dict[str, Any]:
        """Return the EXECUTOR's system prompt and rendered task. Call first.

        Reads EXECUTOR_TASK_PATH only. This tool is hosted here, on a server
        the executor session loads and the watchdog session does not, so the
        watchdog has no tool capable of resolving the executor's task file --
        which matters because the executor's file may deliberately carry extra
        or faulty steps the watchdog must not be told about in advance.

        Returns system_prompt and task as separate fields: load system_prompt
        as your standing instructions and task as your opening task, before
        calling any other tool.
        """

        command = "get_rendered_task"
        try:
            return role_context.build_rendered_task(role_context.EXECUTOR_ROLE)
        except Exception as exc:
            return _error(command, exc)


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
        solve_ik_first: bool = False,
        position_tolerance_m: float | None = None,
        orientation_tolerance_rad: float | None = None,
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
                solve_ik_first=bool(solve_ik_first),
                position_tolerance_m=(
                    _DEFAULT_POSITION_TOLERANCE_M if position_tolerance_m is None
                    else _validate_float("position_tolerance_m", position_tolerance_m)
                ),
                orientation_tolerance_rad=(
                    _DEFAULT_ORIENTATION_TOLERANCE_RAD if orientation_tolerance_rad is None
                    else _validate_float("orientation_tolerance_rad", orientation_tolerance_rad)
                ),
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
        solve_ik_first: bool = False,
        position_tolerance_m: float | None = None,
        orientation_tolerance_rad: float | None = None,
        grasp_center_offset_x: float | None = None,
        grasp_center_offset_y: float | None = None,
        grasp_center_offset_z: float | None = None,
        grasp_center_offset_qx: float | None = None,
        grasp_center_offset_qy: float | None = None,
        grasp_center_offset_qz: float | None = None,
        grasp_center_offset_qw: float | None = None,
        target_offset_x: float | None = None,
        target_offset_y: float | None = None,
        target_offset_z: float | None = None,
        target_offset_qx: float | None = None,
        target_offset_qy: float | None = None,
        target_offset_qz: float | None = None,
        target_offset_qw: float | None = None,
        max_velocity_scaling_factor: float | None = None,
        max_acceleration_scaling_factor: float | None = None,
        allowed_planning_time: float | None = None,
        approach_distance_m: float | None = None,
        clear_octomap_first: bool = False,
    ) -> dict[str, Any]:
        """Move the logical gripper grasp center to a target pose.

        WITH a grasp_center_offset (use this for placing/reorienting a part
        you are already holding): x..qw are the DESTINATION pose of the
        OBJECT -- e.g. a placement or reorient frame you just queried, copied
        through unchanged -- and this tool composes the grasp centre from it,
        rotating the offset position through the destination orientation and
        taking the offset's orientation as the grasp orientation. Do this
        rather than composing by hand: for a destination rotated 90 degrees,
        an offset of (0.049, -0.040, 0) contributes (0.049, 0, 0.040) in
        world, and dropping the rotated x term puts the part ~50mm out with
        no error raised.

        WITHOUT an offset: x..qw are the grasp centre pose directly.

        The three optional arguments exist for insertions and other
        land-exactly moves; leave them alone for ordinary free-space motion.

        solve_ik_first resolves ONE exact IK solution and plans in joint
        space, so the end effector lands at the literal target instead of
        anywhere inside the goal tolerance region. It is a single IK seed with
        no retry, so it can fail on poses the sampling path would solve, and
        it makes the two tolerances below inert (the joint-space path returns
        before they are read) -- do not combine them.

        position_tolerance_m / orientation_tolerance_rad widen or tighten the
        goal region. An insertion usually wants the opposite trade from a
        free-space move: position tight so the part enters the hole,
        orientation loose (e.g. 0.05 rad) so the planner can absorb the small
        tilt the part has in the gripper and still find a collision-free goal.

        target_offset_x/y/z (and optional target_offset_q*) apply a standoff
        from the destination BEFORE the grasp centre is resolved, expressed in
        the DESTINATION FRAME'S OWN axes. Use it for an approach stage: pass
        the placement/insertion frame's pose in x..qw exactly as the pose
        query returned it, and the standoff here -- do NOT pre-add it
        yourself. These frames' local +Z generally does not point up, so an
        offset added in world coordinates stages the approach sideways
        instead of above the slot, and the resulting call is valid and plans
        happily to the wrong place. Omit it for the seated pose itself. The
        orientation defaults to identity, which is meaningful here: same
        orientation, just standing off.

        approach_distance_m offsets the resolved grasp centre along its OWN
        local Z -- the same axis, and the same sign convention, as the approach
        tool's argument of that name. Use it for a LIFT: it backs the gripper
        out along exactly the axis the approach came in on, because the grasp
        orientation is unchanged. That is what makes a lift reliable regardless
        of how the part is sitting.

        The sign is the SAME as the pick's, not its opposite: on both tools a
        negative value means "back away along local Z". A top-down pick uses
        about -0.10 to stage above the part; the matching lift uses about -0.20
        to back further out along that same axis. A positive value drives INTO
        the part on either tool.

        Offsetting in world Z instead, or via target_offset (which is in the
        OBJECT's axes), only lifts straight up while the part happens to rest
        square; for a part standing at a reorient frame it drags it sideways
        across the bench.

        clear_octomap_first clears MoveIt's occupancy map inside THIS call,
        immediately before planning. Use it whenever the move enters space the
        camera currently sees as occupied -- carrying a part away from where
        it was standing, or seating one against others. Prefer it over calling
        clear_octomap as a separate step: the map republishes at the
        point-cloud rate, so a clear from a previous turn has already refilled
        by the time this plans, and the move then fails
        START_STATE_IN_COLLISION.
        """

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

            # A standoff from the destination, applied FIRST and expressed in
            # the destination frame's OWN axes -- so an approach height stays
            # "above the slot" however the fixture is rotated. Doing it here
            # rather than in the caller is the point: these frames' local +Z
            # generally does NOT point up, so adding the offset in world
            # coordinates lands the approach sideways, and a caller that gets
            # it wrong produces a perfectly valid call to the wrong place.
            if any(
                component is not None
                for component in (
                    target_offset_x,
                    target_offset_y,
                    target_offset_z,
                    target_offset_qx,
                    target_offset_qy,
                    target_offset_qz,
                    target_offset_qw,
                )
            ):
                target_offset_pose = make_pose(
                    position={
                        "x": _validate_float("target_offset_x", 0.0 if target_offset_x is None else target_offset_x),
                        "y": _validate_float("target_offset_y", 0.0 if target_offset_y is None else target_offset_y),
                        "z": _validate_float("target_offset_z", 0.0 if target_offset_z is None else target_offset_z),
                    },
                    orientation={
                        "x": _validate_float("target_offset_qx", 0.0 if target_offset_qx is None else target_offset_qx),
                        "y": _validate_float("target_offset_qy", 0.0 if target_offset_qy is None else target_offset_qy),
                        "z": _validate_float("target_offset_qz", 0.0 if target_offset_qz is None else target_offset_qz),
                        # Identity IS meaningful here, unlike grasp_center_offset:
                        # it means "same orientation, just standing off".
                        "w": _validate_float("target_offset_qw", 1.0 if target_offset_qw is None else target_offset_qw),
                    },
                )
                pose = compose_poses(pose, target_offset_pose)

            # When an offset is supplied, x..qw were the DESTINATION pose of the
            # OBJECT and the grasp centre is composed from it here -- the same
            # dual meaning execute_approach_and_preposition has. Without this
            # the caller has to rotate the offset through the destination
            # orientation by hand for every placement, which is exactly the
            # arithmetic that silently mislocates a place step.
            if any(
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
            ):
                pose = _compose_grasp_center_from_object(
                    pose,
                    grasp_center_offset_x,
                    grasp_center_offset_y,
                    grasp_center_offset_z,
                    grasp_center_offset_qx,
                    grasp_center_offset_qy,
                    grasp_center_offset_qz,
                    grasp_center_offset_qw,
                )

            # Offset along the GRASP CENTRE's own local Z, after the grasp centre
            # is resolved. That axis IS the approach axis, so this is how a lift
            # is guaranteed to back out exactly the way the approach came in --
            # same orientation, opposite sign. Offsetting in world Z, or along
            # the OBJECT's local Z, only agrees while the part happens to rest
            # square; for a part standing at a reorient frame it drags the part
            # sideways across the bench instead of lifting it.
            if approach_distance_m is not None:
                dist = _validate_float("approach_distance_m", approach_distance_m)
                pose = compose_poses(
                    pose,
                    make_pose(
                        position={"x": 0.0, "y": 0.0, "z": dist},
                        orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    ),
                )

            backend = _ensure_backend()
            # Clearing must happen in THIS call, not as a separate tool call.
            # The map republishes at the point-cloud rate (10 Hz here), so a
            # clear issued in a previous turn has fully refilled by the time
            # the plan is requested -- measured gaps of 3.5-34 s between a
            # clear and the move it was meant to enable, every one of which
            # then failed START_STATE_IN_COLLISION.
            octomap_cleared = False
            if clear_octomap_first:
                backend.scene.clear_octomap()
                octomap_cleared = True

            pos_tol = (
                _DEFAULT_POSITION_TOLERANCE_M
                if position_tolerance_m is None
                else _validate_float("position_tolerance_m", position_tolerance_m)
            )
            ori_tol = (
                _DEFAULT_ORIENTATION_TOLERANCE_RAD
                if orientation_tolerance_rad is None
                else _validate_float("orientation_tolerance_rad", orientation_tolerance_rad)
            )
            # Speed is per-call: a seating move wants to be slow (the commanded
            # trajectory is tracked more closely, and a part entering a slot is
            # where tracking error becomes a jam), a free-space transit does
            # not. None means "use the cell default", so an omitted argument
            # behaves exactly as before.
            vel = (
                _DEFAULT_MAX_VELOCITY_SCALING_FACTOR
                if max_velocity_scaling_factor is None
                else _validate_scaling("max_velocity_scaling_factor", max_velocity_scaling_factor)
            )
            acc = (
                _DEFAULT_MAX_ACCELERATION_SCALING_FACTOR
                if max_acceleration_scaling_factor is None
                else _validate_scaling("max_acceleration_scaling_factor", max_acceleration_scaling_factor)
            )
            plan_time = (
                _DEFAULT_ALLOWED_PLANNING_TIME
                if allowed_planning_time is None
                else _validate_float("allowed_planning_time", allowed_planning_time)
            )
            result = backend.motion.move_to_pose(
                pose,
                target_is_grasp_center=True,
                gripper_width_m=width,
                solve_ik_first=bool(solve_ik_first),
                position_tolerance_m=pos_tol,
                orientation_tolerance_rad=ori_tol,
                allowed_planning_time=plan_time,
                num_planning_attempts=_DEFAULT_NUM_PLANNING_ATTEMPTS,
                max_velocity_scaling_factor=vel,
                max_acceleration_scaling_factor=acc,
            )
            return _ok(
                command,
                requested_grasp_center_pose=pose,
                gripper_width_m=width,
                octomap_cleared=octomap_cleared,
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
        x, y, z, qx, qy, qz, qw : float
            THE MEANING OF THESE SEVEN DEPENDS ON WHETHER YOU SUPPLY A
            grasp_center_offset. Getting this wrong silently mislocates the
            grasp instead of erroring.

            WITH a grasp_center_offset (the usual case when a task step
            carries one): they are the OBJECT's CURRENT pose, exactly as
            get_object_pose returned it -- copy its position AND its
            orientation through unchanged. Do NOT put the grasp orientation
            here. The tool derives the grasp centre itself by rotating the
            offset position through this orientation and taking the offset's
            own orientation as the grasp orientation. Passing (qx=1, qy=0,
            qz=0, qw=0) here for an object that is actually resting at
            identity flips the offset through 180 degrees about X, i.e.
            (x, y, z) -> (x, -y, -z), landing the grasp on the opposite side
            of the object.

            WITHOUT a grasp_center_offset: they are the desired GRASP CENTRE
            pose directly -- the logical midpoint between the RG2 fingertips
            at the given width, and the orientation the grasp centre should
            have. In that case a standard top-down grasp is (qx=1, qy=0,
            qz=0, qw=0); identity (qw=1) points the gripper upward and is not
            a top-down grasp.
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
            pre-grasp staging position. Defaults to 0.05 m when omitted. The
            pre-grasp position is always the grasp center backed off along
            its OWN local Z axis by the SIGNED distance given (the grasp frame's +Z is conventionally the approach axis,
            pointing into the object, by convention) -- this generalizes to
            any grasp orientation, side approaches included, not just
            top-down. Not independently configurable: the direction is
            determined entirely by qx..qw above.
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
                grasp center by approach_distance_m (signed) along the grasp center's
                own local Z axis, by the signed distance given.
            grasp_tcp_pose : dict
                Fixed TCP target that places the grasp center on the requested
                position.
            gripper_width_m : float
                Effective width used for the TCP offset calculation.
            approach_distance_m : float
                Effective approach distance used.
            approach_vector : dict
                The retreat direction (local -Z if the distance is negative,
                +Z if positive) expressed in frame_id
                coordinates -- world +Z for a top-down grasp, some other
                direction for a side grasp.
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
        grasp_center_offset_x: float | None = None,
        grasp_center_offset_y: float | None = None,
        grasp_center_offset_z: float | None = None,
        grasp_center_offset_qx: float | None = None,
        grasp_center_offset_qy: float | None = None,
        grasp_center_offset_qz: float | None = None,
        grasp_center_offset_qw: float | None = None,
        stop_at_pregrasp: bool = False,
        clear_octomap_first: bool = False,
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
        x, y, z, qx, qy, qz, qw : float
            THE MEANING OF THESE SEVEN DEPENDS ON WHETHER YOU SUPPLY A
            grasp_center_offset. Getting this wrong silently mislocates the
            grasp instead of erroring.

            WITH a grasp_center_offset (the usual case when a task step
            carries one): they are the OBJECT's CURRENT pose, exactly as
            get_object_pose returned it -- copy its position AND its
            orientation through unchanged. Do NOT put the grasp orientation
            here. The tool derives the grasp centre itself by rotating the
            offset position through this orientation and taking the offset's
            own orientation as the grasp orientation. Passing (qx=1, qy=0,
            qz=0, qw=0) here for an object that is actually resting at
            identity flips the offset through 180 degrees about X, i.e.
            (x, y, z) -> (x, -y, -z), landing the grasp on the opposite side
            of the object.

            WITHOUT a grasp_center_offset: they are the desired GRASP CENTRE
            pose directly -- the logical midpoint between the RG2 fingertips
            at the given width, and the orientation the grasp centre should
            have. In that case a standard top-down grasp is (qx=1, qy=0,
            qz=0, qw=0); identity (qw=1) points the gripper upward and is not
            a top-down grasp.
        gripper_width_m : float
            Required. Expected physical finger gap in meters at the moment of
            contact -- the TCP target is computed FROM this value, so there
            is no fallback: an omitted or wrong width places the TCP at the
            wrong height and drives the approach into whatever is there.
            Obtain from the target step's grasp_width_m in the task
            instruction's steps: block.
        approach_distance_m : float, optional
            Distance in meters between the grasp center target and the
            pre-grasp staging position. Defaults to 0.05 m when omitted. The
            pre-grasp position is always the grasp center backed off along
            its OWN local Z axis by the SIGNED distance given (the grasp frame's +Z is conventionally the approach axis,
            pointing into the object, by convention) -- this generalizes to
            any grasp orientation, side approaches included, not just
            top-down. Not independently configurable: the direction is
            determined entirely by qx..qw above.
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
        clear_octomap_first : bool, optional
            Clear MoveIt's occupancy map immediately before planning, inside
            this same call. Use it whenever the move has to enter space the
            camera currently sees as occupied: right after grasping a part
            that was standing on the table (its old voxels remain), or before
            seating a part against others. Prefer this over calling
            clear_octomap as a separate step -- the map republishes at the
            point-cloud rate, so a clear from a previous turn has refilled
            before this plans, and the move fails START_STATE_IN_COLLISION.
        stop_at_pregrasp : bool, optional
            When True the arm stops at the pre-grasp TCP pose and the tool
            returns without proceeding to the grasp position. Defaults to False.

        Sequence
        --------
        1. Robot moves the grasp center to approach_distance_m along the
           grasp orientation's local Z axis from the target, with the
           gripper open (pre-grasp TCP pose). The sign is taken as given --
           NEGATIVE backs off for a top-down grasp.
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
                The retreat direction (local -Z if the distance is negative,
                +Z if positive) expressed in frame_id
                coordinates -- world +Z for a top-down grasp, some other
                direction for a side grasp.
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
            # Same-call clear: see move_grasp_center_to_pose. A clear issued as
            # a separate tool call has already refilled by the time this plans.
            if clear_octomap_first:
                backend.scene.clear_octomap()
                payload["octomap_cleared"] = True

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

    @mcp.tool()
    @emit_skill_events
    def retreat_tcp(
        distance_m: float,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> dict[str, Any]:
        """Back the TCP away along its OWN local Z from a known TCP pose.

        Use after a release, to clear a part that was just placed.

        x..qw are the TCP pose to retreat FROM -- normally the tcp pose the
        preceding placement reported, not the grasp centre. This is
        deliberately a TCP-to-TCP move: grasp-centre targets convert to TCP
        through the gripper opening, and a release has just opened the jaws to
        ~104mm, so routing a retreat that way shifts the TCP by ~41mm on the
        width change alone -- DOWNWARD, into the part just placed.

        distance_m is applied exactly as given along local Z, same convention
        as approach_distance_m: for a top-down TCP whose local +Z points down,
        a NEGATIVE value backs up and away.
        """

        command = "retreat_tcp"
        try:
            base_tcp = _build_pose_dict(
                frame_id=frame_id, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw
            )
            dist = _validate_float("distance_m", distance_m)
            retreat_tcp_pose = compose_poses(
                base_tcp,
                make_pose(
                    position={"x": 0.0, "y": 0.0, "z": dist},
                    orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                ),
            )
            backend = _ensure_backend()
            result = backend.motion.move_to_pose(
                retreat_tcp_pose,
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
                from_tcp_pose=base_tcp,
                distance_m=dist,
                retreat_tcp_pose=retreat_tcp_pose,
                result=result,
            )
        except Exception as exc:
            return _error(command, exc, distance_m=distance_m)


    @mcp.tool()
    @emit_skill_events
    def lift(
        height_m: float,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> dict[str, Any]:
        """Raise the TCP straight up in world Z, keeping its orientation.

        This is the tool for lifting something. height_m is how far UP to go,
        in metres, in the world frame. Positive lifts. There is no sign
        convention to work out and no axis to choose: up is up, whatever the
        gripper is holding and however that part happens to be oriented.

        Use this instead of an approach-axis or object-frame offset whenever
        the intent is "raise it". Those offsets travel along the grasp
        centre's own local Z, or the object's own axes, which point up ONLY
        while the part is resting square with the world. A part standing at a
        reorient frame is rotated roughly ninety degrees from that, so its own
        Z lies flat and an offset along it drags the part sideways across the
        bench instead of raising it. The motion plans and executes; nothing
        errors; the part simply goes the wrong way. Lifting in world Z cannot
        express that mistake.

        x..qw are the TCP pose to lift FROM -- normally the TCP pose the
        preceding motion reported. This is deliberately TCP-to-TCP, like
        retreat_tcp: grasp-centre targets convert to TCP through the gripper
        opening, so routing a lift that way makes the result depend on jaw
        width, which has nothing to do with how far up the part should go.
        Orientation is carried through unchanged, so the grasp stays rigid
        across the lift and the held part rises exactly height_m.
        """

        command = "lift"
        try:
            base_tcp = _build_pose_dict(
                frame_id=frame_id, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw
            )
            height = _validate_float("height_m", height_m)
            # A world-frame translation, NOT compose_poses: composing would
            # apply the offset in the TCP's own axes, which is the very thing
            # this tool exists to avoid.
            lifted_tcp_pose = make_pose(
                position={
                    "x": base_tcp["position"]["x"],
                    "y": base_tcp["position"]["y"],
                    "z": base_tcp["position"]["z"] + height,
                },
                orientation=dict(base_tcp["orientation"]),
                frame_id=base_tcp.get("frame_id", frame_id),
            )
            backend = _ensure_backend()
            result = backend.motion.move_to_pose(
                lifted_tcp_pose,
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
                from_tcp_pose=base_tcp,
                height_m=height,
                lifted_tcp_pose=lifted_tcp_pose,
                result=result,
            )
        except Exception as exc:
            return _error(command, exc, height_m=height_m)


    @mcp.tool()
    @emit_skill_events
    def clear_octomap() -> dict[str, Any]:
        """Drop every voxel in MoveIt's occupancy map; it refills from the camera.

        Needed because octomap voxels are NOT covered by the Allowed Collision
        Matrix -- an ACM pair exempts object-vs-object contact, but nothing can
        exempt perception occupancy. Two cases where that bites:

        - After grasping a part that had been standing on the table, its own
          voxels remain and the next move fails START_STATE_IN_COLLISION
          against the space the attached part now occupies. Attaching does not
          clear them.
        - Before an insertion, the surrounding parts are in the scene twice --
          as synced collision objects (ACM-exempt) and as voxels (not
          exemptable) -- and the part has to enter the space those voxels fill.

        The map rebuilds from the next point clouds, so clear it immediately
        before the move that needs the space, not long in advance.
        """

        command = "clear_octomap"
        try:
            backend = _ensure_backend()
            return _ok(command, result=backend.scene.clear_octomap())
        except Exception as exc:
            return _error(command, exc)


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
