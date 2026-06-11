"""MCP server for pick-and-place skills.

Flow:
Agent / MCP client
    -> pick_place MCP tool
    -> pick_object / place_object skill
    -> RobotBackend (scene + motion + gripper)
    -> MoveIt + Isaac trajectory bridge

All pose parameters use GRASP CENTER coordinates, not raw TCP coordinates.
The server converts internally using the RG2 calibration:
  - pregrasp / preplace: grasp-center -> TCP at gripper-open width
  - grasp / place:       grasp-center -> TCP at the specified grasp_width_m
  - retreat (after pick): grasp-center -> TCP at grasp_width_m (gripper closed)
  - retreat (after place): grasp-center -> TCP at open width (gripper open)

Typical agent workflow:
  1. sync_scene()               -- refresh planning scene from TF
  2. get_scene_objects()        -- inspect what is in the scene
  3. pick_object(...)           -- full grasp sequence
  4. pick_object / place_object -- additional operations as needed
"""

from __future__ import annotations

import atexit
import math
from pathlib import Path
from typing import Any

import yaml

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.pick_place_mcp_server")

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.pick_place_skills import pick_object as pick_object_skill
from robot_backend.skills.pick_place_skills import place_object as place_object_skill
from robot_backend.skills.pose_query_skills import get_object_pose as get_object_pose_skill
from robot_backend.skills.pose_query_skills import get_object_poses as get_object_poses_skill
from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION
from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center


try:
    from mcp.server.fastmcp import FastMCP
except ImportError as exc:
    FastMCP = None  # type: ignore[assignment]
    _MCP_IMPORT_ERROR = exc
else:
    _MCP_IMPORT_ERROR = None


_MIN_QUATERNION_NORM = 1e-8

# Default registry path: resolved relative to this file so no ament_index call is
# needed at import time (ament may not be sourced when the linter runs).
_DEFAULT_REGISTRY_PATH = Path(__file__).resolve().parents[2] / "config" / "model_registry.yaml"

mcp = FastMCP("robot_backend_pick_place") if FastMCP is not None else None
_NODE: Node | None = None
_BACKEND = None
# TF context: separate lightweight node so its spin_thread never races with
# the service-call spin in the main backend node.
_TF_NODE: Node | None = None
_TF_BUFFER: Buffer | None = None
_TF_LISTENER: TransformListener | None = None
_REGISTRY: dict[str, Any] | None = None


def _ensure_backend():
    global _NODE, _BACKEND
    if _BACKEND is not None:
        return _BACKEND
    if not rclpy.ok():
        rclpy.init(args=None)
    _NODE = Node("robot_backend_pick_place_mcp_server")
    _BACKEND = create_ros_moveit_backend(node=_NODE)
    # Initialise event bus with this node (ROS publisher) and the default log path.
    import os
    log_path = os.environ.get(
        "WATCHDOG_EVENT_LOG",
        str(Path.home() / ".watchdog" / "events.jsonl"),
    )
    configure_bus(node=_NODE, log_path=log_path)
    return _BACKEND


def _load_registry() -> dict[str, Any]:
    """Load model_registry.yaml, trying ament share dir then source-relative path."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        path = _DEFAULT_REGISTRY_PATH
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _ensure_tf_context() -> tuple[Buffer, dict[str, Any]]:
    """Lazy-init the TF buffer/listener and registry cache.

    Uses a dedicated TF node with spin_thread=True so transforms are received
    continuously without interfering with the service-call spin on _NODE.
    """
    global _TF_NODE, _TF_BUFFER, _TF_LISTENER, _REGISTRY
    _ensure_backend()  # guarantees rclpy.init() has been called
    if _TF_BUFFER is None:
        _TF_NODE = Node("robot_backend_pick_place_tf_client")
        _TF_BUFFER = Buffer()
        _TF_LISTENER = TransformListener(_TF_BUFFER, _TF_NODE, spin_thread=True)
    if _REGISTRY is None:
        _REGISTRY = _load_registry()
    return _TF_BUFFER, _REGISTRY


def _shutdown_backend() -> None:
    global _NODE, _BACKEND, _TF_NODE, _TF_BUFFER, _TF_LISTENER, _REGISTRY
    shutdown_bus()
    if _TF_NODE is not None:
        _TF_NODE.destroy_node()
        _TF_NODE = None
    _TF_BUFFER = None
    _TF_LISTENER = None
    _REGISTRY = None
    if _NODE is not None:
        _NODE.destroy_node()
        _NODE = None
    _BACKEND = None
    if rclpy.ok():
        rclpy.shutdown()


atexit.register(_shutdown_backend)


def _ok(command: str, **payload: Any) -> dict[str, Any]:
    return {"ok": True, "command": command, **payload}


def _error(command: str, error: Exception | str, **payload: Any) -> dict[str, Any]:
    return {"ok": False, "command": command, "error": str(error), **payload}


def _validate_float(name: str, value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric, not boolean.")
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric.") from exc
    if not math.isfinite(numeric):
        raise ValueError(f"{name} must be finite.")
    return numeric


def _validate_object_id(object_id: Any) -> str:
    if not isinstance(object_id, str) or not object_id.strip():
        raise ValueError("object_id must be a non-empty string.")
    return object_id.strip()


def _validate_width(name: str, width_m: Any) -> float:
    width = _validate_float(name, width_m)
    if width < 0.0:
        raise ValueError(f"{name} must be non-negative.")
    if width > DEFAULT_RG2_CALIBRATION.width_open_m:
        raise ValueError(
            f"{name} exceeds the calibrated RG2 maximum opening of "
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
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a non-empty string.")
    px = _validate_float("x", x)
    py = _validate_float("y", y)
    pz = _validate_float("z", z)
    ox = _validate_float("qx", qx)
    oy = _validate_float("qy", qy)
    oz = _validate_float("qz", qz)
    ow = _validate_float("qw", qw)
    if math.sqrt(ox * ox + oy * oy + oz * oz + ow * ow) <= _MIN_QUATERNION_NORM:
        raise ValueError("Quaternion norm is too close to zero.")
    return {
        "frame_id": frame_id.strip(),
        "position": {"x": px, "y": py, "z": pz},
        "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
    }


def _build_offset_pose(
    *,
    frame_id: str,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> dict[str, Any]:
    """Build a grasp-center pose reusing a pre-validated orientation."""
    return {
        "frame_id": frame_id,
        "position": {"x": x, "y": y, "z": z},
        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
    }


def _require_all_or_none(values: dict[str, float | None], group: str) -> bool:
    """Return True if all values are set, False if all are None, raise if mixed."""
    provided = [v for v in values.values() if v is not None]
    if len(provided) == 0:
        return False
    if len(provided) == len(values):
        return True
    missing = [k for k, v in values.items() if v is None]
    raise ValueError(
        f"{group}: all three coordinates must be provided together. Missing: {missing}"
    )


if mcp is not None:

    @mcp.tool()
    def get_object_pose(object_id: str) -> dict[str, Any]:
        """Return the current TF pose of a registry object in world coordinates.

        Queries TF directly — this is the authoritative real-time pose of the
        object, not MoveIt's cached copy. Call this to locate an object before
        constructing pick coordinates. Does NOT modify the planning scene.

        Parameters
        ----------
        object_id:
            Registry object ID (e.g. "red_cube").
        """
        command = "get_object_pose"
        try:
            oid = _validate_object_id(object_id)
            tf_buffer, registry = _ensure_tf_context()
            return get_object_pose_skill(tf_buffer, registry, oid)
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    def get_object_poses(graspable_only: bool = False) -> dict[str, Any]:
        """Return current TF poses for all enabled registry objects.

        Queries TF for every enabled object in the model registry. Use this
        to survey the scene before planning a pick sequence. Each entry in the
        returned poses list has the same shape as get_object_pose.

        Parameters
        ----------
        graspable_only:
            If True, only return objects with graspable: true in the registry
            (useful when the executor only cares about pickable targets).
        """
        command = "get_object_poses"
        try:
            tf_buffer, registry = _ensure_tf_context()
            return get_object_poses_skill(tf_buffer, registry, graspable_only=graspable_only)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def sync_scene(object_ids: list[str] | None = None) -> dict[str, Any]:
        """Sync collision objects from TF + model registry into MoveIt's planning scene.

        Call this before any pick operation to ensure MoveIt has the latest
        object poses from Isaac Sim. With no arguments, syncs all enabled
        registry objects. Pass a list of object IDs to sync only those.
        """
        command = "sync_scene"
        try:
            backend = _ensure_backend()
            result = backend.scene.sync_collision_objects(
                object_ids=object_ids if object_ids else None
            )
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def get_scene_objects() -> dict[str, Any]:
        """Return all world collision objects and attached objects from MoveIt.

        Use this to inspect the current planning scene before picking or placing.
        World objects are tracked by TF. Attached objects are held by the robot.
        """
        command = "get_scene_objects"
        try:
            backend = _ensure_backend()
            result = backend.scene.get_objects()
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def begin_step(
        step_id: str,
        object_id: str | None = None,
    ) -> dict[str, Any]:
        """Record the start of an execution step and attach an optional contract.

        Publishes a begin_step event to the WatchDog event bus. The WatchDog
        uses the begin_step timestamp to open a sensor-recording window. All
        skill_start / skill_end events that follow (until end_step) are within
        this step's window.

        Parameters
        ----------
        step_id:
            Unique identifier chosen by the agent (e.g. "pick_red_cube_0").
            Must be unique within a session to allow correct begin/end pairing.
        object_id:
            Registry object ID. When provided the corresponding PickContract is
            loaded from the registry spec and serialized into the event payload,
            binding the spec snapshot to this step's timestamp.
        """
        command = "begin_step"
        try:
            if not isinstance(step_id, str) or not step_id.strip():
                raise ValueError("step_id must be a non-empty string.")
            step_id = step_id.strip()

            contract_dict = None
            if object_id is not None:
                from robot_backend.contracts.loader import load_pick_contract
                from robot_backend.contracts.schema import to_json
                import json
                contract = load_pick_contract(_validate_object_id(object_id))
                contract_dict = json.loads(to_json(contract))

            from robot_backend.events.bus import get_bus
            get_bus().emit_begin_step(step_id=step_id, contract=contract_dict)
            return _ok(command, step_id=step_id, contract_attached=contract_dict is not None)
        except Exception as exc:
            return _error(command, exc, step_id=step_id)

    @mcp.tool()
    def end_step(
        step_id: str,
        claimed_outcome: str,
        notes: str = "",
    ) -> dict[str, Any]:
        """Record the end of an execution step and declare the claimed outcome.

        Publishes an end_step event. The WatchDog closes the sensor-recording
        window at this timestamp and compares the claimed_outcome against
        independent observations from the recording.

        Parameters
        ----------
        step_id:
            Must match the step_id used in the corresponding begin_step call.
        claimed_outcome:
            The agent's self-assessed result: "success", "failure", or "partial".
            This is a CLAIM, not a verified result — the WatchDog checks it.
        notes:
            Optional free-text annotation (reason for failure, etc.).
        """
        command = "end_step"
        _VALID_OUTCOMES = {"success", "failure", "partial"}
        try:
            if not isinstance(step_id, str) or not step_id.strip():
                raise ValueError("step_id must be a non-empty string.")
            if claimed_outcome not in _VALID_OUTCOMES:
                raise ValueError(
                    f"claimed_outcome must be one of {sorted(_VALID_OUTCOMES)}, "
                    f"got {claimed_outcome!r}."
                )
            from robot_backend.events.bus import get_bus
            get_bus().emit_end_step(
                step_id=step_id.strip(),
                claimed_outcome=claimed_outcome,
                notes=notes,
            )
            return _ok(command, step_id=step_id.strip(), claimed_outcome=claimed_outcome)
        except Exception as exc:
            return _error(command, exc, step_id=step_id)

    @mcp.tool()
    def pick_object(
        object_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        grasp_width_m: float,
        grasp_width_source: str = "agent_estimate",
        frame_id: str = "world",
        pregrasp_x: float | None = None,
        pregrasp_y: float | None = None,
        pregrasp_z: float | None = None,
        retreat_x: float | None = None,
        retreat_y: float | None = None,
        retreat_z: float | None = None,
    ) -> dict[str, Any]:
        """Pick an object by closing the gripper at a grasp center pose.

        Internally this skill:
          1. Syncs the object's TF pose into MoveIt.
          2. Captures the object's resting TF pose as a reference.
          3. Moves to pregrasp position (if provided), gripper open.
          4. Opens gripper to full width.
          5. Moves to the grasp center pose.
          6. Closes the gripper.
          7. Attaches the object to the robot (removes it from world scene).
          8. Lifts to the retreat position (or a minimal 5 cm lift if no retreat).
          9. Verifies the grasp: compares object TF pose to reference and TCP.
             On failure the attachment is reversed and ok=False with grasp_failed=True
             is returned. On success the object remains attached.
         10. Moves to retreat position (if the lift in step 8 was not already the retreat).

        All positions are in grasp-center coordinates (logical grip point),
        not raw TCP coordinates. The server handles the RG2 TCP offset internally.

        Parameters
        ----------
        object_id:
            Registry object ID to pick (e.g. "red_cube").
        x, y, z:
            Grasp center position in the specified frame.
        qx, qy, qz, qw:
            Grasp center orientation quaternion.
        grasp_width_m:
            Gripper opening width at the grasp point, in meters.
        grasp_width_source:
            Provenance of grasp_width_m: "registry_spec" if the value was taken
            from model_registry.yaml spec.grasp.grasp_width_m, "agent_estimate"
            if the agent chose a value independently. A declared estimate is
            auditable — the WatchDog can compare it against the registry constant.
        frame_id:
            Reference frame for all poses (default: "world").
        pregrasp_x/y/z:
            Optional approach position. Provide all three or none.
            The gripper will be fully open when moving to this position.
        retreat_x/y/z:
            Optional retreat position after grasp. Provide all three or none.
            The gripper will be at grasp_width_m when moving here.
        """
        command = "pick_object"
        _VALID_SOURCES = {"registry_spec", "agent_estimate"}
        try:
            oid = _validate_object_id(object_id)
            if grasp_width_source not in _VALID_SOURCES:
                raise ValueError(
                    f"grasp_width_source must be one of {sorted(_VALID_SOURCES)}, "
                    f"got {grasp_width_source!r}."
                )
            grasp_center = _build_pose_dict(
                frame_id=frame_id, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw
            )
            width = _validate_width("grasp_width_m", grasp_width_m)

            # Pregrasp: grasp-center coords at open width -> TCP
            pregrasp_pose = None
            if _require_all_or_none(
                {"pregrasp_x": pregrasp_x, "pregrasp_y": pregrasp_y, "pregrasp_z": pregrasp_z},
                "pregrasp",
            ):
                pregrasp_center = _build_offset_pose(
                    frame_id=frame_id,
                    x=pregrasp_x, y=pregrasp_y, z=pregrasp_z,
                    qx=qx, qy=qy, qz=qz, qw=qw,
                )
                pregrasp_pose = tcp_target_from_grasp_center(
                    pregrasp_center, DEFAULT_RG2_CALIBRATION.width_open_m
                )

            # Retreat: grasp-center coords at grasp width -> TCP (gripper closed)
            retreat_pose = None
            if _require_all_or_none(
                {"retreat_x": retreat_x, "retreat_y": retreat_y, "retreat_z": retreat_z},
                "retreat",
            ):
                retreat_center = _build_offset_pose(
                    frame_id=frame_id,
                    x=retreat_x, y=retreat_y, z=retreat_z,
                    qx=qx, qy=qy, qz=qz, qw=qw,
                )
                retreat_pose = tcp_target_from_grasp_center(retreat_center, width)

            backend = _ensure_backend()
            try:
                tf_buffer, registry = _ensure_tf_context()
            except Exception:
                tf_buffer, registry = None, None
            result = pick_object_skill(
                backend,
                object_id=oid,
                grasp_center_pose=grasp_center,
                grasp_width_m=width,
                grasp_width_source=grasp_width_source,
                pregrasp_pose=pregrasp_pose,
                retreat_pose=retreat_pose,
                tf_buffer=tf_buffer,
                registry=registry,
            )
            return _ok(command, object_id=oid, result=result)
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    def place_object(
        object_id: str,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        frame_id: str = "world",
        release_width_m: float | None = None,
        preplace_x: float | None = None,
        preplace_y: float | None = None,
        preplace_z: float | None = None,
        retreat_x: float | None = None,
        retreat_y: float | None = None,
        retreat_z: float | None = None,
    ) -> dict[str, Any]:
        """Place an attached object by releasing the gripper at a target pose.

        Internally this skill:
          1. Moves to preplace position (if provided).
          2. Moves to the place pose.
          3. Opens the gripper.
          4. Detaches the object from the robot.
          5. Syncs the object's new TF pose back into MoveIt (world scene).
          6. Moves to retreat position (if provided).

        All positions are in grasp-center coordinates. The server handles
        the RG2 TCP offset internally.

        Parameters
        ----------
        object_id:
            Registry object ID to place (e.g. "red_cube").
        x, y, z:
            Place grasp-center position in the specified frame.
        qx, qy, qz, qw:
            Place orientation quaternion.
        frame_id:
            Reference frame for all poses (default: "world").
        release_width_m:
            Gripper opening width during release (default: fully open at 0.104 m).
        preplace_x/y/z:
            Optional pre-place approach position. Provide all three or none.
        retreat_x/y/z:
            Optional retreat position after release. Provide all three or none.
            The gripper will be fully open when moving here.
        """
        command = "place_object"
        try:
            oid = _validate_object_id(object_id)
            place_width = (
                _validate_width("release_width_m", release_width_m)
                if release_width_m is not None
                else DEFAULT_RG2_CALIBRATION.width_open_m
            )
            place_center = _build_pose_dict(
                frame_id=frame_id, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw
            )

            # Preplace: grasp-center -> TCP at place width
            preplace_pose = None
            if _require_all_or_none(
                {"preplace_x": preplace_x, "preplace_y": preplace_y, "preplace_z": preplace_z},
                "preplace",
            ):
                preplace_center = _build_offset_pose(
                    frame_id=frame_id,
                    x=preplace_x, y=preplace_y, z=preplace_z,
                    qx=qx, qy=qy, qz=qz, qw=qw,
                )
                preplace_pose = tcp_target_from_grasp_center(preplace_center, place_width)

            # Retreat: grasp-center -> TCP at open width (gripper released)
            retreat_pose = None
            if _require_all_or_none(
                {"retreat_x": retreat_x, "retreat_y": retreat_y, "retreat_z": retreat_z},
                "retreat",
            ):
                retreat_center = _build_offset_pose(
                    frame_id=frame_id,
                    x=retreat_x, y=retreat_y, z=retreat_z,
                    qx=qx, qy=qy, qz=qz, qw=qw,
                )
                retreat_pose = tcp_target_from_grasp_center(
                    retreat_center, DEFAULT_RG2_CALIBRATION.width_open_m
                )

            backend = _ensure_backend()
            result = place_object_skill(
                backend,
                object_id=oid,
                place_grasp_center_pose=place_center,
                place_width_m=place_width,
                preplace_pose=preplace_pose,
                retreat_pose=retreat_pose,
            )
            return _ok(command, object_id=oid, result=result)
        except Exception as exc:
            return _error(command, exc, object_id=object_id)


def main() -> None:
    """Run the pick-and-place MCP server."""

    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run pick_place_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
