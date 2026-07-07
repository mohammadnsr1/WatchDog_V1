"""MCP server for scene-state tools: pose queries, scene auditing, and recovery.

Tools:
  get_object_pose      — TF pose of one registry object (planning frame).
  get_object_poses     — TF poses of all enabled registry objects.
  reset_tf             — flush the TF buffer after an Isaac sim reset.
  get_scene_summary    — MoveIt's current beliefs: world + attached objects.
  sync_scene           — push TF-derived poses into MoveIt's collision scene.
  reset_object_state   — idempotent single-object recovery to WORLD_TRACKED.
  recover_scene        — reset all objects + clear octomap.
  begin_step           — open a WatchDog execution-step recording window.
  end_step             — close the window and declare the claimed outcome.
  set_permitted_contacts — apply registry-declared permitted-contact pairs
                           to MoveIt's Allowed Collision Matrix.

Process pattern (same as gripper_mcp_server):
  - Lazy rclpy.init on first tool call.
  - Single shared _NODE + _BACKEND for the process lifetime.
  - Pose queries are thin ROS service clients to /get_object_pose and
    /get_object_poses served by scene_tf_sync_node (the sole TF buffer owner).
  - atexit-registered shutdown closes the node cleanly.
  - Event bus configured on startup (ROS publisher + JSONL log).
"""

from __future__ import annotations

import atexit
import json
import os
from pathlib import Path
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.scene_mcp_server")

import yaml

import rclpy
from rclpy.node import Node

from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.maintenance_skills import get_scene_summary as get_scene_summary_skill
from robot_backend.skills.maintenance_skills import recover_scene as recover_scene_skill
from robot_backend.skills.maintenance_skills import reset_object_state as reset_object_state_skill
from robot_backend.skills.maintenance_skills import set_permitted_contacts as set_permitted_contacts_skill


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

mcp = FastMCP("robot_backend_scene") if FastMCP is not None else None
_NODE: Node | None = None
_BACKEND = None
_REGISTRY: dict[str, Any] | None = None
_POSE_CLIENT = None
_POSES_CLIENT = None
_RESET_TF_CLIENT = None


def _load_registry() -> dict[str, Any]:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        path = _DEFAULT_REGISTRY_PATH
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _ensure_backend():
    global _NODE, _BACKEND, _REGISTRY, _POSE_CLIENT, _POSES_CLIENT, _RESET_TF_CLIENT
    if _BACKEND is not None:
        return _BACKEND
    if not rclpy.ok():
        rclpy.init(args=None)
    _NODE = Node("robot_backend_scene_mcp_server")
    _BACKEND = create_ros_moveit_backend(node=_NODE)
    log_path = os.environ.get(
        "WATCHDOG_EVENT_LOG",
        str(Path.home() / ".watchdog" / "events.jsonl"),
    )
    configure_bus(node=_NODE, log_path=log_path)
    if _REGISTRY is None:
        _REGISTRY = _load_registry()

    from robot_backend_interfaces.srv import GetObjectPose, GetObjectPoses
    from std_srvs.srv import Trigger
    _POSE_CLIENT = _NODE.create_client(GetObjectPose, '/get_object_pose')
    _POSES_CLIENT = _NODE.create_client(GetObjectPoses, '/get_object_poses')
    _RESET_TF_CLIENT = _NODE.create_client(Trigger, '/reset_tf')

    return _BACKEND


def _call_service(client, request, service_name: str, timeout_sec: float = 5.0) -> Any:
    """Call a service synchronously using spin_until_future_complete on _NODE."""
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(
            f"Service {service_name} not available within {timeout_sec:.1f}s. "
            "Is scene_tf_sync_node running?"
        )
    future = client.call_async(request)
    rclpy.spin_until_future_complete(_NODE, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError(
            f"Timed out waiting for {service_name} after {timeout_sec:.1f}s."
        )
    result = future.result()
    if result is None:
        raise RuntimeError(f"Service {service_name} returned no result.")
    return result


def _shutdown_backend() -> None:
    global _NODE, _BACKEND, _REGISTRY, _POSE_CLIENT, _POSES_CLIENT, _RESET_TF_CLIENT
    shutdown_bus()
    _POSE_CLIENT = None
    _POSES_CLIENT = None
    _RESET_TF_CLIENT = None
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


def _validate_object_id(object_id: Any) -> str:
    if not isinstance(object_id, str) or not object_id.strip():
        raise ValueError("object_id must be a non-empty string.")
    return object_id.strip()


if mcp is not None:

    # ------------------------------------------------------------------
    # Pose queries — thin clients of scene_tf_sync_node's services.
    # TF is owned exclusively by that node; no Buffer lives here.
    # ------------------------------------------------------------------

    @mcp.tool()
    def get_object_pose(object_id: str) -> dict[str, Any]:
        """Return the current TF pose of a registry object in its lookup frame.

        Delegates to scene_tf_sync_node's /get_object_pose service, which
        holds the single executor-side TF buffer. Does not read or write
        MoveIt's planning scene.

        Parameters
        ----------
        object_id:
            Registry object ID (e.g. "red_cube").
        """
        command = "get_object_pose"
        try:
            oid = _validate_object_id(object_id)
            _ensure_backend()
            from robot_backend_interfaces.srv import GetObjectPose
            req = GetObjectPose.Request()
            req.object_id = oid
            resp = _call_service(_POSE_CLIENT, req, '/get_object_pose')
            if not resp.ok:
                return {"ok": False, "command": command, "object_id": oid, "error": resp.error}
            p = resp.pose
            return {
                "ok": True,
                "command": command,
                "object_id": resp.object_id,
                "frame_id": resp.frame_id,
                "tf_frame": resp.tf_frame,
                "pose": {
                    "position": {
                        "x": float(p.position.x),
                        "y": float(p.position.y),
                        "z": float(p.position.z),
                    },
                    "orientation": {
                        "x": float(p.orientation.x),
                        "y": float(p.orientation.y),
                        "z": float(p.orientation.z),
                        "w": float(p.orientation.w),
                    },
                },
            }
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    def get_object_poses(graspable_only: bool = False) -> dict[str, Any]:
        """Return current TF poses for all enabled registry objects.

        Delegates to scene_tf_sync_node's /get_object_poses service. Failures
        for individual objects are collected in a 'failed' list rather than
        aborting the entire call.

        Parameters
        ----------
        graspable_only:
            When True, only returns objects whose registry entry has
            graspable: true.
        """
        command = "get_object_poses"
        try:
            _ensure_backend()
            from robot_backend_interfaces.srv import GetObjectPoses
            req = GetObjectPoses.Request()
            req.graspable_only = graspable_only
            req.enabled_only = True
            resp = _call_service(_POSES_CLIENT, req, '/get_object_poses')
            poses = json.loads(resp.poses_json) if resp.poses_json else []
            failed = json.loads(resp.failed_json) if resp.failed_json else []
            return {
                "ok": resp.ok,
                "command": command,
                "poses": poses,
                "failed": failed,
            }
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def reset_tf() -> dict[str, Any]:
        """Flush and recreate the TF buffer in scene_tf_sync_node.

        Call this after an Isaac Sim reset to discard stale pre-reset
        transforms. The node's TF buffer is destroyed and recreated so
        subsequent pose queries reflect the new simulation state.
        """
        command = "reset_tf"
        try:
            _ensure_backend()
            from std_srvs.srv import Trigger
            resp = _call_service(_RESET_TF_CLIENT, Trigger.Request(), '/reset_tf')
            if not resp.success:
                return _error(command, resp.message)
            return _ok(command, message=resp.message)
        except Exception as exc:
            return _error(command, exc)

    # ------------------------------------------------------------------
    # Scene state — MoveIt's planning scene beliefs.
    # ------------------------------------------------------------------

    @mcp.tool()
    def get_scene_summary() -> dict[str, Any]:
        """Return a read-only audit of MoveIt's current planning scene.

        Reports world collision objects (TF-tracked) and attached objects
        (held by the robot), with convenience counts. Does not modify state.
        """
        command = "get_scene_summary"
        try:
            backend = _ensure_backend()
            result = get_scene_summary_skill(backend)
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def sync_scene(object_ids: list[str] | None = None) -> dict[str, Any]:
        """Push TF-derived object poses into MoveIt's collision scene.

        Reads the current TF pose for each object and upserts it as a
        CollisionObject in MoveIt's world scene. With no arguments, syncs
        all enabled registry objects. Pass object_ids to sync a subset.

        Parameters
        ----------
        object_ids:
            Registry object IDs to sync. Syncs all enabled objects when None.
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

    # ------------------------------------------------------------------
    # Scene recovery — repair inconsistent planning scene state.
    # ------------------------------------------------------------------

    @mcp.tool()
    def reset_object_state(object_id: str) -> dict[str, Any]:
        """Reset one object to WORLD_TRACKED state, idempotently.

        Queries MoveIt's current scene to determine whether the object is
        attached. If it is, detaches it first. Then syncs the object's
        TF-based pose back into the world collision scene.

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

        Iterates every enabled object in the model registry, detaches any that
        are attached, and syncs each back to its current TF pose. Clears the
        octomap afterward to flush stale perception voxels.
        """
        command = "recover_scene"
        try:
            backend = _ensure_backend()
            result = recover_scene_skill(backend, registry=_REGISTRY)
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)

    # ------------------------------------------------------------------
    # Step instrumentation — WatchDog execution-window management.
    # ------------------------------------------------------------------

    @mcp.tool()
    def begin_step(
        step_id: str,
        object_id: str | None = None,
    ) -> dict[str, Any]:
        """Publish a begin_step event to the WatchDog event bus.

        Opens a sensor-recording window keyed by step_id. All skill_start /
        skill_end events that follow (until the matching end_step) are within
        this window. When object_id is provided the corresponding PickContract
        is loaded from the registry spec and embedded in the event payload.

        Parameters
        ----------
        step_id:
            Caller-chosen identifier, unique within the session.
        object_id:
            Registry object ID. When provided, its PickContract is attached to
            the event for downstream conformance checking.
        """
        command = "begin_step"
        try:
            if not isinstance(step_id, str) or not step_id.strip():
                raise ValueError("step_id must be a non-empty string.")
            step_id = step_id.strip()

            contract_dict = None
            if object_id is not None:
                import json as _json
                from robot_backend.contracts.loader import load_pick_contract
                from robot_backend.contracts.schema import to_json
                contract = load_pick_contract(_validate_object_id(object_id))
                contract_dict = _json.loads(to_json(contract))

            _ensure_backend()
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
        """Publish an end_step event and declare the claimed outcome.

        Closes the sensor-recording window opened by the matching begin_step.
        The WatchDog compares claimed_outcome against independent sensor
        observations from the recording window.

        Parameters
        ----------
        step_id:
            Must match the step_id from the corresponding begin_step call.
        claimed_outcome:
            Self-assessed result: "success", "failure", or "partial".
        notes:
            Optional free-text annotation.
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
            _ensure_backend()
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
    def set_permitted_contacts() -> dict[str, Any]:
        """Apply the registry's declared permitted-contact pairs to MoveIt's
        Allowed Collision Matrix.

        Reads permitted_contacts from model_registry.yaml and persistently
        allows each declared object pair to be in contact without being
        flagged as a planning-time collision. Does not affect any other
        collision pair.
        """
        command = "set_permitted_contacts"
        try:
            backend = _ensure_backend()
            result = set_permitted_contacts_skill(backend, registry=_REGISTRY or {})
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)


def main() -> None:
    """Run the scene MCP server."""
    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run scene_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
