"""MCP server for scene-state tools: pose queries, scene auditing, and recovery.

Tools:
  get_object_pose      — TF pose of one registry object (planning frame).
  get_object_poses     — TF poses of every enabled registry entry, both
                         kind: object (physical bodies) and kind: frame
                         (pose-only placement/hole/reorient landmarks).
  get_object_spec      — one object's registry spec block (tolerances,
                         limits, grasp thresholds) plus cell frame names.
                         Read-only; does not carry grasp_width_m, which is
                         instruction-sourced (see the tool's docstring).
  reset_tf             — flush the TF buffer after an Isaac sim reset.
  get_scene_summary    — MoveIt's current beliefs: world + attached objects.
  sync_scene           — push TF-derived poses into MoveIt's collision scene.
  reset_object_state   — idempotent single-object recovery to WORLD_TRACKED.
  recover_scene        — reset all objects + clear octomap.
  begin_step           — open a WatchDog execution-step recording window.
                         Blocks on the cell's synchronization gate (see
                         robot_backend.cell.gate) after publishing its event,
                         so a monitor has a chance to arm before anything
                         moves. Fails open if no orchestrator is running.
  mark_phase           — record that a named mid-step phase (e.g. a grasp)
                         has been reached, and block on the gate again.
  end_step             — close the window and declare the claimed outcome.
  set_permitted_contacts — apply registry-declared acm_allowed_object_pairs
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
import re
import time
import sys
from pathlib import Path
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.scene_mcp_server")

import rclpy
from rclpy.node import Node

from robot_backend.cell.gate import request_go
from robot_backend.events.bus import configure_bus, shutdown_bus
from robot_backend.events.decorator import emit_skill_events
from robot_backend.registry import load_registry
from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.watchdog import session as session_helper
from robot_backend.skills.maintenance_skills import get_scene_summary as get_scene_summary_skill
from robot_backend.skills.maintenance_skills import recover_scene as recover_scene_skill
from robot_backend.skills.maintenance_skills import reset_object_state as reset_object_state_skill
from robot_backend.skills.maintenance_skills import set_permitted_contacts as set_permitted_contacts_skill
from robot_backend.mcp_servers import role_context


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
# Must precede the tool definitions below: it patches mcp.tool() so every tool
# registered afterwards records its call to mcp_calls_<role>.jsonl.
role_context.install_call_logging(mcp, "scene")
_NODE: Node | None = None
_BACKEND = None
_REGISTRY: dict[str, Any] | None = None
_POSE_CLIENT = None
_POSES_CLIENT = None
_RESET_TF_CLIENT = None
# Set by begin_step, read by mark_phase, cleared by end_step. mark_phase
# takes its step_id from here rather than from the caller, so a phase event
# can never be misattributed to the wrong step.
_CURRENT_STEP_ID: str | None = None


def _load_registry() -> dict[str, Any]:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        path = _DEFAULT_REGISTRY_PATH
    return load_registry(path).raw_dict


def _ensure_backend():
    global _NODE, _BACKEND, _REGISTRY, _POSE_CLIENT, _POSES_CLIENT, _RESET_TF_CLIENT
    if _BACKEND is not None:
        return _BACKEND
    if not rclpy.ok():
        rclpy.init(args=None)
    _NODE = Node("robot_backend_scene_mcp_server")
    _BACKEND = create_ros_moveit_backend(node=_NODE)
    # /watchdog/events (ROS topic) is the unified cross-process timeline; this
    # JSONL file is a per-process crash-survival artifact, not the merge point.
    log_path = os.environ.get(
        "WATCHDOG_EVENT_LOG",
        session_helper.event_log_default("scene"),
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
    global _NODE, _BACKEND, _REGISTRY, _POSE_CLIENT, _POSES_CLIENT, _RESET_TF_CLIENT, _CURRENT_STEP_ID
    shutdown_bus()
    _POSE_CLIENT = None
    _POSES_CLIENT = None
    _RESET_TF_CLIENT = None
    _REGISTRY = None
    _CURRENT_STEP_ID = None
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
    @emit_skill_events
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
    @emit_skill_events
    def get_object_poses(
        graspable_only: bool = False,
        kind: str | None = None,
    ) -> dict[str, Any]:
        """Return current TF poses for every enabled registry entry.

        This is the tool to call to find out what is actually on the table.
        It returns both kinds of entry the registry declares, each pose tagged
        with its "kind":

        - kind "object" -- a physical body with collision geometry. These are
          the parts and fixtures; the graspable ones are what you can pick.
        - kind "frame" -- a pose-only landmark with no body: placement frames,
          holes and reorient targets. Use these as motion TARGETS. They are
          not pickable and never appear in the planning scene, so you cannot
          collide with one and must not try to grasp one.

        Only entries with enabled: true in the registry are returned, so the
        listing reflects what the running stage publishes. An entry absent
        from the result is not on the table.

        Delegates to scene_tf_sync_node's /get_object_poses service. Failures
        for individual entries are collected in a 'failed' list rather than
        aborting the entire call.

        Parameters
        ----------
        graspable_only:
            When True, only returns entries whose registry entry has
            graspable: true. Implies kind "object" -- no frame is graspable.
        kind:
            "object" or "frame" to return just that kind. None (the default)
            returns both.
        """
        command = "get_object_poses"
        try:
            _VALID_KINDS = {"object", "frame"}
            if kind is not None and kind not in _VALID_KINDS:
                raise ValueError(
                    f"kind must be one of {sorted(_VALID_KINDS)} or omitted, got {kind!r}."
                )
            _ensure_backend()
            from robot_backend_interfaces.srv import GetObjectPoses
            req = GetObjectPoses.Request()
            req.graspable_only = graspable_only
            req.enabled_only = True
            req.kind = kind or ""
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
    def get_object_spec(object_id: str) -> dict[str, Any]:
        """Return one object's registry spec: tolerances, limits, grasp thresholds.

        This is the object-level truth the task file declines to author --
        static per-object reference data read straight from
        model_registry.yaml. It does not change as the run proceeds, so fetch
        an object's spec once and reuse it.

        It deliberately does NOT return grasp_width_m. That value is
        instruction-sourced, not registry-sourced: it lives in your task
        file's steps: block and reaches you through get_rendered_task. The
        registry has no grasp_width_m to give and no default to fall back on.

        cell carries cell-level (not per-object) joint and frame names --
        cell.gripper.effort_joint_name, cell.gripper.tcp_frame, and
        cell.base_frame for MoveIt's planning frame.

        acm_allowed_object_pairs lists object-object pairs whose collision
        checking is disabled in MoveIt's Allowed Collision Matrix so that
        intended contact (stacking, insertion) does not leave the planning
        scene in collision. It is a planner tractability setting, not a
        permission to touch those objects.
        """
        command = "get_object_spec"
        try:
            object_id = _validate_object_id(object_id)
            registry = _load_registry() if _REGISTRY is None else _REGISTRY
            objects = registry.get("objects", {})
            if not isinstance(objects, dict) or object_id not in objects:
                raise KeyError(
                    f"Object {object_id!r} not found in the registry. "
                    f"Known objects: {sorted(objects) if isinstance(objects, dict) else []}"
                )
            entry = objects[object_id]
            spec = entry.get("spec")
            if not isinstance(spec, dict):
                raise ValueError(
                    f"Object {object_id!r} has no 'spec' block in the registry, so "
                    "it carries no tolerances or limits."
                )
            acm_pairs = registry.get("acm_allowed_object_pairs", [])
            return _ok(
                command,
                object_id=object_id,
                spec=spec,
                graspable=bool(entry.get("graspable", False)),
                tf_frame=entry.get("tf_frame"),
                lookup_frame=entry.get("lookup_frame"),
                acm_allowed_object_pairs=acm_pairs if isinstance(acm_pairs, list) else [],
                cell=registry.get("cell", {}),
            )
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    @emit_skill_events
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
    @emit_skill_events
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
        object_ids: list[str] | None = None,
        notes: str = "",
    ) -> dict[str, Any]:
        """Publish a begin_step event to the WatchDog event bus, then wait for
        the cell's synchronization gate before returning.

        Declares the start of a named execution step and the objects in
        scope for it. After the event is published, blocks on the cell's
        request_go gate so a monitor has a chance to arm before anything
        moves -- this call may take a moment to return; that is the cell's
        interlock behaving normally, not an error. If no orchestrator is
        running, this returns promptly (gate result reason "no_orchestrator")
        and standalone runs are unaffected.

        Parameters
        ----------
        step_id:
            Caller-chosen identifier, unique within the session.
        object_ids:
            Registry object IDs in scope for this step.
        notes:
            Optional free-text annotation.
        """
        command = "begin_step"
        global _CURRENT_STEP_ID
        try:
            if not isinstance(step_id, str) or not step_id.strip():
                raise ValueError("step_id must be a non-empty string.")
            step_id = step_id.strip()

            _ensure_backend()
            _CURRENT_STEP_ID = step_id
            from robot_backend.events.bus import get_bus
            get_bus().emit_begin_step(step_id=step_id, object_ids=object_ids, notes=notes)
            # Order matters: the event must be published before we block, or
            # the monitor cannot yet know a step began.
            gate = request_go(_NODE, step_id)
            # An explicit refusal (proceed=False, e.g. reason="aborted") is the
            # cell interlock declining to release this step. Surface it as a
            # tool-level failure so the agent reasons about it rather than
            # acting as if the step was released. Every other outcome
            # (released, timeout, no_orchestrator, error) still proceeds --
            # the gate is fail-open by design.
            if gate.get("proceed") is False:
                return _error(
                    command,
                    "the cell declined to release this step",
                    step_id=step_id,
                    object_ids=object_ids or [],
                    gate=gate,
                )
            return _ok(command, step_id=step_id, object_ids=object_ids or [], gate=gate)
        except Exception as exc:
            return _error(command, exc, step_id=step_id)

    @mcp.tool()
    def end_step(
        step_id: str,
        claimed_outcome: str,
        claim_detail: dict[str, Any] | None = None,
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
        claim_detail:
            The quantities checked to reach claimed_outcome, e.g.
            {"final_width_m": 0.021, "width_tolerance_m": 0.002}. Lets the
            WatchDog compare its own evidence against what the executor
            actually looked at rather than only its conclusion.
        notes:
            Optional free-text annotation.
        """
        command = "end_step"
        _VALID_OUTCOMES = {"success", "failure", "partial"}
        global _CURRENT_STEP_ID
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
                claim_detail=claim_detail,
                notes=notes,
            )
            if _CURRENT_STEP_ID == step_id.strip():
                _CURRENT_STEP_ID = None
            return _ok(command, step_id=step_id.strip(), claimed_outcome=claimed_outcome)
        except Exception as exc:
            return _error(command, exc, step_id=step_id)

    @mcp.tool()
    def end_run(
        status: str,
        reason: str = "",
        detail: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Declare that you have stopped working on this run, and why.

        Call this EXACTLY ONCE, as the last thing you do, on EVERY path that
        ends your run -- you finished the assembly, you hit a fault you could
        not recover from, a part jammed, you gave up, or the cell declined to
        release a step. Call it even when the run went badly; especially then.

        Until this event arrives the WatchDog cannot tell "still working" from
        "stopped", and can only infer the end from silence -- which costs it a
        full no-event timeout cycle on every ending, clean ones included.

        This is a CLAIM about the whole run, exactly as end_step's
        claimed_outcome is a claim about one step. It tells the WatchDog you
        have stopped and is logged for later comparison against the WatchDog's
        own independent verdict. It does not decide that verdict: claiming
        "completed" does not make the run conformant, and "aborted" does not
        make it non-conformant. Report what actually happened.

        Parameters
        ----------
        status:
            "completed" -- you finished the assembly and claim it done.
            "aborted"   -- you stopped before completing, for any reason.
        reason:
            Short free text on why you stopped. For "aborted" say what
            defeated you, e.g. "cell declined step place_u_orange" or
            "no IK solution for hex_red insertion after 3 attempts".
        detail:
            Optional structured detail, e.g. which steps were completed.
        """
        command = "end_run"
        _VALID_STATUS = {"completed", "aborted"}
        global _CURRENT_STEP_ID
        try:
            if status not in _VALID_STATUS:
                raise ValueError(
                    f"status must be one of {sorted(_VALID_STATUS)}, got {status!r}."
                )
            _ensure_backend()
            from robot_backend.events.bus import get_bus
            from robot_backend.events.schema import RunEndedPayload, make_event
            event = make_event(
                "run_ended",
                RunEndedPayload(
                    status=status,  # type: ignore[arg-type]
                    reason=reason,
                    detail=detail,
                ),
            )
            get_bus().publish(event)
            # A run that ends mid-step leaves the step open. Clear it so a
            # later begin_step in the same process cannot be read as nested.
            _CURRENT_STEP_ID = None
            return _ok(command, status=status, reason=reason)
        except Exception as exc:
            return _error(command, exc, status=status)

    @mcp.tool()
    def mark_phase(phase: str, detail: dict[str, Any] | None = None) -> dict[str, Any]:
        """Record that a named phase of the current step has been reached.

        Use this when you establish a physical relationship the rest of the
        step depends on -- e.g. closing the gripper on an object -- so the
        cell can re-arm and release its synchronization gate for what
        follows. step_id is taken from the currently open step (the most
        recent begin_step not yet closed by end_step), not from this call's
        arguments, so a phase can never be misattributed to the wrong step.

        Publishes a phase event, then blocks on the cell's synchronization
        gate before returning, same as begin_step.

        Parameters
        ----------
        phase:
            Short descriptive name for the phase reached (e.g. "grasped").
        detail:
            Optional free-form detail about the phase.
        """
        command = "mark_phase"
        try:
            if not isinstance(phase, str) or not phase.strip():
                raise ValueError("phase must be a non-empty string.")
            phase = phase.strip()

            _ensure_backend()
            step_id = _CURRENT_STEP_ID
            if not step_id:
                raise ValueError(
                    "mark_phase called with no open step -- call begin_step first."
                )

            from robot_backend.events.bus import get_bus
            from robot_backend.events.schema import PhasePayload, make_event
            event = make_event(
                "phase", PhasePayload(step_id=step_id, phase=phase, detail=detail)
            )
            get_bus().publish(event)
            gate = request_go(_NODE, step_id, phase=phase)
            # See begin_step: an explicit proceed=False refusal surfaces as a
            # tool-level failure; every other outcome proceeds (fail-open).
            if gate.get("proceed") is False:
                return _error(
                    command,
                    "the cell declined to release this step",
                    step_id=step_id,
                    phase=phase,
                    gate=gate,
                )
            return _ok(command, step_id=step_id, phase=phase, gate=gate)
        except Exception as exc:
            return _error(command, exc, phase=phase)

    @mcp.tool()
    def set_permitted_contacts() -> dict[str, Any]:
        """Apply the registry's declared acm_allowed_object_pairs to MoveIt's
        Allowed Collision Matrix.

        Reads acm_allowed_object_pairs from model_registry.yaml and
        persistently allows each declared object pair to be in contact
        without being flagged as a planning-time collision. Does not affect
        any other collision pair. This is a planner tractability setting; it
        carries no task-level permission about what the robot may touch.
        """
        command = "set_permitted_contacts"
        try:
            backend = _ensure_backend()
            result = set_permitted_contacts_skill(backend, registry=_REGISTRY or {})
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)


def main() -> None:
    """Run the scene MCP server.

    Eagerly constructs the backend/node/event bus before mcp.run() rather
    than waiting for the first tool call. The runner spawns all three
    executor server subprocesses ~40s before the first LLM turn lands (that
    turn is what triggers the first tool call), so doing this here lets DDS
    discovery of /watchdog/events complete during that idle window instead of
    paying the bounded subscriber wait on the first real publish.

    Non-fatal: if the simulator/ROS graph isn't up yet, this logs a warning
    and falls through to mcp.run() unchanged. The lazy _ensure_backend() path
    inside each tool remains the fallback, exactly as before -- this must
    never make --dry-run (which is expected to work with no simulator
    running) fail to start.
    """
    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run scene_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        _ensure_backend()
    except Exception as exc:
        print(
            f"WARNING: scene_mcp_server eager init failed ({exc}); "
            "continuing -- the first tool call will retry initialization.",
            file=sys.stderr,
        )

    try:
        mcp.run()
    finally:
        _shutdown_backend()


if __name__ == "__main__":
    main()
