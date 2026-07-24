"""MCP server for WatchDog recorder and monitor tools."""

from __future__ import annotations

import atexit
import os
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.watchdog_mcp_server")

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_backend.contracts.loader import load_pick_contract
from robot_backend.events.bus import EventBus
from robot_backend.events.schema import (
    KNOWN_EVENT_TYPES,
    VerdictPayload,
    available_event_type_definitions,
    make_event,
)
from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.halt import HaltClient
from robot_backend.watchdog.monitors import MONITOR_TYPES, monitor_definitions
from robot_backend.watchdog.ready import ReadyClient
from robot_backend.watchdog.recorder import EvidenceRecorder, _load_registry, event_wrapper
from robot_backend.watchdog.timebase import wall_ns

try:
    from mcp.server.fastmcp import FastMCP
except ImportError as exc:
    FastMCP = None  # type: ignore[assignment]
    _MCP_IMPORT_ERROR = exc
else:
    _MCP_IMPORT_ERROR = None


mcp = FastMCP("robot_backend_watchdog") if FastMCP is not None else None


@dataclass
class WatchDogContext:
    node: Node
    executor: MultiThreadedExecutor
    thread: threading.Thread
    registry: dict[str, Any]
    event_bus: EventBus
    recorder: EvidenceRecorder
    engine: MonitorEngine
    halt_client: HaltClient
    ready_client: ReadyClient
    wake_event: threading.Event


_CONTEXT: WatchDogContext | None = None
_CONTEXT_LOCK = threading.Lock()
_VALID_VERDICTS = {"conformant", "non_conformant", "valid_variation", "inconclusive"}
# "<section>.<field>", matching how the loader derives clause_id in
# contracts/loader.py's _make_clause -- e.g. "effects.object_lift_height_m".
_CLAUSE_ID_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*\.[A-Za-z_][A-Za-z0-9_]*$")


def _validate_clause_results(clause_results: list[dict[str, Any]]) -> None:
    """Shape-check clause_results for record_verdict.

    Every entry must carry a well-formed clause_id, or clause_id: null paired
    with source: "instruction" for an instruction-level requirement with no
    contract counterpart. Does not check clause_id against the actual
    contract -- this server does not know which object a step concerned, and
    is not the place to look it up just to validate a string.
    """
    for i, entry in enumerate(clause_results):
        if not isinstance(entry, dict):
            raise ValueError(
                f"clause_results[{i}] must be an object, got {type(entry).__name__}."
            )
        clause_id = entry.get("clause_id")
        if clause_id is None:
            if entry.get("source") != "instruction":
                raise ValueError(
                    f"clause_results[{i}] has clause_id: null but no "
                    f'source: "instruction" -- null clause_id is reserved for '
                    f"instruction-level requirements that have no contract clause."
                )
            continue
        if not isinstance(clause_id, str) or not _CLAUSE_ID_RE.match(clause_id):
            raise ValueError(
                f"clause_results[{i}] has a malformed clause_id {clause_id!r}; "
                f'expected "<section>.<field>", e.g. "effects.object_lift_height_m".'
            )


def _ensure_watchdog() -> WatchDogContext:
    global _CONTEXT
    with _CONTEXT_LOCK:
        if _CONTEXT is not None:
            return _CONTEXT
        if not rclpy.ok():
            rclpy.init(args=None)
        node = Node("watchdog_node")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        wake_event = threading.Event()
        registry = _load_registry()
        log_path = os.environ.get(
            "WATCHDOG_MONITOR_LOG",
            str(Path.home() / ".watchdog" / "monitor_events.jsonl"),
        )
        event_bus = EventBus(node=node, log_path=log_path)
        recorder = EvidenceRecorder(
            node=node,
            registry=registry,
            wake_event=wake_event,
        )
        halt_client = HaltClient(node=node, event_bus=event_bus)
        ready_client = ReadyClient(node=node, event_bus=event_bus)
        engine = MonitorEngine(
            event_bus=event_bus,
            halt_client=halt_client,
            wake_event=wake_event,
            logger=node.get_logger(),
        )
        recorder.engine = engine
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        _CONTEXT = WatchDogContext(
            node=node,
            executor=executor,
            thread=thread,
            registry=registry,
            event_bus=event_bus,
            recorder=recorder,
            engine=engine,
            halt_client=halt_client,
            ready_client=ready_client,
            wake_event=wake_event,
        )
        return _CONTEXT


def _shutdown_watchdog() -> None:
    global _CONTEXT
    with _CONTEXT_LOCK:
        ctx = _CONTEXT
        _CONTEXT = None
    if ctx is None:
        return
    try:
        ctx.executor.shutdown()
    except Exception:
        pass
    try:
        ctx.thread.join(timeout=2.0)
    except Exception:
        pass
    try:
        ctx.node.destroy_node()
    except Exception:
        pass
    try:
        ctx.event_bus.close()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


atexit.register(_shutdown_watchdog)


def _ok(command: str, **payload: Any) -> dict[str, Any]:
    return {"ok": True, "command": command, **payload}


def _error(command: str, error: Exception | str, **payload: Any) -> dict[str, Any]:
    return {"ok": False, "command": command, "error": str(error), **payload}


def _object_spec(ctx: WatchDogContext, object_id: str) -> dict[str, Any] | None:
    objects = ctx.registry.get("objects", {})
    entry = objects.get(object_id) if isinstance(objects, dict) else None
    if isinstance(entry, dict) and isinstance(entry.get("spec"), dict):
        return entry["spec"]
    return None


def _build_monitor(
    ctx: WatchDogContext,
    monitor_type: str,
    params: dict[str, Any],
    clause: dict[str, Any] | None,
    halt_on_fire: bool,
    monitor_id: str | None,
    step_id: str | None,
) -> Any:
    cls = MONITOR_TYPES.get(monitor_type)
    if cls is None:
        raise ValueError(
            "monitor_type must be one of relative_frame, stationarity, region, effort_event."
        )
    resolved_step = step_id or ctx.recorder.current_step()
    if not resolved_step:
        raise ValueError("step_id must be provided when no current step is active.")
    kwargs = {
        "clause": clause,
        "halt_on_fire": halt_on_fire,
        "monitor_id": monitor_id,
        "armed_wall_ns": wall_ns(),
    }
    if monitor_type in {"relative_frame", "stationarity"}:
        return cls(params=params, step_id=resolved_step, recorder=ctx.recorder, **kwargs)
    return cls(params=params, step_id=resolved_step, **kwargs)


def _drain_wait(
    ctx: WatchDogContext, event_types: set[str] | None
) -> dict[str, Any] | None:
    fires = ctx.engine.pop_fires()
    if fires:
        return {"wake": "monitor_fire", "fire": fires[0], "additional_fires": fires[1:]}
    events = ctx.recorder.pop_wait_events(event_types)
    if events:
        first = events[0]
        return {
            "wake": first.get("event_type"),
            "event": event_wrapper(first),
            "additional_events": [event_wrapper(event) for event in events[1:]],
        }
    return None


if mcp is not None:

    @mcp.tool()
    def get_contract(object_id: str) -> dict[str, Any]:
        """Return a pick contract and declared registry provenance for one object.

        cell carries cell-level (not per-object) joint and frame names --
        cell.gripper.effort_joint_name for effort_event's joint_name
        parameter, cell.gripper.tcp_frame for a gripper-side relative_frame
        parameter, cell.base_frame for MoveIt's planning frame. There is no
        grasp-centre TF frame; it does not exist and none should be invented
        -- the grasp centre is computed from commanded grasp width via the
        RG2 calibration table, not published to TF. Any monitor needing a
        gripper-side frame uses cell.gripper.tcp_frame.

        object_frames carries the TF frame id (tf_frame) and MoveIt lookup
        frame (lookup_frame) the recorder already tracks for every enabled
        registry object, keyed by object_id -- the object-side half of a
        relative_frame monitor's parent_frame/child_frame pair.

        acm_allowed_object_pairs lists object-object pairs whose collision
        checking is disabled in MoveIt's Allowed Collision Matrix so that
        intended contact (stacking, insertion) does not leave the planning
        scene permanently in collision. It is a planner tractability
        setting, not a task-level permission -- it says nothing about which
        objects the robot may touch. The specification is the only source
        for that; contact with an object required to stay undisturbed is a
        conformance violation regardless of what this list contains.
        """
        command = "get_contract"
        try:
            ctx = _ensure_watchdog()
            contract = load_pick_contract(object_id)
            acm_pairs = ctx.registry.get("acm_allowed_object_pairs", [])
            return _ok(
                command,
                object_id=object_id,
                contract=contract.model_dump(mode="json"),
                raw_spec=_object_spec(ctx, object_id),
                acm_allowed_object_pairs=acm_pairs if isinstance(acm_pairs, list) else [],
                cell=ctx.registry.get("cell", {}),
                object_frames=ctx.recorder.object_frames,
            )
        except Exception as exc:
            return _error(command, exc, object_id=object_id)

    @mcp.tool()
    def get_execution_state() -> dict[str, Any]:
        """Return buffered recorder counts, current step, monitors, and last event."""
        command = "get_execution_state"
        try:
            ctx = _ensure_watchdog()
            events = ctx.recorder.latest_events(1)
            return _ok(
                command,
                current_step_id=ctx.recorder.current_step(),
                counts=ctx.recorder.counts(),
                armed_monitors=ctx.engine.snapshot(),
                last_executor_event=events[-1]["envelope"] if events else None,
                tf_staleness_counters=ctx.recorder.tf_counters(),
            )
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def query_snapshot(object_ids: list[str] | None = None) -> dict[str, Any]:
        """Return latest buffered object poses, cell frame poses, and joint state.

        frames carries the latest sample of every cell-level TF frame (e.g.
        the gripper tcp_frame), keyed by frame id -- the instantaneous
        counterpart to get_evidence's frames channel.
        """
        command = "query_snapshot"
        try:
            ctx = _ensure_watchdog()
            ids = object_ids if object_ids is not None else ctx.recorder.object_ids()
            return _ok(
                command,
                objects={oid: ctx.recorder.latest_object_pose(oid) for oid in ids},
                frames={
                    fid: ctx.recorder.latest_frame_pose(fid)
                    for fid in ctx.recorder.frame_ids()
                },
                joint_state=ctx.recorder.latest_joint_state(),
            )
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def arm_monitor(
        monitor_type: str,
        params: dict[str, Any],
        clause: dict[str, Any] | None = None,
        halt_on_fire: bool = False,
        monitor_id: str | None = None,
        step_id: str | None = None,
    ) -> dict[str, Any]:
        """Arm one deterministic monitor primitive."""
        command = "arm_monitor"
        try:
            ctx = _ensure_watchdog()
            monitor = _build_monitor(
                ctx, monitor_type, params, clause, halt_on_fire, monitor_id, step_id
            )
            result = ctx.engine.arm(monitor)
            return _ok(command, **result)
        except ValueError as exc:
            return _error(command, exc, monitor_type=monitor_type)
        except Exception as exc:
            return _error(command, exc, monitor_type=monitor_type)

    @mcp.tool()
    def disarm_monitor(monitor_id: str) -> dict[str, Any]:
        """Disarm one monitor by id."""
        command = "disarm_monitor"
        try:
            ctx = _ensure_watchdog()
            removed = ctx.engine.disarm(monitor_id, "agent_disarm")
            return _ok(command, monitor_id=monitor_id, disarmed=removed)
        except Exception as exc:
            return _error(command, exc, monitor_id=monitor_id)

    @mcp.tool()
    def disarm_all_monitors() -> dict[str, Any]:
        """Disarm every armed monitor."""
        command = "disarm_all_monitors"
        try:
            ctx = _ensure_watchdog()
            removed = ctx.engine.disarm_all("agent_disarm_all")
            return _ok(command, disarmed=removed)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def wait_for_event(
        timeout_sec: float = 120.0,
        event_types: list[str] | None = None,
    ) -> dict[str, Any]:
        """Block until a monitor fire, an executor event of the requested types, or timeout.

        event_types must be exact names from available_event_types() -- a
        filter naming a type that does not exist matches nothing and would
        otherwise wait out the full timeout for no reason, believing it is
        watching for something. Rejected immediately instead, naming the
        unknown entries and the valid vocabulary.

        Waits out its full budget rather than returning wake="timeout" the
        moment some OTHER, non-matching event sets the wake condition (e.g.
        a skill_start arriving while filtered on ["phase", "end_step"]) --
        such a wakeup drains nothing matching the filter and must resume
        waiting on the remaining budget, not report a timeout that never
        happened. waited_sec is always returned, on every outcome, so a
        wake="timeout" that actually waited only a fraction of the requested
        timeout_sec is visible on the record rather than passing as a real
        timeout.
        """
        command = "wait_for_event"
        try:
            if event_types:
                unknown = sorted(set(event_types) - set(KNOWN_EVENT_TYPES))
                if unknown:
                    return _error(
                        command,
                        f"unknown event_types {unknown}; valid event types are "
                        f"{sorted(KNOWN_EVENT_TYPES)}. Call available_event_types() "
                        f"for descriptions and payload fields.",
                        event_types=event_types,
                    )
            ctx = _ensure_watchdog()
            timeout = min(max(float(timeout_sec), 0.0), 600.0)
            wanted = set(event_types) if event_types else None

            t0 = time.monotonic()
            deadline = t0 + timeout
            while True:
                immediate = _drain_wait(ctx, wanted)
                if immediate is not None:
                    return _ok(command, waited_sec=time.monotonic() - t0, **immediate)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return _ok(command, wake="timeout", waited_sec=time.monotonic() - t0)
                ctx.wake_event.clear()
                # Re-drain after clearing: an event may have arrived between
                # the drain above and the clear, and clearing would
                # otherwise discard that wakeup.
                immediate = _drain_wait(ctx, wanted)
                if immediate is not None:
                    return _ok(command, waited_sec=time.monotonic() - t0, **immediate)
                ctx.wake_event.wait(remaining)
                # Loop back around: wake_event may have been set by a
                # non-matching event (or spuriously), in which case the next
                # drain finds nothing and the loop waits out what's left of
                # the budget instead of reporting a timeout that didn't
                # happen.
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def available_monitor_definitions() -> dict[str, Any]:
        """Return the type, feed, semantics, and parameter schema of every monitor primitive."""
        command = "available_monitor_definitions"
        try:
            return _ok(command, monitors=monitor_definitions())
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def available_event_types() -> dict[str, Any]:
        """Return every event type the executor and monitor publish, with its
        meaning and payload fields.

        Built from the event schema definitions in events/schema.py, not a
        hand-written list, so it cannot drift from what is actually
        publishable. Call this once at the start of the run, before using
        event_types filters on wait_for_event -- filter names must match
        exactly, and a filter naming a type that does not exist here is
        rejected rather than silently matching nothing.

        Static description only -- does not depend on a live node, same as
        available_monitor_definitions.
        """
        command = "available_event_types"
        try:
            return _ok(command, event_types=available_event_type_definitions())
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def get_events(since_wall_ns: int | None = None) -> dict[str, Any]:
        """Return buffered executor and monitor events from the shared topic tail.

        All timestamps are wall-clock nanoseconds -- take them from each
        event's own wall_ns field, never from timestamp_ns inside envelope
        (a per-process monotonic clock, not comparable across processes).
        """
        command = "get_events"
        try:
            ctx = _ensure_watchdog()
            if since_wall_ns is None:
                events = ctx.recorder.latest_events(100)
            else:
                events = ctx.recorder.get_events_window(int(since_wall_ns), wall_ns())
            return _ok(command, events=[event_wrapper(e) for e in events])
        except Exception as exc:
            return _error(command, exc, since_wall_ns=since_wall_ns)

    @mcp.tool()
    def get_evidence(
        start_wall_ns: int,
        end_wall_ns: int,
        channels: list[str],
        object_ids: list[str] | None = None,
        frame_ids: list[str] | None = None,
        joint_names: list[str] | None = None,
        max_samples: int = 200,
    ) -> dict[str, Any]:
        """Return downsampled evidence windows from recorder buffers.

        channels is any of joint_states, object_poses, frames, events. The
        frames channel carries cell frames such as the gripper TCP, sampled
        on the same tick as object poses so samples pair by index -- use it
        to verify relative_frame clauses from evidence rather than reporting
        that a monitor did not fire.

        start_wall_ns/end_wall_ns are wall-clock nanoseconds -- the same
        wall_ns field every event and every recorder buffer uses, never
        envelope.timestamp_ns (a per-process monotonic clock; using it here
        will silently select an empty window). If the requested window
        overlaps no buffered sample on any requested channel, this returns
        an error carrying the actually buffered range rather than empty
        arrays -- an empty result is reserved for "nothing happened in a
        window that genuinely was buffered", a real and different outcome
        from "this window was never buffered at all".
        """
        command = "get_evidence"
        try:
            ctx = _ensure_watchdog()
            allowed = {"joint_states", "object_poses", "frames", "events"}
            requested = set(channels)
            unknown = requested - allowed
            if unknown:
                raise ValueError(f"unknown channels: {sorted(unknown)}")
            limit = max(1, min(int(max_samples), 1000))
            start_ns, end_ns = int(start_wall_ns), int(end_wall_ns)
            evidence: dict[str, Any] = {}
            if "joint_states" in requested:
                evidence["joint_states"] = ctx.recorder.get_joint_window(
                    start_ns, end_ns, joint_names, limit
                )
            if "object_poses" in requested:
                ids = object_ids if object_ids is not None else ctx.recorder.object_ids()
                evidence["object_poses"] = {
                    oid: ctx.recorder.get_object_pose_window(start_ns, end_ns, oid, limit)
                    for oid in ids
                }
            if "frames" in requested:
                fids = frame_ids if frame_ids is not None else ctx.recorder.frame_ids()
                evidence["frames"] = {
                    fid: ctx.recorder.get_frame_window(start_ns, end_ns, fid, limit)
                    for fid in fids
                }
            if "events" in requested:
                evidence["events"] = [
                    event_wrapper(e)
                    for e in ctx.recorder.get_events_window(start_ns, end_ns)
                ]

            any_samples = any(
                (any(v.values()) if key in ("object_poses", "frames") else bool(v))
                for key, v in evidence.items()
            )
            if not any_samples:
                return _error(
                    command,
                    "requested window does not overlap any buffered evidence",
                    requested={"start_wall_ns": start_ns, "end_wall_ns": end_ns},
                    buffered_range=ctx.recorder.buffered_range(object_ids, frame_ids),
                    now_wall_ns=wall_ns(),
                )
            return _ok(command, evidence=evidence)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def halt_execution(detail: str) -> dict[str, Any]:
        """Cancel active MoveGroup goals through the WatchDog halt path."""
        command = "halt_execution"
        try:
            ctx = _ensure_watchdog()
            step_id = ctx.recorder.current_step() or "agent_manual"
            result = ctx.halt_client.halt(step_id, "agent_manual", detail)
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def signal_ready(
        scope: str,
        step_id: str | None = None,
        phase: str | None = None,
    ) -> dict[str, Any]:
        """Publish a readiness signal for a run, step, or phase.

        Calls the cell's /cell/signal_ready service (releasing any executor
        blocked in request_go for this scope) and publishes a watchdog_ready
        event to the topic for the record.

        For scope="run", this first verifies the recorder buffers are warm:
        at least one buffered pose for every object the registry declares,
        and a non-empty joint-state sample buffer. If either is empty, the
        signal is refused with a clear error rather than sent -- this is the
        warm-up race in which a monitor could arm against a recorder that
        has not sampled anything yet.

        Parameters
        ----------
        scope:
            One of "run", "step", "phase".
        step_id:
            Required for scope "step" or "phase".
        phase:
            Required for scope "phase".
        """
        command = "signal_ready"
        _VALID_SCOPES = {"run", "step", "phase"}
        try:
            if scope not in _VALID_SCOPES:
                raise ValueError(f"scope must be one of {sorted(_VALID_SCOPES)}.")
            ctx = _ensure_watchdog()

            if scope == "run":
                object_ids = ctx.recorder.object_ids()
                counts = ctx.recorder.counts()
                pose_counts = counts.get("object_poses", {})
                joint_count = counts.get("joint_states", 0)
                cold_objects = [
                    oid for oid in object_ids if pose_counts.get(oid, 0) < 1
                ]
                if cold_objects or joint_count < 1:
                    return _error(
                        command,
                        "recorder buffers are not warm yet: "
                        f"objects with no buffered pose={cold_objects}, "
                        f"joint_state samples={joint_count}. Wait and retry "
                        "rather than signalling run-scope readiness.",
                        counts=counts,
                        object_ids=object_ids,
                    )
                result = ctx.ready_client.signal(scope, step_id, phase)
                return _ok(command, result=result, counts=counts, object_ids=object_ids)

            result = ctx.ready_client.signal(scope, step_id, phase)
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc, scope=scope)

    @mcp.tool()
    def record_verdict(
        step_id: str,
        verdict: str,
        clause_results: list[dict[str, Any]],
        explanation: str,
        claimed_outcome: str | None = None,
    ) -> dict[str, Any]:
        """Publish a conformance verdict with free-form clause result dicts.

        clause_results entries are expected to contain field, comparison,
        threshold, observed, and pass keys, plus clause_id: every entry that
        corresponds to a contract clause must carry that clause's clause_id
        (from get_contract) unchanged. An instruction-level requirement with
        no contract clause carries clause_id: null and source: "instruction"
        instead. This is what lets coverage be checked afterward by set
        comparison against the contract's own clause_ids, not free text.

        A malformed clause_id (not "<section>.<field>", and not null with
        source: "instruction") is rejected here. This only checks shape --
        it does not check the id against the actual contract, which would
        require this server to know which object the step concerned.
        """
        command = "record_verdict"
        try:
            if verdict not in _VALID_VERDICTS:
                raise ValueError(f"verdict must be one of {sorted(_VALID_VERDICTS)}.")
            _validate_clause_results(clause_results)
            ctx = _ensure_watchdog()
            payload = VerdictPayload(
                step_id=step_id,
                verdict=verdict,
                claimed_outcome=claimed_outcome,
                clause_results=clause_results,
                explanation=explanation,
            )
            event = make_event("verdict", payload)
            ctx.event_bus.publish(event)
            return _ok(command, event=event.model_dump(mode="json"))
        except Exception as exc:
            return _error(command, exc, step_id=step_id, verdict=verdict)


def main() -> None:
    """Run the WatchDog MCP server.

    Eagerly constructs the node/recorder/engine before mcp.run() rather than
    waiting for the first tool call. available_monitor_definitions()
    deliberately does not boot the node (it returns static description and
    must not depend on a live one -- see that tool's docstring), so without
    this the recorder's TF/joint-state buffers are still empty by the time
    the agent makes its first signal_ready(scope="run") call, and that call's
    warm check fails on the first attempt. Eager init here means the recorder
    has been buffering since process start, so by the time the agent has
    spent a turn on available_monitor_definitions the buffers are warm.

    Non-fatal: if ROS/the simulator isn't up yet, this logs a warning and
    falls through to mcp.run() unchanged -- the lazy _ensure_watchdog() path
    inside each tool remains the fallback, and --dry-run must keep working
    with no simulator running.
    """
    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run watchdog_mcp_server."
        ) from _MCP_IMPORT_ERROR

    try:
        _ensure_watchdog()
    except Exception as exc:
        print(
            f"WARNING: watchdog_mcp_server eager init failed ({exc}); "
            "continuing -- the first tool call will retry initialization.",
            file=sys.stderr,
        )

    try:
        mcp.run()
    finally:
        _shutdown_watchdog()


if __name__ == "__main__":
    main()
