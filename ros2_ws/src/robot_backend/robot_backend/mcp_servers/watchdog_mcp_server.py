"""MCP server for WatchDog recorder and monitor tools."""

from __future__ import annotations

import atexit
import os
import threading
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
from robot_backend.events.schema import VerdictPayload, make_event
from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.halt import HaltClient
from robot_backend.watchdog.monitors import MONITOR_TYPES, monitor_definitions
from robot_backend.watchdog.recorder import EvidenceRecorder, _load_registry
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
    wake_event: threading.Event


_CONTEXT: WatchDogContext | None = None
_CONTEXT_LOCK = threading.Lock()
_VALID_VERDICTS = {"conformant", "non_conformant", "valid_variation", "inconclusive"}


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
            "event": first.get("envelope", first),
            "additional_events": [event.get("envelope", event) for event in events[1:]],
        }
    return None


if mcp is not None:

    @mcp.tool()
    def get_contract(object_id: str) -> dict[str, Any]:
        """Return a pick contract and declared registry provenance for one object."""
        command = "get_contract"
        try:
            ctx = _ensure_watchdog()
            contract = load_pick_contract(object_id)
            permitted = ctx.registry.get("permitted_contacts", [])
            return _ok(
                command,
                object_id=object_id,
                contract=contract.model_dump(mode="json"),
                raw_spec=_object_spec(ctx, object_id),
                permitted_contacts=permitted if isinstance(permitted, list) else [],
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
        """Return latest buffered object poses and joint state."""
        command = "query_snapshot"
        try:
            ctx = _ensure_watchdog()
            ids = object_ids if object_ids is not None else ctx.recorder.object_ids()
            return _ok(
                command,
                objects={oid: ctx.recorder.latest_object_pose(oid) for oid in ids},
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
        """Block until a monitor fire, an executor event of the requested types, or timeout."""
        command = "wait_for_event"
        try:
            ctx = _ensure_watchdog()
            timeout = min(max(float(timeout_sec), 0.0), 600.0)
            wanted = set(event_types) if event_types else None
            immediate = _drain_wait(ctx, wanted)
            if immediate is not None:
                return _ok(command, **immediate)
            ctx.wake_event.clear()
            immediate = _drain_wait(ctx, wanted)
            if immediate is not None:
                return _ok(command, **immediate)
            if not ctx.wake_event.wait(timeout):
                return _ok(command, wake="timeout")
            immediate = _drain_wait(ctx, wanted)
            if immediate is not None:
                return _ok(command, **immediate)
            return _ok(command, wake="timeout")
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
    def get_events(since_wall_ns: int | None = None) -> dict[str, Any]:
        """Return buffered executor and monitor events from the shared topic tail."""
        command = "get_events"
        try:
            ctx = _ensure_watchdog()
            if since_wall_ns is None:
                events = ctx.recorder.latest_events(100)
            else:
                events = ctx.recorder.get_events_window(int(since_wall_ns), wall_ns())
            return _ok(command, events=events)
        except Exception as exc:
            return _error(command, exc, since_wall_ns=since_wall_ns)

    @mcp.tool()
    def get_evidence(
        start_wall_ns: int,
        end_wall_ns: int,
        channels: list[str],
        object_ids: list[str] | None = None,
        joint_names: list[str] | None = None,
        max_samples: int = 200,
    ) -> dict[str, Any]:
        """Return downsampled evidence windows from recorder buffers."""
        command = "get_evidence"
        try:
            ctx = _ensure_watchdog()
            allowed = {"joint_states", "object_poses", "events"}
            requested = set(channels)
            unknown = requested - allowed
            if unknown:
                raise ValueError(f"unknown channels: {sorted(unknown)}")
            limit = max(1, min(int(max_samples), 1000))
            evidence: dict[str, Any] = {}
            if "joint_states" in requested:
                evidence["joint_states"] = ctx.recorder.get_joint_window(
                    int(start_wall_ns), int(end_wall_ns), joint_names, limit
                )
            if "object_poses" in requested:
                ids = object_ids if object_ids is not None else ctx.recorder.object_ids()
                evidence["object_poses"] = {
                    oid: ctx.recorder.get_object_pose_window(
                        int(start_wall_ns), int(end_wall_ns), oid, limit
                    )
                    for oid in ids
                }
            if "events" in requested:
                evidence["events"] = ctx.recorder.get_events_window(
                    int(start_wall_ns), int(end_wall_ns)
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
    def record_verdict(
        step_id: str,
        verdict: str,
        clause_results: list[dict[str, Any]],
        explanation: str,
        claimed_outcome: str | None = None,
    ) -> dict[str, Any]:
        """Publish a conformance verdict with free-form clause result dicts.

        clause_results entries are expected to contain field, comparison,
        threshold, observed, and pass keys.
        """
        command = "record_verdict"
        try:
            if verdict not in _VALID_VERDICTS:
                raise ValueError(f"verdict must be one of {sorted(_VALID_VERDICTS)}.")
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
    """Run the WatchDog MCP server."""
    if mcp is None:
        raise RuntimeError(
            "The Python package 'mcp' is required to run watchdog_mcp_server."
        ) from _MCP_IMPORT_ERROR
    try:
        mcp.run()
    finally:
        _shutdown_watchdog()


if __name__ == "__main__":
    main()
