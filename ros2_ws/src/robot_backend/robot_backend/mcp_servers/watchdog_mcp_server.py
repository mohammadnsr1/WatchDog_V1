"""MCP server for WatchDog recorder and monitor tools."""

from __future__ import annotations

import atexit
import json
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
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_backend.contracts.loader import load_pick_contract
from robot_backend.events.bus import EventBus
from robot_backend.events.schema import (
    FramesCapturedPayload,
    KNOWN_EVENT_TYPES,
    RunVerdictPayload,
    VerdictPayload,
    available_event_type_definitions,
    make_event,
)
from robot_backend.watchdog.engine import MonitorEngine
from robot_backend.watchdog.halt import HaltClient
from robot_backend.utils.png_encode import encode_png
from robot_backend.watchdog.image_buffer import ImageBufferManager, ImageRingBuffer
from robot_backend.watchdog.monitors import MONITOR_TYPES, monitor_definitions
from robot_backend.watchdog.ready import ReadyClient
from robot_backend.watchdog.recorder import (
    EvidenceRecorder,
    _load_registry,
    _uniform_downsample,
    event_wrapper,
)
from robot_backend.watchdog import session as session_helper
from robot_backend.watchdog.timebase import wall_ns

# Image ring buffer bounds, env-configurable. A byte bound is required because
# frame size varies with encoding and a count-only bound cannot protect memory.
_DEFAULT_IMAGE_BUFFER_BYTES = 512 * 1024 * 1024  # 512 MB
_DEFAULT_IMAGE_BUFFER_HZ = 5.0
_DEFAULT_IMAGE_BUFFER_MAX_FRAMES = 2000  # count bound; byte bound is the real cap


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return default
    return value if value > 0 else default


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return default
    return value if value > 0 else default


def _load_task_steps() -> dict[str, dict[str, Any]]:
    """Load the active run's task YAML steps: block, keyed by step_id.

    grasp_width_m and grasp_center_offset are pure per-run conformance/command
    data and live in the task instruction, not the registry (see
    agents/tasks/*.yaml). WATCHDOG_TASK_PATH is exported by run_agent.py
    before this process's MCP servers are spawned, so get_contract can read
    the step data directly from the instruction file rather than have the
    agent pass it as a tool argument. Fails loud: no fallback to defaults or
    to a registry value, since there is no registry value anymore.
    """
    task_path = os.environ.get("WATCHDOG_TASK_PATH")
    if not task_path:
        raise RuntimeError(
            "WATCHDOG_TASK_PATH is not set. get_contract needs it to read the "
            "task instruction's steps: block (grasp_width_m and "
            "grasp_center_offset are instruction-sourced, not registry-sourced). "
            "run_agent.py exports this before launching the MCP servers -- "
            "check that this process was started via run_agent.py --task ..."
        )
    with open(task_path) as f:
        task = yaml.safe_load(f)
    steps = task.get("steps") if isinstance(task, dict) else None
    if not isinstance(steps, list):
        raise ValueError(
            f"task file {task_path!r} has no steps: list. Add a structured "
            "steps: block (step_id, object_id, grasp_width_m, "
            "grasp_center_offset) -- prose fields are not parsed for this data."
        )
    by_step_id: dict[str, dict[str, Any]] = {}
    for entry in steps:
        if not isinstance(entry, dict) or "step_id" not in entry:
            raise ValueError(
                f"task file {task_path!r} has a steps: entry missing step_id: {entry!r}"
            )
        by_step_id[entry["step_id"]] = entry
    return by_step_id


def _load_task_objects() -> dict[str, dict[str, Any]]:
    """Collapse the task's steps: block into one spec per object_id.

    A contract is static per-object reference data: every threshold in it is a
    registry bound for that object, and grasp_width_m is that object's
    conformance width. None of it changes as the run moves from step to step,
    so there is one contract per object and it is addressed by object_id.

    Two fields are folded out of the object's steps:

    grasp_width_m is asserted to be identical across every step that names the
    object. It describes the object's graspable geometry, not the step, so a
    disagreement between two steps is an authoring error and raises here
    rather than silently letting whichever step was asked first decide the
    conformance bound.

    contract_clause_groups is the union across the object's steps. Union, not
    intersection: a group any step authored is a group that applies to this
    object somewhere in the process, and the contract is the object's whole
    specification rather than any one step's slice of it. Where in the process
    each group is judged is the agent's to read from the process map and the
    instruction, not something the shape of this return can say any more.
    """
    by_step_id = _load_task_steps()
    by_object: dict[str, dict[str, Any]] = {}
    for step_id, entry in by_step_id.items():
        object_id = entry.get("object_id")
        if not object_id:
            continue
        spec = by_object.setdefault(
            object_id, {"object_id": object_id, "step_id": f"<object:{object_id}>"}
        )
        width = entry.get("grasp_width_m")
        if width is not None:
            seen = spec.get("grasp_width_m")
            if seen is not None and float(seen) != float(width):
                raise ValueError(
                    f"object_id {object_id!r} is authored with conflicting "
                    f"grasp_width_m across its steps ({seen} and {width}, the "
                    f"latter at step_id {step_id!r}). grasp_width_m describes "
                    f"the object, not the step, so it must agree everywhere "
                    f"the object appears."
                )
            spec["grasp_width_m"] = width
        groups = entry.get("contract_clause_groups")
        if groups:
            spec.setdefault("contract_clause_groups", [])
            for g in groups:
                if g not in spec["contract_clause_groups"]:
                    spec["contract_clause_groups"].append(g)
    return by_object


def _frame_to_png(frame: Any) -> bytes:
    """Return PNG bytes for a buffered frame, converting if needed.

    Everything written to disk is PNG so an analyst (or the operator) can just
    open it. Frames arrive either already PNG-encoded, or as a raw pixel buffer
    that gets encoded here. A JPEG frame cannot be converted without a codec
    this cell does not ship, so it raises rather than writing bytes with a .png
    name that no viewer will open -- see image_buffer's topic preference, which
    exists to keep that case from arising.
    """
    data = frame.data
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return data
    enc = (frame.encoding or "").lower()
    if "jpeg" in enc or "jpg" in enc:
        raise ValueError(
            f"frame on {frame.topic!r} is JPEG-compressed ({frame.encoding!r}) and "
            "cannot be converted to PNG without an image codec. Buffer the raw "
            "image topic instead (WATCHDOG_IMAGE_TOPICS)."
        )
    return encode_png(data, frame.width, frame.height, frame.encoding)


def _write_capture_frames(
    session_dir: Path,
    selected: list[Any],
    *,
    start_ns: int,
    end_ns: int,
    max_frames: int,
    label: str | None,
    step_id: str | None,
    buffered_range: list[int] | None,
) -> dict[str, Any]:
    """Downsample and write selected frames + capture.json to the session dir.

    Module-level and ROS-free so the write path is unit-testable. Returns a
    dict with output_dir, frame_count, covered_start_wall_ns, covered_end_wall_ns.
    Callers guarantee `selected` is non-empty.
    """
    frames = _uniform_downsample(selected, max(1, int(max_frames)))
    name = label or step_id or "capture"
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name)).strip("_") or "capture"
    out_dir = session_dir / "frames" / f"{safe_name}_{start_ns}_{end_ns}"
    out_dir.mkdir(parents=True, exist_ok=True)

    written: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []
    for idx, frame in enumerate(frames):
        try:
            png_bytes = _frame_to_png(frame)
        except Exception as exc:
            # Never write an unviewable file just to keep the count up.
            skipped.append({"wall_ns": frame.wall_ns, "topic": frame.topic,
                            "encoding": frame.encoding, "reason": str(exc)})
            continue
        filename = f"{idx:04d}_{frame.wall_ns}.png"
        (out_dir / filename).write_bytes(png_bytes)
        written.append({
            "file": filename,
            "wall_ns": frame.wall_ns,
            "topic": frame.topic,
            "source_encoding": frame.encoding,
            "width": frame.width,
            "height": frame.height,
            "nbytes": len(png_bytes),
        })
    if not written:
        raise RuntimeError(
            "no frame could be written as PNG: "
            + "; ".join(s["reason"] for s in skipped[:3])
        )

    covered_start = written[0]["wall_ns"]
    covered_end = written[-1]["wall_ns"]
    capture_meta = {
        "requested_window": {"start_wall_ns": start_ns, "end_wall_ns": end_ns},
        "covered_window": {"start_wall_ns": covered_start, "end_wall_ns": covered_end},
        "buffered_range": buffered_range,
        "label": label,
        "step_id": step_id,
        "frame_count": len(written),
        "format": "png",
        "frames": written,
        "skipped": skipped,
    }
    (out_dir / "capture.json").write_text(
        json.dumps(capture_meta, indent=2), encoding="utf-8"
    )
    return {
        "output_dir": str(out_dir),
        "frame_count": len(written),
        "format": "png",
        "skipped_count": len(skipped),
        "covered_start_wall_ns": covered_start,
        "covered_end_wall_ns": covered_end,
    }

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
    session_dir: Path
    image_buffer: ImageRingBuffer
    image_manager: ImageBufferManager
    # Lazily populated on first get_contract call from WATCHDOG_TASK_PATH's
    # steps: block, collapsed to one spec per object_id -- see
    # _load_task_objects. Contracts are addressed by object, not by step: the
    # steps: block remains the watchdog's map of the expected process, but it
    # is no longer a lookup key for loading a contract.
    task_objects: dict[str, dict[str, Any]] | None = None


_CONTEXT: WatchDogContext | None = None
_CONTEXT_LOCK = threading.Lock()
_VALID_VERDICTS = {"conformant", "non_conformant", "valid_variation", "inconclusive"}
# Per-process record of step_ids a verdict has already been published for.
# Detection, not prevention -- a second verdict for a retried step may be
# legitimate, and silently dropping it would be worse than recording it as
# suspicious. Guarded by _CONTEXT_LOCK rather than its own lock since it is
# always touched alongside _CONTEXT.
_RECORDED_VERDICT_STEP_IDS: set[str] = set()
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


def _validate_run_clause_results(clause_results: list[dict[str, Any]]) -> None:
    """Shape-check clause_results for record_run_verdict.

    The step-scoped check plus one addition: every entry must also carry a
    non-empty step_id naming the step it was evaluated under. clause_id is
    "<section>.<field>" (contracts/loader.py's _make_clause) with no object
    and no step in it, so the same clause_id appears once per part in a
    whole-run rollup. Without a per-entry step_id the rollup would validate
    clean while being unreadable -- four identical effects.* entries with no
    way to tell which part each concerned.
    """
    _validate_clause_results(clause_results)
    for i, entry in enumerate(clause_results):
        step_id = entry.get("step_id")
        if not isinstance(step_id, str) or not step_id:
            raise ValueError(
                f"clause_results[{i}] has no usable step_id ({step_id!r}). A "
                f"run-scoped verdict combines several steps, and clause_id "
                f"alone cannot say which step an entry came from -- every "
                f"entry must name its own step_id."
            )


def _ensure_watchdog() -> WatchDogContext:
    global _CONTEXT
    with _CONTEXT_LOCK:
        if _CONTEXT is not None:
            return _CONTEXT
        if not rclpy.ok():
            rclpy.init(args=None)
        node = Node("watchdog_node")
        executor = MultiThreadedExecutor(num_threads=4)
        node.get_logger().info("WatchDog executor starting with num_threads=4")
        executor.add_node(node)
        client_cb_group = MutuallyExclusiveCallbackGroup()
        wake_event = threading.Event()
        registry = _load_registry()
        session_dir = session_helper.ensure_session_dir()
        # Explicit WATCHDOG_MONITOR_LOG still wins; otherwise the log defaults
        # into the session dir (events_watchdog.jsonl) so this process's events
        # sit alongside the rest of the run's record. See session.py.
        log_path = os.environ.get(
            "WATCHDOG_MONITOR_LOG",
            session_helper.monitor_log_default(),
        )
        event_bus = EventBus(node=node, log_path=log_path)
        recorder = EvidenceRecorder(
            node=node,
            registry=registry,
            wake_event=wake_event,
        )
        # Image ring buffer for retrospective visual-evidence capture. Its
        # subscriptions go in a SEPARATE callback group from the recorder's
        # /joint_states and TF callbacks (D2): image callbacks are slow and
        # must never delay monitor evaluation or halt latency.
        image_buffer = ImageRingBuffer(
            max_frames=_env_int("WATCHDOG_IMAGE_BUFFER_MAX_FRAMES", _DEFAULT_IMAGE_BUFFER_MAX_FRAMES),
            max_bytes=_env_int("WATCHDOG_IMAGE_BUFFER_BYTES", _DEFAULT_IMAGE_BUFFER_BYTES),
        )
        image_hz = _env_float("WATCHDOG_IMAGE_BUFFER_HZ", _DEFAULT_IMAGE_BUFFER_HZ)
        image_manager = ImageBufferManager(image_buffer, hz=image_hz, logger=node.get_logger())
        image_cb_group = ReentrantCallbackGroup()
        try:
            image_manager.attach(node, image_cb_group)
        except Exception as exc:
            node.get_logger().warning(f"image buffer attach failed (non-fatal): {exc}")
        halt_client = HaltClient(
            node=node,
            event_bus=event_bus,
            callback_group=client_cb_group,
        )
        ready_client = ReadyClient(
            node=node,
            event_bus=event_bus,
            callback_group=client_cb_group,
        )
        engine = MonitorEngine(
            event_bus=event_bus,
            halt_client=halt_client,
            wake_event=wake_event,
            logger=node.get_logger(),
        )
        recorder.engine = engine
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        # Prewarm client construction outside recorder callbacks. Left lazy,
        # the first halt pays ROS interface imports and client construction
        # inside /joint_states handling, creating a measured 0.17-0.26 s hole
        # in evidence around the fire despite a millisecond-scale cancel call.
        # Failures remain non-fatal: halt()/signal() retain their guarded lazy
        # initialization as the fallback.
        try:
            halt_client.prewarm()
        except Exception as exc:
            node.get_logger().warning(f"halt client prewarm failed: {exc}")
        try:
            ready_client.prewarm()
        except Exception as exc:
            node.get_logger().warning(f"ready client prewarm failed: {exc}")
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
            session_dir=session_dir,
            image_buffer=image_buffer,
            image_manager=image_manager,
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
) -> Any:
    cls = MONITOR_TYPES.get(monitor_type)
    if cls is None:
        raise ValueError(
            "monitor_type must be one of relative_frame, stationarity, region, effort_event."
        )
    kwargs = {
        "clause": clause,
        "halt_on_fire": halt_on_fire,
        "monitor_id": monitor_id,
        "armed_wall_ns": wall_ns(),
    }
    if monitor_type in {"relative_frame", "stationarity"}:
        return cls(params=params, recorder=ctx.recorder, **kwargs)
    return cls(params=params, **kwargs)


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
        """Return one object's contract, plus cell and frame reference data.

        This is static per-object reference data. Every threshold in it is a
        registry bound for that object and grasp_width_m is that object's
        conformance width, read structurally from the running task file
        (WATCHDOG_TASK_PATH) rather than passed by you, so the bound never
        travels through your reasoning. None of it changes as the run
        proceeds. Fetch each object's contract once and reuse it; calling
        again for the same object returns the same thing and buys nothing.

        It is addressed by object_id alone. There is no step argument: a
        contract is not a per-step event, and asking for one says nothing
        about where the run has got to. What the contract does not tell you is
        WHEN each clause is judged -- that you read from the process and the
        instruction specification.

        cell carries cell-level (not per-object) joint and frame names --
        cell.gripper.effort_joint_name, cell.gripper.tcp_frame, and
        cell.base_frame for MoveIt's planning frame. There is no grasp-centre
        TF frame; it does not exist and none should be invented -- the grasp
        centre is computed from commanded grasp width via the RG2 calibration
        table, not published to TF.

        object_frames carries the TF frame id (tf_frame) and MoveIt lookup
        frame (lookup_frame) the recorder already tracks for every enabled
        registry object, keyed by object_id.

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
            if ctx.task_objects is None:
                ctx.task_objects = _load_task_objects()
            object_spec = ctx.task_objects.get(object_id)
            if object_spec is None:
                raise ValueError(
                    f"object_id {object_id!r} is not named by any step in the "
                    f"running task. Objects with a contract: "
                    f"{sorted(ctx.task_objects)}"
                )
            contract = load_pick_contract(object_id, object_spec)
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
    ) -> dict[str, Any]:
        """Arm one deterministic monitor primitive.

        A monitor has no step affiliation. It lives from the moment you arm it
        until the moment you disarm it, however many steps that spans, and
        nothing takes it down at a step boundary. Arming and disarming are
        yours alone, at whatever moments your reading of the process calls for.

        Which step was under way when a monitor armed or fired is recovered
        afterwards from the event's wall_time_iso against the recorded event
        logs. That absolute timestamp is the only field comparable across
        processes, so it is the correlation key, and it is why a monitor needs
        no step label of its own.

        A monitor that is already armed and still applies does not need
        rebuilding. Re-arming it costs a turn, and where it measures against a
        reference captured at arm time, re-arming re-takes that reference from
        wherever things have drifted to by then -- so a requirement rebuilt
        repeatedly can be violated by degrees without any single rebuild
        registering it. get_execution_state lists everything currently armed.
        """
        command = "arm_monitor"
        try:
            ctx = _ensure_watchdog()
            monitor = _build_monitor(
                ctx, monitor_type, params, clause, halt_on_fire, monitor_id
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
        """Disarm every armed monitor.

        It means all, including any monitor holding a requirement that spans
        the whole run. Nothing is exempt, and the rest of the run proceeds
        without whatever this takes down.
        """
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
    def capture_frames(
        start_wall_ns: int,
        end_wall_ns: int,
        max_frames: int = 20,
        label: str | None = None,
    ) -> dict[str, Any]:
        """Write buffered camera frames from a wall-clock window to the session directory.

        This tool PRESERVES visual evidence; it does not perceive. The agent
        harness sends text only -- WatchDog cannot see the frames it captures.
        The tool exists so that visual evidence for a moment the monitor
        identified as significant survives for later analysis. It does not
        return image data and it does not inform any verdict, which rests on
        the proprioceptive and TF evidence WatchDog can actually read,
        independent of any vision pipeline.

        Selects buffered frames whose wall_ns falls in the window, downsamples
        uniformly to max_frames, and writes them under
        <session_dir>/frames/<label or step_id>_<start>_<end>/ with a
        capture.json listing each file, its wall_ns, source topic, and
        encoding, plus the requested window and the actual buffered range.
        Returns the output directory, frame count, and covered time span --
        never the images themselves.

        If the window overlaps no buffered frames, returns an error carrying
        the buffered range and now_wall_ns, matching get_evidence -- an empty
        result is never silently indistinguishable from "nothing happened".
        """
        command = "capture_frames"
        try:
            ctx = _ensure_watchdog()
            start_ns, end_ns = int(start_wall_ns), int(end_wall_ns)
            limit = max(1, int(max_frames))
            selected = ctx.image_buffer.select_window(start_ns, end_ns)
            if not selected:
                return _error(
                    command,
                    "requested window does not overlap any buffered frames",
                    requested={"start_wall_ns": start_ns, "end_wall_ns": end_ns},
                    buffered_range=ctx.image_buffer.buffered_range(),
                    subscribed_topics=list(ctx.image_manager.subscribed_topics),
                    now_wall_ns=wall_ns(),
                )
            step_id = ctx.recorder.current_step()
            result = _write_capture_frames(
                ctx.session_dir,
                selected,
                start_ns=start_ns,
                end_ns=end_ns,
                max_frames=limit,
                label=label,
                step_id=step_id,
                buffered_range=ctx.image_buffer.buffered_range(),
            )
            # Publish frames_captured so the capture is on the shared timeline.
            # Self-emitted (see _SELF_EMITTED_EVENT_TYPES) -- must not wake the
            # agent on its own capture.
            ctx.event_bus.publish(make_event(
                "frames_captured",
                FramesCapturedPayload(
                    output_dir=result["output_dir"],
                    frame_count=result["frame_count"],
                    requested_start_wall_ns=start_ns,
                    requested_end_wall_ns=end_ns,
                    covered_start_wall_ns=result["covered_start_wall_ns"],
                    covered_end_wall_ns=result["covered_end_wall_ns"],
                    label=label,
                    step_id=step_id,
                ),
            ))
            return _ok(command, **result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def halt_execution(detail: str) -> dict[str, Any]:
        """Cancel active MoveGroup goals through the WatchDog halt path."""
        command = "halt_execution"
        try:
            ctx = _ensure_watchdog()
            result = ctx.halt_client.halt(
                "agent_manual", detail, step_id=ctx.recorder.current_step()
            )
            return _ok(command, result=result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def abort_run(reason: str) -> dict[str, Any]:
        """Signal that execution should not continue beyond the next step boundary.

        Sends a scope="abort" readiness signal to the cell's orchestrator,
        which then refuses every subsequent request_go with reason="aborted"
        and releases any gate already blocked. This does NOT stop motion
        already underway -- halt_execution is the only immediate lever for
        that; the two are used together when both apply. The executor learns
        only that the cell declined to release a step; it is told nothing
        about the monitor or the reason. The reason is recorded on the
        WatchDog timeline and in the orchestrator log.
        """
        command = "abort_run"
        try:
            if not isinstance(reason, str) or not reason.strip():
                raise ValueError("reason must be a non-empty string.")
            ctx = _ensure_watchdog()
            result = ctx.ready_client.signal("abort", detail=reason.strip())
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

        Called on a finding, not on a schedule: an adjudicated monitor fire,
        or an end_step evidence check that fails a clause with nothing having
        fired. A run that produces no findings records nothing here and calls
        record_run_verdict once at the end instead.

        A repeated step_id is therefore normal -- a step with two fires
        produces two verdicts. It still publishes, and the return carries
        duplicate_step_id: true with a warning logged, so a genuinely
        unintended repeat stays visible.
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
            with _CONTEXT_LOCK:
                is_duplicate = step_id in _RECORDED_VERDICT_STEP_IDS
                _RECORDED_VERDICT_STEP_IDS.add(step_id)
            if is_duplicate:
                ctx.node.get_logger().warning(
                    f"record_verdict: a second verdict was recorded for "
                    f"step_id={step_id!r}. Legitimate for a step with two "
                    f"separate findings, or for a retried step, but check."
                )
            return _ok(
                command,
                event=event.model_dump(mode="json"),
                duplicate_step_id=is_duplicate,
            )
        except Exception as exc:
            return _error(command, exc, step_id=step_id, verdict=verdict)

    @mcp.tool()
    def record_run_verdict(
        verdict: str,
        clause_results: list[dict[str, Any]],
        explanation: str,
        steps_covered: list[str],
        claimed_outcome: str | None = None,
    ) -> dict[str, Any]:
        """Publish one conformance verdict scoped to the whole run.

        For the run that produced no findings: no monitor fire was adjudicated
        as a physical departure, and no end_step evidence check failed a
        clause. Instead of a verdict per clean step, evaluate every step's
        effects together and record them here, once, at the true end of the
        run. A run that already recorded a finding through record_verdict
        needs no call here -- those recordings are that run's record, and a
        rollup alongside them would double-count.

        clause_results carries the same entries record_verdict takes, with one
        addition: every entry must also name its own step_id. clause_id is
        "<section>.<field>" and is identical across all four parts, so without
        it the rollup cannot say which part a result belongs to.

        steps_covered lists every step_id this verdict accounts for. It is a
        real argument rather than a sentence in explanation so coverage can be
        checked by set comparison against the task's steps: block, the same way
        clause_id makes clause coverage checkable.

        claimed_outcome is optional here and, when given, should summarize the
        executor's claims across the run rather than any single step's.
        """
        command = "record_run_verdict"
        try:
            if verdict not in _VALID_VERDICTS:
                raise ValueError(f"verdict must be one of {sorted(_VALID_VERDICTS)}.")
            if not steps_covered or not all(
                isinstance(s, str) and s for s in steps_covered
            ):
                raise ValueError(
                    "steps_covered must be a non-empty list of step_id strings "
                    "naming every step this run-scoped verdict accounts for."
                )
            _validate_run_clause_results(clause_results)
            uncovered = sorted(
                {e["step_id"] for e in clause_results} - set(steps_covered)
            )
            if uncovered:
                raise ValueError(
                    f"clause_results name step_ids absent from steps_covered: "
                    f"{uncovered}. steps_covered must account for every step a "
                    f"clause result was evaluated under."
                )
            ctx = _ensure_watchdog()
            payload = RunVerdictPayload(
                verdict=verdict,
                claimed_outcome=claimed_outcome,
                clause_results=clause_results,
                steps_covered=steps_covered,
                explanation=explanation,
            )
            event = make_event("run_verdict", payload)
            ctx.event_bus.publish(event)
            return _ok(
                command,
                event=event.model_dump(mode="json"),
                steps_covered=steps_covered,
            )
        except Exception as exc:
            return _error(command, exc, verdict=verdict)


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
