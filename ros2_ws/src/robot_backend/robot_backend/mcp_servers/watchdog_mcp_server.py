"""MCP server for WatchDog recorder and monitor tools."""

from __future__ import annotations

import atexit
import json
import math
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
from robot_backend.watchdog.authored import AuthoredCodeError, AuthoredMonitor
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
from robot_backend.mcp_servers import role_context

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
    from mcp.types import ToolAnnotations
except ImportError as exc:
    FastMCP = None  # type: ignore[assignment]
    ToolAnnotations = None  # type: ignore[assignment]
    _MCP_IMPORT_ERROR = exc
else:
    _MCP_IMPORT_ERROR = None

# Claude Code caps an MCP tool result at roughly 50 KB of characters
# (independently of MAX_MCP_OUTPUT_TOKENS): anything larger is saved to a
# session file and the model gets a ~2 KB preview it is expected to Read.
# These sessions deny Read by design, so an over-limit result is simply LOST
# to the agent. This vendor annotation is the documented per-tool override.
# It goes on every tool whose full result the agent must receive inline:
# get_rendered_task crossed the default at 51.5 KB when the S2 prompt landed
# (observed 2026-09-25), and get_evidence/get_events can grow past it on a
# long window. ToolAnnotations(extra="allow") carries the namespaced key.
_LARGE_RESULT_ANNOTATIONS = (
    ToolAnnotations(**{"anthropic/maxResultSizeChars": 400_000})
    if ToolAnnotations is not None
    else None
)


class _UnservableToolRegistry:
    """Stand-in for FastMCP that registers tools as plain functions.

    Used when FastMCP cannot be imported, so the tool block below still
    DEFINES its functions at module scope instead of being skipped entirely.

    Why this exists rather than `mcp = None`: the tools live inside an
    `if mcp is not None:` block, so a missing FastMCP silently removed all 18
    of them from the module. Every unit test that calls one then failed at
    import-time attribute lookup -- `module has no attribute
    'record_run_verdict'` -- which reads as a broken test rather than a
    missing dependency. Five tests covering executor_terminal_status were dead
    that way, and the field shipped wrong (aborted recorded as
    no_terminal_event) with its regression tests passing by not running.

    Tool bodies are ordinary Python and do not need a server to be exercised,
    so their testability should not depend on which MCP library happens to be
    installed. Serving still does: main() refuses to start on this shim rather
    than pretending, and call logging is not installed on it because there is
    no session to log to.
    """

    def tool(self, *args: Any, **kwargs: Any):
        def register(func: Any) -> Any:
            return func

        return register


# True only when this process can actually serve MCP. Kept separate from
# `mcp` being non-None, which no longer implies servability.
_MCP_SERVABLE = FastMCP is not None

mcp = FastMCP("robot_backend_watchdog") if _MCP_SERVABLE else _UnservableToolRegistry()
# Must precede the tool definitions below: it patches mcp.tool() so every tool
# registered afterwards records its call to mcp_calls_<role>.jsonl. Passed None
# on the shim -- there is no run to log, and the wrapper would try to open a
# session file during a unit test.
role_context.install_call_logging(mcp if _MCP_SERVABLE else None, "watchdog")

# --- scenario gating --------------------------------------------------------
# Which tool surface this server exposes. Read once at import, like the role
# env vars: the scenario is a property of the launch, not of any call.
_SCENARIO = os.environ.get("WATCHDOG_SCENARIO")
_VALID_SCENARIOS = {"S1", "S2", "S3"}


def _tool_for(*scenarios: str):
    """mcp.tool() when this scenario serves the tool, else a no-op decorator.

    The no-op keeps the function defined at module scope, so unit tests can
    call it whatever WATCHDOG_SCENARIO is; it just is not registered with the
    live server. On the shim (FastMCP unimportable) everything registers,
    since the shim registers nothing anyway and main() refuses to serve it.
    """
    if not _MCP_SERVABLE or _SCENARIO in scenarios:
        return mcp.tool()

    def _unregistered(func: Any) -> Any:
        return func

    return _unregistered


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
# "incomplete" is run-scoped only: the executor stopped before the end-state
# could exist, and nothing fired. It is deliberately distinct from
# "inconclusive" (WatchDog's own evidence was insufficient to judge) and from
# "non_conformant" (a violation was actually observed), so the scorer's
# confusion matrix can separate "executor gave up" from "WatchDog caught
# something" from "WatchDog could not tell".
_VALID_VERDICTS = {
    "conformant", "non_conformant", "valid_variation", "inconclusive", "incomplete",
}
# Step verdicts that are NOT findings, i.e. do not put their step into the
# final rollup's findings_during_run. A conformant step went right; a
# valid_variation went right by an unexpected route, which the scorer counts
# as correct behaviour. Everything else -- non_conformant, and inconclusive,
# where WatchDog could not establish that the step was sound -- is something
# that happened to this run and belongs on its timeline.
_NON_FINDING_VERDICTS = {"conformant", "valid_variation"}
# Per-process record of step_ids a verdict has already been published for.
# Detection, not prevention -- a second verdict for a retried step may be
# legitimate, and silently dropping it would be worse than recording it as
# suspicious. Guarded by _CONTEXT_LOCK rather than its own lock since it is
# always touched alongside _CONTEXT.
_RECORDED_VERDICT_STEP_IDS: set[str] = set()

# Sentinel terminal status for a run that produced no run_ended event at all.
# Deliberately a non-null string rather than None: None is indistinguishable
# from "the field was never populated", and the scorer needs to tell a crash
# apart from a bookkeeping miss.
_NO_TERMINAL_EVENT = "no_terminal_event"
# Consecutive wait_for_event timeouts after which the server finalizes the run
# ITSELF. Not a suggestion to the agent -- see _timeout_or_finalize. Two, so a
# single long legitimate gap (the cell gate can block up to gate_timeout_sec,
# 90s, well under one 180s wait) can never reach it.
_MAX_CONSECUTIVE_TIMEOUTS = 2


@dataclass
class _RunTermination:
    """Server-side run state the termination guarantee is built on.

    Lives here, in the MCP server, because for the two-session Claude Code
    path there IS no harness loop -- the only code that runs between agent
    turns is this server answering tool calls. A backstop whose whole purpose
    is bounding a run when things have gone wrong cannot be implemented as an
    instruction to the agent that has already stopped behaving.

    observed_terminal_status is populated from the run_ended event as it is
    drained, NOT from an argument the agent passes. The agent getting it right
    on a clean run and wrong on an aborted one is exactly what happened before
    this existed.
    """

    observed_terminal_status: str | None = None
    steps_seen: list[str] = None  # type: ignore[assignment]
    # step_ids a step-scoped finding was recorded for, in order, de-duplicated.
    # Ordered, unlike _RECORDED_VERDICT_STEP_IDS, because it is reported on the
    # final run_verdict as the run's finding timeline and the sequence is part
    # of what a scorer reads. Populated from observed record_verdict calls, not
    # from anything the agent asserts at the end.
    findings_recorded: list[str] = None  # type: ignore[assignment]
    consecutive_timeouts: int = 0
    finalized: bool = False

    def __post_init__(self) -> None:
        if self.steps_seen is None:
            self.steps_seen = []
        if self.findings_recorded is None:
            self.findings_recorded = []


_RUN = _RunTermination()
_RUN_LOCK = threading.Lock()


def _resolve_observed_terminal_status(ctx: Any) -> str | None:
    """The run's terminal status as OBSERVED, by any delivery path, or None.

    Consults the recorder's ingest buffer first and the drain-time capture
    second. The two disagree in exactly one direction that matters: the drain
    capture sees an event only if wait_for_event happened to drain it, while
    the ingest buffer holds every event the subscriber ever received, so a
    run_ended the agent read through get_events or get_evidence is visible
    here and was invisible before. See EvidenceRecorder.observed_terminal_status
    for the run this silently got wrong.

    The drain capture is kept as a fallback rather than deleted because the
    ingest buffer is bounded (maxlen 5000) and a very long run could in
    principle evict a run_ended that the capture still holds. Neither source
    can invent a status, so consulting both can only ever find more truth.
    """
    recorder = getattr(ctx, "recorder", None)
    if recorder is not None:
        try:
            status = recorder.observed_terminal_status()
        except Exception:  # pragma: no cover - never let bookkeeping break a verdict
            status = None
        if status:
            return status
    with _RUN_LOCK:
        return _RUN.observed_terminal_status


def _observe_drained_events(drained: dict[str, Any]) -> None:
    """Track run_ended status and observed step_ids off a wait_for_event drain.

    Called for every drain so the server's own picture of the run is built
    from observed events rather than anything the agent reports. A real event
    also resets the consecutive-timeout counter: the run is demonstrably alive.
    """
    records = []
    if drained.get("event") is not None:
        records.append(drained["event"])
    records.extend(drained.get("additional_events") or [])

    with _RUN_LOCK:
        _RUN.consecutive_timeouts = 0
        for rec in records:
            if not isinstance(rec, dict):
                continue
            envelope = rec.get("envelope") or {}
            payload = envelope.get("payload") or {}
            etype = rec.get("event_type")
            if etype == "run_ended":
                status = payload.get("status")
                if isinstance(status, str) and status:
                    _RUN.observed_terminal_status = status
            elif etype in ("begin_step", "end_step"):
                step_id = payload.get("step_id")
                if isinstance(step_id, str) and step_id and step_id not in _RUN.steps_seen:
                    _RUN.steps_seen.append(step_id)
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
        # Resolve WITHOUT minting a session for a run that has none.
        # ensure_session_dir() creates whatever resolve_session_dir() returns,
        # including the fresh timestamped path produced when the pointer is
        # missing or dangling -- so merely starting this server against a dead
        # session left an empty directory behind for a run that
        # get_rendered_task's guard was about to refuse anyway. Worse, that
        # directory is a plausible-looking session an operator can later mistake
        # for a real one. active_session_dir() returns None instead of inventing
        # a path, which is the same choice monitor_log_default() already makes
        # for the log location just below.
        active = session_helper.active_session_dir()
        if active is not None:
            active.mkdir(parents=True, exist_ok=True)
            session_dir = active
        else:
            # No session established: fall back to the legacy per-user location
            # rather than fabricate one. Frame captures land under
            # ~/.watchdog/frames, matching where monitor_log_default() puts the
            # event log in this same case.
            session_dir = Path.home() / ".watchdog"
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
            scenario=_SCENARIO,
        )
        engine = MonitorEngine(
            event_bus=event_bus,
            halt_client=halt_client,
            wake_event=wake_event,
            logger=node.get_logger(),
        )
        recorder.engine = engine
        # The authored lane runs whatever the scenario: arming is what the
        # scenario gates, and a lane with nothing armed just sleeps on its
        # wake event.
        engine.attach_recorder(recorder)
        engine.start_authored_lane()
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
        ctx.engine.stop_authored_lane()
    except Exception:
        pass
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


# Prefix for the dotted paths in threshold_source. "raw_spec" is the key
# get_contract returns an object's spec under, so a source string names the
# path the agent itself was handed -- the same convention the agent already
# uses in prose ("Threshold from registry raw_spec.tolerances.position_m").
_SPEC_PATH_PREFIX = "raw_spec"
# Relative tolerance for calling an agent-passed threshold equal to the
# registry's. Not exact equality: the value makes a JSON round trip between
# two processes, and a scorer must not see a spurious mismatch from a float
# repr. Far tighter than any real difference between a registry value and a
# hand-picked default.
_THRESHOLD_MATCH_REL_TOL = 1e-9


def _lookup_spec_field(spec: dict[str, Any] | None, dotted: str) -> Any:
    """Walk a dotted path (e.g. "tolerances.grasp_slip_tolerance_m") in a spec."""
    node: Any = spec
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node


def _resolve_arm_object_id(ctx: WatchDogContext, monitor: Any) -> str | None:
    """The object an arm is about, resolved exactly as a fire's object_id is.

    A monitor that is intrinsically about an object answers for itself via
    subject_object_id. effort_event watches a joint and has none, so the
    object is whatever the gripper holds -- asked of the engine at ARM time
    here, as the fire path asks it at fire time. The two can legitimately
    differ (a monitor armed on an empty gripper that fires while holding a
    part), and that difference is information, not an inconsistency.
    """
    subject = getattr(monitor, "subject_object_id", None)
    if subject:
        return subject
    held = getattr(ctx.engine, "held_object_id", None)
    return held() if callable(held) else held


def _resolve_threshold_provenance(
    ctx: WatchDogContext, monitor: Any, object_id: str | None
) -> dict[str, Any]:
    """Establish where this monitor's threshold actually came from.

    Monitors are armed from agent-supplied params; nothing in the arm path
    reads the registry. So provenance cannot be asserted from the code path --
    it has to be MEASURED, by comparing the value the agent passed against
    what the registry holds for this object and this family's field.

    The rule the whole field rests on: threshold_source names a registry path
    only when the values actually agree. Any other outcome says what happened
    instead. A source string that claimed the registry for a value that did
    not come from there would make the field worse than absent -- it reads as
    valid, so nothing downstream would question it, and the Class 4 mechanism
    (corrupt a registry value, prove the watchdog armed from the corrupted
    one) would report success for a watchdog that had ignored the registry
    entirely.

    On a Class 4 run the registry value IS the corrupted one, so a monitor
    armed from it verifies true against it. That is the intended result: the
    claim being checked is "armed from the registry as it stands", not "armed
    from a correct value".
    """
    field = getattr(monitor, "threshold_registry_field", None)
    param = getattr(monitor, "threshold_param", None)

    if not field or not param:
        # No registry-backed threshold for this family (region). A real
        # answer, distinct from a failure to look.
        return {
            "threshold_source": "structural",
            "threshold_verified": False,
            "threshold_registry_field": None,
            "threshold_registry_value": None,
        }

    qualified = f"{_SPEC_PATH_PREFIX}.{field}"

    if not object_id:
        # An effort monitor armed on an empty gripper: there is no object, so
        # no per-object registry field to check against. Not a mismatch.
        return {
            "threshold_source": "object_unresolved",
            "threshold_verified": False,
            "threshold_registry_field": qualified,
            "threshold_registry_value": None,
        }

    registry_value = _lookup_spec_field(_object_spec(ctx, object_id), field)
    try:
        registry_float = float(registry_value)
    except (TypeError, ValueError):
        registry_float = None

    try:
        passed = float(monitor.threshold)
    except (TypeError, ValueError):
        passed = None

    if registry_float is None:
        return {
            "threshold_source": "no_registry_field",
            "threshold_verified": False,
            "threshold_registry_field": qualified,
            "threshold_registry_value": None,
        }

    matches = (
        passed is not None
        and math.isfinite(passed)
        and math.isclose(passed, registry_float, rel_tol=_THRESHOLD_MATCH_REL_TOL)
    )
    return {
        # The one place a registry path is asserted, and only behind `matches`.
        "threshold_source": qualified if matches else "agent_literal",
        "threshold_verified": bool(matches),
        "threshold_registry_field": qualified,
        "threshold_registry_value": registry_float,
    }


def _resolve_param_provenance(
    ctx: WatchDogContext, monitor: Any, object_id: str | None
) -> list[dict[str, Any]]:
    """Verify every registry-sourced authored param against the registry.

    One entry per param whose declared source starts "raw_spec.": the same
    measurement _resolve_threshold_provenance makes for the one threshold
    param, applied to each of them, with the same tolerance and the same rule
    -- verified is True only when the passed value matches what the registry
    holds for this object and field.
    """
    entries: list[dict[str, Any]] = []
    sources = getattr(monitor, "param_sources", None) or {}
    spec = _object_spec(ctx, object_id) if object_id else None
    prefix = _SPEC_PATH_PREFIX + "."
    for name, source in sources.items():
        if not isinstance(source, str) or not source.startswith(prefix):
            continue
        raw = monitor.params.get(name)
        value = raw.get("value") if isinstance(raw, dict) else raw
        registry_value = _lookup_spec_field(spec, source[len(prefix):])
        try:
            registry_float = float(registry_value)
        except (TypeError, ValueError):
            registry_float = None
        try:
            passed = float(value)
        except (TypeError, ValueError):
            passed = None
        verified = (
            registry_float is not None
            and passed is not None
            and math.isfinite(passed)
            and math.isclose(passed, registry_float, rel_tol=_THRESHOLD_MATCH_REL_TOL)
        )
        entries.append({
            "name": name,
            "value": value,
            "source": source,
            "verified": bool(verified),
            "registry_value": registry_float,
        })
    return entries


def _save_authored_source(session_dir: Path, monitor_id: str, source: str) -> Path:
    """Write authored source under <session_dir>/authored_monitors/.

    Never overwrites: an existing file gets a numeric suffix instead, so two
    arms with the same monitor_id (a re-arm after a fire, or a collision)
    both leave their exact source on the record.
    """
    out_dir = session_dir / "authored_monitors"
    out_dir.mkdir(parents=True, exist_ok=True)
    safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(monitor_id)).strip("_") or "monitor"
    path = out_dir / f"{safe}.py"
    suffix = 1
    while path.exists():
        path = out_dir / f"{safe}_{suffix}.py"
        suffix += 1
    path.write_text(source, encoding="utf-8")
    return path


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
        drained = {
            "wake": first.get("event_type"),
            "event": event_wrapper(first),
            "additional_events": [event_wrapper(event) for event in events[1:]],
        }
        _observe_drained_events(drained)
        return drained
    return None


def _observed_findings() -> list[str]:
    """step_ids the server saw a step-scoped finding recorded for, in order."""
    with _RUN_LOCK:
        return list(_RUN.findings_recorded)


def _note_finding(step_id: str) -> None:
    """Record that a step-scoped finding was published for this step.

    Appended in call order and de-duplicated: a step with two fires produces
    two verdicts but is one place the run went wrong, and the final rollup
    reports places, not events.
    """
    with _RUN_LOCK:
        if step_id not in _RUN.findings_recorded:
            _RUN.findings_recorded.append(step_id)


def _publish_run_verdict(
    ctx: WatchDogContext,
    *,
    verdict: str,
    clause_results: list[dict[str, Any]],
    steps_covered: list[str],
    explanation: str,
    claimed_outcome: str | None,
    executor_terminal_status: str | None,
    findings_during_run: list[str] | None = None,
) -> Any:
    """Build and publish one run-scoped verdict event. Returns the envelope.

    Shared by record_run_verdict (the agent's path) and _timeout_or_finalize
    (the server's own path) so a server-recorded rollup is the same event,
    on the same channel, in the same shape as an agent-recorded one. A scorer
    reading run_verdict events cannot tell -- and should not need to -- which
    of the two wrote it, beyond the explanation saying so.
    """
    # Both derived from what the server observed, never from an argument. The
    # caller may pass them only so the two finalize paths can reuse a list they
    # already read under the lock; omitting them re-reads it here.
    findings = list(findings_during_run if findings_during_run is not None
                    else _observed_findings())
    payload = RunVerdictPayload(
        verdict=verdict,
        claimed_outcome=claimed_outcome,
        clause_results=clause_results,
        steps_covered=steps_covered,
        explanation=explanation,
        executor_terminal_status=executor_terminal_status,
        findings_during_run=findings,
        # A run "recovered" only if it went wrong AND ended right. Always
        # computed, never a parameter: it is a function of the other two
        # fields, and allowing it to be passed would allow it to disagree
        # with them.
        recovered=bool(findings) and verdict == "conformant",
    )
    event = make_event("run_verdict", payload)
    ctx.event_bus.publish(event)
    return event


def _timeout_or_finalize(ctx: WatchDogContext, command: str, t0: float) -> dict[str, Any]:
    """Return a wait_for_event timeout, finalizing the run if it has run out.

    THE TERMINATION GUARANTEE. The failure this exists to stop: a real run
    where the executor was killed mid-step, both 180s waits timed out exactly
    as designed, and the agent read each empty result as "keep waiting" --
    still looping at 456s, no verdict ever recorded, no exit. The timeout
    firing was never the problem; the timeout firing not CAUSING anything was.

    So the server does not ask. On the Nth consecutive timeout it records the
    incomplete rollup itself, off its own observed state, and every later
    wait_for_event returns immediately instead of blocking another full
    timeout. An agent that ignores the terminal result cannot hang the run and
    cannot cost it its verdict, because the verdict is already written.
    """
    waited = time.monotonic() - t0
    with _RUN_LOCK:
        # Already finalized: never block again, and never record twice.
        if _RUN.finalized:
            return _ok(
                command,
                wake="terminated",
                terminal=True,
                waited_sec=waited,
                run_finalized=True,
                finalized_by="server",
                message=(
                    "This run is over and its verdict is already recorded. Stop "
                    "calling tools and end your turn."
                ),
            )
        _RUN.consecutive_timeouts += 1
        count = _RUN.consecutive_timeouts
        if count < _MAX_CONSECUTIVE_TIMEOUTS:
            return _ok(
                command,
                wake="timeout",
                waited_sec=waited,
                consecutive_timeouts=count,
                timeouts_before_forced_finalize=_MAX_CONSECUTIVE_TIMEOUTS - count,
                message=(
                    "No event within the timeout. If the executor has stopped "
                    "without a run_ended event it has probably crashed: finalize "
                    f"now. After {_MAX_CONSECUTIVE_TIMEOUTS - count} more such "
                    "timeout(s) this server will record the incomplete rollup "
                    "for you and end the run."
                ),
            )
        # Nth timeout: finalize here, whatever the agent does next.
        steps = list(_RUN.steps_seen)
        _RUN.finalized = True

    # Resolved outside the lock, and from the same any-delivery-path source
    # record_run_verdict uses. This path chooses between two very different
    # explanations, so reading only the drain capture could assert "the
    # executor probably crashed" about a run that had cleanly reported
    # aborted through get_events.
    observed = _resolve_observed_terminal_status(ctx)
    terminal_status = observed or _NO_TERMINAL_EVENT
    findings = _observed_findings()
    # Findings are stated here rather than left implicit because this
    # explanation is the only account a server-finalized run gets. Saying "no
    # clause check failed" unconditionally, as this did, would have been flatly
    # untrue on a run that recorded findings and then went silent.
    if findings:
        findings_note = (
            f" Findings were recorded during the run at: {findings}. Those step "
            f"verdicts stand as the run's timeline; this rollup does not "
            f"re-judge them, and it cannot say whether they were subsequently "
            f"corrected, because the run stopped before that could be observed."
        )
    else:
        findings_note = (
            " No monitor fire was adjudicated and no clause check failed on the "
            "evidence gathered."
        )
    if observed is None:
        explanation = (
            f"Run terminated on the no-event timeout after "
            f"{_MAX_CONSECUTIVE_TIMEOUTS} consecutive waits with no executor "
            f"event and no run_ended: the executor probably crashed. Recorded "
            f"by the server, not the monitoring agent. Steps observed before "
            f"the silence: {steps or 'none'}.{findings_note} The verdict is "
            f"incomplete rather than non_conformant because the run stopped "
            f"before its end-state could exist, which is a statement about "
            f"reachability, not about whether anything went wrong."
        )
    else:
        explanation = (
            f"Run ended with executor status {observed!r}, and the monitoring "
            f"agent did not record the final verdict before falling silent for "
            f"{_MAX_CONSECUTIVE_TIMEOUTS} consecutive timeouts. Recorded by the "
            f"server as a backstop. Steps observed: {steps or 'none'}."
            f"{findings_note} The server cannot measure the end-state itself, "
            f"so this is incomplete: it records that the run terminated without "
            f"an end-state assessment, not that the end-state was wrong."
        )

    try:
        _publish_run_verdict(
            ctx,
            verdict="incomplete",
            clause_results=[],
            steps_covered=steps,
            explanation=explanation,
            claimed_outcome=None,
            executor_terminal_status=terminal_status,
            findings_during_run=findings,
        )
        recorded = True
    except Exception as exc:  # pragma: no cover - publish failure is not fatal
        ctx.node.get_logger().error(f"forced finalize could not publish: {exc}")
        recorded = False

    return _ok(
        command,
        wake="terminated",
        terminal=True,
        waited_sec=waited,
        run_finalized=True,
        finalized_by="server",
        verdict_recorded=recorded,
        verdict="incomplete",
        executor_terminal_status=terminal_status,
        steps_covered=steps,
        findings_during_run=findings,
        message=(
            "Run terminated: no executor event across "
            f"{_MAX_CONSECUTIVE_TIMEOUTS} consecutive timeouts. The server has "
            "recorded run_verdict=incomplete on your behalf. Do not call "
            "record_run_verdict. Stop calling tools and end your turn."
        ),
    )


# Always entered: `mcp` is either a real FastMCP or the shim above, so the tool
# functions are defined at module scope either way. See
# _UnservableToolRegistry for why this was effectively conditional before and
# what that cost.
if mcp is not None:

    @mcp.tool(annotations=_LARGE_RESULT_ANNOTATIONS)
    def get_rendered_task(part: str | None = None) -> dict[str, Any]:
        """Return the WATCHDOG's system prompt and rendered task. Call first.

        Reads WATCHDOG_TASK_PATH only. This tool is hosted here, on the server
        the watchdog session loads and the executor session does not, so
        neither role has a tool capable of resolving the other's task file.
        That separation is load-bearing: the watchdog's file states only what
        conformant execution looks like, while the executor's may carry extra
        or faulty steps, and a watchdog that could read the executor's file
        would be judging against the fault rather than against the spec.

        Returns system_prompt and task as separate fields: load system_prompt
        as your standing instructions and task as your opening task, before
        calling any other tool.

        part selects which of the two large fields the result carries:
        "system_prompt" returns everything except task, "task" returns
        everything except system_prompt, and omitting it returns both.
        The combined result is ~50 KB, which some MCP clients truncate to a
        short preview; the two part results are each safely under that. The
        remaining fields (role, scenario, sources, session_dir, session_rule,
        instructions) are identical on every call.
        """

        command = "get_rendered_task"
        try:
            if part not in (None, "system_prompt", "task"):
                raise ValueError(
                    'part must be "system_prompt", "task", or omitted.'
                )
            result = role_context.build_rendered_task(role_context.WATCHDOG_ROLE)
            result["scenario"] = _SCENARIO
            if part == "system_prompt":
                result.pop("task", None)
                result["part"] = part
            elif part == "task":
                result.pop("system_prompt", None)
                result["part"] = part
            return result
        except Exception as exc:
            return _error(command, exc)


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

    @_tool_for("S1", "S2")
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
            # Resolve before arming: engine.arm emits the arm event, and the
            # event is the artifact the scorer reads, so provenance has to be
            # on the monitor by then rather than attached afterwards.
            arm_object_id = _resolve_arm_object_id(ctx, monitor)
            monitor.provenance = {
                "object_id": arm_object_id,
                **_resolve_threshold_provenance(ctx, monitor, arm_object_id),
            }
            result = ctx.engine.arm(monitor)
            # Returned in the tool result as well as published on the event,
            # so mcp_calls_watchdog.jsonl carries the object, threshold and
            # source without the scorer having to join to events_watchdog.
            return _ok(command, **result)
        except ValueError as exc:
            return _error(command, exc, monitor_type=monitor_type)
        except Exception as exc:
            return _error(command, exc, monitor_type=monitor_type)

    @_tool_for("S2", "S3")
    def arm_custom_monitor(
        code: str,
        description: str,
        params: dict[str, Any],
        clause: dict[str, Any] | None = None,
        object_id: str | None = None,
        threshold_param: str | None = None,
        monitor_id: str | None = None,
    ) -> dict[str, Any]:
        """Arm one monitor whose predicate is the supplied Python source.

        Code contract. `code` must define exactly one top-level function
        check(ctx) taking one positional argument. Its return value is read
        as: None -- the monitored condition holds; a dict with a finite
        numeric "value" key and an optional str "detail" key -- a violation,
        which fires with that value; anything else disarms the monitor with
        reason authored_bad_return. Top level may contain only function
        definitions (helper functions are allowed) and assignments of literal
        constants, at most 8000 characters in total.

        First call. check(ctx) runs once during this call, with ctx.state an
        empty dict. Its return value is stored and returned here as
        first_call_result; it does NOT fire. State written to ctx.state on
        that call persists across every later evaluation of the same monitor.

        Evaluation. After arming, check(ctx) runs on a dedicated thread, woken
        by incoming samples: TF ticks arrive at 20 Hz and joint-state messages
        at the publisher's rate, and the monitor is evaluated at most once per
        wake. A monitor fires at most once and is then disarmed.

        Budgets and disarm reasons. Each call has a 20000-line execution
        budget and a 20 ms wall-time budget. Exceeding either counts one
        strike (a line-budget overrun also suppresses any fire from that
        call); at 5 strikes the monitor is disarmed with reason
        authored_timeout. A return outside the contract disarms with
        authored_bad_return. Any other exception disarms with authored_error.

        The ctx object. check receives a MonitorContext and nothing else:
          ctx.params -- dict of the plain param values passed here.
          ctx.state -- dict, persists across calls of this monitor.
          ctx.object_ids() / ctx.frame_ids() -> list[str].
          ctx.joint_names() -> list[str], names in the latest joint sample.
          ctx.object_pose(object_id) -> {"wall_ns", "object_id", "pose":
              {"position": {x,y,z}, "orientation": {x,y,z,w}}} or None.
              Positions in meters.
          ctx.frame_pose(frame_id) -> same shape with "frame_id", or None.
          ctx.transform(parent, child) -> {"translation": {x,y,z},
              "rotation": {x,y,z,w}}. Meters; raises if the TF lookup fails.
          ctx.joint(name) -> {"wall_ns", "position", "effort"} or None.
              Effort in the units reported on /joint_states, not newtons.
          ctx.joint_window(name, seconds) -> list of {"wall_ns", "position",
              "effort"} over [now-seconds, now], at most 1000 samples.
          ctx.object_pose_window(object_id, seconds) /
          ctx.frame_pose_window(frame_id, seconds) -> lists of the pose
              sample shape above, at most 1000 samples.
          ctx.held_object() -> object_id the gripper holds, or None.
          ctx.now_wall_ns() -> int wall-clock nanoseconds.
          ctx.dist(a, b) -> float, Euclidean distance between two {x,y,z}
              dicts, meters.
        All returned dicts are fresh copies. wall_ns values are wall-clock
        nanoseconds, the same axis every event and evidence buffer uses.

        Allowed names. Builtins: abs, all, any, bool, dict, enumerate, float,
        int, len, list, max, min, range, reversed, round, set, sorted, str,
        sum, tuple, zip, isinstance, ValueError, ZeroDivisionError. The
        stdlib `math` module is provided as `math`. Nothing else -- no
        imports and no numpy.

        Forbidden constructs, rejected before compiling (the error names the
        stage and line): import, while, class, async def, await, yield,
        global, nonlocal; any name or attribute starting with "_"; eval,
        exec, compile, open, getattr, setattr, delattr, globals, locals,
        vars, dir, input, breakpoint, help, memoryview, type, super, object.

        Params and provenance. params is {name: {"value": v, "source": s}}
        where source is a dotted path like
        "raw_spec.tolerances.disturbance_tolerance_m" naming where the value
        was read from, or null for a value with no registry source. The plain
        values reach check as ctx.params. threshold_param, when given, names
        the param whose value is this monitor's threshold (it must be finite
        numeric); that param's source feeds threshold verification exactly as
        a library monitor's is verified, and every param with a raw_spec
        source is verified the same way and reported in param_provenance.
        Without threshold_param the recorded threshold is 0.0. clause is
        carried unchanged onto the arm and fire events.

        object_id, when given, is the object this monitor is about and is
        used for fire attribution and provenance lookup; when omitted, the
        part currently held by the gripper is used, as for effort_event.

        The source is saved verbatim under the session directory
        (authored_monitors/<monitor_id>.py) and its sha256 is on the arm and
        fire events as code_sha256. channels_read lists the ctx methods the
        first call actually used.
        """
        command = "arm_custom_monitor"
        try:
            ctx = _ensure_watchdog()
            try:
                monitor = AuthoredMonitor(
                    source=code,
                    description=description,
                    params=params,
                    recorder=ctx.recorder,
                    engine=ctx.engine,
                    clause=clause,
                    object_id=object_id,
                    threshold_param=threshold_param,
                    monitor_id=monitor_id,
                    armed_wall_ns=wall_ns(),
                )
            except AuthoredCodeError as exc:
                return _error(
                    command, exc, stage=exc.stage, lineno=exc.lineno
                )
            arm_object_id = _resolve_arm_object_id(ctx, monitor)
            monitor.provenance = {
                "object_id": arm_object_id,
                **_resolve_threshold_provenance(ctx, monitor, arm_object_id),
                "param_provenance": _resolve_param_provenance(
                    ctx, monitor, arm_object_id
                ),
            }
            saved_path = _save_authored_source(
                ctx.session_dir, monitor.monitor_id, monitor.source
            )
            result = dict(ctx.engine.arm(monitor))
            # The arm dict already carries code_sha256/channels_read for an
            # authored monitor; update() rather than kwargs so the merge can
            # never raise on the duplicates.
            result.update(
                first_call_result=monitor.first_call_result,
                channels_read=monitor.channels_read,
                code_sha256=monitor.code_sha256,
                saved_path=str(saved_path),
            )
            return _ok(command, **result)
        except Exception as exc:
            return _error(command, exc)

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
        # 180s, not 120s. Measured worst-case LEGITIMATE executor silence:
        # inter-event gaps on a clean 12-step fmb_assembly_2 run peaked at
        # 17.1s (p95 6.7s), but the dominant gap is the cell gate wait and
        # gate_timeout_sec is 90.0 -- a gate that runs to its limit yields a
        # ~95s gap with nothing wrong. At 120s that left only ~25s of headroom
        # on a single wait. Tripping mid-run truncates a live run, which is far
        # worse than waiting longer on a real crash; and now that run_ended
        # covers every executor-initiated ending, this timeout fires only on a
        # hard crash, so erring long costs essentially nothing.
        timeout_sec: float = 180.0,
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
            # Already finalized: return at once rather than blocking out
            # another full timeout. An agent that keeps calling after the run
            # was force-ended costs a round trip, not 180 seconds.
            with _RUN_LOCK:
                already_final = _RUN.finalized
            if already_final:
                return _timeout_or_finalize(ctx, command, t0)

            deadline = t0 + timeout
            while True:
                immediate = _drain_wait(ctx, wanted)
                if immediate is not None:
                    return _ok(command, waited_sec=time.monotonic() - t0, **immediate)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return _timeout_or_finalize(ctx, command, t0)
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

    @_tool_for("S1", "S2")
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

    @mcp.tool(annotations=_LARGE_RESULT_ANNOTATIONS)
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

    @mcp.tool(annotations=_LARGE_RESULT_ANNOTATIONS)
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
                    # The recorder samples exactly the registry's enabled
                    # entries, so every id named here is one the registry
                    # claims the stage publishes. If an id stays cold across
                    # several retries it is a registry/stage disagreement, not
                    # slow warm-up: either the prim is missing from the stage
                    # or the entry should be enabled: false. Say so, because
                    # the failure this replaces was silent -- the agent
                    # retried, gave up, moved to step scope, and every
                    # executor gate then paid its full timeout waiting on a
                    # run-scope signal that could never come.
                    frames = getattr(ctx.recorder, "object_frames", {}) or {}
                    cold_by_kind: dict[str, list[str]] = {}
                    for oid in cold_objects:
                        entry_kind = (frames.get(oid) or {}).get("kind", "object")
                        cold_by_kind.setdefault(entry_kind, []).append(oid)
                    return _error(
                        command,
                        "recorder buffers are not warm yet: "
                        f"enabled entries with no buffered pose={cold_objects} "
                        f"(by kind: {cold_by_kind}), "
                        f"joint_state samples={joint_count}. Retry a few times; "
                        "if the same ids stay cold, the stage is not publishing "
                        "them and the registry's enabled flag is wrong for those "
                        "entries -- report that rather than signalling run-scope "
                        "readiness, because leaving run scope unsignalled makes "
                        "every executor gate wait out its full timeout.",
                        counts=counts,
                        object_ids=object_ids,
                        cold_by_kind=cold_by_kind,
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

        explanation must summarise the DECISION BASIS, not just state the
        conclusion: what you observed (the evidence and the values that
        mattered), why that made this conformant or a departure -- including
        why something suspicious was judged within bounds when you did not
        flag it -- and, for a departure, what you believe physically happened
        (collision, wrong grasp, slip, wrong approach). Name the mechanism, and
        say plainly when the evidence does not settle it. A summary of the
        basis, not a transcript of your reasoning. This text is the only place
        a reviewer can tell a correct judgement from a lucky one.
        """
        command = "record_verdict"
        try:
            if verdict not in _VALID_VERDICTS:
                raise ValueError(f"verdict must be one of {sorted(_VALID_VERDICTS)}.")
            if verdict == "incomplete":
                # Run-scoped only: it describes the executor stopping early,
                # which is a property of the run, not of any one step. A step
                # the executor abandoned is a "failure"/"partial" claim whose
                # conformance is still judged on its own evidence.
                raise ValueError(
                    "verdict 'incomplete' is run-scoped -- use it on "
                    "record_run_verdict, not on a per-step record_verdict."
                )
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
            if verdict not in _NON_FINDING_VERDICTS:
                # Tracked here, at the moment of publication, so the final
                # rollup's findings_during_run is built from what was actually
                # recorded rather than from what the agent remembers at the end.
                _note_finding(step_id)
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
        executor_terminal_status: str | None = None,
    ) -> dict[str, Any]:
        """Publish the run's single final verdict. ALWAYS call this, once.

        Every run ends with exactly one call here, at termination, whether or
        not you recorded findings along the way. Findings and this rollup are
        complementary: your step verdicts are the TIMELINE (what went wrong,
        when), and this is the END-STATE assessment (did the run ultimately
        succeed). A run with three findings still gets exactly one of these.

        This replaces the old rule that a run with a finding records no
        rollup. That rule lost a real distinction: a run that misseated a part
        and then reseated it correctly produced the same record as one that
        ended broken -- a single non_conformant step verdict and nothing else,
        with nothing anywhere saying how the run ENDED.

        What the verdict value means here. It is about the END-STATE at
        termination, not a summary of everything that happened:

          conformant     every part's final pose meets its placement frame
                         within tolerance, and no invariant violation still
                         stands at run end. A finding you recorded earlier
                         that was subsequently corrected does NOT make this
                         non_conformant -- the run recovered, and that is a
                         real and different outcome from both a clean run and
                         a failed one.
          non_conformant at least one part's end-state is wrong at run end, or
                         a violation stands uncorrected. The run ended broken.
          incomplete     the executor aborted or crashed before the end-state
                         could exist.

        Judge it on YOUR OWN evidence: query each placement frame and compare
        each part's final pose against it. The executor's run_ended status
        tells you the run STOPPED, and it is recorded beside your verdict for
        the independence comparison, but it must not be what decides
        conformant versus non_conformant. An executor that claims "completed"
        on a run that ended misseated is precisely the case this separation
        exists to catch.

        When you recorded findings, explanation must tie the timeline to the
        end-state -- name what was found and say what became of it. For
        example: "a non-conformance was recorded at place_u_orange (u_orange
        misseated by 0.032 m); the executor then ran an unplanned
        reseat_u_orange and u_orange's final pose is 0.004 m from
        u_orange_placement, within the 0.02 m tolerance, so the run's
        end-state is conformant."

        findings_during_run and recovered are filled in by this server from
        the record_verdict calls it actually saw; you do not pass them and
        cannot override them.

        clause_results carries the same entries record_verdict takes, with one
        addition: every entry must also name its own step_id. clause_id is
        "<section>.<field>" and is identical across all four parts, so without
        it the rollup cannot say which part a result belongs to.

        steps_covered lists every step_id this verdict accounts for. It is a
        real argument rather than a sentence in explanation so coverage can be
        checked by set comparison against the task's steps: block, the same way
        clause_id makes clause coverage checkable.

        explanation must summarise the DECISION BASIS, the same as
        record_verdict's: what you observed across the run and what it
        established about the end-state, including why anything that looked
        suspicious was judged within bounds. On a run with no findings, "no
        findings" is a claim about evidence you examined, and this is where
        that examination is shown. State plainly that the verdict is the
        run's final end-state assessment.

        claimed_outcome is optional here and, when given, should summarize the
        executor's claims across the run rather than any single step's.

        verdict may be "incomplete" here and only here: the executor stopped
        before the end-state could exist. That is distinct from "inconclusive"
        (your own evidence was insufficient to judge) and from
        "non_conformant" (the end-state is wrong). Say in explanation which
        parts were and were not reached.

        A recorded finding no longer forces the run verdict. It used to, on
        the old cadence where a finding suppressed this call entirely. Now the
        question here is only what the END-STATE was, so an abort that
        happened because you caught something is "incomplete" if the remaining
        parts were never reached, and "non_conformant" if what was reached
        ended wrong -- the finding itself is already on the record as its own
        step verdict and is reported here in findings_during_run either way.

        executor_terminal_status is the status from the executor's run_ended
        event -- "completed", "aborted", or None if none arrived and you
        terminated on the no-event timeout. Record it verbatim. It is logged
        BESIDE your verdict for the scorer to compare; it must never change
        what your verdict is. An executor claiming "completed" on a run you
        judge non_conformant is exactly the comparison this field exists for.
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
            # CODE-populated, not agent-supplied. The agent passed None on
            # both the clean and the aborted run, so the argument is only a
            # fallback for a caller that somehow has the status when the
            # server does not; observation wins.
            #
            # Resolved from what was OBSERVED by any delivery path, not from
            # the wait_for_event drain alone. Keying it to the drain made the
            # sentinel appear on a run whose run_ended was read through
            # get_events -- collapsing "aborted" (executor gave up and said
            # so) onto "no_terminal_event" (hard crash, said nothing), which
            # are the two most different outcomes this field exists to
            # separate, and doing it silently with a value that looks valid.
            #
            # The sentinel must therefore mean exactly one thing: no run_ended
            # was ever ingested. It is still an explicit string rather than
            # None so a crash stays distinguishable from a field nobody filled
            # in.
            observed_status = _resolve_observed_terminal_status(ctx)
            resolved_status = observed_status or executor_terminal_status or _NO_TERMINAL_EVENT
            if (
                observed_status is not None
                and executor_terminal_status is not None
                and executor_terminal_status != observed_status
            ):
                ctx.node.get_logger().warning(
                    f"record_run_verdict: agent passed executor_terminal_status="
                    f"{executor_terminal_status!r} but the observed run_ended said "
                    f"{observed_status!r}; recording the observed value."
                )
            event = _publish_run_verdict(
                ctx,
                verdict=verdict,
                clause_results=clause_results,
                steps_covered=steps_covered,
                explanation=explanation,
                claimed_outcome=claimed_outcome,
                executor_terminal_status=resolved_status,
            )
            with _RUN_LOCK:
                _RUN.finalized = True
            # EventEnvelope.payload is a plain dict (typed payloads are dumped
            # on construction), so read the published values back by key.
            published = event.payload
            return _ok(
                command,
                event=event.model_dump(mode="json"),
                steps_covered=steps_covered,
                executor_terminal_status=resolved_status,
                # Echoed so the agent can see what the server concluded about
                # its own timeline, and so the call log carries it without a
                # join to the event stream.
                findings_during_run=list(published.get("findings_during_run", [])),
                recovered=bool(published.get("recovered", False)),
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
    if not _MCP_SERVABLE:
        # The tool functions exist (on the shim) but nothing can serve them.
        # Fail here, loudly, rather than letting a shim reach mcp.run(): a
        # server that starts and exposes zero tools looks to the session like
        # a working server with nothing to offer.
        raise RuntimeError(
            "Cannot serve watchdog_mcp_server: FastMCP could not be imported "
            f"({_MCP_IMPORT_ERROR}). This server targets the mcp 1.x API "
            "(mcp.server.fastmcp.FastMCP); mcp 2.x renamed it to "
            "mcp.server.mcpserver.MCPServer. Pin 'mcp<2' in the workspace "
            "virtualenv or migrate this module to the 2.x API."
        ) from _MCP_IMPORT_ERROR

    if _SCENARIO not in _VALID_SCENARIOS:
        # Refuse to serve rather than default: an unset scenario would
        # silently serve a tool surface nobody chose, and which tools a run
        # had is exactly what the scenarios exist to control.
        raise RuntimeError(
            f"WATCHDOG_SCENARIO={_SCENARIO!r} is not a valid scenario. Set the "
            f"WATCHDOG_SCENARIO environment variable to one of "
            f"{sorted(_VALID_SCENARIOS)} (the watchdog*.mcp.json env block is "
            f"where the launch sets it)."
        )

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
