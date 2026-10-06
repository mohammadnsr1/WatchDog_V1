"""MCP server for the S4 EXECUTOR-SIDE monitor host.

S4 runs with no WatchDog at all: the executor authors and runs its own
monitors and records its own verdicts. This server is the executor session's
monitoring surface — arm_custom_monitor / dry_run_custom_monitor /
disarm_monitor / disarm_all_monitors / get_monitor_fires / query_snapshot /
get_evidence / record_observation / record_verdict / record_run_verdict —
re-hosted from the WatchDog server's machinery, in the EXECUTOR role.

DELIBERATELY A SEPARATE SERVER, not an S4 surface on watchdog_mcp_server.
s4_scorer's validity gate requires events_watchdog.jsonl to hold no monitor
or verdict events, and that gate is only evidence of independence if it holds
because no watchdog process ran — not because a watchdog process special-cased
its logging. This process:

  * logs its tool calls as the EXECUTOR (role_context derives role from
    EXECUTOR_TASK_PATH), into mcp_calls_executor.jsonl beside the
    scene/motion/gripper servers' calls;
  * publishes its events to the shared /watchdog/events topic and logs them
    to <session>/events_executor_monitor.jsonl — a non-watchdog events file
    that s4_scorer.load_s4_events picks up automatically — and never touches
    events_watchdog.jsonl;
  * owns its OWN MonitorEngine and EvidenceRecorder, subscribed directly to
    the ROS streams (TF, /joint_states, the event topic, the camera). It is
    a fresh instance in this process, never a client of an engine a watchdog
    process owns.

Engine-level behaviour (budgets, lanes, halt-on-fire, fire payloads with
trigger_wall_ns) is identical to S1–S3 by construction: the engine, the
authored-monitor machinery and the arm-time preparation path
(_prepare_authored_monitor) are the same code, reached with this server's
context. The measured S4 difference is adjudication latency and independence,
never halt speed — see s4_scorer.py.

Timestamps: wall_time_iso / wall_ns (wall clock) are the only values compared
across processes; payload timestamp_ns is per-process monotonic.
"""

from __future__ import annotations

import atexit
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.mcp_servers.executor_monitor_mcp_server")

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_backend.events.bus import EventBus
from robot_backend.events.schema import RunVerdictPayload, VerdictPayload, make_event
from robot_backend.vision.observation import (
    ObservationDeps,
    ObservationStore,
    record_observation as _run_record_observation,
)
from robot_backend.watchdog.authored import MONITOR_PURPOSES, json_safe, warm_numpy
from robot_backend.watchdog.engine import ARRAY_WALL_BUDGET_MS, MonitorEngine
from robot_backend.watchdog.halt import HaltClient
from robot_backend.watchdog.image_buffer import CameraStreams
from robot_backend.watchdog.recorder import (
    EvidenceRecorder,
    _load_registry,
    event_wrapper,
)
from robot_backend.watchdog import session as session_helper
from robot_backend.watchdog.timebase import wall_ns
from robot_backend.mcp_servers import role_context

# The shared monitor-host machinery. Importing the watchdog server module
# defines its tools on its OWN (never-served) FastMCP instance and boots no
# node, bus or log file — everything stateful there is created lazily by
# _ensure_watchdog(), which this process never calls: every shared helper
# used here either is pure or takes this server's context explicitly.
from robot_backend.mcp_servers.watchdog_mcp_server import (
    _DRY_RUN_MAX_TICKS,
    _DRY_RUN_TICK_PERIOD_SEC,
    _LARGE_RESULT_ANNOTATIONS,
    _MCP_IMPORT_ERROR,
    _MCP_SERVABLE,
    _NO_TERMINAL_EVENT,
    _UnservableToolRegistry,
    _VALID_VERDICTS,
    _NON_FINDING_VERDICTS,
    _clamp_rate_hz,
    _dry_run_error,
    _prepare_authored_monitor,
    _save_authored_source,
    _validate_clause_results,
    _validate_run_clause_results,
    _would_fire,
)

try:
    from mcp.server.fastmcp import FastMCP
except ImportError:
    FastMCP = None  # type: ignore[assignment]

mcp = (
    FastMCP("robot_backend_executor_monitor")
    if _MCP_SERVABLE and FastMCP is not None
    else _UnservableToolRegistry()
)
# Must precede the tool definitions: patches mcp.tool() so every call lands in
# mcp_calls_<role>.jsonl. Role is derived from EXECUTOR_TASK_PATH, so these
# calls append to the SAME mcp_calls_executor.jsonl the scene/motion/gripper
# servers write — which is where s4_scorer's serialization metrics read
# monitor-vs-action interleaving from.
role_context.install_call_logging(mcp if _MCP_SERVABLE else None, "executor_monitor")


@dataclass
class ExecutorMonitorContext:
    """Everything the tools need. Duck-type-compatible with WatchDogContext
    where the shared helpers (_prepare_authored_monitor and the provenance
    resolvers) read it: registry, engine, recorder, camera_streams,
    session_dir."""

    node: Node
    executor: MultiThreadedExecutor
    thread: threading.Thread
    registry: dict[str, Any]
    event_bus: EventBus
    recorder: EvidenceRecorder
    engine: MonitorEngine
    halt_client: HaltClient
    wake_event: threading.Event
    session_dir: Path
    camera_streams: CameraStreams


_CONTEXT: ExecutorMonitorContext | None = None
_CONTEXT_LOCK = threading.Lock()
_OBSERVATIONS = ObservationStore()
# Per-process verdict bookkeeping, mirroring the watchdog host's: findings are
# tracked at publication so the final rollup's findings_during_run comes from
# what was recorded, never from what the agent remembers at the end.
_FINDINGS_RECORDED: list[str] = []
_RECORDED_VERDICT_STEP_IDS: set[str] = set()
_RUN_VERDICTS_PUBLISHED = 0


def _event_log_default() -> str:
    """Default events_executor_monitor.jsonl path.

    events_<name>.jsonl in the session directory, like every other server's
    event log — and NOT events_watchdog.jsonl, whose emptiness is s4_scorer's
    validity evidence that no watchdog ran.
    """
    base = session_helper.active_session_dir() or (Path.home() / ".watchdog")
    return str(base / "events_executor_monitor.jsonl")


def _ensure_monitor_host() -> ExecutorMonitorContext:
    global _CONTEXT
    with _CONTEXT_LOCK:
        if _CONTEXT is not None:
            return _CONTEXT
        if not rclpy.ok():
            rclpy.init(args=None)
        node = Node("executor_monitor_node")
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        client_cb_group = MutuallyExclusiveCallbackGroup()
        wake_event = threading.Event()
        registry = _load_registry()
        active = session_helper.active_session_dir()
        if active is not None:
            active.mkdir(parents=True, exist_ok=True)
            session_dir = active
        else:
            session_dir = Path.home() / ".watchdog"
        log_path = os.environ.get("EXECUTOR_MONITOR_EVENT_LOG", _event_log_default())
        event_bus = EventBus(node=node, log_path=log_path)
        recorder = EvidenceRecorder(node=node, registry=registry, wake_event=wake_event)
        # Camera path for authored array-lane monitors (ctx.rgb()/ctx.depth()).
        # This host has no capture_frames, so no ring buffer or manager — the
        # streams own their subscriptions directly, RGB included.
        camera_streams = CameraStreams(logger=node.get_logger())
        try:
            camera_streams.attach(node, ReentrantCallbackGroup(), subscribe_rgb=True)
        except Exception as exc:
            node.get_logger().warning(f"camera streams attach failed (non-fatal): {exc}")
        halt_client = HaltClient(
            node=node, event_bus=event_bus, callback_group=client_cb_group
        )
        engine = MonitorEngine(
            event_bus=event_bus,
            halt_client=halt_client,
            wake_event=wake_event,
            logger=node.get_logger(),
        )
        recorder.engine = engine
        engine.attach_recorder(recorder)
        engine.start_authored_lane()
        try:
            warm_numpy()
        except Exception as exc:
            node.get_logger().warning(f"numpy warm-up failed (non-fatal): {exc}")
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            halt_client.prewarm()
        except Exception as exc:
            node.get_logger().warning(f"halt client prewarm failed: {exc}")
        _CONTEXT = ExecutorMonitorContext(
            node=node,
            executor=executor,
            thread=thread,
            registry=registry,
            event_bus=event_bus,
            recorder=recorder,
            engine=engine,
            halt_client=halt_client,
            wake_event=wake_event,
            session_dir=session_dir,
            camera_streams=camera_streams,
        )
        return _CONTEXT


def _shutdown_monitor_host() -> None:
    global _CONTEXT
    with _CONTEXT_LOCK:
        ctx = _CONTEXT
        _CONTEXT = None
    if ctx is None:
        return
    for action in (
        lambda: ctx.engine.stop_authored_lane(),
        lambda: ctx.executor.shutdown(),
        lambda: ctx.thread.join(timeout=2.0),
        lambda: ctx.node.destroy_node(),
        lambda: ctx.event_bus.close(),
        lambda: rclpy.shutdown() if rclpy.ok() else None,
    ):
        try:
            action()
        except Exception:
            pass


atexit.register(_shutdown_monitor_host)


def _ok(command: str, **payload: Any) -> dict[str, Any]:
    return {"ok": True, "command": command, **payload}


def _error(command: str, error: Exception | str, **payload: Any) -> dict[str, Any]:
    return {"ok": False, "command": command, "error": str(error), **payload}


def _executor_look_ids(ctx: ExecutorMonitorContext) -> list[str]:
    """look_ids of the EXECUTOR's own looks, recovered from the event stream.

    look() lives on the scene server, a separate process, so this host's
    in-process look registry is empty by construction. The look events arrive
    over the shared event topic and sit in this recorder's ingest buffer;
    reading them from there keeps record_observation's evidence_look_ids
    validation meaningful across the process boundary.
    """
    ids: list[str] = []
    for record in ctx.recorder.latest_events(5000):
        if record.get("event_type") != "look":
            continue
        payload = record.get("payload") or {}
        look_id = payload.get("look_id")
        if payload.get("role") == "executor" and isinstance(look_id, str):
            ids.append(look_id)
    return ids


def _note_finding(step_id: str) -> None:
    with _CONTEXT_LOCK:
        if step_id not in _FINDINGS_RECORDED:
            _FINDINGS_RECORDED.append(step_id)


def _observed_findings() -> list[str]:
    with _CONTEXT_LOCK:
        return list(_FINDINGS_RECORDED)


# Always entered: `mcp` is a real FastMCP or the shim, so the tool functions
# exist at module scope for unit tests either way.
if mcp is not None:

    @mcp.tool()
    def arm_custom_monitor(
        code: str,
        description: str,
        params: dict[str, Any],
        clause: dict[str, Any] | None = None,
        object_id: str | None = None,
        threshold_param: str | None = None,
        monitor_id: str | None = None,
        rate_hz: float | None = None,
        purpose: str = "clause",
        threshold_rationale: str | None = None,
        halt_on_fire: bool = False,
    ) -> dict[str, Any]:
        """Arm one monitor whose predicate is the supplied Python source.

        THE RETURN CONTRACT. `code` must define exactly one top-level function
        check(ctx) taking one positional argument. Its return value is read
        as: None — the monitored condition holds; a dict with a finite
        numeric "value" key and an optional str "detail" key — a violation,
        which FIRES with that value and disarms the monitor; anything else
        disarms the monitor with reason authored_bad_return. Top level may
        contain only function definitions (helper functions are allowed) and
        assignments of literal constants, at most 8000 characters in total.
        Read params with ctx.value(name): ctx.params[name] is a
        {value, source} record, not the number. ctx.depth()/ctx.rgb() may
        return None.

        First call. check(ctx) runs once during this call, with ctx.state an
        empty dict. Its return value is stored and returned here as
        first_call_result; it does NOT fire. State written to ctx.state on
        that call persists across every later evaluation of the same monitor.

        Evaluation. After arming, check(ctx) runs on a dedicated thread. With
        rate_hz omitted (the scalar lane), it is woken by incoming samples:
        TF ticks arrive at 20 Hz and joint-state messages at the publisher's
        rate, and the monitor is evaluated at most once per wake. With
        rate_hz set (the ARRAY lane), it runs on its own period instead —
        rate_hz is clamped to at most 2.0 (the result reports rate_hz_clamped
        when that happened) — on a third thread, so array work never delays
        the other lanes. A monitor fires at most once and is then disarmed.
        A monitor has no step affiliation: it lives from this call until it
        fires, is disarmed by you, or is disarmed for a failure, however many
        steps that spans. Nothing takes it down at a step boundary, including
        a release — an armed monitor on a just-released object keeps running.

        Budgets and disarm reasons. Each call has a 20000-line execution
        budget and a wall-time budget: 20 ms on the scalar lane, 150 ms on
        the array lane (numpy C work emits no Python lines, so the wall clock
        is the real bound there). Exceeding either counts one strike (a
        line-budget overrun also suppresses any fire from that call); at 5
        strikes the monitor is disarmed with reason authored_timeout. A
        return outside the contract disarms with authored_bad_return. Any
        other exception disarms with authored_error. On the array lane,
        ctx.state may hold ndarrays (a previous frame, an ROI) up to 32 MB in
        total; exceeding that disarms with authored_bad_return.

        ctx API. This is the COMPLETE surface of the ctx object check
        receives; nothing else exists on it.
          ctx.params — dict. ctx.params[name] is the exact {"value": ...,
              "source": ...} mapping passed here for that param.
          ctx.value(name) -> the plain number inside params[name] ("value"
              if the entry is a mapping, else the entry itself).
          ctx.state — dict, persists across every call of this monitor.
              Empty on the arm-time first call.
          ctx.channels_read — list[str], the ctx methods (and first
              arguments) called so far, as "method:arg" strings.
          ctx.object_ids() -> list[str].
          ctx.frame_ids() -> list[str].
          ctx.joint_names() -> list[str], names in the latest joint sample;
              [] before the first sample.
          ctx.object_pose(object_id) -> {"object_id", "pose": {"position":
              {"x","y","z"}, "orientation": {"x","y","z","w"}}, "wall_ns",
              "tf_stamp_ns"} or None. Positions in meters. Samples from one
              recorder tick share one TF time (tf_stamp_ns).
          ctx.frame_pose(frame_id) -> {"frame_id", "pose": {"position":
              {"x","y","z"}, "orientation": {"x","y","z","w"}}, "wall_ns",
              "tf_stamp_ns"} or None. Same tick, same tf_stamp_ns.
          NOTE: ctx.object_ids, ctx.frame_ids, ctx.object_pose,
              ctx.frame_pose and their windows cover only the objects and
              frames registered for this cell; they cannot show anything
              else that is in the workspace.
          ctx.transform(parent, child) -> exactly {"translation":
              {"x","y","z"}, "rotation": {"x","y","z","w"}}: the child
              frame's pose expressed in the parent frame, meters. This is a
              single time-consistent TF lookup — the whole parent->child
              chain is evaluated at one common time. It never returns None;
              a lookup that cannot be resolved raises.
          ctx.joint(name) -> {"wall_ns", "position", "effort"} or None.
              Effort in the units reported on /joint_states, not newtons.
          ctx.joint_window(name, seconds) -> list of {"wall_ns", "position",
              "effort"} over [now-seconds, now], at most 1000 samples.
          ctx.object_pose_window(object_id, seconds) -> list of the
              object_pose shape above, at most 1000 samples.
          ctx.frame_pose_window(frame_id, seconds) -> list of the
              frame_pose shape above, at most 1000 samples.
          ctx.held_object() -> object_id the gripper holds, or None. Its
              value changes on the gripper's skill events: it becomes the
              object id at grasp_skill skill_start, the grasp is confirmed
              at a successful grasp_skill skill_end, and it returns None
              from release_skill skill_start onward and after a failed
              grasp.
          ctx.now_wall_ns() -> int wall-clock nanoseconds.
          ctx.dist(a, b) -> float, Euclidean distance between two
              {"x","y","z"} dicts, meters.
        Camera accessors, available ONLY when the monitor is armed with
        rate_hz (the array lane); on the scalar lane they raise an error
        naming rate_hz:
          ctx.rgb() -> HxWx3 uint8 ndarray (read-only, writeable=False) or
              None when no frame has arrived or the latest is older than
              1.0 s.
          ctx.depth() -> HxW float32 metres, NaN = no return at that pixel
              (read-only), or None on the same conditions as rgb.
          ctx.camera() -> {"width", "height", "K": 3x3 list, "frame_id",
              "world_T_camera": {"translation": [x,y,z],
              "rotation_xyzw": [x,y,z,w]}} or None without camera_info.
          ctx.deproject(u, v, z) -> {"x","y","z"} in the world frame, from K
              and world_T_camera; arrays of u, v, z are accepted and return
              an Nx3 ndarray.
          ctx.frame_stamps() -> {"rgb_wall_ns", "depth_wall_ns", "age_ms"}
              for this call's snapshot (age from the oldest present frame).
        Snapshot semantics: within ONE check(ctx) call, every camera accessor
        returns the same frames — one snapshot is taken per tick. Code must
        handle None from rgb()/depth().

        A pose accessor returns None when that frame or object has no
        buffered sample yet; code must guard for it. All returned dicts are
        fresh copies. wall_ns values are wall-clock nanoseconds, the same
        axis every event and evidence buffer uses.

        Allowed names. Builtins: abs, all, any, bool, dict, enumerate, float,
        int, len, list, max, min, range, reversed, round, set, sorted, str,
        sum, tuple, zip, isinstance, ValueError, ZeroDivisionError. The
        stdlib `math` module is provided as `math`. On the array lane numpy
        is provided as `np` (provided, not imported — import stays
        forbidden). Nothing else.

        Forbidden constructs, rejected before compiling (the error names the
        stage and line): import, while, class, async def, await, yield,
        global, nonlocal; any name or attribute starting with "_"; eval,
        exec, compile, open, getattr, setattr, delattr, globals, locals,
        vars, dir, input, breakpoint, help, memoryview, type, super, object.

        Params and provenance. params is {name: {"value": v, "source": s}},
        e.g. params={"change_m": {"value": 0.02}} — every entry is a mapping
        with a "value" key, never a bare number. source is a dotted path like
        "raw_spec.tolerances.disturbance_tolerance_m" naming where the value
        was read from, or null for a value with no registry source. Each
        entry reaches check unchanged as ctx.params[name]; ctx.value(name)
        returns the plain number. threshold_param, when given, names
        the param whose value is this monitor's threshold (it must be finite
        numeric); that param's value is compared against the registry this
        process loaded and the outcome is recorded as threshold_source and
        threshold_verified, and every param with a raw_spec source is
        verified the same way and reported in param_provenance. Without
        threshold_param the recorded threshold is 0.0. clause is carried
        unchanged onto the arm and fire events.

        object_id, when given, is the object this monitor is about and is
        used for fire attribution and provenance lookup; when omitted, the
        part currently held by the gripper is used.

        purpose is "clause" (default: a conformance monitor, exactly as
        described above) or "observation": a monitor whose threshold only
        decides when to look, not whether a requirement held. For
        "observation": halt_on_fire is refused; clause may be null; the
        threshold is recorded as threshold_source "agent_chosen" with
        threshold_verified false; threshold_param must name the params key
        holding the trigger value; and threshold_rationale (a non-empty
        string saying why this value) is REQUIRED and carried on the arm and
        fire events. Either purpose may use either lane.

        The source is saved verbatim under the session directory
        (authored_monitors/<monitor_id>.py) and its sha256 is on the arm and
        fire events as code_sha256. channels_read lists the ctx methods the
        first call actually used. Each fire this monitor produces carries
        monitor_id, triggering_value, threshold, comparison, detail, clause,
        object_id and trigger_wall_ns (wall-clock nanoseconds at trigger).
        """
        command = "arm_custom_monitor"
        try:
            if purpose not in MONITOR_PURPOSES:
                raise ValueError(
                    f"purpose must be one of {list(MONITOR_PURPOSES)}, got {purpose!r}."
                )
            if purpose == "observation":
                if halt_on_fire:
                    raise ValueError(
                        "halt_on_fire is refused for purpose=\"observation\": an "
                        "observation trigger decides when to look, never whether "
                        "to stop the arm."
                    )
                if not isinstance(threshold_rationale, str) or not threshold_rationale.strip():
                    raise ValueError(
                        "purpose=\"observation\" requires a non-empty "
                        "threshold_rationale saying why this threshold value."
                    )
            effective_rate, rate_clamped = (None, False)
            if rate_hz is not None:
                effective_rate, rate_clamped = _clamp_rate_hz(rate_hz)
            ctx = _ensure_monitor_host()
            monitor, err = _prepare_authored_monitor(
                code,
                description,
                params,
                clause,
                object_id,
                threshold_param,
                monitor_id,
                rate_hz=effective_rate,
                purpose=purpose,
                threshold_rationale=(
                    threshold_rationale.strip() if threshold_rationale else None
                ),
                halt_on_fire=halt_on_fire,
                ctx=ctx,
            )
            if err is not None:
                return _error(
                    command, err["exc"], stage=err["stage"], lineno=err["lineno"]
                )
            saved_path = _save_authored_source(
                ctx.session_dir, monitor.monitor_id, monitor.source
            )
            result = dict(ctx.engine.arm(monitor))
            result.update(
                first_call_result=monitor.first_call_result,
                channels_read=monitor.channels_read,
                code_sha256=monitor.code_sha256,
                saved_path=str(saved_path),
            )
            if effective_rate is not None:
                result.update(rate_hz=effective_rate, lane="array")
                if rate_clamped:
                    result.update(
                        rate_hz_clamped=True, rate_hz_requested=float(rate_hz)
                    )
            if isinstance(monitor.first_call_result, dict):
                result.update(warning=(
                    "This monitor fires on every tick. Returning a dict IS a "
                    "fire; return None when nothing is wrong. Armed as is, it "
                    "would fire on its first tick and disarm."
                ))
            return _ok(command, **result)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def dry_run_custom_monitor(
        code: str,
        params: dict[str, Any],
        object_id: str | None = None,
        ticks: int = 3,
        description: str = "dry run",
        rate_hz: float | None = None,
        purpose: str = "clause",
    ) -> dict[str, Any]:
        """Run authored code a few times WITHOUT arming it, and show what it saw.

        A dry run arms nothing, watches nothing, and covers no requirement
        for any part of the time it takes. It cannot fire, it emits no
        monitor_armed, monitor_fire or monitor_disarmed event, and it leaves
        no monitor behind. The only record it leaves is its own line in the
        call log.

        Mechanics. The code goes through exactly the same preparation as
        arm_custom_monitor — same validator, same restricted namespace, same
        params normalisation, same provenance resolution — so a dry run that
        prepares cleanly will arm cleanly. `code` must therefore satisfy the
        whole contract in arm_custom_monitor's description: one top-level
        check(ctx), the same allowed names, the same forbidden constructs.

        Preparation itself calls check(ctx) ONCE, exactly as arming does:
        that is the arm-time reference capture, and it is reported separately
        as first_call. The `ticks` calls happen after it, so ctx.state
        arrives at tick 1 already holding whatever the first call put there.
        This is what the live monitor would see.

        Ticks are spaced by 50 ms, the TF feed's own period, so successive
        calls see genuinely different samples. A dry run therefore takes
        roughly ticks * 50 ms. With rate_hz set (the array lane; clamped to
        at most 2.0), ticks are spaced at 1/rate_hz instead — the monitor's
        own live period — still clamped to the same maximum tick count, each
        tick takes a fresh camera snapshot, and wall_ms is judged against
        the array lane's 150 ms budget (reported as wall_budget_ms) rather
        than the scalar lane's 20 ms. purpose is accepted for symmetry with
        arm_custom_monitor and echoed; a dry run judges nothing either way.

        Per call the result carries:
          return_value    — what check returned, coerced to JSON. None means
                            "condition holds"; a dict with a numeric "value"
                            is what WOULD have fired, had this been armed.
          fired           — whether a live monitor would have fired on this
                            return. Informational only; nothing fired.
          error           — on an exception: its type, message, and the last
                            traceback frame, which names the line in the
                            supplied source.
          lines           — Python lines executed. Reported even when the
                            call raised or blew the budget. Work inside a C
                            extension emits no line events, so this
                            undercounts array work; the wall clock is the
                            real bound there.
          wall_ms         — how long the call took. The live scalar lane's
                            budget is 20 ms per call and five overruns
                            disarm a monitor.
          budget_exceeded — the line budget was passed; a live fire from
                            such a call is suppressed and counts a strike.
          state           — ctx.state after the call.
          channels_read   — cumulative ctx methods used, as "method:arg".

        params takes the same {name: {"value": v, "source": s}} shape as
        arm_custom_monitor; object_id behaves the same way. threshold_param
        is deliberately absent: a dry run judges nothing, so a threshold has
        nothing to be verified against here.
        """
        command = "dry_run_custom_monitor"
        try:
            if purpose not in MONITOR_PURPOSES:
                raise ValueError(
                    f"purpose must be one of {list(MONITOR_PURPOSES)}, got {purpose!r}."
                )
            effective_rate, rate_clamped = (None, False)
            if rate_hz is not None:
                effective_rate, rate_clamped = _clamp_rate_hz(rate_hz)
            tick_period_sec = (
                1.0 / effective_rate if effective_rate is not None
                else _DRY_RUN_TICK_PERIOD_SEC
            )
            wall_budget_ms = (
                ARRAY_WALL_BUDGET_MS if effective_rate is not None else 20.0
            )
            ctx = _ensure_monitor_host()
            monitor, err = _prepare_authored_monitor(
                code,
                description,
                params,
                None,        # clause: a dry run judges no requirement
                object_id,
                None,        # threshold_param: nothing to verify against
                None,        # monitor_id: this monitor is never registered
                rate_hz=effective_rate,
                purpose=purpose,
                for_arm=False,
                ctx=ctx,
            )
            if err is not None:
                return _error(
                    command, err["exc"], stage=err["stage"], lineno=err["lineno"]
                )

            requested_ticks = int(ticks)
            tick_count = max(1, min(requested_ticks, _DRY_RUN_MAX_TICKS))

            results: list[dict[str, Any]] = []
            first_call = {
                "call": "first_call",
                "return_value": json_safe(monitor.first_call_result),
                "fired": _would_fire(monitor.first_call_result),
                "lines": None,
                "wall_ms": round(monitor.last_wall_time_sec * 1000.0, 3),
                "state": json_safe(dict(monitor.ctx.state)),
                "channels_read": list(monitor.channels_read),
                "note": (
                    "This call happened during preparation, exactly as it would "
                    "during arming. Its return value never fires, and its line "
                    "count is not measured — the arm path does not record one."
                ),
            }

            from robot_backend.watchdog.authored import AuthoredBudgetExceeded

            for index in range(tick_count):
                time.sleep(tick_period_sec)
                usage: dict[str, Any] = {}
                entry: dict[str, Any] = {"call": index + 1}
                try:
                    returned = monitor.dry_call(usage=usage)
                    entry["return_value"] = json_safe(returned)
                    entry["fired"] = _would_fire(returned)
                except AuthoredBudgetExceeded as exc:
                    entry["error"] = _dry_run_error(exc)
                    entry["budget_exceeded"] = True
                    entry["fired"] = False
                except Exception as exc:
                    entry["error"] = _dry_run_error(exc)
                    entry["fired"] = False
                entry["lines"] = usage.get("lines")
                wall_sec = usage.get("wall_sec")
                entry["wall_ms"] = (
                    round(wall_sec * 1000.0, 3) if wall_sec is not None else None
                )
                entry["state"] = json_safe(dict(monitor.ctx.state))
                entry["channels_read"] = list(monitor.channels_read)
                results.append(entry)

            fired_count = sum(1 for entry in results if entry.get("fired"))
            extras: dict[str, Any] = {"fires": f"{fired_count}/{tick_count}"}
            if fired_count == tick_count and tick_count >= 1:
                extras["warning"] = (
                    "This monitor fires on every tick. Returning a dict IS a "
                    "fire; return None when nothing is wrong. Armed as is, it "
                    "would fire on its first tick and disarm."
                )
            return _ok(
                command,
                armed=False,
                **extras,
                code_sha256=monitor.code_sha256,
                description=monitor.description,
                object_id=monitor.provenance.get("object_id"),
                param_provenance=monitor.provenance.get("param_provenance"),
                tick_period_ms=tick_period_sec * 1000.0,
                wall_budget_ms=wall_budget_ms,
                purpose=purpose,
                **(
                    {"rate_hz": effective_rate, "lane": "array",
                     **({"rate_hz_clamped": True} if rate_clamped else {})}
                    if effective_rate is not None else {}
                ),
                ticks_requested=requested_ticks,
                ticks_run=tick_count,
                first_call=first_call,
                calls=results,
                note=(
                    "Nothing was armed and nothing was watched. No monitor_armed, "
                    "monitor_fire or monitor_disarmed event was emitted and no "
                    "strike was counted. This run covered no requirement."
                ),
            )
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def disarm_monitor(monitor_id: str) -> dict[str, Any]:
        """Disarm one monitor by id."""
        command = "disarm_monitor"
        try:
            ctx = _ensure_monitor_host()
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
            ctx = _ensure_monitor_host()
            removed = ctx.engine.disarm_all("agent_disarm_all")
            return _ok(command, disarmed=removed)
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def get_monitor_fires() -> dict[str, Any]:
        """Return and clear every monitor fire queued since the last call.

        Does not block. Fires queue in arrival order until read; a fire is
        returned exactly once. Each fire carries monitor_id, monitor_type,
        triggering_value, threshold, comparison, detail, clause, object_id,
        action_taken and trigger_wall_ns — the wall-clock nanosecond stamp
        taken at the moment the monitor triggered, which is the fire's own
        time; this call's own timestamp is only when the queue was drained.
        A fired monitor is already disarmed.
        """
        command = "get_monitor_fires"
        try:
            ctx = _ensure_monitor_host()
            fires = ctx.engine.pop_fires()
            return _ok(
                command,
                fires=fires,
                count=len(fires),
                armed_monitors_remaining=len(ctx.engine.snapshot()),
            )
        except Exception as exc:
            return _error(command, exc)

    @mcp.tool()
    def query_snapshot(object_ids: list[str] | None = None) -> dict[str, Any]:
        """Return latest buffered object poses, cell frame poses, and joint state.

        Covers only the objects and frames registered for this cell; it
        cannot show anything else that is in the workspace.

        frames carries the latest sample of every cell-level TF frame (e.g.
        the gripper tcp_frame), keyed by frame id — the instantaneous
        counterpart to get_evidence's frames channel. Samples from one
        recorder tick share one TF time (tf_stamp_ns), so poses from the same
        tick are mutually consistent.
        """
        command = "query_snapshot"
        try:
            ctx = _ensure_monitor_host()
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
        """Return downsampled evidence windows from this host's recorder buffers.

        Covers only the objects and frames registered for this cell; it
        cannot show anything else that is in the workspace.

        Samples from one recorder tick share one TF time (tf_stamp_ns), so an
        object_poses sample and a frames sample with the same wall_ns are
        mutually consistent.

        channels is any of joint_states, object_poses, frames, events. The
        frames channel carries cell frames such as the gripper TCP, sampled
        on the same tick as object poses so samples pair by index.

        start_wall_ns/end_wall_ns are wall-clock nanoseconds — the same
        wall_ns field every event and every recorder buffer uses, never
        envelope.timestamp_ns (a per-process monotonic clock; using it here
        will silently select an empty window). If the requested window
        overlaps no buffered sample on any requested channel, this returns
        an error carrying the actually buffered range rather than empty
        arrays — an empty result is reserved for "nothing happened in a
        window that genuinely was buffered", a real and different outcome
        from "this window was never buffered at all".
        """
        command = "get_evidence"
        try:
            ctx = _ensure_monitor_host()
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
    def record_observation(
        summary: str,
        category: str,
        location: dict[str, Any] | None,
        first_seen_wall_ns: int | None,
        evidence_look_ids: list[str],
        evidence_monitor_ids: list[str] | None = None,
        identity: str | None = None,
        confidence: str = "medium",
        requirement_impact: str = "none",
        impact_detail: str | None = None,
        observation_id: str | None = None,
        status: str = "new",
    ) -> dict[str, Any]:
        """Record one structured observation of something unexpected in the scene.

        An observation is a structured note that the scene holds something
        the process description did not account for: an object that is not
        on any list, an object that moved when nothing should have moved it,
        an object that is gone, or anything else off-script.

        An observation is NOT a finding and NOT a verdict. It does not count
        against the run, and the presence of something unexpected is not by
        itself a violation of anything. An observation whose subject also
        breaks a requirement carries requirement_impact="violates" with
        impact_detail naming the requirement; the requirement's own verdict
        is recorded separately and this tool never substitutes for it.

        category: one of unlisted_object (something present that no listing
        names), object_moved (a known object is not where it should be),
        object_missing (a known object is gone), other_unexpected.

        location: where the thing is, in either or both of two forms —
        {"frame": "world", "x": ..., "y": ..., "z": ...} (metres), and/or
        {"pixel": {"u": ..., "v": ...}, "look_id": "<id>"} naming the pixel
        in a specific look. At least one form is required unless category is
        object_missing.

        first_seen_wall_ns: when the thing first became visible, typically
        the wall_ns of the look that revealed it. Null if unknown.

        evidence_look_ids: ids of looks that show what is being described.
        They are checked against the looks this role actually made in this
        session; an empty list is allowed but the observation is then on
        record as unsupported by any visual evidence.

        identity: what the thing is believed to be, if the evidence says.
        confidence: low | medium | high.
        requirement_impact: none | violates | unknown. "violates" requires
        impact_detail naming the requirement, and a separate verdict.

        status: "new" records a new observation and generates its
        observation_id (never pass one). "update" revises an existing
        observation, and "resolved" closes it; both take the observation_id
        this tool returned.
        """
        ctx = _ensure_monitor_host()
        deps = ObservationDeps(
            role="executor",
            store=_OBSERVATIONS,
            publish_event=lambda event: ctx.event_bus.publish(event),
            current_step_id=lambda: ctx.recorder.current_step(),
            # look() lives on the scene server, a different process, so the
            # valid ids come from the look events this recorder ingested over
            # the shared topic rather than from the in-process registry.
            look_ids=lambda: _executor_look_ids(ctx),
        )
        return _run_record_observation(
            deps,
            summary=summary,
            category=category,
            location=location,
            first_seen_wall_ns=first_seen_wall_ns,
            evidence_look_ids=evidence_look_ids,
            evidence_monitor_ids=evidence_monitor_ids,
            identity=identity,
            confidence=confidence,
            requirement_impact=requirement_impact,
            impact_detail=impact_detail,
            observation_id=observation_id,
            status=status,
        )

    @mcp.tool()
    def record_verdict(
        step_id: str,
        verdict: str,
        clause_results: list[dict[str, Any]],
        explanation: str,
        claimed_outcome: str | None = None,
    ) -> dict[str, Any]:
        """Publish one step-scoped conformance verdict event.

        verdict is one of non_conformant, valid_variation, conformant or
        inconclusive. "incomplete" is run-scoped and is refused here — it
        describes the run stopping early, which is a property of the run,
        not of any one step.

        clause_results entries carry field, comparison, threshold, observed
        and pass keys, plus clause_id: an entry corresponding to a contract
        clause carries that clause's clause_id ("<section>.<field>")
        unchanged; an instruction-level requirement with no contract clause
        carries clause_id: null and source: "instruction". A malformed
        clause_id is rejected. This checks shape only, not the id against
        the actual contract.

        A repeated step_id publishes normally — a step with two separate
        findings produces two verdicts — and the return carries
        duplicate_step_id: true so an unintended repeat stays visible.

        explanation summarises the DECISION BASIS, not just the conclusion:
        the evidence and the values that mattered, why that made this
        conformant or a departure — including why something suspicious was
        judged within bounds when it was not flagged — and, for a departure,
        the believed physical mechanism (collision, wrong grasp, slip, wrong
        approach), stated plainly when the evidence does not settle it.
        """
        command = "record_verdict"
        try:
            if verdict not in _VALID_VERDICTS:
                raise ValueError(f"verdict must be one of {sorted(_VALID_VERDICTS)}.")
            if verdict == "incomplete":
                raise ValueError(
                    "verdict 'incomplete' is run-scoped — use it on "
                    "record_run_verdict, not on a per-step record_verdict."
                )
            _validate_clause_results(clause_results)
            ctx = _ensure_monitor_host()
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
        """Publish the run's single final verdict event.

        Every run carries exactly one of these; step verdicts are the
        timeline (what went wrong, when) and this is the end-state
        assessment. A second call publishes but the return carries
        duplicate_run_verdict: true, since a run with two rollups is a
        cadence violation on the record.

        What the verdict value means here — the END-STATE at termination,
        not a summary of everything that happened:

          conformant     every part's final pose meets its placement frame
                         within tolerance, and no violation still stands at
                         run end. An earlier finding that was subsequently
                         corrected does not make this non_conformant.
          non_conformant at least one part's end-state is wrong at run end,
                         or a violation stands uncorrected.
          incomplete     the run stopped before the end-state could exist.
                         Distinct from "inconclusive" (the evidence was
                         insufficient to judge) and from "non_conformant"
                         (the end-state is wrong).

        findings_during_run and recovered are filled in by this server from
        the record_verdict calls it actually saw; they are not parameters
        and cannot be overridden.

        clause_results carries the same entries record_verdict takes, with
        one addition: every entry also names its own step_id, because
        clause_id is "<section>.<field>" with no step in it and a whole-run
        rollup otherwise cannot say which part an entry belongs to.

        steps_covered lists every step_id this verdict accounts for, as a
        real argument so coverage is checkable by set comparison against the
        task's steps.

        explanation summarises the decision basis across the run: what was
        observed and what it established about the end-state, including why
        anything that looked suspicious was judged within bounds; when
        findings were recorded, it says what became of each.

        claimed_outcome, when given, summarises the run's claimed outcome.
        executor_terminal_status is the status from the run_ended event;
        the server records the status it observed on the event stream when
        the two disagree, and no run_ended at all is recorded as
        "no_terminal_event".
        """
        command = "record_run_verdict"
        global _RUN_VERDICTS_PUBLISHED
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
            ctx = _ensure_monitor_host()
            try:
                observed_status = ctx.recorder.observed_terminal_status()
            except Exception:
                observed_status = None
            resolved_status = (
                observed_status or executor_terminal_status or _NO_TERMINAL_EVENT
            )
            if (
                observed_status is not None
                and executor_terminal_status is not None
                and executor_terminal_status != observed_status
            ):
                ctx.node.get_logger().warning(
                    f"record_run_verdict: caller passed executor_terminal_status="
                    f"{executor_terminal_status!r} but the observed run_ended said "
                    f"{observed_status!r}; recording the observed value."
                )
            findings = _observed_findings()
            payload = RunVerdictPayload(
                verdict=verdict,
                claimed_outcome=claimed_outcome,
                clause_results=clause_results,
                steps_covered=steps_covered,
                explanation=explanation,
                executor_terminal_status=resolved_status,
                findings_during_run=findings,
                # Computed, never a parameter: a run "recovered" only if it
                # went wrong AND ended right.
                recovered=bool(findings) and verdict == "conformant",
            )
            event = make_event("run_verdict", payload)
            ctx.event_bus.publish(event)
            with _CONTEXT_LOCK:
                _RUN_VERDICTS_PUBLISHED += 1
                duplicate = _RUN_VERDICTS_PUBLISHED > 1
            published = event.payload
            return _ok(
                command,
                event=event.model_dump(mode="json"),
                steps_covered=steps_covered,
                executor_terminal_status=resolved_status,
                findings_during_run=list(published.get("findings_during_run", [])),
                recovered=bool(published.get("recovered", False)),
                duplicate_run_verdict=duplicate,
            )
        except Exception as exc:
            return _error(command, exc, verdict=verdict)


def main() -> None:
    """Run the S4 executor-side monitor MCP server.

    Eagerly constructs the node/recorder/engine before mcp.run() so the
    recorder's TF/joint-state buffers are warm by the time the first
    arm_custom_monitor call captures its reference. Non-fatal when ROS or the
    simulator is not up yet: the lazy _ensure_monitor_host() path inside each
    tool remains the fallback.
    """
    if not _MCP_SERVABLE or FastMCP is None:
        raise RuntimeError(
            "Cannot serve executor_monitor_mcp_server: FastMCP could not be "
            f"imported ({_MCP_IMPORT_ERROR}). This server targets the mcp 1.x "
            "API (mcp.server.fastmcp.FastMCP); pin 'mcp<2' in the workspace "
            "virtualenv or migrate this module to the 2.x API."
        )

    # This host serves the EXECUTOR session (S4). Refuse any other launch:
    # a monitor host inside a watchdog session would put executor-role events
    # beside watchdog ones and quietly break both scorers' role attribution.
    role = role_context.detect_role()
    if role != role_context.EXECUTOR_ROLE:
        raise RuntimeError(
            f"executor_monitor_mcp_server detected role {role!r}. This server "
            "is the S4 executor-side monitor host and must be launched in the "
            "executor session (EXECUTOR_TASK_PATH set, WATCHDOG_TASK_PATH "
            "unset) — see executor_s4.mcp.json."
        )

    try:
        _ensure_monitor_host()
    except Exception as exc:
        print(
            f"WARNING: executor_monitor_mcp_server eager init failed ({exc}); "
            "continuing — the first tool call will retry initialization.",
            file=sys.stderr,
        )

    try:
        mcp.run()
    finally:
        _shutdown_monitor_host()


if __name__ == "__main__":
    main()
