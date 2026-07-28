"""The orchestrator: a plain ROS node, the cell's timing interlock.

Serves /cell/request_go and /cell/signal_ready. Relays between them. Nothing
else.

This package's only dependencies are rclpy and cell_sync_interfaces (see
package.xml, which says so explicitly and forbids robot_backend /
robot_backend_interfaces). It lives in its own colcon package, separate from
both robot_backend (the executor stack it gates) and agents/ (the LLM
harness, which has pip dependencies and no ROS dependency at all). Splitting
by dependency type keeps colcon builds clean and the harness's venv free of
ROS; putting the orchestrator inside robot_backend would additionally make
the executor package contain its own supervisor, which is the wrong reading
of a dependency graph for a claim of independent supervision.

Why it exists as a third process rather than folding into WatchDog: if
WatchDog served RequestGo directly, the executor could not run at all without
WatchDog up, and WatchDog -- the independent monitor -- would be sitting
literally in the cell's control path. That is indefensible alongside a claim
of independent supervision. With the orchestrator serving both sides, the
cell has an interlock that happens to consult a monitor; if the monitor
vanishes, the interlock times out and opens rather than jamming shut.

Rules this node exists to keep, not to negotiate:
  - It never inspects event payloads beyond what the service request/response
    carries. It has no subscription to /watchdog/events at all.
  - It never reads contracts, monitor state, claims, or verdicts.
  - It never tells the monitor anything -- WatchDog learns of steps from the
    event topic directly, exactly as it does today.
  - The RequestGo response carries nothing the executor did not already send
    it: step_id and phase came from the executor's own call. This is timing
    coupling, not information coupling.
  - The gate never refuses. proceed is always True. Timing out just means the
    step ran unguarded, which is recorded, not prevented.

Concurrency: RequestGo can hold its response open for tens of seconds while
SignalReady must be serviced in the meantime, or the very call that would
release a blocked RequestGo could never run. A SingleThreadedExecutor
deadlocks here. Requires MultiThreadedExecutor with both services in the same
ReentrantCallbackGroup -- same precedent as
robot_backend.nodes.scene_tf_sync_node.SceneTFSyncNode, which hit the
equivalent problem (a service handler that must wait on another callback) and
solved it the same way.

guard_mode -- whether a run is monitored is experimental metadata and must be
declared, not inferred. A heuristic that detects the absence of a monitor
could silently classify a misconfigured run as an intentionally unmonitored
one, and those two must never be confusable in results. The "guard_mode" ROS
parameter ("guarded" | "unguarded", default "guarded") is read live on every
RequestGo call, so `ros2 param set /cell_orchestrator guard_mode unguarded`
takes effect on the next gate with no restart. Three, and only three, reason
codes reach the executor and the log:
  unguarded_by_config -- the operator declared this run unmonitored. Expected.
  timeout_gate         -- a monitor was present and failed to arm in time for
                           this one step. A coverage failure, counted in
                           results.
  unguarded_latched    -- guard_mode was "guarded" but no monitor ever
                           signalled anything on this run's first gate. A
                           misconfiguration: this run's results cover
                           nothing. Logged at warning level once, when the
                           latch trips; every gate after that returns
                           unguarded_latched immediately without waiting. If
                           a monitor signals at any point after latching, the
                           latch clears and normal gating resumes from the
                           next request onward.
"""

from __future__ import annotations

import json
import os
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
import rclpy.executors
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult
from cell_sync_interfaces.srv import RequestGo, SignalReady

try:
    from .timebase import wall_ns
except ImportError:
    # Supports running as `python3 node.py` directly, in addition to the
    # installed `ros2 run cell_orchestrator orchestrator_node` entry point.
    from timebase import wall_ns


def _default_log_dir() -> Path:
    """Default orchestrator log dir, honoring WATCHDOG_SESSION_DIR.

    Inlined here rather than imported from robot_backend.watchdog.session: this
    package deliberately depends only on rclpy and cell_sync_interfaces (see the
    module docstring) and must not pull in robot_backend. When bringup launches
    this node it passes an explicit log_dir already rooted in the session, so
    this default matters mainly for a direct `ros2 run` of the orchestrator.
    """
    session = os.environ.get("WATCHDOG_SESSION_DIR")
    if session and session.strip():
        return Path(os.path.expanduser(session.strip())) / "orchestrator"
    return Path.home() / ".watchdog" / "orchestrator"


_DEFAULT_LOG_DIR = _default_log_dir()
_DEFAULT_GATE_TIMEOUT_SEC = 90.0
_DEFAULT_GUARD_MODE = "guarded"
_VALID_GUARD_MODES = {"guarded", "unguarded"}


def _gate_key(step_id: str, phase: str) -> tuple[str, str]:
    return (step_id, phase or "")


class OrchestratorNode(Node):
    """Relays between RequestGo (executor side) and SignalReady (monitor side)."""

    def __init__(self, log_dir: Path | None = None) -> None:
        super().__init__("cell_orchestrator")

        # All three exposed as launch arguments (see
        # launch/orchestrator.launch.py). gate_timeout_sec is used only as a
        # fallback when a RequestGo caller sends timeout_sec <= 0 -- the
        # normal path is driven entirely by the per-call timeout_sec the
        # caller (robot_backend.cell.gate.request_go) sends, which must stay
        # in sync with this default; see that module's config note on the
        # MCP-client-vs-gate-timeout ordering requirement.
        #
        # guard_mode is experimental metadata that must be declared, not
        # inferred -- see module docstring addendum below. It is dynamically
        # settable at runtime (`ros2 param set /cell_orchestrator guard_mode
        # unguarded`) without restarting the node, since the simulator can
        # take minutes to come up and flipping between guarded/unguarded runs
        # must not require bouncing the cell.
        self.declare_parameter("gate_timeout_sec", _DEFAULT_GATE_TIMEOUT_SEC)
        self.declare_parameter("log_dir", str(_DEFAULT_LOG_DIR))
        self.declare_parameter("guard_mode", _DEFAULT_GUARD_MODE)
        self._default_gate_timeout_sec = float(
            self.get_parameter("gate_timeout_sec").value
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)

        run_id = f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_{uuid.uuid4().hex[:8]}"
        resolved_log_dir = log_dir or Path(
            str(self.get_parameter("log_dir").value)
        ).expanduser()
        resolved_log_dir.mkdir(parents=True, exist_ok=True)
        self._log_path = resolved_log_dir / f"orchestrator_{run_id}.jsonl"
        self._log_file = self._log_path.open("a", encoding="utf-8", buffering=1)

        # State this node is allowed to keep: whether run-scope readiness has
        # arrived, a map of pending gates keyed by (step_id, phase), and the
        # guard-mode safety latch (see _handle_request_go). Nothing about
        # contracts, monitors, or verdicts.
        self._run_ready_event = threading.Event()
        self._any_signal_received = threading.Event()
        self._state_lock = threading.Lock()
        self._gates: dict[tuple[str, str], threading.Event] = {}
        self._first_gate_attempted = False
        self._latched_unguarded = False
        # Run-scoped abort. Set by a SignalReady(scope="abort") and never
        # cleared for the rest of the run: once the monitor declares the run
        # should not continue, every subsequent gate refuses immediately with
        # reason="aborted", and any gate already blocked is released to the
        # same refusal rather than left to time out. This is the executor's
        # only signal, and it is told nothing about who aborted or why -- it
        # sees a cell interlock declining to release a step.
        self._aborted = threading.Event()
        self._abort_detail: str = ""
        # (monotonic_time, phase) per step_id, for every phase-scope
        # SignalReady this node has ever received -- lets a timed-out
        # phase gate check whether a signal arrived for the same step_id
        # under a DIFFERENT phase name during its own wait. See
        # _find_near_miss_phases.
        self._phase_signal_log: dict[str, list[tuple[float, str]]] = {}

        self._log(
            "orchestrator_start",
            guard_mode=self._get_guard_mode(),
            gate_timeout_sec=self._default_gate_timeout_sec,
        )

        # Both services share one ReentrantCallbackGroup so the
        # MultiThreadedExecutor can run a blocked RequestGo call and an
        # incoming SignalReady call concurrently -- without this, rclpy's
        # default MutuallyExclusiveCallbackGroup would serialize them even
        # under a MultiThreadedExecutor, and the blocked call would never see
        # the release.
        cb_group = ReentrantCallbackGroup()

        self.create_service(
            RequestGo, "/cell/request_go", self._handle_request_go, callback_group=cb_group
        )
        self.create_service(
            SignalReady, "/cell/signal_ready", self._handle_signal_ready, callback_group=cb_group
        )

        self.get_logger().info(
            f"cell_orchestrator up; serving /cell/request_go and "
            f"/cell/signal_ready; default gate_timeout_sec="
            f"{self._default_gate_timeout_sec}; guard_mode="
            f"{self._get_guard_mode()}; log at {self._log_path}"
        )

    # ------------------------------------------------------------------
    # guard_mode -- read live on every request so `ros2 param set` takes
    # effect on the very next gate, with no node restart.
    # ------------------------------------------------------------------

    def _get_guard_mode(self) -> str:
        return str(self.get_parameter("guard_mode").value)

    def _on_set_parameters(self, params: list) -> SetParametersResult:
        for param in params:
            if param.name == "guard_mode" and param.value not in _VALID_GUARD_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"guard_mode must be one of {sorted(_VALID_GUARD_MODES)}, "
                        f"got {param.value!r}"
                    ),
                )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Logging -- one JSONL record per transition, each carrying wall_ns.
    # ------------------------------------------------------------------

    def _log(self, event_type: str, **fields: Any) -> None:
        record = {"type": event_type, "wall_ns": wall_ns(), **fields}
        try:
            self._log_file.write(json.dumps(record, default=str) + "\n")
            self._log_file.flush()
        except Exception:
            pass  # the orchestrator's own log must never break the gate

    def _get_gate_event(self, step_id: str, phase: str) -> threading.Event:
        key = _gate_key(step_id, phase)
        with self._state_lock:
            event = self._gates.get(key)
            if event is None:
                event = threading.Event()
                self._gates[key] = event
            return event

    def _abort_response(
        self,
        response: RequestGo.Response,
        step_id: str,
        phase: str,
        waited_sec: float = 0.0,
    ) -> RequestGo.Response:
        """Fill a RequestGo response for an aborted run: proceed=False.

        The only place the gate refuses. reason="aborted" is the executor's
        sole signal; it learns nothing about who aborted or why (the detail is
        kept in this node's log only). timed_out is False -- an abort is a
        deliberate refusal, not a coverage timeout.
        """
        response.proceed = False
        response.timed_out = False
        response.waited_sec = waited_sec
        response.reason = "aborted"
        self._log(
            "gate_aborted",
            step_id=step_id,
            phase=phase,
            waited_sec=waited_sec,
            abort_detail=self._abort_detail,
        )
        return response

    def _find_near_miss_phases(
        self, step_id: str, phase: str, t0: float, t1: float
    ) -> list[str]:
        """Phase names signalled for step_id, other than phase, within [t0, t1].

        A phase gate timing out while a *different* phase name was signalled
        for the same step during the same wait is a name mismatch between
        the executor and the monitor, not a slow monitor -- surfacing it
        here turns a cross-transcript comparison into one log line.
        """
        with self._state_lock:
            entries = list(self._phase_signal_log.get(step_id, []))
        return sorted({p for (t, p) in entries if t0 <= t <= t1 and p != phase})

    # ------------------------------------------------------------------
    # SignalReady -- monitor side. Record the signal, release the matching
    # event, return immediately. Never blocks.
    # ------------------------------------------------------------------

    def _handle_signal_ready(
        self, request: SignalReady.Request, response: SignalReady.Response
    ) -> SignalReady.Response:
        scope = request.scope

        # Any signal at all, of any scope, is proof a monitor is present.
        # This is what the safety latch checks to distinguish "no monitor
        # ever appeared" (unguarded_latched) from an ordinary slow step.
        self._any_signal_received.set()
        with self._state_lock:
            was_latched = self._latched_unguarded
            self._latched_unguarded = False
        if was_latched:
            self.get_logger().warning(
                "cell_orchestrator: a monitor signalled after this run had "
                "already latched unguarded -- clearing the latch and "
                "resuming normal gating from the next request onward."
            )
            self._log("latch_cleared", scope=scope, step_id=request.step_id)

        if scope == "abort":
            # Run-scoped and irreversible: latch the flag, then release every
            # currently pending gate (and run-readiness) so a RequestGo that is
            # already blocked wakes and refuses with reason="aborted" rather
            # than sitting out its full timeout. The abort check inside
            # _handle_request_go turns the wakeup into an "aborted" outcome.
            detail = getattr(request, "detail", "") or ""
            with self._state_lock:
                self._abort_detail = detail
                pending = list(self._gates.values())
            self._aborted.set()
            self._run_ready_event.set()
            for event in pending:
                event.set()
            self.get_logger().warning(
                f"cell_orchestrator: run aborted by monitor signal "
                f"(detail={detail!r}). Every subsequent gate will refuse with "
                f"reason='aborted'."
            )
            self._log("abort", detail=detail)
            response.ok = True
            response.message = "abort recorded"
        elif scope == "run":
            self._run_ready_event.set()
            self._log("run_ready")
            response.ok = True
            response.message = "run-scope readiness recorded"
        elif scope == "step":
            event = self._get_gate_event(request.step_id, "")
            event.set()
            response.ok = True
            response.message = f"step readiness recorded for step_id={request.step_id!r}"
        elif scope == "phase":
            event = self._get_gate_event(request.step_id, request.phase)
            event.set()
            with self._state_lock:
                self._phase_signal_log.setdefault(request.step_id, []).append(
                    (time.monotonic(), request.phase)
                )
            response.ok = True
            response.message = (
                f"phase readiness recorded for step_id={request.step_id!r}, "
                f"phase={request.phase!r}"
            )
        else:
            response.ok = False
            response.message = (
                f"unknown scope {scope!r}; must be run, step, phase, or abort"
            )
        return response

    # ------------------------------------------------------------------
    # RequestGo -- executor side. Blocks (on this callback's own thread,
    # concurrently with everything else the ReentrantCallbackGroup allows)
    # until released or timed out. Always proceeds.
    # ------------------------------------------------------------------

    def _handle_request_go(
        self, request: RequestGo.Request, response: RequestGo.Response
    ) -> RequestGo.Response:
        step_id = request.step_id
        phase = request.phase
        guard_mode = self._get_guard_mode()

        # aborted: the monitor declared this run should not continue. Takes
        # precedence over everything below and never waits -- once set, every
        # gate refuses immediately. Checked first so an abort stops execution
        # regardless of guard_mode or latch state.
        if self._aborted.is_set():
            return self._abort_response(response, step_id, phase)

        # unguarded_by_config: the operator declared this run unmonitored.
        # Expected, not a defect -- never waits, ever.
        if guard_mode == "unguarded":
            response.proceed = True
            response.timed_out = False
            response.waited_sec = 0.0
            response.reason = "unguarded_by_config"
            self._log(
                "gate_unguarded_by_config",
                step_id=step_id,
                phase=phase,
                guard_mode=guard_mode,
            )
            return response

        # unguarded_latched: this guarded run already discovered, on its
        # first gate, that no monitor ever appeared. Every subsequent gate
        # returns immediately rather than paying the full timeout again --
        # see the latch-detection branch below for why.
        with self._state_lock:
            already_latched = self._latched_unguarded
        if already_latched:
            response.proceed = True
            response.timed_out = False
            response.waited_sec = 0.0
            response.reason = "unguarded_latched"
            self._log(
                "gate_unguarded_latched",
                step_id=step_id,
                phase=phase,
                guard_mode=guard_mode,
            )
            return response

        with self._state_lock:
            is_first_gate = not self._first_gate_attempted
            self._first_gate_attempted = True

        timeout_sec = float(request.timeout_sec) if request.timeout_sec > 0 else (
            self._default_gate_timeout_sec
        )
        t0 = time.monotonic()
        # A single deadline shared by both waits below: the caller asked for
        # a bound on THIS CALL, not on each internal stage of it. Computing
        # remaining budget from one deadline (rather than giving each wait
        # its own fresh timeout_sec) is what keeps total wall time in this
        # handler from exceeding timeout_sec, regardless of which stage ends
        # up consuming the budget.
        deadline = t0 + timeout_sec

        # Wait for run-scope readiness first, if it hasn't arrived yet, out
        # of the shared deadline above. This is what lets the very first
        # begin_step of a run wait for the monitor to finish warming up
        # rather than racing it.
        if not self._run_ready_event.is_set():
            remaining = max(0.0, deadline - time.monotonic())
            run_ready = self._run_ready_event.wait(remaining)
            # An abort sets _run_ready_event to wake this wait; a call blocked
            # here when the abort fired must refuse, not fall through to the
            # gate wait.
            if self._aborted.is_set():
                return self._abort_response(
                    response, step_id, phase, time.monotonic() - t0
                )
            if not run_ready:
                waited_sec = time.monotonic() - t0
                response.proceed = True  # the gate never refuses
                response.waited_sec = waited_sec

                # Latch only on the first gate of the run, and only if no
                # monitor has signalled ANYTHING at all (any scope) by this
                # point. Without the "first gate" guard, every gate in an
                # unmonitored run would re-pay the full timeout_sec waiting
                # on the same never-coming run_ready event -- at 90s and two
                # gates per step, a single-step run gains three minutes of
                # dead waiting. With it, the run pays the timeout exactly
                # once, then latches.
                if is_first_gate and not self._any_signal_received.is_set():
                    with self._state_lock:
                        self._latched_unguarded = True
                    self.get_logger().warning(
                        f"cell_orchestrator: guard_mode=guarded but no monitor "
                        f"signalled anything within {waited_sec:.1f}s on this "
                        f"run's first gate -- latching this run as unguarded. "
                        f"Every subsequent gate will return immediately with "
                        f"reason='unguarded_latched'. This run's results "
                        f"cover nothing."
                    )
                    response.timed_out = True
                    response.reason = "unguarded_latched"
                    self._log(
                        "gate_unguarded_latched",
                        step_id=step_id,
                        phase=phase,
                        waited_sec=waited_sec,
                        guard_mode=guard_mode,
                    )
                    return response

                # Not the first gate, or a monitor has signalled something
                # (just not run-scope readiness yet) -- an ordinary coverage
                # failure for this step, not a whole-run misconfiguration.
                response.timed_out = True
                response.reason = "timeout_gate"
                self._log(
                    "gate_timeout",
                    step_id=step_id,
                    phase=phase,
                    waited_sec=waited_sec,
                    guard_mode=guard_mode,
                )
                return response

        gate_event = self._get_gate_event(step_id, phase)
        remaining = max(0.0, deadline - time.monotonic())
        released = gate_event.wait(remaining)
        waited_sec = time.monotonic() - t0

        # An abort released this gate to wake it: a call blocked in the gate
        # wait when the abort fired sees released=True but must refuse with
        # reason="aborted", not "released".
        if self._aborted.is_set():
            return self._abort_response(response, step_id, phase, waited_sec)

        response.proceed = True  # the gate never refuses -- see module docstring
        response.waited_sec = waited_sec

        if released:
            response.timed_out = False
            response.reason = "released"
            self._log(
                "gate_open",
                step_id=step_id,
                phase=phase,
                waited_sec=waited_sec,
                guard_mode=guard_mode,
            )
            return response

        # Reaching this point already implies run-scope readiness was
        # received (the branch above returns early otherwise), so this is
        # necessarily "a monitor is present but this one step didn't finish
        # arming in time" -- a coverage failure counted in results, not a
        # whole-run misconfiguration (that's unguarded_latched, above).
        #
        # Diagnostic only, does not change the outcome: a phase gate timing
        # out is still timeout_gate even when a near-miss name is found --
        # the executor really did wait the full timeout regardless of why.
        # This just makes the likely cause (a name mismatch, not a slow
        # monitor) visible in one log line instead of a cross-transcript
        # comparison. No fuzzy matching or auto-correction -- names must
        # match exactly; this only reports that they didn't.
        if phase:
            near_misses = self._find_near_miss_phases(step_id, phase, t0, time.monotonic())
            if near_misses:
                self.get_logger().warning(
                    f"cell_orchestrator: RequestGo for step_id={step_id!r} "
                    f"phase={phase!r} timed out, but signal_ready was received "
                    f"for the same step_id under a different phase name during "
                    f"the wait: {near_misses}. This looks like a phase-name "
                    f"mismatch between the executor and the monitor, not a slow "
                    f"monitor."
                )
                self._log(
                    "gate_timeout_near_miss_phase",
                    step_id=step_id,
                    requested_phase=phase,
                    signalled_phases=near_misses,
                )

        response.timed_out = True
        response.reason = "timeout_gate"
        self._log(
            "gate_timeout",
            step_id=step_id,
            phase=phase,
            waited_sec=waited_sec,
            guard_mode=guard_mode,
        )
        return response

    def destroy_node(self) -> bool:
        try:
            self._log_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: OrchestratorNode | None = None
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        node = OrchestratorNode()
        executor.add_node(node)
        executor.spin()
    finally:
        if node is not None:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
