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
"""

from __future__ import annotations

import json
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

from cell_sync_interfaces.srv import RequestGo, SignalReady

try:
    from .timebase import wall_ns
except ImportError:
    # Supports running as `python3 node.py` directly, in addition to the
    # installed `ros2 run cell_orchestrator orchestrator_node` entry point.
    from timebase import wall_ns

_DEFAULT_LOG_DIR = Path.home() / ".watchdog" / "orchestrator"
_DEFAULT_GATE_TIMEOUT_SEC = 90.0


def _gate_key(step_id: str, phase: str) -> tuple[str, str]:
    return (step_id, phase or "")


class OrchestratorNode(Node):
    """Relays between RequestGo (executor side) and SignalReady (monitor side)."""

    def __init__(self, log_dir: Path | None = None) -> None:
        super().__init__("cell_orchestrator")

        # Both exposed as launch arguments (see launch/orchestrator.launch.py).
        # gate_timeout_sec is used only as a fallback when a RequestGo caller
        # sends timeout_sec <= 0 -- the normal path is driven entirely by the
        # per-call timeout_sec the caller (robot_backend.cell.gate.request_go)
        # sends, which must stay in sync with this default; see that module's
        # config note on the MCP-client-vs-gate-timeout ordering requirement.
        self.declare_parameter("gate_timeout_sec", _DEFAULT_GATE_TIMEOUT_SEC)
        self.declare_parameter("log_dir", str(_DEFAULT_LOG_DIR))
        self._default_gate_timeout_sec = float(
            self.get_parameter("gate_timeout_sec").value
        )

        run_id = f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_{uuid.uuid4().hex[:8]}"
        resolved_log_dir = log_dir or Path(
            str(self.get_parameter("log_dir").value)
        ).expanduser()
        resolved_log_dir.mkdir(parents=True, exist_ok=True)
        self._log_path = resolved_log_dir / f"orchestrator_{run_id}.jsonl"
        self._log_file = self._log_path.open("a", encoding="utf-8", buffering=1)

        # State this node is allowed to keep: whether run-scope readiness has
        # arrived, and a map of pending gates keyed by (step_id, phase).
        # Nothing about contracts, monitors, or verdicts.
        self._run_ready_event = threading.Event()
        self._state_lock = threading.Lock()
        self._gates: dict[tuple[str, str], threading.Event] = {}

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
            f"{self._default_gate_timeout_sec}; log at {self._log_path}"
        )

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

    # ------------------------------------------------------------------
    # SignalReady -- monitor side. Record the signal, release the matching
    # event, return immediately. Never blocks.
    # ------------------------------------------------------------------

    def _handle_signal_ready(
        self, request: SignalReady.Request, response: SignalReady.Response
    ) -> SignalReady.Response:
        scope = request.scope

        if scope == "run":
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
            response.ok = True
            response.message = (
                f"phase readiness recorded for step_id={request.step_id!r}, "
                f"phase={request.phase!r}"
            )
        else:
            response.ok = False
            response.message = f"unknown scope {scope!r}; must be run, step, or phase"
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
            if not run_ready:
                # The monitoring agent never came up (or never warmed) within
                # the deadline. This is categorically worse than a single
                # slow step: every step in the run is unguarded, not just
                # this one. Distinguish it in the response and the log so a
                # run full of these is never read as merely a slow run.
                waited_sec = time.monotonic() - t0
                response.proceed = True  # the gate never refuses
                response.timed_out = True
                response.waited_sec = waited_sec
                response.reason = "timeout_run_ready"
                self._log(
                    "gate_timeout_run_ready",
                    step_id=step_id,
                    phase=phase,
                    waited_sec=waited_sec,
                )
                return response

        gate_event = self._get_gate_event(step_id, phase)
        remaining = max(0.0, deadline - time.monotonic())
        released = gate_event.wait(remaining)
        waited_sec = time.monotonic() - t0

        response.proceed = True  # the gate never refuses -- see module docstring
        response.waited_sec = waited_sec

        if released:
            response.timed_out = False
            response.reason = "released"
            self._log("gate_open", step_id=step_id, phase=phase, waited_sec=waited_sec)
            return response

        # Reaching this point already implies run-scope readiness was
        # received (the branch above returns early otherwise), so this is
        # necessarily the "one step didn't finish arming in time" case, not
        # the "no monitor at all" case -- that one is timeout_run_ready,
        # above. Distinct reason and distinct log type on purpose: a run
        # full of timeout_run_ready is a broken setup, a run with one
        # timeout_gate is a slow turn on one step, and conflating them would
        # make a broken run look like a merely slow one.
        response.timed_out = True
        response.reason = "timeout_gate"
        self._log("gate_timeout", step_id=step_id, phase=phase, waited_sec=waited_sec)
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
