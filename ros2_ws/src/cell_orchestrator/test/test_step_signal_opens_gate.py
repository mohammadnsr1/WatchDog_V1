"""A step-scope signal must open its gate even if run scope never arrives.

Regression tests for the stall measured in session 20260921T204929Z, where a
single-part pick-and-place took 8 minutes instead of ~1. The orchestrator
waited on run-scope readiness FIRST, out of the gate's whole budget, and only
then looked at the step's own event. The watchdog's
signal_ready(scope="run") was refused by its own warm-up precondition (four
registry entries the stage never published could never buffer a pose), so the
agent fell back to signalling step scope for each step -- which arrived 35s
into a 90s gate and was never read. All four gates ran to their full timeout:
4 x 90s = 6 minutes of dead waiting, with a monitor present and arming the
whole time.

The registry side of that bug is fixed separately (enabled now means "the
stage publishes this", so run readiness is reachable). These tests cover the
orchestrator's half: a gate must not be able to sit out its timeout while the
signal that names it specifically is already set.

Style follows test_guard_mode.py: the service handlers are plain blocking
methods, so they are called directly -- no spin, no transport, no simulator.

Run with: pytest test/test_step_signal_opens_gate.py -v
"""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from cell_sync_interfaces.srv import RequestGo, SignalReady

from cell_orchestrator.node import OrchestratorNode

# Long enough that a regressed build cannot pass by finishing early, short
# enough to keep the suite fast. The assertions below compare against a small
# fraction of this rather than against an absolute wall-clock figure.
_GATE_TIMEOUT_SEC = 2.0


@pytest.fixture(autouse=True)
def _rclpy_context():
    rclpy.init(args=None)
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_node(tmp_path) -> OrchestratorNode:
    return OrchestratorNode(log_dir=tmp_path)


def _request_go(
    node: OrchestratorNode, step_id: str, phase: str = ""
) -> RequestGo.Response:
    req = RequestGo.Request()
    req.step_id = step_id
    req.phase = phase
    req.timeout_sec = _GATE_TIMEOUT_SEC
    return node._handle_request_go(req, RequestGo.Response())


def _signal_ready(node: OrchestratorNode, scope: str, step_id: str = "", phase: str = ""):
    req = SignalReady.Request()
    req.scope = scope
    req.step_id = step_id
    req.phase = phase
    return node._handle_signal_ready(req, SignalReady.Response())


def test_step_signal_opens_gate_without_run_readiness(tmp_path):
    """The exact production stall: step scope signalled, run scope never."""
    node = _make_node(tmp_path)
    try:
        _signal_ready(node, "step", step_id="pick_u_brown")
        assert not node._run_ready_event.is_set(), (
            "test setup error: run readiness must NOT be signalled here -- that "
            "is the whole point of the regression"
        )

        t0 = time.monotonic()
        response = _request_go(node, "pick_u_brown")
        elapsed = time.monotonic() - t0

        assert response.reason == "released", (
            f"gate reported {response.reason!r}; a step signal naming this exact "
            "step was already set, so the gate had nothing left to wait for"
        )
        assert not response.timed_out
        assert response.proceed
        assert elapsed < _GATE_TIMEOUT_SEC / 2, (
            f"gate took {elapsed:.2f}s of its {_GATE_TIMEOUT_SEC}s budget -- it "
            "is still waiting out run readiness before reading the step event, "
            "which is the 90s-per-gate stall this test exists to catch"
        )
    finally:
        node.destroy_node()


def test_phase_signal_opens_phase_gate_without_run_readiness(tmp_path):
    """mark_phase gates stall the same way; two of the four in production were phases."""
    node = _make_node(tmp_path)
    try:
        _signal_ready(node, "phase", step_id="pick_u_brown", phase="grasped")

        t0 = time.monotonic()
        response = _request_go(node, "pick_u_brown", phase="grasped")
        elapsed = time.monotonic() - t0

        assert response.reason == "released"
        assert not response.timed_out
        assert elapsed < _GATE_TIMEOUT_SEC / 2, (
            f"phase gate took {elapsed:.2f}s of its {_GATE_TIMEOUT_SEC}s budget"
        )
    finally:
        node.destroy_node()


def test_step_signal_arriving_mid_wait_releases_a_blocked_gate(tmp_path):
    """The production ordering: the gate blocks first, the signal lands later.

    test_step_signal_opens_gate_without_run_readiness pre-sets the event, which
    a correct implementation could pass by checking it once before waiting.
    Here the gate is already parked when the signal arrives, which is what
    actually happened -- the signal landed 35s into the gate.
    """
    node = _make_node(tmp_path)
    try:
        signal_delay_sec = 0.3

        def _signal_later():
            time.sleep(signal_delay_sec)
            _signal_ready(node, "step", step_id="place_u_brown")

        signaller = threading.Thread(target=_signal_later, daemon=True)
        signaller.start()

        t0 = time.monotonic()
        response = _request_go(node, "place_u_brown")
        elapsed = time.monotonic() - t0
        signaller.join(timeout=1.0)

        assert response.reason == "released", (
            f"gate reported {response.reason!r} -- a step signal arriving while "
            "the gate is parked must wake it"
        )
        assert elapsed >= signal_delay_sec, (
            "gate returned before the signal was even sent, so it cannot have "
            "been released by it"
        )
        assert elapsed < _GATE_TIMEOUT_SEC / 2, (
            f"gate took {elapsed:.2f}s -- it waited out run readiness instead of "
            "waking on the step signal"
        )
    finally:
        node.destroy_node()


def test_run_readiness_still_opens_a_gate_nothing_else_signalled(tmp_path):
    """The ordinary path must keep working: run scope alone releases the gate."""
    node = _make_node(tmp_path)
    try:
        _signal_ready(node, "run")

        t0 = time.monotonic()
        response = _request_go(node, "pick_u_brown")
        elapsed = time.monotonic() - t0

        # No step signal was sent, so the gate passes the run-readiness check
        # and then times out on its own event. That is a per-step coverage
        # failure, which is the correct and unchanged outcome here -- the
        # point of this test is that widening the run-readiness wait did not
        # turn run scope into a blanket release for every step.
        assert response.reason == "timeout_gate"
        assert response.proceed
        assert elapsed >= _GATE_TIMEOUT_SEC * 0.5, (
            "the gate should have spent its budget waiting on its own event"
        )
    finally:
        node.destroy_node()


def test_unrelated_step_signal_does_not_open_this_gate(tmp_path):
    """A signal naming a DIFFERENT step must not release this one.

    The widened wait accepts this gate's own event as an alternative to run
    readiness. It must not accept any signal at all, or a monitor arming step
    N would silently wave step N+1 through unguarded.
    """
    node = _make_node(tmp_path)
    try:
        _signal_ready(node, "step", step_id="some_other_step")

        response = _request_go(node, "pick_u_brown")

        assert response.reason == "timeout_gate", (
            f"gate reported {response.reason!r} -- a signal for 'some_other_step' "
            "must not release the gate for 'pick_u_brown'"
        )
        assert response.timed_out
        assert response.proceed  # the gate never refuses
    finally:
        node.destroy_node()
