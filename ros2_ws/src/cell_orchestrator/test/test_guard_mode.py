"""Functional tests for cell_orchestrator's guard_mode and safety latch.

Calls OrchestratorNode's service handlers directly (no executor spin, no
service transport) since they are plain blocking Python methods -- this
keeps the tests fast (short timeouts) and simulator-free, matching the old
F3/F4/F4b-style manual verification these replace/extend.

Run with: pytest test/test_guard_mode.py -v
"""

from __future__ import annotations

import time

import pytest
import rclpy
from cell_sync_interfaces.srv import RequestGo, SignalReady

from cell_orchestrator.node import OrchestratorNode

_SHORT_GATE_TIMEOUT_SEC = 0.2


@pytest.fixture(autouse=True)
def _rclpy_context():
    rclpy.init(args=None)
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_node(tmp_path, guard_mode: str | None = None) -> OrchestratorNode:
    # gate_timeout_sec is a launch-time parameter cached once at construction
    # (node.py's __init__ reads it into self._default_gate_timeout_sec, by
    # design -- see the module docstring's note on why guard_mode is read
    # live but this is not). Tests therefore always pass an explicit
    # request.timeout_sec instead of relying on the node's default, exactly
    # as the real caller (robot_backend.cell.gate.request_go) always does.
    node = OrchestratorNode(log_dir=tmp_path)
    if guard_mode is not None:
        node.set_parameters([rclpy.parameter.Parameter("guard_mode", value=guard_mode)])
    return node


def _request_go(node: OrchestratorNode, step_id: str, phase: str = "") -> RequestGo.Response:
    req = RequestGo.Request()
    req.step_id = step_id
    req.phase = phase
    req.timeout_sec = _SHORT_GATE_TIMEOUT_SEC
    return node._handle_request_go(req, RequestGo.Response())


def _signal_ready(node: OrchestratorNode, scope: str, step_id: str = "", phase: str = ""):
    req = SignalReady.Request()
    req.scope = scope
    req.step_id = step_id
    req.phase = phase
    return node._handle_signal_ready(req, SignalReady.Response())


class TestUnguardedByConfig:
    def test_gate_returns_immediately_with_unguarded_by_config(self, tmp_path) -> None:
        node = _make_node(tmp_path, guard_mode="unguarded")
        t0 = time.monotonic()
        resp = _request_go(node, "step1")
        elapsed = time.monotonic() - t0

        assert resp.proceed is True
        assert resp.timed_out is False
        assert resp.reason == "unguarded_by_config"
        assert elapsed < 0.05, "unguarded gates must return in milliseconds, not wait"
        node.destroy_node()

    def test_unguarded_by_config_never_appears_when_a_monitor_signals(self, tmp_path) -> None:
        # Sanity check the reason codes don't cross-contaminate: a guarded
        # run with a monitor must never see unguarded_by_config.
        node = _make_node(tmp_path, guard_mode="guarded")
        _signal_ready(node, "run")
        _signal_ready(node, "step", step_id="step1")
        resp = _request_go(node, "step1")
        assert resp.reason != "unguarded_by_config"
        assert resp.reason == "released"
        node.destroy_node()


class TestGuardedWithMonitor:
    def test_gate_released_when_monitor_signals_first(self, tmp_path) -> None:
        # Run-scope readiness unblocks the first wait stage; the gate itself
        # still needs its own step/phase-scope signal (a run-scope signal
        # alone does not release a specific step's gate).
        node = _make_node(tmp_path, guard_mode="guarded")
        _signal_ready(node, "run")
        _signal_ready(node, "step", step_id="step1")
        resp = _request_go(node, "step1")

        assert resp.proceed is True
        assert resp.timed_out is False
        assert resp.reason == "released"
        node.destroy_node()


class TestGuardedNoMonitorLatches:
    def test_first_gate_times_out_and_latches(self, tmp_path) -> None:
        node = _make_node(tmp_path, guard_mode="guarded")
        t0 = time.monotonic()
        resp = _request_go(node, "step1")
        elapsed = time.monotonic() - t0

        assert resp.proceed is True
        assert resp.timed_out is True
        assert resp.reason == "unguarded_latched"
        assert elapsed >= _SHORT_GATE_TIMEOUT_SEC * 0.9, (
            "the first gate must actually pay the timeout before latching"
        )
        node.destroy_node()

    def test_subsequent_gates_return_immediately_once_latched(self, tmp_path) -> None:
        node = _make_node(tmp_path, guard_mode="guarded")
        _request_go(node, "step1")  # pays the timeout, latches

        t0 = time.monotonic()
        resp = _request_go(node, "step2", phase="grasp")
        elapsed = time.monotonic() - t0

        assert resp.proceed is True
        assert resp.timed_out is False
        assert resp.reason == "unguarded_latched"
        assert elapsed < 0.05, "latched gates must return immediately, not re-pay the timeout"
        node.destroy_node()

    def test_latch_clears_when_a_monitor_signals(self, tmp_path) -> None:
        node = _make_node(tmp_path, guard_mode="guarded")
        _request_go(node, "step1")  # latches

        _signal_ready(node, "run")  # monitor shows up late
        _signal_ready(node, "step", step_id="step2")

        resp = _request_go(node, "step2")
        assert resp.reason == "released"
        assert resp.reason != "unguarded_latched"
        node.destroy_node()


class TestDynamicParamUpdate:
    def test_ros2_param_set_takes_effect_on_next_gate_without_restart(self, tmp_path) -> None:
        node = _make_node(tmp_path, guard_mode="guarded")
        _signal_ready(node, "run")
        _signal_ready(node, "step", step_id="step1")
        first = _request_go(node, "step1")
        assert first.reason == "released"

        node.set_parameters([rclpy.parameter.Parameter("guard_mode", value="unguarded")])

        second = _request_go(node, "step2")
        assert second.reason == "unguarded_by_config"
        node.destroy_node()

    def test_invalid_guard_mode_value_is_rejected(self, tmp_path) -> None:
        node = _make_node(tmp_path)
        result = node.set_parameters(
            [rclpy.parameter.Parameter("guard_mode", value="sideways")]
        )
        assert result[0].successful is False
        # Value must remain at whatever it was before the rejected set.
        assert node._get_guard_mode() == "guarded"
        node.destroy_node()


class TestLogging:
    def test_gate_records_carry_guard_mode(self, tmp_path) -> None:
        node = _make_node(tmp_path, guard_mode="unguarded")
        _request_go(node, "step1")
        node.destroy_node()

        log_lines = list(tmp_path.glob("orchestrator_*.jsonl"))
        assert len(log_lines) == 1
        content = log_lines[0].read_text()
        assert '"guard_mode": "unguarded"' in content
        assert '"type": "orchestrator_start"' in content
        assert '"type": "gate_unguarded_by_config"' in content
