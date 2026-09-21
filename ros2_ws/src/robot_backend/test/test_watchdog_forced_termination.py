"""The code-enforced termination backstop, and code-populated terminal status.

The failure these cover, from a real hard-crash run: the executor was killed
mid-step, both 180s wait_for_event calls timed out exactly as designed, and the
agent read each empty result as "keep waiting" -- still looping at 456s, no
run_verdict ever recorded, no exit. The timeout firing was never the problem.
The timeout firing not CAUSING anything was.

Termination is therefore enforced in this server, not asked of the agent: on
the Nth consecutive timeout the server records the incomplete rollup itself and
every later wait returns at once. These tests drive _timeout_or_finalize
directly, which is the whole guarantee.

Same import guard as the other watchdog tests -- importing anything under
mcp_servers runs ensure_mcp_server_environment(), which os.execve()s the
process into the workspace venv when this flag is unset.
"""

from __future__ import annotations

import os
import threading

import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

import robot_backend.mcp_servers.watchdog_mcp_server as ws  # noqa: E402


class _RecordingBus:
    def __init__(self) -> None:
        self.events = []
        self.lock = threading.Lock()

    def publish(self, event) -> None:
        with self.lock:
            self.events.append(event)

    def of_type(self, event_type):
        return [e for e in self.events if e.event_type == event_type]


class _StubLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _StubNode:
    def __init__(self):
        self._logger = _StubLogger()

    def get_logger(self):
        return self._logger


class _StubContext:
    def __init__(self):
        self.event_bus = _RecordingBus()
        self.node = _StubNode()


@pytest.fixture
def ctx(monkeypatch):
    """Fresh context AND fresh run state -- _RUN is module-level and sticky."""
    stub = _StubContext()
    monkeypatch.setattr(ws, "_CONTEXT", stub)
    monkeypatch.setattr(ws, "_RECORDED_VERDICT_STEP_IDS", set())
    monkeypatch.setattr(ws, "_RUN", ws._RunTermination())
    return stub


def _drain(event_type, payload):
    """A wait_for_event drain in the shape _observe_drained_events consumes."""
    return {
        "wake": event_type,
        "event": {"event_type": event_type, "envelope": {"payload": payload}},
        "additional_events": [],
    }


# --- 1. the termination guarantee -------------------------------------------

def test_first_timeout_warns_but_does_not_finalize(ctx):
    result = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    assert result["wake"] == "timeout"
    assert result.get("terminal") is not True
    assert result["consecutive_timeouts"] == 1
    # The agent is told how long it has left to act on its own.
    assert result["timeouts_before_forced_finalize"] == 1
    assert ctx.event_bus.of_type("run_verdict") == []


def test_second_timeout_finalizes_without_the_agent(ctx):
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    result = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    assert result["wake"] == "terminated"
    assert result["terminal"] is True
    assert result["run_finalized"] is True
    assert result["finalized_by"] == "server"
    assert result["verdict_recorded"] is True

    # THE point: the verdict exists even though no agent ever called a tool.
    events = ctx.event_bus.of_type("run_verdict")
    assert len(events) == 1
    assert events[0].payload["verdict"] == "incomplete"
    assert events[0].payload["run_scope"] is True


def test_hard_crash_records_the_no_terminal_event_sentinel(ctx):
    # No run_ended was ever drained, so there is no status to observe.
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    # Non-null sentinel, not None: a crash must be distinguishable from a
    # field nobody populated.
    assert payload["executor_terminal_status"] == "no_terminal_event"
    assert payload["executor_terminal_status"] is not None
    assert "probably crashed" in payload["explanation"]


def test_forced_finalize_reports_the_steps_it_observed(ctx):
    ws._observe_drained_events(_drain("begin_step", {"step_id": "pick_u_brown"}))
    ws._observe_drained_events(_drain("end_step", {"step_id": "pick_u_brown"}))
    ws._observe_drained_events(_drain("begin_step", {"step_id": "place_u_brown"}))
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    # Ordered, de-duplicated, and built from observation rather than a claim.
    assert payload["steps_covered"] == ["pick_u_brown", "place_u_brown"]


def test_further_waits_after_finalize_return_at_once_and_record_nothing(ctx):
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    # An agent that ignores the terminal result must not be able to hang the
    # run or add a second verdict.
    for _ in range(5):
        again = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
        assert again["terminal"] is True
        assert again["run_finalized"] is True
    assert len(ctx.event_bus.of_type("run_verdict")) == 1


def test_a_real_event_resets_the_timeout_counter(ctx):
    # A long but legitimate gap (the cell gate can block ~90s) must not walk
    # the run toward a forced finalize.
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    ws._observe_drained_events(_drain("skill_start", {"skill_name": "grasp"}))
    result = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    assert result["wake"] == "timeout"
    assert result["consecutive_timeouts"] == 1
    assert ctx.event_bus.of_type("run_verdict") == []


def test_finalize_after_an_observed_abort_uses_the_observed_status(ctx):
    # Executor aborted, then the agent fell silent. The backstop still fires,
    # and records what was actually observed rather than the crash sentinel.
    ws._observe_drained_events(_drain("run_ended", {"status": "aborted"}))
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["executor_terminal_status"] == "aborted"
    assert payload["verdict"] == "incomplete"


# --- 2. executor_terminal_status is code-populated --------------------------

def _rollup(**kw):
    base = dict(
        verdict="conformant",
        clause_results=[],
        steps_covered=["place_u_brown"],
        explanation="rollup",
    )
    base.update(kw)
    return ws.record_run_verdict(**base)


def test_aborted_status_threads_onto_the_rollup_without_the_agent(ctx):
    # The Issue-2 regression: run_ended said "aborted" and the recorded verdict
    # carried None because the agent simply did not pass it.
    ws._observe_drained_events(_drain("run_ended", {"status": "aborted"}))
    assert _rollup(verdict="incomplete")["ok"] is True
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["executor_terminal_status"] == "aborted"


def test_completed_status_threads_onto_the_rollup(ctx):
    ws._observe_drained_events(_drain("run_ended", {"status": "completed"}))
    assert _rollup()["ok"] is True
    assert (
        ctx.event_bus.of_type("run_verdict")[0].payload["executor_terminal_status"]
        == "completed"
    )


def test_observed_status_beats_a_wrong_agent_argument(ctx):
    ws._observe_drained_events(_drain("run_ended", {"status": "aborted"}))
    _rollup(executor_terminal_status="completed")
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["executor_terminal_status"] == "aborted"
    # And the disagreement is surfaced rather than silently resolved.
    assert any("observed" in w for w in ctx.node.get_logger().warnings)


def test_rollup_with_no_run_ended_gets_the_sentinel_not_none(ctx):
    assert _rollup()["ok"] is True
    assert (
        ctx.event_bus.of_type("run_verdict")[0].payload["executor_terminal_status"]
        == "no_terminal_event"
    )


def test_agent_rollup_marks_the_run_finalized(ctx):
    # Once the agent has recorded, the backstop must not add a second verdict.
    ws._observe_drained_events(_drain("run_ended", {"status": "completed"}))
    _rollup()
    after = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    assert after["terminal"] is True
    assert len(ctx.event_bus.of_type("run_verdict")) == 1


# --- 3. placement frames are in the independent evidence channel ------------

def _registry_path():
    """The source registry, next to this test's own package -- not the
    installed copy, whose share/ layout differs from site-packages."""
    import pathlib

    return pathlib.Path(__file__).resolve().parent.parent / "config" / "model_registry.yaml"


def test_placement_hole_and_reorient_frames_are_sampled():
    """The seating check's target must come from the watchdog's own buffer.

    These were excluded because the recorder filtered on `enabled`, which is
    collision-geometry scope (scene_tf_sync_node skips disabled entries when
    building collision objects) and not evidence scope. The effect was that
    query_snapshot returned null for every placement frame, so "the part is
    seated at its placement frame within tolerance" had nowhere to get the
    target except the executor's own report -- collapsing independence on the
    one measurement the channel exists for.
    """
    import yaml

    from robot_backend.watchdog.recorder import _sampled_object_frames

    registry = yaml.safe_load(_registry_path().read_text())
    frames = _sampled_object_frames(registry)

    for frame_id in (
        "u_brown_placement",
        "u_orange_placement",
        "line_green_placement",
        "inverted_u_yellow_placement",
        "hex_blue_hole",
        "hex_red_hole",
        "u_green_reorient",
        "inverted_u_brown_reorient",
    ):
        assert frame_id in frames, f"{frame_id} is not in the watchdog's evidence buffer"
        assert frames[frame_id]["tf_frame"]
        assert frames[frame_id]["lookup_frame"]


def test_ordinary_parts_and_fixtures_are_still_sampled():
    import yaml

    from robot_backend.watchdog.recorder import _sampled_object_frames

    registry = yaml.safe_load(_registry_path().read_text())
    frames = _sampled_object_frames(registry)
    for frame_id in ("u_brown", "u_orange", "line_green", "base1", "base2"):
        assert frame_id in frames
