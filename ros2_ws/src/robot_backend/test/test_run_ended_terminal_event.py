"""The executor's terminal event and the run-scoped "incomplete" verdict.

Before run_ended existed the WatchDog could only infer that a run had stopped
from silence -- two consecutive wait_for_event timeouts -- which cost a full
timeout cycle on EVERY ending, clean ones included, and could not tell "the
executor finished" from "the executor gave up" from "the executor crashed".
These tests cover the three things that replaced that:

  1. run_ended itself: a real event type, executor-origin, with a validated
     status.
  2. "incomplete": a run-scoped verdict distinct from both non_conformant
     (a violation was observed) and inconclusive (evidence was missing).
  3. The independence constraint: the executor's status is recorded BESIDE
     the WatchDog's verdict and never constrains it.

Same import guard as test_watchdog_run_verdict.py -- importing anything under
mcp_servers runs ensure_mcp_server_environment(), which os.execve()s the
process into the workspace venv when this flag is unset, replacing the pytest
process mid-collection with no output.
"""

from __future__ import annotations

import os
import threading

import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

from robot_backend.events.schema import (  # noqa: E402
    EVENT_TYPE_PAYLOAD_CLASSES,
    KNOWN_EVENT_TYPES,
    RunEndedPayload,
    RunVerdictPayload,
    event_type_origin,
    make_event,
)
import robot_backend.mcp_servers.watchdog_mcp_server as ws  # noqa: E402


# --- stubs, mirroring test_watchdog_run_verdict.py --------------------------

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

    def warning(self, msg):
        self.warnings.append(msg)


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
    stub = _StubContext()
    monkeypatch.setattr(ws, "_CONTEXT", stub)
    monkeypatch.setattr(ws, "_RECORDED_VERDICT_STEP_IDS", set())
    return stub


def _entry(step_id, passed=True):
    return {
        "step_id": step_id,
        "clause_id": "effects.object_seated_position_error_m",
        "field": "object_seated_position_error_m",
        "comparison": "le",
        "threshold": 0.005,
        "observed": 0.001,
        "pass": passed,
    }


# --- 1. run_ended is a first-class event type -------------------------------

def test_run_ended_is_a_known_executor_event_type():
    assert "run_ended" in KNOWN_EVENT_TYPES
    assert "run_ended" in EVENT_TYPE_PAYLOAD_CLASSES
    # It is the executor's declaration, not the WatchDog's -- available_event_types()
    # reports origin so the monitoring agent knows which side produces it.
    assert event_type_origin("run_ended") == "executor"


def test_known_event_types_and_payload_classes_cannot_drift():
    # KNOWN_EVENT_TYPES is read off EventEnvelope's Literal; the payload map is
    # hand-written. A type added to one and not the other is the failure mode.
    assert set(KNOWN_EVENT_TYPES) == set(EVENT_TYPE_PAYLOAD_CLASSES)


@pytest.mark.parametrize("status", ["completed", "aborted"])
def test_run_ended_accepts_both_terminal_statuses(status):
    event = make_event("run_ended", RunEndedPayload(status=status, reason="x"))
    assert event.event_type == "run_ended"
    assert event.payload["status"] == status
    assert event.wall_time_iso  # the field the WatchDog measures gaps against


def test_run_ended_rejects_an_unknown_status():
    # "gave_up", "failed", "success" are all plausible-looking and all wrong;
    # a typo must fail at construction, not silently become a third status the
    # WatchDog's precedence has no branch for.
    with pytest.raises(Exception):
        RunEndedPayload(status="gave_up")


# --- 2. "incomplete" is accepted, and is run-scoped only --------------------

def test_record_run_verdict_accepts_incomplete(ctx):
    result = ws.record_run_verdict(
        verdict="incomplete",
        clause_results=[_entry("place_u_green")],
        steps_covered=["pick_u_green", "place_u_green"],
        explanation="Executor aborted after seating u_green; hex_blue and "
                    "hex_red were never reached. No monitor fired and no "
                    "clause failed on the evidence gathered.",
        executor_terminal_status="aborted",
    )
    assert result["ok"] is True
    events = ctx.event_bus.of_type("run_verdict")
    assert len(events) == 1
    assert events[0].payload["verdict"] == "incomplete"
    assert events[0].payload["run_scope"] is True


def test_incomplete_is_rejected_on_a_per_step_verdict(ctx):
    # It describes the run stopping early, which is not a property of any one
    # step. A step the executor abandoned still gets judged on its own evidence.
    result = ws.record_verdict(
        step_id="place_hex_red",
        verdict="incomplete",
        clause_results=[_entry("place_hex_red")],
        explanation="should be rejected",
    )
    assert result["ok"] is False
    assert "run-scoped" in str(result).lower()
    assert ctx.event_bus.of_type("verdict") == []


def test_the_other_four_verdicts_still_work(ctx):
    # Adding a fifth value must not disturb the existing vocabulary.
    for verdict in ("conformant", "non_conformant", "valid_variation", "inconclusive"):
        assert ws.record_run_verdict(
            verdict=verdict,
            clause_results=[_entry("place_u_green")],
            steps_covered=["place_u_green"],
            explanation=f"rollup: {verdict}",
        )["ok"] is True
    assert len(ctx.event_bus.of_type("run_verdict")) == 4


def test_a_bogus_verdict_is_still_rejected(ctx):
    assert ws.record_run_verdict(
        verdict="probably_fine",
        clause_results=[_entry("place_u_green")],
        steps_covered=["place_u_green"],
        explanation="x",
    )["ok"] is False


# --- 3. independence: status is recorded beside the verdict, never into it ---

def test_executor_status_is_a_separate_field_from_the_verdict(ctx):
    ws.record_run_verdict(
        verdict="conformant",
        clause_results=[_entry("place_u_green")],
        steps_covered=["place_u_green"],
        explanation="clean rollup",
        executor_terminal_status="completed",
    )
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    # Two distinct fields the scorer can cross-tabulate, not one merged value.
    assert payload["verdict"] == "conformant"
    assert payload["executor_terminal_status"] == "completed"
    assert payload["claimed_outcome"] is None  # and distinct from the per-step claim


def test_executor_claiming_completed_does_not_force_a_conformant_verdict(ctx):
    # THE run-level independence case: the executor says it finished, the
    # WatchDog's own evidence says a clause failed. Both are recorded, and the
    # verdict is the WatchDog's.
    result = ws.record_run_verdict(
        verdict="non_conformant",
        clause_results=[_entry("place_hex_red", passed=False)],
        steps_covered=["place_hex_red"],
        explanation="Executor claimed the run completed; hex_red seated "
                    "position error exceeded its clause threshold.",
        claimed_outcome="success",
        executor_terminal_status="completed",
    )
    assert result["ok"] is True
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["verdict"] == "non_conformant"
    assert payload["executor_terminal_status"] == "completed"


def test_executor_aborting_does_not_force_incomplete(ctx):
    # The other direction: an executor that aborted BECAUSE the WatchDog caught
    # something is still non_conformant. The finding outranks the status.
    ws.record_run_verdict(
        verdict="non_conformant",
        clause_results=[_entry("place_u_green", passed=False)],
        steps_covered=["place_u_green"],
        explanation="Collision monitor fired; executor then aborted.",
        executor_terminal_status="aborted",
    )
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["verdict"] == "non_conformant"
    assert payload["executor_terminal_status"] == "aborted"


def test_terminal_status_is_optional_for_the_crash_path(ctx):
    # Hard crash: no run_ended ever arrived, so there is no status to record.
    # The field must accept None rather than forcing a fabricated value.
    ws.record_run_verdict(
        verdict="incomplete",
        clause_results=[_entry("place_u_green")],
        steps_covered=["place_u_green"],
        explanation="Terminated on the no-event timeout with no run_ended; "
                    "the executor probably crashed.",
    )
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["verdict"] == "incomplete"
    assert payload["executor_terminal_status"] is None


def test_run_verdict_payload_defaults_terminal_status_to_none():
    # Backward compatibility: every existing caller omits the new field.
    payload = RunVerdictPayload(
        verdict="conformant",
        claimed_outcome=None,
        clause_results=[],
        steps_covered=["a"],
        explanation="x",
    )
    assert payload.executor_terminal_status is None
