"""executor_terminal_status must come from the OBSERVED run_ended, any path.

The bug these pin, from the abort run of 2026-09-21 (session 20260921T215124Z):

  run_verdict.executor_terminal_status == "no_terminal_event"
  run_verdict.explanation            == "... run_ended reported status 'aborted' ..."

One payload, two answers. The executor HAD emitted run_ended(status="aborted")
and the watchdog HAD seen it -- the explanation is written from it -- but the
structured field carried the hard-crash sentinel.

Mechanism: the status was captured only by _observe_drained_events, whose sole
call site is _drain_wait, i.e. the wait_for_event path. Timeline from the run:

  21:54:57  last wait_for_event starts, drains place_u_brown events, returns
  21:56:06  executor emits run_ended(status="aborted")     <- no wait open
  21:56:08  agent calls get_events, SEES the abort, writes the explanation
  21:57:50  record_run_verdict -> field falls through to the sentinel

get_events, get_evidence and get_execution_state never called the capture, so
run_ended reached the agent's reasoning but not the field. The clean run
(20260921T213117Z) recorded "completed" correctly only by luck: its run_ended
fired 2s into an already-open blocking wait.

Why it matters enough for its own file: executor_terminal_status is the
run-level independence column, and "aborted" (executor gave up and said so)
versus "no_terminal_event" (hard crash, said nothing) are the two most
different executor outcomes there are. Collapsing them loses the distinction
silently, behind a value that looks valid.

The existing tests in test_watchdog_forced_termination.py could not catch this:
they drive the capture through a _drain(...) helper, which simulates the one
delivery path that always worked. Every test here goes through the recorder's
ingest buffer instead -- what the subscriber received, regardless of which tool
surfaced it.

Same import guard as the other watchdog tests.
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


class _IngestOnlyRecorder:
    """Recorder whose events arrived by ingest and were never drained.

    This is the production situation the bug needed: the subscriber received
    run_ended and buffered it, but no wait_for_event was open to drain it, so
    _observe_drained_events never ran. Mirrors EvidenceRecorder.events (records
    with top-level event_type/payload) and its observed_terminal_status scan.
    """

    def __init__(self) -> None:
        self.events: list[dict] = []

    def ingest(self, event_type: str, payload: dict) -> None:
        self.events.append({"event_type": event_type, "payload": payload})

    def observed_terminal_status(self) -> str | None:
        for record in reversed(self.events):
            if record.get("event_type") != "run_ended":
                continue
            payload = record.get("payload")
            if isinstance(payload, dict):
                status = payload.get("status")
                if isinstance(status, str) and status:
                    return status
        return None


class _StubContext:
    def __init__(self):
        self.event_bus = _RecordingBus()
        self.node = _StubNode()
        self.recorder = _IngestOnlyRecorder()


@pytest.fixture
def ctx(monkeypatch):
    """Fresh context AND fresh run state -- _RUN is module-level and sticky."""
    stub = _StubContext()
    monkeypatch.setattr(ws, "_CONTEXT", stub)
    monkeypatch.setattr(ws, "_RECORDED_VERDICT_STEP_IDS", set())
    monkeypatch.setattr(ws, "_RUN", ws._RunTermination())
    return stub


def _rollup(**kw):
    base = dict(
        verdict="incomplete",
        clause_results=[],
        steps_covered=["place_u_brown"],
        explanation="rollup",
    )
    base.update(kw)
    return ws.record_run_verdict(**base)


def _recorded_status(ctx) -> str | None:
    events = ctx.event_bus.of_type("run_verdict")
    assert len(events) == 1, f"expected exactly one run_verdict, got {len(events)}"
    return events[0].payload["executor_terminal_status"]


# --- the four cases, each driven through ingest, not through the drain -------

def test_abort_observed_only_via_ingest_records_aborted(ctx):
    """THE BUG: run_ended=aborted arrives with no wait open -> must be "aborted"."""
    ctx.recorder.ingest("run_ended", {"status": "aborted"})
    # Nothing was ever drained, exactly as in the production run.
    assert ws._RUN.observed_terminal_status is None, (
        "test setup error: the drain capture must be empty -- that is the whole "
        "point of this test"
    )

    assert _rollup()["ok"] is True

    assert _recorded_status(ctx) == "aborted", (
        "the sentinel was recorded for a run whose run_ended said 'aborted'; the "
        "field is still keyed to the wait_for_event drain rather than to what was "
        "observed"
    )


def test_clean_observed_only_via_ingest_records_completed(ctx):
    ctx.recorder.ingest("run_ended", {"status": "completed"})
    assert _rollup(verdict="conformant")["ok"] is True
    assert _recorded_status(ctx) == "completed"


def test_hard_crash_with_no_run_ended_records_the_sentinel(ctx):
    """The ONLY case the sentinel is correct: nothing terminal was ever ingested."""
    ctx.recorder.ingest("begin_step", {"step_id": "pick_u_brown"})
    # No run_ended of any status -- the executor died mid-step.
    assert _rollup()["ok"] is True
    assert _recorded_status(ctx) == "no_terminal_event"


def test_abort_and_hard_crash_do_not_collapse_to_the_same_value():
    """The distinction itself, asserted directly rather than inferred.

    Both cases produce verdict="incomplete", so executor_terminal_status is the
    only thing in the payload that separates "the executor gave up and told us"
    from "the executor vanished". A test that checks them in isolation can pass
    while they are equal, which is how this shipped -- so compare them.
    """
    statuses = {}
    for label, terminal in (("abort", {"status": "aborted"}), ("crash", None)):
        stub = _StubContext()
        run = ws._RunTermination()
        original_ctx, original_run = ws._CONTEXT, ws._RUN
        original_ids = ws._RECORDED_VERDICT_STEP_IDS
        ws._CONTEXT, ws._RUN, ws._RECORDED_VERDICT_STEP_IDS = stub, run, set()
        try:
            if terminal is not None:
                stub.recorder.ingest("run_ended", terminal)
            _rollup()
            statuses[label] = _recorded_status(stub)
        finally:
            ws._CONTEXT, ws._RUN = original_ctx, original_run
            ws._RECORDED_VERDICT_STEP_IDS = original_ids

    assert statuses["abort"] == "aborted"
    assert statuses["crash"] == "no_terminal_event"
    assert statuses["abort"] != statuses["crash"], (
        f"abort and hard crash both recorded {statuses['abort']!r} -- the two most "
        "different executor outcomes are indistinguishable in the verdict payload"
    )


# --- the sentinel must never appear when a run_ended WAS observed ------------

@pytest.mark.parametrize("status", ["aborted", "completed", "failed", "cancelled"])
def test_sentinel_never_appears_when_any_run_ended_was_observed(ctx, status):
    """Whatever status the executor used, the sentinel is the wrong answer.

    The field passes the status through rather than validating it against a
    known set, so an unexpected status must still not become the sentinel --
    recording "no run_ended arrived" about a run that ended is worse than
    recording an unfamiliar value the scorer can see and question.
    """
    ctx.recorder.ingest("run_ended", {"status": status})
    _rollup()
    recorded = _recorded_status(ctx)
    assert recorded == status
    assert recorded != ws._NO_TERMINAL_EVENT


def test_last_run_ended_wins_when_several_were_ingested(ctx):
    ctx.recorder.ingest("run_ended", {"status": "completed"})
    ctx.recorder.ingest("run_ended", {"status": "aborted"})
    _rollup()
    assert _recorded_status(ctx) == "aborted"


def test_a_run_ended_with_no_usable_status_is_not_mistaken_for_one(ctx):
    """An empty or non-string status is not an observation, so the sentinel stands."""
    ctx.recorder.ingest("run_ended", {"status": ""})
    ctx.recorder.ingest("run_ended", {})
    _rollup()
    assert _recorded_status(ctx) == "no_terminal_event"


# --- the drain path must keep working, and the two sources must agree --------

def test_drained_status_still_works_when_the_recorder_has_nothing(ctx):
    """The pre-existing path is a fallback now, not the only source.

    Kept because the ingest buffer is bounded (maxlen 5000) and a long run
    could in principle evict a run_ended the drain capture still holds.
    """
    ws._observe_drained_events(
        {
            "wake": "run_ended",
            "event": {"event_type": "run_ended", "envelope": {"payload": {"status": "aborted"}}},
            "additional_events": [],
        }
    )
    assert ctx.recorder.observed_terminal_status() is None
    _rollup()
    assert _recorded_status(ctx) == "aborted"


def test_agent_argument_cannot_override_an_observation(ctx):
    """The field is code-populated; a wrong agent argument must lose and be logged."""
    ctx.recorder.ingest("run_ended", {"status": "aborted"})
    _rollup(executor_terminal_status="completed")

    assert _recorded_status(ctx) == "aborted"
    warnings = " ".join(ctx.node.get_logger().warnings)
    assert "executor_terminal_status" in warnings, (
        "a disagreement between the agent's claim and the observation must be "
        "logged, not silently resolved"
    )


def test_agent_argument_is_still_a_fallback_when_nothing_was_observed(ctx):
    """Neither source has it, so a caller that somehow does is better than the sentinel."""
    _rollup(executor_terminal_status="aborted")
    assert _recorded_status(ctx) == "aborted"


# --- the forced-finalize backstop resolves from the same source --------------

def test_forced_finalize_uses_an_ingested_abort_not_the_crash_explanation(ctx):
    """The backstop chooses between two very different explanations.

    Reading only the drain capture here would assert "the executor probably
    crashed" about a run that had cleanly reported aborted through get_events.
    """
    ctx.recorder.ingest("run_ended", {"status": "aborted"})
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    result = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    assert result["executor_terminal_status"] == "aborted"
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["executor_terminal_status"] == "aborted"
    assert "probably crashed" not in payload["explanation"], (
        "the run reported a terminal status, so the crash explanation is wrong"
    )


def test_forced_finalize_still_reports_a_real_crash_as_such(ctx):
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    result = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    assert result["executor_terminal_status"] == "no_terminal_event"
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["executor_terminal_status"] == "no_terminal_event"
    assert "probably crashed" in payload["explanation"]


# --- finding+abort: the step-scoped verdict carries no status (choice (a)) ---

def test_step_verdict_carries_no_terminal_status_field(ctx):
    """Confirm the choice-(a) design did not regress into stamping the sentinel.

    A run with a finding records step-scoped verdicts through record_verdict and
    may never call record_run_verdict at all. Those payloads deliberately have
    no executor_terminal_status -- a step verdict makes no run-level claim, and
    the scorer recovers the terminal status by joining the scene event stream.
    The risk worth pinning is the opposite of the run-scope bug: a step verdict
    must not acquire the field and report the sentinel as if it were a
    run-level finding.
    """
    ctx.recorder.ingest("run_ended", {"status": "aborted"})
    result = ws.record_verdict(
        step_id="place_u_brown",
        verdict="non_conformant",
        clause_results=[
            {
                "clause_id": "effects.object_tcp_offset_m",
                "field": "in_hand_slip_m",
                "comparison": "le",
                "threshold": 0.02,
                "observed": 0.0212,
                "pass": False,
                "detail": "slip",
            }
        ],
        explanation="finding",
    )
    assert result["ok"] is True

    payload = ctx.event_bus.of_type("verdict")[0].payload
    assert payload.get("run_scope") is not True
    assert "executor_terminal_status" not in payload, (
        "a step-scoped verdict must not carry a run-level terminal status"
    )
    # And the run-scope path is untouched: the abort is still recoverable.
    assert ctx.recorder.observed_terminal_status() == "aborted"


# --- the recorder method itself, against a real EvidenceRecorder buffer ------

def test_recorder_scans_its_real_ingest_buffer():
    """Pins the method against the real record shape, not the stub's.

    The stub above mirrors EvidenceRecorder.events by hand, so it would keep
    passing if the real record shape changed underneath it. This asserts
    against the real class: records carry top-level event_type and payload
    (event_wrapper's nested "envelope" shape is the agent-facing projection,
    which is a different thing and was part of why this was easy to get wrong).
    """
    from robot_backend.watchdog.recorder import EvidenceRecorder

    recorder = EvidenceRecorder.__new__(EvidenceRecorder)
    recorder.lock = threading.Lock()
    recorder.events = __import__("collections").deque(maxlen=5000)

    assert recorder.observed_terminal_status() is None

    recorder.events.append({"event_type": "begin_step", "payload": {"step_id": "pick"}})
    assert recorder.observed_terminal_status() is None

    recorder.events.append({"event_type": "run_ended", "payload": {"status": "aborted"}})
    assert recorder.observed_terminal_status() == "aborted"
