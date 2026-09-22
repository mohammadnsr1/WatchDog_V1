"""Every run ends with exactly one final run_verdict, findings or not.

The case that forced this. The executor misseated u_orange at
place_u_orange, the watchdog correctly recorded non_conformant, the executor
then ran an unplanned reseat_u_orange, fixed it, and finished with all four
parts seated. The record afterwards held ONE non_conformant step verdict and
nothing else -- identical to what a run that misseated u_orange and gave up
would have produced. Nothing anywhere said the run had recovered.

That was the old cadence working as designed: "a finding was recorded -> no
rollup", on the reasoning that a rollup would double-count. It does not
double-count. Step verdicts are the TIMELINE (what went wrong, when); the
final verdict is the END-STATE (did the run ultimately succeed). Suppressing
the second because the first exists throws away the answer to the only
question that distinguishes recovery from failure.

So the rule is now: exactly one final run_verdict always, plus zero or more
step findings. These tests pin the four outcomes apart, and in particular pin
recovery and stay-broken as DISTINGUISHABLE -- they were not, which is the
whole defect.

findings_during_run and recovered are code-populated from the record_verdict
calls the server saw, never from an argument, for the same reason
executor_terminal_status is: an agent that forgot a finding could otherwise
report a recovery that never happened.

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
    """Supplies the observed run_ended without needing a live event bus."""

    def __init__(self, terminal_status=None):
        self._status = terminal_status

    def observed_terminal_status(self):
        return self._status


class _StubContext:
    def __init__(self, terminal_status=None):
        self.event_bus = _RecordingBus()
        self.node = _StubNode()
        self.recorder = _IngestOnlyRecorder(terminal_status)


@pytest.fixture
def ctx(monkeypatch):
    stub = _StubContext(terminal_status="completed")
    monkeypatch.setattr(ws, "_CONTEXT", stub)
    monkeypatch.setattr(ws, "_RECORDED_VERDICT_STEP_IDS", set())
    monkeypatch.setattr(ws, "_RUN", ws._RunTermination())
    return stub


def _finding(step_id, verdict="non_conformant"):
    return ws.record_verdict(
        step_id=step_id,
        verdict=verdict,
        clause_results=[
            {
                "clause_id": "effects.object_tcp_offset_m",
                "field": "placement_position_error_m",
                "comparison": "le",
                "threshold": 0.02,
                "observed": 0.032,
                "pass": False,
                "detail": "u_orange misseated",
            }
        ],
        explanation="finding",
    )


def _final(verdict, steps=("place_u_orange",), **kw):
    base = dict(
        verdict=verdict,
        clause_results=[],
        explanation="end-state assessment",
        steps_covered=list(steps),
    )
    base.update(kw)
    return ws.record_run_verdict(**base)


class _Payload(dict):
    """Attribute access over the envelope's plain-dict payload.

    EventEnvelope.payload is dumped to a dict on construction, so the typed
    model is not available downstream; this keeps the assertions readable
    without pretending otherwise.
    """

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc


def _run_verdict(ctx):
    events = ctx.event_bus.of_type("run_verdict")
    assert len(events) == 1, f"expected exactly one run_verdict, got {len(events)}"
    return _Payload(events[0].payload)


# --- the four outcomes, each distinguishable from the others ----------------

def test_recovery_run_is_conformant_and_flagged_recovered(ctx):
    """The u_orange case: finding recorded, then corrected, ends conformant."""
    _finding("place_u_orange")
    assert _final("conformant")["ok"] is True

    payload = _run_verdict(ctx)
    assert payload.verdict == "conformant"
    assert payload.findings_during_run == ["place_u_orange"]
    assert payload.recovered is True


def test_stay_broken_run_is_non_conformant_and_not_recovered(ctx):
    _finding("place_u_orange")
    _final("non_conformant")

    payload = _run_verdict(ctx)
    assert payload.verdict == "non_conformant"
    assert payload.findings_during_run == ["place_u_orange"]
    assert payload.recovered is False


def test_clean_run_is_conformant_with_no_findings_and_not_recovered(ctx):
    _final("conformant")

    payload = _run_verdict(ctx)
    assert payload.verdict == "conformant"
    assert payload.findings_during_run == []
    # Nothing went wrong, so there was nothing to recover from.
    assert payload.recovered is False


def test_aborted_run_is_incomplete(ctx):
    ctx.recorder = _IngestOnlyRecorder("aborted")
    _final("incomplete")

    payload = _run_verdict(ctx)
    assert payload.verdict == "incomplete"
    assert payload.executor_terminal_status == "aborted"


def test_recovery_and_stay_broken_are_distinguishable():
    """The defect, asserted directly rather than inferred from the pair above.

    Both runs record one non_conformant step verdict at the same step. Under
    the old cadence that was their ENTIRE record, so they were byte-identical.
    Compare them explicitly, because two tests that each pass in isolation can
    both pass while the two outcomes remain indistinguishable.
    """
    outcomes = {}
    for label, final_verdict in (("recovered", "conformant"), ("broken", "non_conformant")):
        stub = _StubContext(terminal_status="completed")
        saved = (ws._CONTEXT, ws._RUN, ws._RECORDED_VERDICT_STEP_IDS)
        ws._CONTEXT, ws._RUN, ws._RECORDED_VERDICT_STEP_IDS = (
            stub,
            ws._RunTermination(),
            set(),
        )
        try:
            _finding("place_u_orange")
            _final(final_verdict)
            payload = _run_verdict(stub)
            outcomes[label] = (payload.verdict, payload.recovered)
        finally:
            ws._CONTEXT, ws._RUN, ws._RECORDED_VERDICT_STEP_IDS = saved

    assert outcomes["recovered"] == ("conformant", True)
    assert outcomes["broken"] == ("non_conformant", False)
    assert outcomes["recovered"] != outcomes["broken"], (
        "a run that recovered and a run that ended broken produced the same "
        "record -- the distinction this cadence exists to preserve is gone"
    )


# --- findings_during_run is built from what the server SAW ------------------

def test_a_finding_no_longer_suppresses_the_final_verdict(ctx):
    """The old rule was 'finding -> no rollup'. It must be gone."""
    _finding("place_u_orange")
    result = _final("conformant")
    assert result["ok"] is True
    assert len(ctx.event_bus.of_type("run_verdict")) == 1
    assert len(ctx.event_bus.of_type("verdict")) == 1


def test_findings_are_ordered_and_deduplicated(ctx):
    _finding("place_u_brown")
    _finding("place_u_orange")
    _finding("place_u_orange")  # a second fire in the same step
    _final("conformant", steps=("place_u_brown", "place_u_orange"))

    payload = _run_verdict(ctx)
    # One entry per PLACE the run went wrong, in the order they occurred --
    # not one per finding event.
    assert payload.findings_during_run == ["place_u_brown", "place_u_orange"]
    assert payload.recovered is True


def test_a_conformant_step_verdict_is_not_a_finding(ctx):
    """findings_during_run is departures, not every step verdict ever written."""
    _finding("place_u_brown", verdict="conformant")
    _final("conformant")

    payload = _run_verdict(ctx)
    assert payload.findings_during_run == []
    assert payload.recovered is False


def test_inconclusive_step_verdict_counts_as_a_finding(ctx):
    """'I could not establish this step was sound' belongs on the timeline."""
    _finding("place_u_orange", verdict="inconclusive")
    _final("conformant")

    payload = _run_verdict(ctx)
    assert payload.findings_during_run == ["place_u_orange"]


def test_recovered_cannot_be_asserted_by_the_agent(ctx):
    """recovered is a function of the other two fields, not an agent claim."""
    # No findings at all, but the agent tries to claim a recovery.
    with pytest.raises(TypeError):
        ws.record_run_verdict(
            verdict="conformant",
            clause_results=[],
            explanation="x",
            steps_covered=["place_u_orange"],
            recovered=True,
        )


def test_recovered_is_false_when_findings_exist_but_verdict_is_incomplete(ctx):
    """Recovery means ended RIGHT, not merely ended."""
    ctx.recorder = _IngestOnlyRecorder("aborted")
    _finding("place_u_orange")
    _final("incomplete")

    payload = _run_verdict(ctx)
    assert payload.findings_during_run == ["place_u_orange"]
    assert payload.recovered is False


# --- independence: the end-state verdict is not the executor's claim --------

def test_verdict_is_not_derived_from_executor_terminal_status(ctx):
    """executor said completed, watchdog measured broken -- both recorded."""
    _final("non_conformant")

    payload = _run_verdict(ctx)
    assert payload.executor_terminal_status == "completed"
    assert payload.verdict == "non_conformant", (
        "the executor's terminal status must not steer the verdict"
    )


def test_conformant_is_possible_on_an_aborted_executor(ctx):
    """The two axes are independent in both directions.

    Guards the inverse of the test above: nothing may coerce a verdict from
    the terminal status, including coercing 'aborted' to incomplete.
    """
    ctx.recorder = _IngestOnlyRecorder("aborted")
    _final("conformant")

    payload = _run_verdict(ctx)
    assert payload.executor_terminal_status == "aborted"
    assert payload.verdict == "conformant"


# --- the server's own backstop obeys the same cadence -----------------------

def test_forced_finalize_reports_findings_it_observed(ctx):
    """A server-finalized run still joins its timeline to its final verdict."""
    _finding("place_u_orange")
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    result = ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    payload = _run_verdict(ctx)
    assert payload.findings_during_run == ["place_u_orange"]
    assert result["findings_during_run"] == ["place_u_orange"]
    # The old explanation asserted "no clause check failed" unconditionally,
    # which was flatly untrue on a run like this one.
    assert "no clause check failed" not in payload.explanation
    assert "place_u_orange" in payload.explanation


def test_forced_finalize_on_a_clean_run_still_says_nothing_failed(ctx):
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)

    payload = _run_verdict(ctx)
    assert payload.findings_during_run == []
    assert "no clause check failed" in payload.explanation


def test_exactly_one_final_verdict_even_via_the_backstop(ctx):
    """The backstop and the agent must not both produce one."""
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    assert len(ctx.event_bus.of_type("run_verdict")) == 1
    # Further waits after finalize add nothing.
    for _ in range(3):
        ws._timeout_or_finalize(ctx, "wait_for_event", 0.0)
    assert len(ctx.event_bus.of_type("run_verdict")) == 1
