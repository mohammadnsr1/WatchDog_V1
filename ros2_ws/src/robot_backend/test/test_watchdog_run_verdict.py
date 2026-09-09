"""The run-scoped rollup path: record_run_verdict and its validation.

Every real run so far has produced a finding, so the "clean run -> exactly one
conformant run-scoped rollup" branch has never executed against the cell.
These tests exercise it directly.

Both tools validate their arguments BEFORE calling _ensure_watchdog(), so the
rejection cases need no ROS at all. The happy paths install a stub context so
the publish path runs without booting a node.
"""

from __future__ import annotations

import os
import threading

import pytest

# Importing anything under mcp_servers runs ensure_mcp_server_environment(),
# which os.execve()s the process into the workspace venv when this flag is
# unset -- replacing the pytest process mid-collection, silently, with no
# output. Set it before the import so the test process survives.
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

    def warning(self, msg):
        self.warnings.append(msg)


class _StubNode:
    def __init__(self):
        self._logger = _StubLogger()

    def get_logger(self):
        return self._logger


class _StubContext:
    """Minimal stand-in for WatchDogContext: only what the verdict tools use."""

    def __init__(self):
        self.event_bus = _RecordingBus()
        self.node = _StubNode()


@pytest.fixture
def ctx(monkeypatch):
    """Install a stub watchdog context so _ensure_watchdog() short-circuits."""
    stub = _StubContext()
    monkeypatch.setattr(ws, "_CONTEXT", stub)
    monkeypatch.setattr(ws, "_RECORDED_VERDICT_STEP_IDS", set())
    return stub


def _entry(step_id, clause_id="effects.object_seated_position_error_m", passed=True):
    return {
        "step_id": step_id,
        "clause_id": clause_id,
        "field": "object_seated_position_error_m",
        "comparison": "le",
        "threshold": 0.005,
        "observed": 0.001,
        "pass": passed,
    }


CLEAN_ROLLUP = [
    _entry("place_u_green"),
    _entry("place_inverted_u_brown"),
    _entry("place_hex_blue"),
    _entry("place_hex_red"),
]
ALL_STEPS = [
    "pick_u_green", "reorient_u_green", "repick_u_green",
    "place_approach_u_green", "place_u_green",
    "pick_inverted_u_brown", "reorient_inverted_u_brown",
    "repick_inverted_u_brown", "place_approach_inverted_u_brown",
    "place_inverted_u_brown",
    "pick_hex_blue", "place_approach_hex_blue", "place_hex_blue",
    "pick_hex_red", "place_approach_hex_red", "place_hex_red",
]


# --- 1. the happy path that has never run against the cell ------------------

def test_clean_run_rollup_publishes_exactly_one_run_scoped_verdict(ctx):
    result = ws.record_run_verdict(
        verdict="conformant",
        clause_results=CLEAN_ROLLUP,
        steps_covered=ALL_STEPS,
        explanation="Whole-run rollup. Every clause evaluated at each part's "
                    "milestone passed; no monitor fired across the run.",
        claimed_outcome="success",
    )
    assert result["ok"] is True

    run_events = ctx.event_bus.of_type("run_verdict")
    assert len(run_events) == 1
    # and nothing leaked onto the step-scoped channel
    assert ctx.event_bus.of_type("verdict") == []

    payload = run_events[0].payload
    assert payload["verdict"] == "conformant"
    assert payload["run_scope"] is True


def test_rollup_retains_per_entry_step_id_through_to_the_event(ctx):
    """clause_id alone cannot disambiguate parts; step_id must survive."""
    ws.record_run_verdict(
        verdict="conformant",
        clause_results=CLEAN_ROLLUP,
        steps_covered=ALL_STEPS,
        explanation="rollup",
    )
    published = ctx.event_bus.of_type("run_verdict")[0].payload["clause_results"]

    assert [e["step_id"] for e in published] == [
        "place_u_green", "place_inverted_u_brown",
        "place_hex_blue", "place_hex_red",
    ]
    # The same clause_id recurs once per part -- step_id is the only separator.
    assert len({e["clause_id"] for e in published}) == 1
    assert len({e["step_id"] for e in published}) == 4


def test_rollup_steps_covered_present_and_covers_referenced_steps(ctx):
    ws.record_run_verdict(
        verdict="conformant",
        clause_results=CLEAN_ROLLUP,
        steps_covered=ALL_STEPS,
        explanation="rollup",
    )
    payload = ctx.event_bus.of_type("run_verdict")[0].payload
    assert payload["steps_covered"] == ALL_STEPS
    referenced = {e["step_id"] for e in payload["clause_results"]}
    assert referenced <= set(payload["steps_covered"])


# --- 2. per-entry step_id is enforced, not merely documented ----------------

def test_rollup_rejects_entry_missing_step_id():
    bad = [_entry("place_u_green"), {k: v for k, v in _entry("x").items()
                                     if k != "step_id"}]
    result = ws.record_run_verdict(
        verdict="conformant",
        clause_results=bad,
        steps_covered=["place_u_green"],
        explanation="rollup",
    )
    assert result["ok"] is False
    assert "step_id" in result["error"]


@pytest.mark.parametrize("value", [None, "", 7])
def test_validate_run_clause_results_rejects_unusable_step_id(value):
    entry = _entry("place_u_green")
    entry["step_id"] = value
    with pytest.raises(ValueError, match="step_id"):
        ws._validate_run_clause_results([entry])


def test_validate_run_clause_results_accepts_a_well_formed_rollup():
    ws._validate_run_clause_results(CLEAN_ROLLUP)   # must not raise


# --- 3. steps_covered accounting -------------------------------------------

def test_rollup_rejects_clause_step_absent_from_steps_covered():
    """Intended contract, read from record_run_verdict: this is a hard reject."""
    result = ws.record_run_verdict(
        verdict="conformant",
        clause_results=[_entry("place_u_green"), _entry("place_hex_red")],
        steps_covered=["place_u_green"],          # hex_red omitted
        explanation="rollup",
    )
    assert result["ok"] is False
    assert "place_hex_red" in result["error"]
    assert "steps_covered" in result["error"]


def test_rollup_allows_steps_covered_beyond_those_with_clause_results(ctx):
    """Coverage is a superset: a step can be watched with no clause due at it."""
    result = ws.record_run_verdict(
        verdict="conformant",
        clause_results=[_entry("place_u_green")],
        steps_covered=ALL_STEPS,
        explanation="rollup",
    )
    assert result["ok"] is True


def test_rollup_rejects_empty_steps_covered():
    result = ws.record_run_verdict(
        verdict="conformant",
        clause_results=CLEAN_ROLLUP,
        steps_covered=[],
        explanation="rollup",
    )
    assert result["ok"] is False
    assert "steps_covered" in result["error"]


def test_rollup_rejects_invalid_verdict():
    result = ws.record_run_verdict(
        verdict="probably_fine",
        clause_results=CLEAN_ROLLUP,
        steps_covered=ALL_STEPS,
        explanation="rollup",
    )
    assert result["ok"] is False


# --- 4. the step-scoped fire path is unaffected -----------------------------

def test_step_scoped_verdict_still_records_without_run_scope(ctx):
    result = ws.record_verdict(
        step_id="place_hex_red",
        verdict="non_conformant",
        clause_results=[{
            "clause_id": "invariants.grasp_effort_instantaneous",
            "field": "grasp_effort", "comparison": "le",
            "threshold": 16.0, "observed": 31.1, "pass": False,
        }],
        explanation="Effort exceeded the clamp bound during the close.",
        claimed_outcome="success",
    )
    assert result["ok"] is True
    assert result["duplicate_step_id"] is False

    events = ctx.event_bus.of_type("verdict")
    assert len(events) == 1
    payload = events[0].payload
    assert payload["step_id"] == "place_hex_red"
    assert payload["verdict"] == "non_conformant"
    # Step-scoped payloads carry no run_scope key at all -- the two verdict
    # kinds are distinguishable by shape, not by a sentinel value.
    assert "run_scope" not in payload
    assert ctx.event_bus.of_type("run_verdict") == []


def test_step_scoped_verdict_does_not_require_per_entry_step_id(ctx):
    """The rollup's extra requirement must not have leaked into the step path."""
    result = ws.record_verdict(
        step_id="place_u_green",
        verdict="conformant",
        clause_results=[{
            "clause_id": "effects.object_seated_position_error_m",
            "field": "e", "comparison": "le", "threshold": 0.005,
            "observed": 0.001, "pass": True,
        }],
        explanation="ok",
    )
    assert result["ok"] is True


def test_repeated_step_id_flags_duplicate_but_still_publishes(ctx):
    args = dict(
        step_id="place_u_green",
        verdict="non_conformant",
        clause_results=[{
            "clause_id": "effects.object_seated_position_error_m",
            "field": "e", "comparison": "le", "threshold": 0.005,
            "observed": 0.02, "pass": False,
        }],
        explanation="two findings on one step",
    )
    first = ws.record_verdict(**args)
    second = ws.record_verdict(**args)
    assert first["duplicate_step_id"] is False
    assert second["duplicate_step_id"] is True
    assert len(ctx.event_bus.of_type("verdict")) == 2


# --- the cadence itself is NOT enforced in code ----------------------------

def test_cadence_is_not_enforced_by_the_tools(ctx):
    """Documents the seam: nothing in code couples the two verdict paths.

    A rollup after a finding, and a per-step verdict on a clean step, are both
    accepted. The "clean run -> one rollup" cadence lives only in the watchdog
    prompt, so it is agent-dependent, not guaranteed. Pinning it here so the
    fact is visible if that ever changes.
    """
    ws.record_verdict(
        step_id="place_u_green", verdict="non_conformant",
        clause_results=[{
            "clause_id": "effects.object_seated_position_error_m",
            "field": "e", "comparison": "le", "threshold": 0.005,
            "observed": 0.02, "pass": False,
        }],
        explanation="a finding",
    )
    rollup = ws.record_run_verdict(
        verdict="conformant",
        clause_results=CLEAN_ROLLUP,
        steps_covered=ALL_STEPS,
        explanation="rollup recorded despite the finding above",
    )
    # Not rejected: no cross-check exists.
    assert rollup["ok"] is True
    assert len(ctx.event_bus.of_type("verdict")) == 1
    assert len(ctx.event_bus.of_type("run_verdict")) == 1
