"""Tests for the execution event bus — no ROS required.

Covers:
  - EventEnvelope schema construction and JSON round-trip
  - EventBus file logging (temp file)
  - EventBus no-op mode (node=None, log_path=None)
  - @emit_skill_events decorator — happy path, exception path, param capture

Run with: pytest test/test_event_bus.py -v
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from robot_backend.events.bus import EventBus, configure_bus, get_bus, shutdown_bus
from robot_backend.events.decorator import emit_skill_events, _sanitize_params
from robot_backend.events.schema import (
    BeginStepPayload,
    EndStepPayload,
    EventEnvelope,
    SkillEndPayload,
    SkillStartPayload,
    make_event,
)


# ---------------------------------------------------------------------------
# Schema tests
# ---------------------------------------------------------------------------

class TestSchema:

    def test_make_event_begin_step(self) -> None:
        event = make_event("begin_step", BeginStepPayload(step_id="s1"))
        assert event.event_type == "begin_step"
        assert event.payload["step_id"] == "s1"
        assert event.payload["object_ids"] == []
        assert event.payload["notes"] == ""
        assert len(event.event_id) == 36  # UUID4
        assert event.timestamp_ns > 0

    def test_make_event_end_step(self) -> None:
        event = make_event(
            "end_step",
            EndStepPayload(step_id="s1", claimed_outcome="success", notes="ok"),
        )
        assert event.event_type == "end_step"
        assert event.payload["claimed_outcome"] == "success"
        assert event.payload["notes"] == "ok"

    def test_make_event_skill_start(self) -> None:
        event = make_event(
            "skill_start",
            SkillStartPayload(
                skill_name="pick_object",
                params={"object_id": "red_cube"},
            ),
        )
        assert event.payload["skill_name"] == "pick_object"
        assert event.payload["params"] == {"object_id": "red_cube"}

    def test_make_event_skill_end(self) -> None:
        event = make_event(
            "skill_end",
            SkillEndPayload(skill_name="pick_object", duration_ns=12345, ok=True),
        )
        assert event.payload["ok"] is True
        assert event.payload["duration_ns"] == 12345
        assert event.payload["error"] is None

    def test_event_envelope_is_json_serializable(self) -> None:
        event = make_event("begin_step", BeginStepPayload(step_id="x"))
        json_str = event.model_dump_json()
        parsed = json.loads(json_str)
        assert parsed["event_type"] == "begin_step"
        assert "timestamp_ns" in parsed
        assert "wall_time_iso" in parsed

    def test_end_step_invalid_outcome_raises(self) -> None:
        with pytest.raises(Exception):
            EndStepPayload(step_id="s", claimed_outcome="maybe")  # type: ignore[arg-type]

    def test_timestamps_are_monotonically_increasing(self) -> None:
        e1 = make_event("begin_step", BeginStepPayload(step_id="a"))
        e2 = make_event("end_step", EndStepPayload(step_id="a", claimed_outcome="success"))
        assert e2.timestamp_ns >= e1.timestamp_ns

    def test_event_ids_are_unique(self) -> None:
        events = [make_event("begin_step", BeginStepPayload(step_id=str(i))) for i in range(10)]
        ids = {e.event_id for e in events}
        assert len(ids) == 10

    def test_begin_step_object_ids_and_notes(self) -> None:
        event = make_event(
            "begin_step",
            BeginStepPayload(step_id="s", object_ids=["red_cube"], notes="picking up"),
        )
        assert event.payload["object_ids"] == ["red_cube"]
        assert event.payload["notes"] == "picking up"


# ---------------------------------------------------------------------------
# EventBus tests
# ---------------------------------------------------------------------------

class TestEventBus:

    def test_noop_bus_does_not_raise(self) -> None:
        bus = EventBus(node=None, log_path=None)
        event = make_event("begin_step", BeginStepPayload(step_id="s"))
        bus.publish(event)  # must not raise

    def test_file_logging(self, tmp_path: Path) -> None:
        log_file = tmp_path / "events.jsonl"
        bus = EventBus(node=None, log_path=log_file)
        event = make_event("begin_step", BeginStepPayload(step_id="s"))
        bus.publish(event)
        bus.close()

        lines = log_file.read_text().strip().splitlines()
        assert len(lines) == 1
        parsed = json.loads(lines[0])
        assert parsed["event_type"] == "begin_step"

    def test_file_logging_multiple_events(self, tmp_path: Path) -> None:
        log_file = tmp_path / "events.jsonl"
        bus = EventBus(node=None, log_path=log_file)
        bus.publish(make_event("begin_step", BeginStepPayload(step_id="s")))
        bus.publish(make_event("end_step", EndStepPayload(step_id="s", claimed_outcome="success")))
        bus.publish(make_event(
            "skill_start",
            SkillStartPayload(skill_name="pick_object", params={})
        ))
        bus.close()

        lines = log_file.read_text().strip().splitlines()
        assert len(lines) == 3
        types = [json.loads(l)["event_type"] for l in lines]
        assert types == ["begin_step", "end_step", "skill_start"]

    def test_emit_begin_step_helper(self, tmp_path: Path) -> None:
        bus = EventBus(node=None, log_path=tmp_path / "e.jsonl")
        bus.emit_begin_step("s1", object_ids=["red_cube"], notes="picking up")
        bus.close()
        lines = (tmp_path / "e.jsonl").read_text().strip().splitlines()
        parsed = json.loads(lines[0])
        assert parsed["event_type"] == "begin_step"
        assert parsed["payload"]["object_ids"] == ["red_cube"]
        assert parsed["payload"]["notes"] == "picking up"

    def test_emit_end_step_helper(self, tmp_path: Path) -> None:
        bus = EventBus(node=None, log_path=tmp_path / "e.jsonl")
        bus.emit_end_step("s1", "failure", notes="dropped")
        bus.close()
        parsed = json.loads((tmp_path / "e.jsonl").read_text())
        assert parsed["payload"]["claimed_outcome"] == "failure"
        assert parsed["payload"]["notes"] == "dropped"

    def test_emit_skill_start_helper(self, tmp_path: Path) -> None:
        bus = EventBus(node=None, log_path=tmp_path / "e.jsonl")
        bus.emit_skill_start(
            "pick_object",
            params={"object_id": "red_cube"},
        )
        bus.close()
        parsed = json.loads((tmp_path / "e.jsonl").read_text())
        assert parsed["payload"]["skill_name"] == "pick_object"
        assert parsed["payload"]["params"] == {"object_id": "red_cube"}

    def test_emit_skill_end_helper(self, tmp_path: Path) -> None:
        bus = EventBus(node=None, log_path=tmp_path / "e.jsonl")
        bus.emit_skill_end("pick_object", duration_ns=500_000, ok=False, error="gripper miss")
        bus.close()
        parsed = json.loads((tmp_path / "e.jsonl").read_text())
        assert parsed["payload"]["ok"] is False
        assert "gripper miss" in parsed["payload"]["error"]

    def test_singleton_configure_and_get(self, tmp_path: Path) -> None:
        configure_bus(log_path=tmp_path / "e.jsonl")
        bus = get_bus()
        assert bus is not None
        bus.emit_begin_step("s")
        shutdown_bus()
        lines = (tmp_path / "e.jsonl").read_text().strip().splitlines()
        assert len(lines) == 1

    def test_get_bus_returns_noop_without_configure(self) -> None:
        shutdown_bus()  # ensure no singleton
        bus = get_bus()
        # Should be a no-op bus — publish must not raise
        bus.publish(make_event("begin_step", BeginStepPayload(step_id="s")))

    def test_close_is_idempotent(self, tmp_path: Path) -> None:
        bus = EventBus(node=None, log_path=tmp_path / "e.jsonl")
        bus.close()
        bus.close()  # second close must not raise


# ---------------------------------------------------------------------------
# Decorator tests
# ---------------------------------------------------------------------------

class TestEmitSkillEventsDecorator:

    def _make_logging_bus(self, tmp_path: Path) -> tuple[EventBus, Path]:
        log = tmp_path / "events.jsonl"
        bus = EventBus(node=None, log_path=log)
        return bus, log

    def test_decorator_emits_start_and_end(self, tmp_path: Path) -> None:
        bus, log = self._make_logging_bus(tmp_path)
        configure_bus(log_path=log)

        @emit_skill_events
        def my_skill(backend: Any, *, object_id: str) -> dict:
            return {"ok": True, "command": "my_skill"}

        my_skill(None, object_id="red_cube")
        shutdown_bus()

        lines = log.read_text().strip().splitlines()
        assert len(lines) == 2
        types = [json.loads(l)["event_type"] for l in lines]
        assert types == ["skill_start", "skill_end"]

    def test_decorator_captures_kwargs_in_params(self, tmp_path: Path) -> None:
        bus, log = self._make_logging_bus(tmp_path)
        configure_bus(log_path=log)

        @emit_skill_events
        def pick_object(backend: Any, *, object_id: str, width_m: float) -> dict:
            return {"ok": True, "command": "pick_object"}

        pick_object(None, object_id="red_cube", width_m=0.065)
        shutdown_bus()

        lines = log.read_text().strip().splitlines()
        start_event = json.loads(lines[0])
        assert start_event["event_type"] == "skill_start"
        assert start_event["payload"]["params"]["object_id"] == "red_cube"
        assert start_event["payload"]["params"]["width_m"] == pytest.approx(0.065)

    def test_decorator_omits_positional_args_from_params(self, tmp_path: Path) -> None:
        configure_bus(log_path=tmp_path / "e.jsonl")

        @emit_skill_events
        def pick_object(backend: Any, object_id: str) -> dict:
            return {"ok": True, "command": "pick_object"}

        pick_object(None, "red_cube")
        shutdown_bus()

        lines = (tmp_path / "e.jsonl").read_text().strip().splitlines()
        payload = json.loads(lines[0])["payload"]
        assert payload["params"] == {}

    def test_decorator_skill_end_ok_false_on_dict_result(self, tmp_path: Path) -> None:
        configure_bus(log_path=tmp_path / "e.jsonl")

        @emit_skill_events
        def failing_skill(backend: Any) -> dict:
            return {"ok": False, "command": "failing_skill", "error": "attach failed"}

        failing_skill(None)
        shutdown_bus()

        lines = (tmp_path / "e.jsonl").read_text().strip().splitlines()
        end_event = json.loads(lines[1])
        assert end_event["event_type"] == "skill_end"
        assert end_event["payload"]["ok"] is False

    def test_decorator_propagates_exception(self, tmp_path: Path) -> None:
        configure_bus(log_path=tmp_path / "e.jsonl")

        @emit_skill_events
        def broken_skill(backend: Any) -> dict:
            raise RuntimeError("motion failed")

        with pytest.raises(RuntimeError, match="motion failed"):
            broken_skill(None)
        shutdown_bus()

        lines = (tmp_path / "e.jsonl").read_text().strip().splitlines()
        end_event = json.loads(lines[1])
        assert end_event["payload"]["ok"] is False
        assert "motion failed" in end_event["payload"]["error"]

    def test_decorator_records_duration(self, tmp_path: Path) -> None:
        configure_bus(log_path=tmp_path / "e.jsonl")

        @emit_skill_events
        def slow_skill(backend: Any) -> dict:
            time.sleep(0.01)
            return {"ok": True}

        slow_skill(None)
        shutdown_bus()

        lines = (tmp_path / "e.jsonl").read_text().strip().splitlines()
        end_event = json.loads(lines[1])
        # At least 10 ms (10_000_000 ns)
        assert end_event["payload"]["duration_ns"] >= 10_000_000

    def test_decorator_preserves_function_name(self) -> None:
        @emit_skill_events
        def my_named_skill(backend: Any) -> dict:
            return {"ok": True}

        assert my_named_skill.__name__ == "my_named_skill"

    def test_sanitize_params_drops_complex_objects(self) -> None:
        class FakeBackend:
            pass

        sanitized = _sanitize_params({
            "backend": FakeBackend(),
            "object_id": "red_cube",
            "grasp_width_m": 0.065,
            "count": 3,
            "flag": True,
        })
        assert "backend" not in sanitized
        assert sanitized["object_id"] == "red_cube"
        assert sanitized["grasp_width_m"] == pytest.approx(0.065)
        assert sanitized["count"] == 3
        assert sanitized["flag"] is True

    def test_sanitize_params_keeps_json_safe_dicts(self) -> None:
        sanitized = _sanitize_params({
            "pose": {"x": 0.1, "y": 0.2, "z": 0.3}
        })
        assert sanitized["pose"] == {"x": 0.1, "y": 0.2, "z": 0.3}
