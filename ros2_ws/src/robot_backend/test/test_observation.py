"""record_observation: the shared core, the event schema wiring, and the
observation-threshold arm requirement (Stage 3).

Core-level, no ROS node: ObservationDeps takes a recording publish callable
and an injected look-id list, exactly as the unit-test seam was designed.
"""

from __future__ import annotations

import os

import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

from robot_backend.events.schema import (  # noqa: E402
    EVENT_TYPE_PAYLOAD_CLASSES,
    KNOWN_EVENT_TYPES,
    ObservationPayload,
    available_event_type_definitions,
    event_type_origin,
)
from robot_backend.vision.observation import (  # noqa: E402
    ObservationDeps,
    ObservationStore,
    known_look_ids,
    record_observation,
    register_look,
    reset_look_registry,
)
from robot_backend.watchdog.recorder import (  # noqa: E402
    _WAKE_OPT_IN_EVENT_TYPES,
    _is_self_emitted,
)


class _RecordingPublisher:
    def __init__(self) -> None:
        self.events = []

    def __call__(self, event) -> None:
        self.events.append(event)


def _deps(looks=("0001-aaaa", "0002-bbbb"), publisher=None, step_id="step_x"):
    return ObservationDeps(
        role="watchdog",
        store=ObservationStore(),
        publish_event=publisher,
        current_step_id=(lambda: step_id),
        look_ids=(lambda: list(looks)),
    )


def _valid_kwargs(**overrides):
    kwargs = dict(
        summary="an unlisted object sits on the table",
        category="unlisted_object",
        location={"frame": "world", "x": 0.4, "y": -0.2, "z": 0.78},
        first_seen_wall_ns=123456789,
        evidence_look_ids=["0001-aaaa"],
    )
    kwargs.update(overrides)
    return kwargs


# --- schema wiring -----------------------------------------------------------


def test_observation_is_a_known_event_type():
    assert "observation" in KNOWN_EVENT_TYPES
    assert EVENT_TYPE_PAYLOAD_CLASSES["observation"] is ObservationPayload
    assert event_type_origin("observation") == "watchdog"


def test_observation_is_in_available_event_types_with_a_meaning():
    definitions = {d["event_type"]: d for d in available_event_type_definitions()}
    entry = definitions["observation"]
    assert entry["description"]
    assert "observation" in entry["description"].lower()
    assert "role" in entry["payload_fields"]


def test_observation_wake_rules_follow_look():
    assert "observation" in _WAKE_OPT_IN_EVENT_TYPES

    class _Envelope:
        event_type = "observation"
        payload = {"role": "watchdog"}

    assert _is_self_emitted(_Envelope()) is True
    _Envelope.payload = {"role": "executor"}
    assert _is_self_emitted(_Envelope()) is False


# --- the happy path ----------------------------------------------------------


def test_new_observation_publishes_event_and_returns_id():
    publisher = _RecordingPublisher()
    deps = _deps(publisher=publisher)
    result = record_observation(deps, **_valid_kwargs())
    assert result["ok"] is True, result
    assert result["observation_id"].startswith("obs-")
    assert result["unsupported_by_looks"] is False

    assert len(publisher.events) == 1
    event = publisher.events[0]
    assert event.event_type == "observation"
    payload = event.payload
    assert payload["observation_id"] == result["observation_id"]
    assert payload["role"] == "watchdog"
    assert payload["step_id"] == "step_x"
    assert payload["wall_ns"] is not None
    assert payload["first_seen_wall_ns"] == 123456789
    assert payload["evidence_look_ids"] == ["0001-aaaa"]
    assert payload["category"] == "unlisted_object"
    assert payload["status"] == "new"


def test_empty_evidence_is_allowed_but_flagged_unsupported():
    result = record_observation(_deps(), **_valid_kwargs(evidence_look_ids=[]))
    assert result["ok"] is True
    assert result["unsupported_by_looks"] is True
    assert any("unsupported" in n for n in result["notes"])


def test_pixel_location_form_with_valid_look():
    result = record_observation(_deps(), **_valid_kwargs(
        location={"pixel": {"u": 320, "v": 180}, "look_id": "0002-bbbb"},
    ))
    assert result["ok"] is True
    location = result["event"]["payload"]["location"]
    assert location == {"pixel": {"u": 320.0, "v": 180.0}, "look_id": "0002-bbbb"}


def test_update_and_resolve_lifecycle():
    deps = _deps()
    created = record_observation(deps, **_valid_kwargs())
    obs_id = created["observation_id"]

    updated = record_observation(deps, **_valid_kwargs(
        summary="still there; now identified",
        identity="a mug",
        observation_id=obs_id,
        status="update",
    ))
    assert updated["ok"] is True, updated
    assert updated["observation_id"] == obs_id
    assert updated["previous_status"] == "new"

    resolved = record_observation(deps, **_valid_kwargs(
        summary="it was removed from the cell",
        observation_id=obs_id,
        status="resolved",
    ))
    assert resolved["ok"] is True
    assert resolved["previous_status"] == "update"


# --- validation --------------------------------------------------------------


def test_unknown_evidence_look_ids_error_lists_valid_ones():
    result = record_observation(_deps(), **_valid_kwargs(
        evidence_look_ids=["nope-1234"],
    ))
    assert result["ok"] is False
    assert "nope-1234" in result["error"]
    assert "0001-aaaa" in result["error"]


def test_bad_category_confidence_status_impact():
    for overrides, needle in (
        ({"category": "weird"}, "category"),
        ({"confidence": "sure"}, "confidence"),
        ({"status": "open"}, "status"),
        ({"requirement_impact": "maybe"}, "requirement_impact"),
    ):
        result = record_observation(_deps(), **_valid_kwargs(**overrides))
        assert result["ok"] is False, overrides
        assert needle in result["error"]


def test_violates_requires_impact_detail():
    result = record_observation(_deps(), **_valid_kwargs(
        requirement_impact="violates",
    ))
    assert result["ok"] is False
    assert "impact_detail" in result["error"]

    result = record_observation(_deps(), **_valid_kwargs(
        requirement_impact="violates",
        impact_detail="blocks the required clear approach volume",
    ))
    assert result["ok"] is True


def test_location_required_unless_object_missing():
    result = record_observation(_deps(), **_valid_kwargs(location=None))
    assert result["ok"] is False
    assert "location" in result["error"]

    result = record_observation(_deps(), **_valid_kwargs(
        category="object_missing", location=None,
    ))
    assert result["ok"] is True


def test_world_location_form_is_validated():
    for bad in (
        {"frame": "camera", "x": 0, "y": 0, "z": 0},
        {"frame": "world", "x": 0, "y": 0},
        {"frame": "world", "x": "wide", "y": 0, "z": 0},
        {"something": "else"},
    ):
        result = record_observation(_deps(), **_valid_kwargs(location=bad))
        assert result["ok"] is False, bad


def test_pixel_location_requires_known_look():
    result = record_observation(_deps(), **_valid_kwargs(
        location={"pixel": {"u": 1, "v": 2}, "look_id": "unknown-look"},
    ))
    assert result["ok"] is False
    assert "unknown-look" in result["error"]

    result = record_observation(_deps(), **_valid_kwargs(
        location={"pixel": {"u": 1, "v": 2}},
    ))
    assert result["ok"] is False
    assert "look_id" in result["error"]


def test_new_refuses_a_passed_observation_id():
    result = record_observation(_deps(), **_valid_kwargs(
        observation_id="obs-001-deadbeef",
    ))
    assert result["ok"] is False
    assert "server-generated" in result["error"]


def test_update_requires_an_existing_id():
    deps = _deps()
    result = record_observation(deps, **_valid_kwargs(status="update"))
    assert result["ok"] is False

    result = record_observation(deps, **_valid_kwargs(
        status="update", observation_id="obs-999-00000000",
    ))
    assert result["ok"] is False
    assert "does not exist" in result["error"]


def test_publish_failure_is_noted_not_fatal():
    def boom(_event):
        raise RuntimeError("bus down")

    deps = _deps(publisher=boom)
    result = record_observation(deps, **_valid_kwargs())
    assert result["ok"] is True
    assert any("not published" in n for n in result["notes"])


# --- the look registry and look() integration --------------------------------


def test_look_registry_roundtrip():
    reset_look_registry()
    try:
        assert known_look_ids("watchdog") == []
        register_look("watchdog", "0001-abcd")
        register_look("watchdog", "0001-abcd")  # idempotent
        register_look("executor", "0001-ffff")
        assert known_look_ids("watchdog") == ["0001-abcd"]
        assert known_look_ids("executor") == ["0001-ffff"]
    finally:
        reset_look_registry()


def test_look_tool_registers_its_look_id(tmp_path):
    """A look that runs (even with every view failing) becomes citable."""
    from robot_backend.vision.look import LookDeps, look

    class _DeadStreams:
        stale_sec = 1.0

        def ages(self):
            return {}

        def stats(self):
            raise RuntimeError("no stats")

        def get_camera(self):
            return None

        def get_rgb(self):
            return None

        def never_received_text(self, _name):
            return "never received"

    reset_look_registry()
    try:
        deps = LookDeps(
            streams=_DeadStreams(), session_dir=tmp_path, role="watchdog",
            file_prefix="look_",
        )
        look(deps, reason="test", views=["rgb"])
        ids = known_look_ids("watchdog")
        assert len(ids) == 1

        deps_obs = ObservationDeps(role="watchdog", store=ObservationStore())
        result = record_observation(deps_obs, **_valid_kwargs(
            evidence_look_ids=[ids[0]],
        ))
        assert result["ok"] is True
    finally:
        reset_look_registry()


# --- item 4a: observation arms require threshold_param ------------------------


def test_observation_arm_requires_threshold_param():
    import robot_backend.mcp_servers.watchdog_mcp_server as ws

    monitor, err = ws._prepare_authored_monitor(
        "def check(ctx):\n    return None\n",
        "obs",
        {"change_m": {"value": 0.02, "source": None}},
        purpose="observation",
        threshold_rationale="look on change",
    )
    assert monitor is None
    assert err is not None
    assert "threshold_param" in str(err["exc"])
    assert err["stage"] == "params"


def test_observation_dry_run_path_is_exempt(monkeypatch):
    """for_arm=False skips the requirement -- it reaches construction instead.

    _ensure_watchdog is stubbed to raise, which proves the arm-path check
    returned BEFORE touching the context while the dry-run path got past it
    (and then failed on the stub, not on threshold_param).
    """
    import robot_backend.mcp_servers.watchdog_mcp_server as ws

    def _no_ctx():
        raise RuntimeError("no watchdog context in this test")

    monkeypatch.setattr(ws, "_ensure_watchdog", _no_ctx)

    _monitor, err = ws._prepare_authored_monitor(
        "def check(ctx):\n    return None\n",
        "obs",
        {"change_m": {"value": 0.02, "source": None}},
        purpose="observation",
        threshold_rationale="look on change",
    )
    assert err is not None and "threshold_param" in str(err["exc"])

    with pytest.raises(RuntimeError, match="no watchdog context"):
        ws._prepare_authored_monitor(
            "def check(ctx):\n    return None\n",
            "obs",
            {"change_m": {"value": 0.02, "source": None}},
            purpose="observation",
            threshold_rationale="look on change",
            for_arm=False,
        )
