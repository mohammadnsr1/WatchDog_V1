"""threshold_source must name the registry ONLY when the value came from it.

Why this field exists. Monitors are armed from AGENT-SUPPLIED params -- the
server never resolves a threshold from the registry itself -- so "the watchdog
read the registry" is a claim about one particular arm call, not a property of
the code path. It has to be measured, by comparing the passed value against
what the registry holds for that object and field at arm time.

Why it has to be honest. Every FMB tolerance is currently a uniform 0.02, so a
watchdog that read the registry and one that hard-coded 0.02 arm IDENTICAL
monitors and behave identically for the whole run. threshold_source is the
only thing that distinguishes them. A source string that named a registry path
for a value that did not come from there would be worse than no field at all:
it reads as valid, so nothing downstream would question it, and Class 4
(corrupt a registry value, prove the watchdog armed from the corrupted one)
would report success for a watchdog that ignored the registry entirely.

So the tests here come in pairs: for each monitor family, one arm with the
registry value and one with a deliberately wrong value, asserting the two are
distinguishable and that the wrong one never claims a registry path.

Same import guard as the other watchdog tests.
"""

from __future__ import annotations

import os

import pytest
import yaml

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

import robot_backend.mcp_servers.watchdog_mcp_server as ws  # noqa: E402
from robot_backend.watchdog.engine import MonitorEngine  # noqa: E402


def _registry_path():
    from pathlib import Path

    return Path(__file__).resolve().parents[1] / "config" / "model_registry.yaml"


class _RecordingBus:
    def __init__(self) -> None:
        self.events = []

    def publish(self, event) -> None:
        self.events.append(event)

    def payloads(self, event_type: str):
        return [
            e.model_dump(mode="json")["payload"]
            for e in self.events
            if e.event_type == event_type
        ]


class _StubRecorder:
    """Enough recorder for the two families that capture an arm reference."""

    def lookup_transform(self, parent, child):
        return {"translation": {"x": 0.0, "y": 0.0, "z": 0.09}}

    def latest_object_pose(self, object_id):
        return {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}}


class _Ctx:
    def __init__(self, registry, held=None):
        self.registry = registry
        self.recorder = _StubRecorder()
        self.engine = MonitorEngine(_RecordingBus())
        self._held = held

    @property
    def bus(self):
        return self.engine.event_bus

    def held_object_id(self):
        return self._held


@pytest.fixture(scope="module")
def registry():
    return yaml.safe_load(_registry_path().read_text())


@pytest.fixture
def ctx(registry):
    c = _Ctx(registry)
    # effort_event has no intrinsic object; the engine's held-part tracking is
    # what supplies one. Patch it rather than replaying grasp events.
    c.engine.held_object_id = lambda: c._held
    return c


def _arm(ctx, monitor_type, params):
    """Arm through the real server path, exactly as the tool does."""
    monitor = ws._build_monitor(ctx, monitor_type, params, None, False, None)
    object_id = ws._resolve_arm_object_id(ctx, monitor)
    monitor.provenance = {
        "object_id": object_id,
        **ws._resolve_threshold_provenance(ctx, monitor, object_id),
    }
    return monitor, ctx.engine.arm(monitor)


def _spec(registry, object_id, dotted):
    node = registry["objects"][object_id]["spec"]
    for part in dotted.split("."):
        node = node[part]
    return node


# --- the registry-backed families, each as a matched honest/dishonest pair ---

def test_relative_frame_registry_value_names_the_registry_path(ctx, registry):
    value = _spec(registry, "u_brown", "tolerances.grasp_slip_tolerance_m")
    _, result = _arm(
        ctx,
        "relative_frame",
        {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": value},
    )
    assert result["threshold_source"] == "raw_spec.tolerances.grasp_slip_tolerance_m"
    assert result["threshold_verified"] is True
    assert result["threshold_registry_value"] == value
    assert result["object_id"] == "u_brown"


def test_relative_frame_wrong_value_does_not_claim_the_registry(ctx, registry):
    value = _spec(registry, "u_brown", "tolerances.grasp_slip_tolerance_m")
    _, result = _arm(
        ctx,
        "relative_frame",
        {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": value * 2.5},
    )
    assert result["threshold_source"] == "agent_literal"
    assert result["threshold_verified"] is False
    assert "raw_spec" not in result["threshold_source"]
    # The field that WAS checked is still reported, so the mismatch is auditable.
    assert result["threshold_registry_field"] == "raw_spec.tolerances.grasp_slip_tolerance_m"
    assert result["threshold_registry_value"] == value


def test_stationarity_registry_value_names_the_registry_path(ctx, registry):
    value = _spec(registry, "u_orange", "tolerances.disturbance_tolerance_m")
    _, result = _arm(ctx, "stationarity", {"object_id": "u_orange", "epsilon_m": value})
    assert result["threshold_source"] == "raw_spec.tolerances.disturbance_tolerance_m"
    assert result["threshold_verified"] is True
    assert result["object_id"] == "u_orange"


def test_stationarity_wrong_value_does_not_claim_the_registry(ctx, registry):
    _, result = _arm(ctx, "stationarity", {"object_id": "u_orange", "epsilon_m": 0.0777})
    assert result["threshold_source"] == "agent_literal"
    assert result["threshold_verified"] is False


def test_effort_event_registry_value_names_the_registry_path(ctx, registry):
    ctx._held = "u_brown"
    value = _spec(registry, "u_brown", "limits.max_clamp_effort")
    _, result = _arm(
        ctx,
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": value},
    )
    assert result["threshold_source"] == "raw_spec.limits.max_clamp_effort"
    assert result["threshold_verified"] is True
    # Resolved from the held part, since effort_event is joint-scoped.
    assert result["object_id"] == "u_brown"


def test_effort_event_wrong_value_does_not_claim_the_registry(ctx, registry):
    ctx._held = "u_brown"
    _, result = _arm(
        ctx,
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": 999.0},
    )
    assert result["threshold_source"] == "agent_literal"
    assert result["threshold_verified"] is False


def test_effort_event_on_an_empty_gripper_says_object_unresolved(ctx):
    """No held part means no per-object field to check -- not a mismatch."""
    ctx._held = None
    _, result = _arm(
        ctx,
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": 100.0},
    )
    assert result["threshold_source"] == "object_unresolved"
    assert result["threshold_verified"] is False
    assert result["object_id"] is None


def test_region_is_structural_not_a_failed_registry_lookup(ctx):
    """A region has no registry-backed threshold; that is an answer, not a gap."""
    _, result = _arm(
        ctx,
        "region",
        {
            "object_id": "u_brown",
            "mode": "stay_inside",
            "center": {"x": 0.0, "y": 0.0, "z": 0.0},
            "half_extents": {"x": 1.0, "y": 1.0, "z": 1.0},
        },
    )
    assert result["threshold_source"] == "structural"
    assert result["threshold_registry_field"] is None


def test_another_objects_value_is_not_accepted_for_this_object(ctx, registry):
    """The check is per-object, so a value valid elsewhere must still fail here.

    max_clamp_effort is NOT uniform across parts (u_brown 100, u_green 500),
    which makes this the one provenance case that is falsifiable by value
    alone rather than only by the source string.
    """
    ctx._held = "u_brown"
    other = _spec(registry, "u_green", "limits.max_clamp_effort")
    mine = _spec(registry, "u_brown", "limits.max_clamp_effort")
    assert other != mine, "test assumes these differ; if they were unified, pick another pair"
    _, result = _arm(
        ctx,
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": other},
    )
    assert result["threshold_source"] == "agent_literal"
    assert result["threshold_registry_value"] == mine


def test_a_corrupted_registry_value_is_what_gets_verified_against(ctx, registry):
    """Class 4: the watchdog must arm from the registry AS IT STANDS.

    The claim provenance certifies is "armed from the registry", not "armed
    from a correct value". So when the registry is corrupted, a monitor armed
    with the corrupted value must verify TRUE, and one armed with the original
    (uncorrupted) value must not -- that inversion is the Class 4 signal.
    """
    corrupted = 0.0731
    registry["objects"]["u_brown"]["spec"]["tolerances"]["grasp_slip_tolerance_m"] = corrupted
    try:
        _, armed_from_corrupt = _arm(
            ctx,
            "relative_frame",
            {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": corrupted},
        )
        _, armed_from_original = _arm(
            ctx,
            "relative_frame",
            {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": 0.02},
        )
    finally:
        registry["objects"]["u_brown"]["spec"]["tolerances"]["grasp_slip_tolerance_m"] = 0.02

    assert armed_from_corrupt["threshold_verified"] is True
    assert armed_from_corrupt["threshold_source"] == "raw_spec.tolerances.grasp_slip_tolerance_m"
    assert armed_from_original["threshold_verified"] is False
    assert armed_from_original["threshold_source"] == "agent_literal"


# --- the fields land on the EVENTS, which is what the scorer actually reads ---

def test_arm_event_carries_object_threshold_and_provenance(ctx, registry):
    value = _spec(registry, "u_brown", "tolerances.grasp_slip_tolerance_m")
    _arm(
        ctx,
        "relative_frame",
        {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": value},
    )
    payload = ctx.bus.payloads("monitor_armed")[0]
    for field in (
        "monitor_id",
        "monitor_type",
        "object_id",
        "threshold",
        "comparison",
        "threshold_source",
        "threshold_verified",
        "threshold_registry_field",
        "threshold_registry_value",
        "reference_values",
    ):
        assert field in payload, f"{field} missing from monitor_armed"
    assert payload["object_id"] == "u_brown"
    assert payload["threshold"] == value
    assert payload["threshold_source"] == "raw_spec.tolerances.grasp_slip_tolerance_m"
    # The arm reference is on the event, not only in the tool result.
    assert payload["reference_values"]["translation"]


def test_fire_restates_the_arm_time_provenance(ctx, registry):
    value = _spec(registry, "u_brown", "tolerances.grasp_slip_tolerance_m")
    monitor, _ = _arm(
        ctx,
        "relative_frame",
        {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": value},
    )
    ctx.engine._handle_fire(monitor, value * 1.6)

    fire = ctx.bus.payloads("monitor_fire")[0]
    arm = ctx.bus.payloads("monitor_armed")[0]
    assert fire["threshold_source"] == arm["threshold_source"]
    assert fire["threshold_verified"] == arm["threshold_verified"]
    assert fire["threshold_registry_value"] == arm["threshold_registry_value"]
    # monitor_id joins fire back to arm.
    assert fire["monitor_id"] == arm["monitor_id"]


def test_fire_of_an_unverified_monitor_stays_unverified(ctx):
    """The dishonesty guard has to hold on fires too, not just arms."""
    monitor, _ = _arm(
        ctx,
        "relative_frame",
        {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": 0.05},
    )
    ctx.engine._handle_fire(monitor, 0.09)
    fire = ctx.bus.payloads("monitor_fire")[0]
    assert fire["threshold_source"] == "agent_literal"
    assert fire["threshold_verified"] is False


def test_monitor_id_joins_arm_fire_and_disarm(ctx, registry):
    value = _spec(registry, "u_orange", "tolerances.disturbance_tolerance_m")
    monitor, _ = _arm(ctx, "stationarity", {"object_id": "u_orange", "epsilon_m": value})
    ctx.engine._handle_fire(monitor, value * 3)

    arm = ctx.bus.payloads("monitor_armed")[0]
    fire = ctx.bus.payloads("monitor_fire")[0]
    disarm = ctx.bus.payloads("monitor_disarmed")[0]
    assert arm["monitor_id"] == fire["monitor_id"] == disarm["monitor_id"]
    assert disarm["reason"] == "fired"


def test_every_monitor_event_carries_wall_time_iso(ctx, registry):
    """wall_time_iso is the cross-process correlation key; nothing may lack it."""
    value = _spec(registry, "u_orange", "tolerances.disturbance_tolerance_m")
    monitor, _ = _arm(ctx, "stationarity", {"object_id": "u_orange", "epsilon_m": value})
    ctx.engine._handle_fire(monitor, value * 3)

    for event in ctx.bus.events:
        dumped = event.model_dump(mode="json")
        assert dumped.get("wall_time_iso"), f"{dumped['event_type']} has no wall_time_iso"


def test_threshold_is_json_safe_for_every_family(ctx):
    """NaN is not valid strict JSON; a family without a scalar threshold sends null."""
    import json

    _arm(
        ctx,
        "region",
        {
            "object_id": "u_brown",
            "mode": "stay_inside",
            "center": {"x": 0.0, "y": 0.0, "z": 0.0},
            "half_extents": {"x": 1.0, "y": 1.0, "z": 1.0},
        },
    )
    for payload in ctx.bus.payloads("monitor_armed"):
        text = json.dumps(payload, allow_nan=False)  # raises on NaN/Infinity
        assert "NaN" not in text


# --- declared object_id on arm_monitor (PREBATCH_FIXES item 8) -----------------
#
# These go through the REAL tool (ws.arm_monitor, _ensure_watchdog patched)
# because the feature is the tool's: a declared object_id that resolves
# threshold provenance for a primitive that cannot resolve an object itself,
# without touching behaviour, fire attribution, or held-object logic.

@pytest.fixture
def tool_ctx(ctx, monkeypatch):
    monkeypatch.setattr(ws, "_ensure_watchdog", lambda: ctx)
    return ctx


def test_pre_grasp_effort_event_verifies_against_declared_object(tool_ctx, registry):
    tool_ctx._held = None
    value = _spec(registry, "u_brown", "limits.max_clamp_effort")
    result = ws.arm_monitor(
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": value},
        object_id="u_brown",
    )
    assert result["ok"] is True, result
    assert result["object_id"] == "u_brown"
    assert result["provenance_object_source"] == "declared"
    assert result["threshold_source"] == "raw_spec.limits.max_clamp_effort"
    assert result["threshold_verified"] is True
    armed = tool_ctx.bus.payloads("monitor_armed")[-1]
    assert armed["provenance_object_source"] == "declared"
    assert armed["threshold_verified"] is True


def test_without_declared_object_the_result_is_unchanged_from_today(tool_ctx, registry):
    tool_ctx._held = None
    value = _spec(registry, "u_brown", "limits.max_clamp_effort")
    result = ws.arm_monitor(
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": value},
    )
    assert result["ok"] is True, result
    assert result["object_id"] is None
    assert result["provenance_object_source"] is None
    assert result["threshold_source"] == "object_unresolved"
    assert result["threshold_verified"] is False


def test_unregistered_declared_object_is_rejected(tool_ctx):
    result = ws.arm_monitor(
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": 1.0},
        object_id="no_such_part",
    )
    assert result["ok"] is False
    assert "not in the registry" in result["error"]
    assert tool_ctx.engine.snapshot() == []  # nothing armed


def test_declared_object_loses_to_the_held_part(tool_ctx, registry):
    tool_ctx._held = "u_orange"
    value = _spec(registry, "u_orange", "limits.max_clamp_effort")
    result = ws.arm_monitor(
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": value},
        object_id="u_brown",
    )
    assert result["ok"] is True, result
    assert result["object_id"] == "u_orange"
    assert result["provenance_object_source"] == "held"
    assert result["object_id_conflict"]["declared"] == "u_brown"
    assert result["object_id_conflict"]["used"] == "u_orange"


def test_declared_object_loses_to_the_primitive_subject(tool_ctx, registry):
    value = _spec(registry, "u_brown", "tolerances.grasp_slip_tolerance_m")
    result = ws.arm_monitor(
        "relative_frame",
        {"parent_frame": "tcp", "child_frame": "u_brown", "max_delta_m": value},
        object_id="u_orange",
    )
    assert result["ok"] is True, result
    assert result["object_id"] == "u_brown"
    assert result["provenance_object_source"] == "params"
    assert result["object_id_conflict"]["declared"] == "u_orange"


def test_fire_payload_carries_the_object_source(tool_ctx, registry):
    tool_ctx._held = None
    value = _spec(registry, "u_brown", "limits.max_clamp_effort")
    result = ws.arm_monitor(
        "effort_event",
        {"joint_name": "finger_joint", "comparison": "le", "threshold": value},
        object_id="u_brown",
        monitor_id="prov_fire",
    )
    assert result["ok"] is True, result
    monitor = tool_ctx.engine.monitors["prov_fire"]
    tool_ctx.engine._handle_fire(monitor, value + 1.0)
    fire = tool_ctx.bus.payloads("monitor_fire")[-1]
    assert fire["provenance_object_source"] == "declared"
    assert fire["threshold_verified"] is True
