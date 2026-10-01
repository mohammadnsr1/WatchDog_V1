"""arm_custom_monitor / dry_run_custom_monitor: purpose and rate_hz surface.

Server-level, against a fake context (monkeypatched _ensure_watchdog), so no
ROS node is needed. Covers: observation validation and provenance, observation
fires never becoming findings, the rate clamp, the dry run's rate-spaced
ticks, and that the rate_hz=None path still produces the pre-Stage-2 result
surface.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

import robot_backend.mcp_servers.watchdog_mcp_server as ws  # noqa: E402
from robot_backend.watchdog.engine import MonitorEngine  # noqa: E402
from robot_backend.watchdog.timebase import wall_ns  # noqa: E402


class _RecordingBus:
    def __init__(self) -> None:
        self.events = []

    def publish(self, event) -> None:
        self.events.append(event)

    def payloads(self, event_type):
        return [
            e.model_dump(mode="json")["payload"]
            for e in self.events
            if e.event_type == event_type
        ]


class _StubRecorder:
    def object_ids(self):
        return ["u_brown"]

    def frame_ids(self):
        return []

    def latest_object_pose(self, object_id):
        return {
            "wall_ns": 1000, "object_id": object_id,
            "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                     "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        }

    def latest_frame_pose(self, frame_id):
        return None

    def latest_joint_state(self):
        return None

    def lookup_transform(self, parent, child):
        raise RuntimeError("no TF in this fake")

    def get_joint_window(self, *a):
        return []

    def get_object_pose_window(self, *a):
        return []

    def get_frame_window(self, *a):
        return []


class _FakeStreams:
    def array_snapshot(self):
        now = wall_ns()
        depth = np.full((36, 64), 1.7, dtype=np.float32)
        depth.flags.writeable = False
        return {"taken_wall_ns": now, "rgb": None, "depth": (depth, now),
                "camera": None}


class _Ctx:
    def __init__(self, registry, session_dir):
        self.registry = registry
        self.recorder = _StubRecorder()
        self.engine = MonitorEngine(_RecordingBus())
        self.session_dir = Path(session_dir)
        self.camera_streams = _FakeStreams()

    @property
    def bus(self):
        return self.engine.event_bus


@pytest.fixture(scope="module")
def registry():
    path = Path(__file__).resolve().parents[1] / "config" / "model_registry.yaml"
    return yaml.safe_load(path.read_text())


@pytest.fixture
def ctx(registry, tmp_path, monkeypatch):
    c = _Ctx(registry, tmp_path)
    monkeypatch.setattr(ws, "_ensure_watchdog", lambda: c)
    return c


PASSIVE = "def check(ctx):\n    return None\n"


# --- purpose="observation" -------------------------------------------------------

def test_observation_requires_a_rationale(ctx):
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="obs", params={}, purpose="observation"
    )
    assert result["ok"] is False
    assert "threshold_rationale" in result["error"]
    assert ctx.engine.snapshot() == []


def test_observation_refuses_halt_on_fire(ctx):
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="obs", params={}, purpose="observation",
        threshold_rationale="a chosen look trigger", halt_on_fire=True,
    )
    assert result["ok"] is False
    assert "halt_on_fire" in result["error"]


def test_bad_purpose_is_rejected(ctx):
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="x", params={}, purpose="diagnostic"
    )
    assert result["ok"] is False
    assert "purpose" in result["error"]


def test_observation_arm_and_fire_carry_purpose_and_agent_chosen(ctx):
    findings_before = list(ws._observed_findings())
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="obs", params={
            "trigger": {"value": 0.05, "source": None},
        },
        threshold_param="trigger",
        purpose="observation",
        threshold_rationale="look when the scene shifts by ~5 cm",
        object_id="u_brown",
        monitor_id="obs1",
    )
    assert result["ok"] is True, result
    assert result["purpose"] == "observation"
    assert result["threshold_rationale"].startswith("look when")
    assert result["threshold_source"] == "agent_chosen"
    assert result["threshold_verified"] is False
    armed = ctx.bus.payloads("monitor_armed")[0]
    assert armed["purpose"] == "observation"
    assert armed["threshold_source"] == "agent_chosen"
    assert armed["threshold_rationale"].startswith("look when")

    # A fire wakes the wait and carries purpose, but is NOT a finding.
    monitor = ctx.engine.monitors["obs1"]
    ctx.engine._handle_fire(monitor, 0.07)
    fire = ctx.bus.payloads("monitor_fire")[0]
    assert fire["purpose"] == "observation"
    assert fire["threshold_source"] == "agent_chosen"
    assert fire["threshold_rationale"].startswith("look when")
    assert ctx.engine.wake_event.is_set()
    # Finding bookkeeping keys on record_verdict calls, never on fires.
    assert list(ws._observed_findings()) == findings_before


def test_clause_purpose_keeps_registry_provenance(ctx, registry):
    value = registry["objects"]["u_brown"]["spec"]["tolerances"][
        "disturbance_tolerance_m"
    ]
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="clause", params={
            "eps": {"value": value,
                    "source": "raw_spec.tolerances.disturbance_tolerance_m"},
        },
        threshold_param="eps", object_id="u_brown",
    )
    assert result["ok"] is True
    assert result["purpose"] == "clause"
    assert result["threshold_source"] == "raw_spec.tolerances.disturbance_tolerance_m"
    assert result["threshold_verified"] is True


# --- rate_hz ----------------------------------------------------------------------

def test_rate_hz_is_clamped_and_reported(ctx):
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="array", params={}, rate_hz=10.0,
        monitor_id="arr1",
    )
    assert result["ok"] is True
    assert result["lane"] == "array"
    assert result["rate_hz"] == 2.0
    assert result["rate_hz_clamped"] is True
    assert result["rate_hz_requested"] == 10.0
    assert ctx.engine.monitors["arr1"].feed == "authored_array"


def test_rate_hz_zero_is_an_error(ctx):
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="array", params={}, rate_hz=0.0
    )
    assert result["ok"] is False
    assert "rate_hz" in result["error"]


def test_rate_hz_none_keeps_the_scalar_surface(ctx):
    """The pre-Stage-2 result keys are all present with unchanged meanings."""
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="scalar", params={}, monitor_id="sc1"
    )
    assert result["ok"] is True
    for key in ("monitor_id", "monitor_type", "object_id", "threshold",
                "comparison", "reference_values", "threshold_source",
                "threshold_verified", "first_call_result", "channels_read",
                "code_sha256", "saved_path", "origin", "description"):
        assert key in result, key
    assert "rate_hz" not in result and "lane" not in result
    assert ctx.engine.monitors["sc1"].feed == "authored"


# --- dry run with rate_hz ----------------------------------------------------------

def test_dry_run_with_rate_spaces_ticks_and_arms_nothing(ctx):
    started = time.monotonic()
    result = ws.dry_run_custom_monitor(
        code=PASSIVE, params={}, ticks=2, rate_hz=2.0
    )
    elapsed = time.monotonic() - started
    assert result["ok"] is True
    assert result["armed"] is False
    assert result["lane"] == "array"
    assert result["tick_period_ms"] == 500.0
    assert result["wall_budget_ms"] == ws.ARRAY_WALL_BUDGET_MS
    assert result["ticks_run"] == 2
    assert elapsed >= 1.0  # two ticks, 500 ms apart
    assert ctx.engine.snapshot() == []
    assert ctx.bus.events == []


FIRES_ALWAYS = "def check(ctx):\n    return {'value': 1.0, 'detail': 'always'}\n"


def test_dry_run_counts_fires_and_warns_on_every_tick(ctx):
    result = ws.dry_run_custom_monitor(code=FIRES_ALWAYS, params={}, ticks=2)
    assert result["ok"] is True
    assert result["fires"] == "2/2"
    assert "fires on every tick" in result["warning"]
    assert "Returning a dict IS a fire" in result["warning"]


def test_dry_run_no_fires_has_no_warning(ctx):
    result = ws.dry_run_custom_monitor(code=PASSIVE, params={}, ticks=2)
    assert result["fires"] == "0/2"
    assert "warning" not in result


def test_dry_run_clause_with_rate_notes_the_purpose_default(ctx):
    result = ws.dry_run_custom_monitor(
        code=PASSIVE, params={}, ticks=1, rate_hz=2.0
    )
    assert 'purpose="observation"' in result["note_purpose"]
    obs = ws.dry_run_custom_monitor(
        code=PASSIVE, params={}, ticks=1, rate_hz=2.0, purpose="observation"
    )
    assert "note_purpose" not in obs


def test_arm_warns_when_the_first_call_already_fired(ctx):
    result = ws.arm_custom_monitor(
        code=FIRES_ALWAYS, description="always", params={}, monitor_id="warn1"
    )
    assert result["ok"] is True                      # armed anyway
    assert "fires on every tick" in result["warning"]
    assert "warn1" in ctx.engine.monitors            # not refused


def test_arm_without_a_first_call_dict_has_no_warning(ctx):
    result = ws.arm_custom_monitor(
        code=PASSIVE, description="quiet", params={}, monitor_id="quiet1"
    )
    assert result["ok"] is True
    assert "warning" not in result


def test_dry_run_without_rate_keeps_the_50ms_ticks(ctx):
    result = ws.dry_run_custom_monitor(code=PASSIVE, params={}, ticks=1)
    assert result["ok"] is True
    assert result["tick_period_ms"] == 50.0
    assert result["wall_budget_ms"] == 20.0
    assert "lane" not in result
