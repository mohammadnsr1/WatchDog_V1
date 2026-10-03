"""inject_scenarios' pure core (observation-duty scenarios): start parsing,
view refusal, visit geometry, record schema, presets, and the no-Isaac-import
guarantee. The Isaac layer is byte-compiled only (it needs a live stage).
"""

from __future__ import annotations

import math
import py_compile
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(_REPO_ROOT / "isaac" / "script_editor"))

import inject_intruder as ii  # noqa: E402
import inject_scenarios as sc  # noqa: E402


def test_module_imports_without_isaac_and_byte_compiles():
    assert not any(m.split(".")[0] in ("omni", "pxr", "isaacsim")
                   for m in sys.modules), "Isaac modules loaded at import time"
    for name in ("inject_scenarios.py", "inject_intruder.py",
                 "spawn_random_intruders.py"):
        py_compile.compile(
            str(_REPO_ROOT / "isaac" / "script_editor" / name), doraise=True
        )


# --- start parsing ---------------------------------------------------------------

def test_parse_start_forms():
    assert sc.parse_start("now") == ("now", None)
    assert sc.parse_start("after_s=2.5") == ("after_s", 2.5)
    assert sc.parse_start("at_step=place_u_green") == ("at_step", "place_u_green")
    for bad in ("later", "after_s=", "after_s=x", "after_s=-1", "at_step="):
        with pytest.raises(ValueError):
            sc.parse_start(bad)


def test_start_trigger_text():
    assert sc.start_trigger_text("now") == "immediate"
    assert sc.start_trigger_text("after_s=3") == "delay:3"
    assert sc.start_trigger_text("at_step=s1") == "at_step:s1"


# --- view checks -----------------------------------------------------------------

def test_check_view_refuses_out_of_view_and_thin_margins(capsys):
    with pytest.raises(sc.ViewRefusal):
        sc.check_view((-0.75, -0.1, 0.16), "thing")  # outside the image
    assert "REFUSED" in capsys.readouterr().out

    # In view but under the 60 px default margin (margin ~36 px here).
    with pytest.raises(sc.ViewRefusal):
        sc.check_view((0.45, 0.05, 0.04), "thing")

    # Same point passes with a lower bar, or with the override.
    assert sc.check_view((0.45, 0.05, 0.04), "thing", min_edge_margin_px=30)
    fields = sc.check_view((-0.75, -0.1, 0.16), "thing", allow_out_of_view=True)
    assert fields["in_camera_view"] is False


def test_check_view_passes_a_good_position():
    fields = sc.check_view((0.45, -0.05, 0.04), "thing")
    assert fields["in_camera_view"] is True
    assert fields["edge_margin_px"] >= sc.MIN_EDGE_MARGIN_PX


# --- visit geometry ---------------------------------------------------------------

def test_visit_geometry_defaults():
    hand = sc.visit_geometry("hand", (0.45, -0.10))
    assert hand["dwell"][2] == pytest.approx(ii.BENCH_TOP_Z + sc.HAND_HEIGHT_ABOVE_BENCH)
    # Default entry: slides in from the x- side, STARTING out of view.
    assert hand["start"][0] < hand["dwell"][0]
    assert ii.camera_view_fields(hand["start"])["in_camera_view"] is False

    person = sc.visit_geometry("person", (0.10, -0.45))
    total = sc.PERSON_HEIGHT + 2 * sc.PERSON_RADIUS
    assert person["dwell"][2] == pytest.approx(ii.BENCH_TOP_Z + total / 2)

    with pytest.raises(ValueError):
        sc.visit_geometry("dog", (0, 0))


def test_visit_geometry_explicit_offset_and_z():
    g = sc.visit_geometry("hand", (0.4, 0.0, 0.3), entry_offset=(0.0, -0.9, 0.0))
    assert g["dwell"] == (0.4, 0.0, 0.3)
    assert g["start"] == (0.4, -0.9, 0.3)


# --- records -----------------------------------------------------------------------

def _record(event_kind, action, position=(0.45, -0.05, 0.04), **kw):
    return sc.scenario_record(
        scenario="x", event_kind=event_kind, episode_id="ep_abc",
        action=action, name="n", prim_path="/World/WatchdogIntruders/n",
        session_dir="/tmp/s", position_xyz_world=position, **kw,
    )


@pytest.mark.parametrize("event_kind,action", [
    ("foreign_object", "place"),
    ("transient_visitor", "enter_start"),
    ("transient_visitor", "in_view"),
    ("transient_visitor", "dwell_start"),
    ("transient_visitor", "leave_start"),
    ("transient_visitor", "out_of_view"),
    ("transient_visitor", "removed"),
    ("external_displacement", "nudge"),
    ("removal", "removed"),
])
def test_every_record_type_carries_the_visibility_triple(event_kind, action):
    record = _record(event_kind, action)
    assert record["pixel_uv"] is not None
    assert record["in_camera_view"] is True
    assert record["edge_margin_px"] > 0
    assert record["scenario"] == "x"
    assert record["event_kind"] == event_kind
    assert record["episode_id"] == "ep_abc"
    assert record["action"] == action
    assert record["wall_ns"] > 0
    assert len(record["scenarios_sha256"]) == 64


def test_record_rejects_unknown_event_kind():
    with pytest.raises(ValueError, match="event_kind"):
        _record("mystery", "place")


def test_record_extras_and_impact_vocabulary():
    record = _record("external_displacement", "nudge",
                     extra={"pre_pose_world": {"position": [0, 0, 0]}})
    assert record["pre_pose_world"] == {"position": [0, 0, 0]}
    assert sc.validate_scenario_impact("violation") == "violation"
    with pytest.raises(ValueError):
        sc.validate_scenario_impact("violates")  # scenarios use "violation"


# --- presets -----------------------------------------------------------------------

def test_presets_are_in_view_and_clear_of_base1():
    for key, preset in sc.PRESETS.items():
        position = preset["position"]
        if position is None:  # nudge targets an existing object
            continue
        x, y, z = position
        if z is None:
            z = ii.default_spawn_z(0.05)
        fields = ii.camera_view_fields((x, y, z))
        assert fields["in_camera_view"], (key, fields)
        assert fields["edge_margin_px"] >= sc.MIN_EDGE_MARGIN_PX, (key, fields)
        clearance = math.hypot(x - sc.BASE1_XY[0], y - sc.BASE1_XY[1])
        assert clearance >= sc.MIN_BASE1_CLEARANCE_M, (key, clearance)


def test_presets_prints_every_scenario_and_cancel_all(capsys):
    sc.presets()
    out = capsys.readouterr().out
    for needle in ("place_object(", "visit(", "nudge_object(", "cancel_all()",
                   "importlib.reload"):
        assert needle in out, needle


# --- the per-frame pose tracker (pure core) --------------------------------------


def test_tracker_adopts_new_prims_silently():
    state = {}
    assert sc.tracker_tick(state, {"a": (0.4, 0.0, 0.03)}, now=0.0) == []
    assert "a" in state


def test_tracker_records_motion_with_rate_limit_and_settle():
    state = {}
    sc.tracker_tick(state, {"a": (0.4, 0.0, 0.03)}, now=0.0)
    # 2 mm wiggle: below TRACK_MOVE_EPS_M, nothing recorded.
    assert sc.tracker_tick(state, {"a": (0.402, 0.0, 0.03)}, now=0.5) == []
    # 2 cm displacement: one "moved"...
    out = sc.tracker_tick(state, {"a": (0.42, 0.0, 0.03)}, now=1.5)
    assert out == [("a", "moved", (0.42, 0.0, 0.03))]
    # ...and further motion within the emit interval is rate-limited...
    assert sc.tracker_tick(state, {"a": (0.44, 0.0, 0.03)}, now=1.9) == []
    # ...then recorded again once the interval has passed.
    out = sc.tracker_tick(state, {"a": (0.46, 0.0, 0.03)}, now=2.6)
    assert out == [("a", "moved", (0.46, 0.0, 0.03))]
    # Still for TRACK_SETTLE_S after moving -> exactly one "settled".
    assert sc.tracker_tick(state, {"a": (0.46, 0.0, 0.03)}, now=3.0) == []
    out = sc.tracker_tick(state, {"a": (0.46, 0.0, 0.03)}, now=3.7)
    assert out == [("a", "settled", (0.46, 0.0, 0.03))]
    assert sc.tracker_tick(state, {"a": (0.46, 0.0, 0.03)}, now=5.0) == []


def test_tracker_vanish_and_forget():
    state = {}
    sc.tracker_tick(state, {"a": (0.4, 0.0, 0.03)}, now=0.0)
    out = sc.tracker_tick(state, {}, now=1.0)
    assert out == [("a", "vanished", (0.4, 0.0, 0.03))]
    assert state == {}
    # forget: a module removal must not produce a vanish afterwards.
    sc.tracker_tick(state, {"b": (0.4, 0.0, 0.03)}, now=2.0)
    sc.tracker_forget(state, "b")
    assert sc.tracker_tick(state, {}, now=3.0) == []


def test_tracked_move_is_a_known_event_kind():
    assert "tracked_move" in sc.EVENT_KINDS
    record = _record("tracked_move", "moved")
    assert record["event_kind"] == "tracked_move"
    assert record["in_camera_view"] is True
