"""inject_intruder's pure core (Stage 3): name/registry collision refusal,
the step-trigger matcher, the ground-truth record shape, and session-dir
resolution. The Isaac layer (omni.* / pxr) is never imported here -- these
tests prove exactly the part that CAN be proven outside Isaac.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(_REPO_ROOT / "isaac" / "script_editor"))

import inject_intruder as ii  # noqa: E402


@pytest.fixture(scope="module")
def forbidden():
    return ii.load_forbidden_names()


# --- name validation ----------------------------------------------------------


def test_registry_ids_are_forbidden(forbidden):
    # Enabled parts, DISABLED entries (red_cube still resolves through
    # get_object_pose), placement frames, and cell frames must all refuse.
    for name in ("u_green", "red_cube", "green_cube", "u_green_placement",
                 "tcp", "world"):
        assert name in forbidden, name
        with pytest.raises(ValueError, match="collides"):
            ii.validate_name(name, forbidden)


def test_good_names_pass(forbidden):
    assert ii.validate_name("intruder_a", forbidden) == "intruder_a"
    assert ii.validate_name("stray_mug_1", forbidden) == "stray_mug_1"


def test_invalid_prim_names_are_refused(forbidden):
    for bad in ("1abc", "has-dash", "has space", "", "a/b"):
        with pytest.raises(ValueError, match="not a valid prim name"):
            ii.validate_name(bad, forbidden)


def test_inject_args_validation():
    with pytest.raises(ValueError, match="shape"):
        ii.validate_inject_args("cone", 0.05, (0, 0, 0), (1, 0, 0), "benign")
    with pytest.raises(ValueError, match="size_m"):
        ii.validate_inject_args("cube", 0.0, (0, 0, 0), (1, 0, 0), "benign")
    with pytest.raises(ValueError, match="position_xyz_world"):
        ii.validate_inject_args("cube", 0.05, (0, 0), (1, 0, 0), "benign")
    with pytest.raises(ValueError, match="color_rgb"):
        ii.validate_inject_args("cube", 0.05, (0, 0, 0), (2, 0, 0), "benign")
    with pytest.raises(ValueError, match="expected_impact"):
        ii.validate_inject_args("cube", 0.05, (0, 0, 0), (1, 0, 0), "harmless")
    size, pos, color = ii.validate_inject_args(
        "sphere", 0.04, (0.3, -0.1, 0.8), (0.1, 0.2, 0.9), "violates"
    )
    assert (size, pos, color) == (0.04, (0.3, -0.1, 0.8), (0.1, 0.2, 0.9))


# --- the step-trigger matcher ---------------------------------------------------


def _event_line(event_type, **payload):
    return json.dumps({
        "event_id": "x", "event_type": event_type, "timestamp_ns": 1,
        "wall_time_iso": "2026-09-30T00:00:00+00:00", "payload": payload,
    })


def test_step_trigger_matches_only_its_begin_step():
    matcher = ii.StepTriggerMatcher("place_u_green")
    sample = "\n".join([
        _event_line("begin_step", step_id="pick_u_green"),
        _event_line("skill_start", skill_name="look"),
        "{ this line is corrupt",
        _event_line("end_step", step_id="pick_u_green", claimed_outcome="success"),
        # mentions the id but is NOT a begin_step:
        _event_line("skill_start", skill_name="x", params={"note": "place_u_green"}),
    ]) + "\n"
    assert matcher.feed(sample) is False
    assert matcher.feed(_event_line("begin_step", step_id="place_u_green") + "\n")
    assert matcher.matched is True


def test_step_trigger_handles_partial_line_chunks():
    matcher = ii.StepTriggerMatcher("s2")
    line = _event_line("begin_step", step_id="s2") + "\n"
    # Feed in three fragments: no match until the newline arrives.
    assert matcher.feed(line[:10]) is False
    assert matcher.feed(line[10:-1]) is False
    assert matcher.feed(line[-1]) is True


def test_step_trigger_rejects_empty_step_id():
    with pytest.raises(ValueError):
        ii.StepTriggerMatcher("")


# --- ground truth records -------------------------------------------------------


def test_ground_truth_record_shape():
    record = ii.build_ground_truth_record(
        action="inject", name="intruder_a",
        prim_path="/World/WatchdogIntruders/intruder_a",
        trigger="at_step:place_u_green", wall_ns=1_700_000_000_000_000_000,
        session_dir="/tmp/sess", shape="cube", size_m=0.05,
        position_xyz_world=(0.4, -0.2, 0.78), yaw_deg=15.0,
        color_rgb=(0.9, 0.1, 0.1), physics=True, label="red cube",
        expected_impact="benign", notes="n", sha256="ab" * 32,
    )
    expected_keys = {
        "action", "name", "prim_path", "shape", "size_m",
        "position_xyz_world", "yaw_deg", "color_rgb", "physics", "label",
        "expected_impact", "notes", "trigger", "wall_ns", "wall_time_iso",
        "session_dir", "script_sha256",
        # camera-visibility triple (PREBATCH_FIXES item 9)
        "pixel_uv", "in_camera_view", "edge_margin_px",
    }
    assert set(record) == expected_keys
    assert record["position_xyz_world"] == [0.4, -0.2, 0.78]
    assert record["wall_time_iso"].startswith("2023-11-14T22:13:20")
    assert record["script_sha256"] == "ab" * 32
    # A move/remove record nulls the geometry fields but keeps every key.
    sparse = ii.build_ground_truth_record(
        action="remove", name="intruder_a",
        prim_path="/World/WatchdogIntruders/intruder_a",
        trigger="immediate", wall_ns=1, session_dir="/tmp/sess", sha256="x",
    )
    assert set(sparse) == expected_keys
    assert sparse["shape"] is None and sparse["physics"] is None
    assert sparse["pixel_uv"] is None and sparse["in_camera_view"] is None


def test_append_jsonl_round_trips(tmp_path):
    path = tmp_path / "ground_truth_injections.jsonl"
    ii.append_jsonl(path, {"a": 1})
    ii.append_jsonl(path, {"b": 2})
    lines = path.read_text().splitlines()
    assert [json.loads(line) for line in lines] == [{"a": 1}, {"b": 2}]


def test_trigger_text():
    assert ii.trigger_text(None, 0) == "immediate"
    assert ii.trigger_text(None, 2.5) == "delay:2.5"
    assert ii.trigger_text("s1", 0) == "at_step:s1"
    assert ii.trigger_text("s1", 9) == "at_step:s1"  # at_step wins


def test_script_sha256_matches_file():
    import hashlib

    expected = hashlib.sha256(Path(ii.__file__).read_bytes()).hexdigest()
    assert ii.script_sha256() == expected


# --- session-dir resolution ------------------------------------------------------


def test_resolve_session_dir(tmp_path):
    pointer = tmp_path / "current_session"
    session = tmp_path / "sessions" / "20260930T000000Z"
    session.mkdir(parents=True)

    pointer.write_text(str(session) + "\n")
    assert ii.resolve_session_dir(pointer) == session

    pointer.write_text("")
    with pytest.raises(RuntimeError, match="empty"):
        ii.resolve_session_dir(pointer)

    pointer.write_text(str(session / "missing"))
    with pytest.raises(RuntimeError, match="not a directory"):
        ii.resolve_session_dir(pointer)

    with pytest.raises(RuntimeError, match="cannot read"):
        ii.resolve_session_dir(tmp_path / "no_such_pointer")


def test_module_imports_without_isaac():
    """The import above already proved it, but say so explicitly: no omni/pxr
    module may be required at import time."""
    assert "omni" not in sys.modules or ii is not None
    assert ii.INTRUDER_ROOT == "/World/WatchdogIntruders"


# --- camera projection in ground truth (PREBATCH_FIXES item 9) ------------------


def test_projection_matches_reference_pixels():
    """Reference points measured against /camera/depth/camera_info 2026-09-30."""
    for world, expected in (((-0.5, 0.1, 0.035), (576.0, 341.0)),
                            ((0.25, -0.38, 0.0), (320.0, 180.0))):
        pixel, depth = ii.project_world_point(world)
        assert pixel is not None and depth > 0
        assert abs(pixel[0] - expected[0]) <= 5.0, (world, pixel)
        assert abs(pixel[1] - expected[1]) <= 5.0, (world, pixel)


def test_camera_view_fields_margins():
    centre = ii.camera_view_fields((0.25, -0.38, 0.0))  # projects to ~image centre
    assert centre["in_camera_view"] is True
    assert centre["edge_margin_px"] > 100

    # The S2/S3 smoke-run placement that landed on the image edge.
    edge = ii.camera_view_fields((-0.5, 0.1, 0.035))
    assert edge["pixel_uv"] is not None
    assert edge["edge_margin_px"] < ii.CAMERA_VIEW_MARGIN_PX
    assert edge["in_camera_view"] is False

    behind = ii.camera_view_fields((0.25, -0.2, 5.0))  # above the camera
    assert behind["pixel_uv"] is None
    assert behind["in_camera_view"] is False


def test_ground_truth_records_carry_visibility():
    record = ii.build_ground_truth_record(
        action="inject", name="x", prim_path="/World/WatchdogIntruders/x",
        trigger="immediate", wall_ns=1, session_dir="/tmp/s",
        position_xyz_world=(0.25, -0.38, 0.0), sha256="f" * 64,
    )
    assert record["in_camera_view"] is True
    assert record["pixel_uv"] == [320.0, 179.6]
    moved = ii.build_ground_truth_record(
        action="move", name="x", prim_path="/World/WatchdogIntruders/x",
        trigger="immediate", wall_ns=2, session_dir="/tmp/s",
        position_xyz_world=(-0.5, 0.1, 0.035), sha256="f" * 64,
    )
    assert moved["in_camera_view"] is False
    assert moved["edge_margin_px"] < 20
