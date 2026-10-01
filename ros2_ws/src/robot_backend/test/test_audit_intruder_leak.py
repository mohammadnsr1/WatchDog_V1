"""audit_intruder_leak.py on synthetic session dirs (Stage 3).

The audit's one job: say whether an intruder's name reached any channel the
agent can read outside the camera-mediated look/observation text. These tests
build session dirs by hand and check both verdicts and the exemptions.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(_REPO_ROOT / "scripts"))

import audit_intruder_leak as audit  # noqa: E402


def _ground_truth_line(name="intruder_a"):
    return json.dumps({
        "action": "inject", "name": name,
        "prim_path": f"/World/WatchdogIntruders/{name}",
        "shape": "cube", "size_m": 0.05,
        "position_xyz_world": [0.4, -0.2, 0.78], "yaw_deg": 0.0,
        "color_rgb": [0.9, 0.1, 0.1], "physics": True, "label": "red cube",
        "expected_impact": "benign", "notes": "", "trigger": "immediate",
        "wall_ns": 1, "wall_time_iso": "2026-09-30T00:00:00+00:00",
        "session_dir": "/tmp/x", "script_sha256": "f" * 64,
    })


def _event_line(event_type, **payload):
    return json.dumps({
        "event_id": "e", "event_type": event_type, "timestamp_ns": 1,
        "wall_time_iso": "2026-09-30T00:00:00+00:00", "payload": payload,
    })


def _call_line(tool, **extra):
    return json.dumps({
        "wall_time_iso": "2026-09-30T00:00:00+00:00", "server": "watchdog",
        "tool": tool, "pid": 1, **extra,
    })


@pytest.fixture
def session(tmp_path):
    (tmp_path / audit.GROUND_TRUTH_FILENAME).write_text(
        _ground_truth_line() + "\n"
    )
    return tmp_path


def test_clean_session_has_no_leak(session):
    (session / "events_scene.jsonl").write_text(
        _event_line("begin_step", step_id="pick_u_green") + "\n"
    )
    (session / "mcp_calls_watchdog.jsonl").write_text(
        _call_line("get_events", arguments={}) + "\n"
    )
    result = audit.audit_session(session)
    assert result["leak"] is False
    assert result["hits"] == []
    assert result["intruder_names"] == ["intruder_a"]
    # The bag is reported as unavailable, not silently skipped.
    assert any(
        c["source"] == "bag" and "unavailable" in c["status"]
        for c in result["checked"]
    )


def test_look_and_observation_text_are_exempt(session):
    (session / "events_watchdog.jsonl").write_text("\n".join([
        _event_line("look", look_id="0001-aa", role="watchdog",
                    reason="a red cube (intruder_a?) appeared on the table"),
        _event_line("observation", observation_id="obs-001", role="watchdog",
                    summary="unlisted red cube, calling it intruder_a"),
    ]) + "\n")
    (session / "mcp_calls_watchdog.jsonl").write_text("\n".join([
        _call_line("look", arguments={"reason": "checking on intruder_a"}),
        _call_line("record_observation",
                   arguments={"summary": "intruder_a is present"}),
    ]) + "\n")
    result = audit.audit_session(session)
    assert result["leak"] is False, result["hits"]


def test_event_leak_is_detected(session):
    (session / "events_watchdog.jsonl").write_text(
        _event_line("monitor_armed", monitor_id="m1",
                    params={"child_frame": "intruder_a"}) + "\n"
    )
    result = audit.audit_session(session)
    assert result["leak"] is True
    (hit,) = result["hits"]
    assert hit["source"] == "events"
    assert hit["event_type"] == "monitor_armed"
    assert hit["name"] == "intruder_a"


def test_mcp_call_leak_is_detected(session):
    (session / "mcp_calls_watchdog.jsonl").write_text(
        _call_line("arm_monitor",
                   arguments={"params": {"child_frame": "intruder_a"}}) + "\n"
    )
    result = audit.audit_session(session)
    assert result["leak"] is True
    (hit,) = result["hits"]
    assert hit["source"] == "mcp_calls"
    assert hit["tool"] == "arm_monitor"


def test_authored_monitor_leak_is_detected(session):
    monitors = session / "authored_monitors"
    monitors.mkdir()
    (monitors / "m1.py").write_text(
        "def check(ctx):\n"
        "    return ctx.transform('world', 'intruder_a')\n"
    )
    result = audit.audit_session(session)
    assert result["leak"] is True
    (hit,) = result["hits"]
    assert hit["source"] == "authored_monitors"
    assert "intruder_a" in hit["snippet"]


def test_prim_leaf_name_is_searched_even_when_it_differs(tmp_path):
    record = json.loads(_ground_truth_line())
    record["name"] = "intruder_b"
    record["prim_path"] = "/World/WatchdogIntruders/intruder_b_prim"
    (tmp_path / audit.GROUND_TRUTH_FILENAME).write_text(
        json.dumps(record) + "\n"
    )
    (tmp_path / "events_executor.jsonl").write_text(
        _event_line("skill_start", skill_name="get_object_pose",
                    params={"object_id": "intruder_b_prim"}) + "\n"
    )
    result = audit.audit_session(tmp_path)
    assert sorted(result["intruder_names"]) == ["intruder_b", "intruder_b_prim"]
    assert result["leak"] is True


def test_missing_ground_truth_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        audit.audit_session(tmp_path)


def test_main_writes_audit_file_and_exit_code(session, capsys):
    (session / "events_watchdog.jsonl").write_text(
        _event_line("monitor_armed", params={"frame": "intruder_a"}) + "\n"
    )
    code = audit.main(["audit_intruder_leak.py", str(session)])
    assert code == 1  # leak found
    saved = json.loads((session / audit.AUDIT_FILENAME).read_text())
    assert saved["leak"] is True
    printed = json.loads(capsys.readouterr().out)
    assert printed == saved

    # Clean run: exit 0, file overwritten.
    (session / "events_watchdog.jsonl").unlink()
    code = audit.main(["audit_intruder_leak.py", str(session)])
    assert code == 0
    assert json.loads((session / audit.AUDIT_FILENAME).read_text())["leak"] is False


def test_main_rejects_bad_usage(tmp_path, capsys):
    assert audit.main(["audit_intruder_leak.py"]) == 2
    assert audit.main(["audit_intruder_leak.py", str(tmp_path / "nope")]) == 2
    assert audit.main(["audit_intruder_leak.py", str(tmp_path)]) == 2  # no gt file
