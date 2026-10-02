"""The RENDERED per-scenario surface (PREBATCH_FIXES items 3 and 5-7).

Renders every scenario's system prompt + task through the real
role_context.build_rendered_task -- the exact path a live session takes -- and
asserts the text matches each scenario's actual tool surface. The smoke runs
showed why file-level checks are not enough: a run note in a SHARED task file
rendered "arm_custom_monitor" into the S1 WatchDog's task, a tool S1 does not
serve, and S3 agents emulated a "library-style" channel because of it.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

import pytest

os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")

REPO = Path(__file__).resolve().parents[4]

import robot_backend.mcp_servers.role_context as rc  # noqa: E402
import robot_backend.mcp_servers.watchdog_mcp_server as ws  # noqa: E402
import robot_backend.mcp_servers.scene_mcp_server as scene  # noqa: E402

SCOPE_SENTENCE = (
    "Covers only the objects and frames registered for this cell; it cannot "
    "show anything else that is in the workspace."
)

WATCHDOG_COMBOS = [
    ("S1", "fmb_assembly", None),
    ("S1", "fmb_assembly_2", None),
    ("S2", "fmb_assembly", "watchdog_agent_s2.yaml"),
    ("S2", "fmb_assembly_2", "watchdog_agent_s2.yaml"),
    ("S3", "fmb_assembly", "watchdog_agent_s3.yaml"),
    ("S3", "fmb_assembly_2", "watchdog_agent_s3.yaml"),
]
EXECUTOR_COMBOS = [
    ("S1-S3", "fmb_assembly", None),
    ("S1-S3", "fmb_assembly_2", None),
    ("S4", "fmb_assembly_s4", "executor_agent_s4.yaml"),
    ("S4", "fmb_assembly_2_s4", "executor_agent_s4.yaml"),
]

_CLEAR = [
    "WATCHDOG_TASK_PATH", "EXECUTOR_TASK_PATH",
    "WATCHDOG_AGENT_CONFIG", "EXECUTOR_AGENT_CONFIG", "WATCHDOG_SCENARIO",
]


def _norm(text: str) -> str:
    return " ".join(text.split())


def _render(monkeypatch, tmp_path, role: str, task_name: str, config: str | None):
    for key in _CLEAR:
        monkeypatch.delenv(key, raising=False)
    monkeypatch.setenv("WATCHDOG_SESSION_DIR", str(tmp_path))
    suffix = "watchdog" if role == "watchdog" else "executor"
    monkeypatch.setenv(
        f"{suffix.upper()}_TASK_PATH",
        str(REPO / "agents" / "tasks" / f"{task_name}.{suffix}.yaml"),
    )
    if config is not None:
        monkeypatch.setenv(
            f"{suffix.upper()}_AGENT_CONFIG",
            str(REPO / "agents" / "config" / config),
        )
    result = rc.build_rendered_task(role)
    return result["system_prompt"], result["task"]


def _bare_arm_monitor(text: str) -> list[str]:
    """arm_monitor( tokens that are neither disarm_monitor( nor arm_custom_monitor(."""
    return re.findall(r"(?<!dis)(?<!custom_)\barm_monitor\(", text)


# --- item 3: no scenario-gated tool named outside its scenario -----------------

@pytest.mark.parametrize("scenario,task_name,config", WATCHDOG_COMBOS)
def test_watchdog_render_matches_scenario_surface(
    monkeypatch, tmp_path, scenario, task_name, config
):
    prompt, task = _render(monkeypatch, tmp_path, "watchdog", task_name, config)
    full = prompt + "\n" + task
    if scenario == "S1":
        assert "arm_custom_monitor" not in full
        assert "dry_run_custom_monitor" not in full
    if scenario == "S3":
        assert "available_monitor_definitions" not in full
        assert not _bare_arm_monitor(full)
    # The task text never names ANY monitor-arming tool, whatever the
    # scenario: tasks are shared files and tool surfaces are not.
    for name in ("arm_custom_monitor", "dry_run_custom_monitor",
                 "available_monitor_definitions"):
        assert name not in task, name
    assert not _bare_arm_monitor(task)
    assert "Run notes" not in task


@pytest.mark.parametrize("scenario,task_name,config", EXECUTOR_COMBOS)
def test_executor_render_has_no_run_notes_with_tool_names(
    monkeypatch, tmp_path, scenario, task_name, config
):
    _prompt, task = _render(monkeypatch, tmp_path, "executor", task_name, config)
    assert "Run notes" not in task
    assert "arm_custom_monitor" not in task
    assert not _bare_arm_monitor(task)


# --- item 5: the registered-only scope sentence, everywhere it belongs ----------

@pytest.mark.parametrize("scenario,task_name,config", WATCHDOG_COMBOS)
def test_scope_sentence_in_the_four_prompt_entries(
    monkeypatch, tmp_path, scenario, task_name, config
):
    prompt, _task = _render(monkeypatch, tmp_path, "watchdog", task_name, config)
    # get_contract, get_execution_state, query_snapshot, get_evidence -- all
    # four entries exist in every scenario's prompt.
    assert _norm(prompt).count(_norm(SCOPE_SENTENCE)) == 4


def test_scope_sentence_in_watchdog_tool_docstrings():
    for tool in (ws.query_snapshot, ws.get_execution_state,
                 ws.get_evidence, ws.get_contract):
        assert _norm(SCOPE_SENTENCE) in _norm(tool.__doc__), tool.__name__
    # MonitorContext: the one note covering the identity/pose accessors.
    assert (
        "cover only the objects and frames registered for this cell"
        in _norm(ws.arm_custom_monitor.__doc__)
    )


def test_scope_sentence_in_executor_tool_docstrings():
    for tool in (scene.get_object_pose, scene.get_object_poses,
                 scene.get_scene_summary):
        assert _norm(SCOPE_SENTENCE) in _norm(tool.__doc__), tool.__name__


# --- items 6 and 7: held-span semantics and arming-vs-inspecting ---------------

def test_held_span_semantics_are_documented():
    doc = _norm(ws.arm_custom_monitor.__doc__)
    assert "becomes the object id at grasp_skill skill_start" in doc
    assert "confirmed at a successful grasp_skill skill_end" in doc
    assert "returns None from release_skill skill_start onward" in doc
    assert (
        "relative_frame monitors are disarmed automatically at release_skill "
        "skill_start" in doc
    )
    assert "authored monitors are NOT" in doc


def test_arming_vs_inspecting_is_documented():
    doc = _norm(ws.arm_custom_monitor.__doc__)
    assert "Arming is monitoring and is recorded as such." in doc
    assert "To inspect what ctx returns without arming, use dry_run_custom_monitor." in doc
    # ctx.transform's exact contract.
    assert '{"translation": {"x","y","z"}, "rotation": {"x","y","z","w"}}' in doc
    assert "child frame's pose expressed in the parent frame" in doc
    assert "time-consistent" in doc
    assert "It never returns None; a lookup that cannot be resolved raises." in doc


# --- item 2's agent-facing sentence ---------------------------------------------

def test_tf_stamp_sentence_is_documented():
    sentence = "Samples from one recorder tick share one TF time (tf_stamp_ns)"
    for tool in (ws.query_snapshot, ws.get_evidence, ws.arm_custom_monitor):
        assert sentence in _norm(tool.__doc__), tool.__name__


# --- observation duty + leak hygiene (OBSERVATION_DUTY_AND_SCENARIOS) -----------

DUTY_HEAD = "Watching the scene. The working area is yours to observe for the whole run"
DUTY_TAIL = (
    "where it is, and when it appeared -- and when it left, if it did. An "
    "unexpected event is not a violation by itself"
)
# Words that would leak the injection machinery to an agent. Checked on the
# RENDERED text (YAML comments never render, so "fault-injection" comments in
# task files are not a leak).
LEAK_WORDS = ("inject", "intruder", "watchdogintruders", "ground truth",
              "planted", "test object")


@pytest.mark.parametrize("scenario,task_name,config", WATCHDOG_COMBOS)
def test_observation_duty_in_every_watchdog_render(
    monkeypatch, tmp_path, scenario, task_name, config
):
    prompt, task = _render(monkeypatch, tmp_path, "watchdog", task_name, config)
    assert _norm(prompt).count(_norm(DUTY_HEAD)) == 1
    assert _norm(prompt).count(_norm(DUTY_TAIL)) == 1
    # and the task's workspace-observation requirement agrees with it
    assert "observe and monitor the working area for the whole" in _norm(task)


def test_observation_duty_in_the_s4_monitoring_section_only(
    monkeypatch, tmp_path
):
    # S4 executor: carries the role-adapted duty ("while you work").
    prompt, _task = _render(monkeypatch, tmp_path, "executor",
                            "fmb_assembly_s4", "executor_agent_s4.yaml")
    assert _norm(prompt).count(_norm(DUTY_HEAD)) == 1
    assert "while you work" in _norm(prompt)
    # S1-S3 executor: no observation duty at all.
    prompt, _task = _render(monkeypatch, tmp_path, "executor",
                            "fmb_assembly", None)
    assert _norm(DUTY_HEAD) not in _norm(prompt)
    assert "record_observation" not in prompt


@pytest.mark.parametrize("scenario,task_name,config",
                         WATCHDOG_COMBOS + EXECUTOR_COMBOS)
def test_no_leak_hygiene_words_on_any_rendered_surface(
    monkeypatch, tmp_path, scenario, task_name, config
):
    role = "watchdog" if (scenario, task_name, config) in [
        tuple(c) for c in WATCHDOG_COMBOS
    ] else "executor"
    prompt, task = _render(monkeypatch, tmp_path, role, task_name, config)
    surface = (prompt + "\n" + task).lower()
    for word in LEAK_WORDS:
        assert word not in surface, (scenario, task_name, word)
