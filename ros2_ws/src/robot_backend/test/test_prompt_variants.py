"""Pin every S2/S3/S4 prompt and task variant to its source, mechanically.

The variants are checked-in files (prompt rendering reads files, not
builders), so nothing at runtime guarantees they still equal what
prompt_variants.py derives from the S1 sources. This test is that guarantee:
each variant must equal its builder's output byte for byte, every anchor must
still match exactly once in its source, and the derived S4 task and config
files must differ from their sources only where the derivation says they do.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[4]
PROMPTS = REPO / "agents" / "prompts"
TASKS = REPO / "agents" / "tasks"
CONFIGS = REPO / "agents" / "config"

sys.path.insert(0, str(PROMPTS))
import prompt_variants as pv  # noqa: E402


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def s1_watchdog():
    return _read(PROMPTS / "watchdog_system.txt")


@pytest.fixture(scope="module")
def s1_executor():
    return _read(PROMPTS / "executor_system.txt")


@pytest.fixture(scope="module")
def s2_text():
    return _read(PROMPTS / "watchdog_system_s2.txt")


@pytest.fixture(scope="module")
def s3_text():
    return _read(PROMPTS / "watchdog_system_s3.txt")


@pytest.fixture(scope="module")
def s4_text():
    return _read(PROMPTS / "executor_system_s4.txt")


# --- the checked-in variants equal the builders' output -----------------------

def test_s2_equals_builder_output(s1_watchdog, s2_text):
    assert pv.build_watchdog(s1_watchdog, "S2") == s2_text


def test_s3_equals_builder_output(s1_watchdog, s3_text):
    assert pv.build_watchdog(s1_watchdog, "S3") == s3_text


def test_s4_executor_equals_builder_output(s1_executor, s4_text):
    assert pv.build_executor_s4(s1_executor) == s4_text


def test_builder_rejects_unknown_scenario(s1_watchdog):
    with pytest.raises(ValueError):
        pv.build_watchdog(s1_watchdog, "S1")


# --- every anchor matches exactly once in its source --------------------------

def _count_lines(text, anchor, prefix):
    return sum(
        1
        for line in text.split("\n")
        if (line.startswith(anchor) if prefix else line == anchor)
    )


def test_watchdog_anchors_match_exactly_once(s1_watchdog):
    assert _count_lines(s1_watchdog, pv.A_START_LINE, prefix=False) == 1
    assert _count_lines(s1_watchdog, pv.A_END_LINE, prefix=False) == 1
    assert _count_lines(s1_watchdog, pv.B_START_PREFIX, prefix=True) == 1
    assert _count_lines(s1_watchdog, pv.DISARM_PREFIX, prefix=True) == 1
    assert _count_lines(s1_watchdog, pv.E_ANCHOR_PREFIX, prefix=True) == 1
    assert s1_watchdog.count(pv.D_OLD) == 1


def test_executor_anchors_match_exactly_once(s1_executor):
    assert _count_lines(s1_executor, pv.F_ANCHOR_LINE, prefix=False) == 1
    assert s1_executor.count(pv.G_OLD) == 1
    assert s1_executor.count(pv.H_OLD) == 1


def test_task_extraction_anchors_match_exactly_once():
    for name in ("fmb_assembly.watchdog.yaml", "fmb_assembly_2.watchdog.yaml"):
        data = yaml.safe_load(_read(TASKS / name))
        spec, objects = data["instruction_spec"], data["object_ids"]
        assert _count_lines(spec, pv.WD_REQ_START_PREFIX, prefix=True) == 1, name
        assert _count_lines(spec, pv.WD_REQ_END_PREFIX, prefix=True) == 1, name
        assert _count_lines(objects, pv.WD_BYS_PREFIX, prefix=True) == 1, name


def test_op_raises_on_a_missing_anchor():
    with pytest.raises(ValueError, match="A_DELETE"):
        pv.WATCHDOG_OPS["A_DELETE"]("no anchors here\n")


# --- ablation: S3 is S2 minus the library-monitor material --------------------

def test_s3_is_s2_minus_library_material(s2_text, s3_text):
    ablated = pv.apply_watchdog_ops(
        s2_text, ["A_DELETE", "B_DELETE", "D_REPLACE"]
    )
    assert ablated == s3_text


# --- tool-surface strings per scenario -----------------------------------------

def _arm_monitor_mentions(text):
    """Occurrences of arm_monitor( that are not part of disarm_monitor(."""
    return re.findall(r"(?<!dis)arm_monitor\(", text)


def test_s1_names_no_authored_tool(s1_watchdog):
    assert "arm_custom_monitor" not in s1_watchdog


def test_s2_names_both_arm_tools(s2_text):
    assert _arm_monitor_mentions(s2_text)
    assert "arm_custom_monitor(" in s2_text
    assert "available_monitor_definitions" in s2_text


def test_s3_names_only_the_authored_tool(s3_text):
    assert "arm_custom_monitor(" in s3_text
    assert "available_monitor_definitions" not in s3_text
    assert not _arm_monitor_mentions(s3_text)


def test_s4_executor_monitors_itself_and_never_names_watchdog(s4_text):
    assert "## Monitoring your own execution" in s4_text
    assert "WatchDog" not in s4_text


# --- the S4 task files ----------------------------------------------------------

S4_TASK_SOURCES = {
    "fmb_assembly_s4.executor.yaml": (
        "fmb_assembly.executor.yaml", "fmb_assembly.watchdog.yaml"
    ),
    "fmb_assembly_2_s4.executor.yaml": (
        "fmb_assembly_2.executor.yaml", "fmb_assembly_2.watchdog.yaml"
    ),
}


@pytest.mark.parametrize("s4_name", sorted(S4_TASK_SOURCES))
def test_s4_task_matches_derivation(s4_name):
    exec_name, wd_name = S4_TASK_SOURCES[s4_name]
    s4_data = yaml.safe_load(_read(TASKS / s4_name))
    exec_data = yaml.safe_load(_read(TASKS / exec_name))
    wd_data = yaml.safe_load(_read(TASKS / wd_name))

    # Every key except the two derived blocks is the source's value, compared
    # as parsed data (steps included).
    assert set(s4_data) == set(exec_data)
    for key in exec_data:
        if key in ("instruction_spec", "object_ids"):
            continue
        assert s4_data[key] == exec_data[key], f"{s4_name}: {key} differs"

    expected = pv.s4_task_expected(exec_data, wd_data)
    assert s4_data["instruction_spec"].rstrip() == expected["instruction_spec"].rstrip()
    assert s4_data["object_ids"].rstrip() == expected["object_ids"].rstrip()


@pytest.mark.parametrize("s4_name", sorted(S4_TASK_SOURCES))
def test_s4_object_ids_carry_only_the_accurate_acm_statement(s4_name):
    exec_name, _ = S4_TASK_SOURCES[s4_name]
    s4_objects = yaml.safe_load(_read(TASKS / s4_name))["object_ids"]
    source_objects = yaml.safe_load(_read(TASKS / exec_name))["object_ids"]
    # The appended WatchDog paragraph (the one that matches the registry's
    # full ten-object clique) is present; the executor's own single-assembly
    # paragraph is dropped.
    assert "ALL TEN FMB" in s4_objects
    assert "covers every pair among u_" not in s4_objects
    # And the source executor YAML still carries its own paragraph untouched:
    # the removal is an S4-only derivation, never an edit to the S1-S3 files.
    assert "covers every pair among u_" in source_objects


# --- agent configs differ only in system_prompt ---------------------------------

CONFIG_SOURCES = {
    "watchdog_agent_s2.yaml": (
        "watchdog_agent.yaml", "prompts/watchdog_system_s2.txt"
    ),
    "watchdog_agent_s3.yaml": (
        "watchdog_agent.yaml", "prompts/watchdog_system_s3.txt"
    ),
    "executor_agent_s4.yaml": (
        "executor_agent.yaml", "prompts/executor_system_s4.txt"
    ),
}


@pytest.mark.parametrize("variant", sorted(CONFIG_SOURCES))
def test_agent_config_differs_only_in_system_prompt(variant):
    source_name, expected_prompt = CONFIG_SOURCES[variant]
    variant_data = yaml.safe_load(_read(CONFIGS / variant))
    source_data = yaml.safe_load(_read(CONFIGS / source_name))
    assert variant_data["system_prompt"] == expected_prompt
    assert variant_data["system_prompt"] != source_data["system_prompt"]
    variant_rest = {k: v for k, v in variant_data.items() if k != "system_prompt"}
    source_rest = {k: v for k, v in source_data.items() if k != "system_prompt"}
    assert variant_rest == source_rest
    # The prompt file the variant names must exist.
    assert (REPO / "agents" / expected_prompt).is_file()


# --- mcp.json wiring -------------------------------------------------------------

MCP_SOURCES = {
    "watchdog_s2.mcp.json": ("watchdog.mcp.json", "S2", "watchdog_agent_s2.yaml"),
    "watchdog_s3.mcp.json": ("watchdog.mcp.json", "S3", "watchdog_agent_s3.yaml"),
    "watchdog_2_s2.mcp.json": ("watchdog_2.mcp.json", "S2", "watchdog_agent_s2.yaml"),
    "watchdog_2_s3.mcp.json": ("watchdog_2.mcp.json", "S3", "watchdog_agent_s3.yaml"),
}


@pytest.mark.parametrize("variant", sorted(MCP_SOURCES))
def test_mcp_json_scenario_and_config_agree(variant):
    source_name, scenario, config_name = MCP_SOURCES[variant]
    variant_env = json.loads(_read(REPO / variant))["mcpServers"][
        "robot-backend-watchdog"
    ]["env"]
    source_env = json.loads(_read(REPO / source_name))["mcpServers"][
        "robot-backend-watchdog"
    ]["env"]
    assert variant_env["WATCHDOG_SCENARIO"] == scenario
    config_path = Path(variant_env["WATCHDOG_AGENT_CONFIG"])
    assert config_path.is_absolute()
    assert config_path.name == config_name
    assert config_path.is_file()
    # Same assembly as the S1 json it derives from.
    assert variant_env["WATCHDOG_TASK_PATH"] == source_env["WATCHDOG_TASK_PATH"]


# --- Stage 4: the scene-observation passages, exactly where they belong ---------
#
# Each passage is asserted by whitespace-normalised count, so the same words
# may wrap differently in a prompt (one indent) and a task YAML (another)
# without weakening "appears exactly once / nowhere else".

def _norm(text):
    return " ".join(text.split())


def _count(haystack, needle):
    return _norm(haystack).count(_norm(needle))


P_LOOK_WD = """look(reason, views=["rgb"], max_width=640)
    Returns the current camera image of the workspace to you: "rgb" (colour)
    and/or "depth" (grayscale, near = bright, with its metre range stated).
    The camera is fixed above the bench. Every look is recorded with your
    reason. A look that returns no image means the camera was unavailable,
    not that the scene is clear."""

P_RECORD_OBS = """record_observation(summary, category, location, first_seen_wall_ns,
                   evidence_look_ids, ...)
    Records something you noticed in the scene that you did not expect: an
    object not listed among the task's objects, an object that moved or
    disappeared without a step accounting for it, or any other unexpected
    event. Say what it is as far as the evidence allows, where it is, when
    you believe it appeared, and which of your looks show it. An observation
    is not a finding and does not change a verdict by itself. Update or
    resolve it as you learn more."""

P_CAPTURE = """Preserves camera frames from a past window to disk as a record. You do
    not see these frames. To see the scene, use look."""

P_THRESHOLD_EXEMPT = """A threshold used only to decide when to look or to trigger an observation,
and never to judge a clause, may be chosen by you when the specification
provides none. Report it as agent-chosen, with its value and why."""

P_SCENE_CHANGE_WD = """Watching the scene. The working area is yours to observe for the whole run,
not only at step boundaries. The workspace changes constantly while the
executor works -- the arm moves, and parts are picked, carried and placed --
and those changes are expected when a step accounts for them. Report any
unexpected event: an unidentified object appears in the scene; an object
moves when the robot did not move it; someone or something enters the
workspace and later leaves."""

P_SCENE_CHANGE_S4 = """Watching the scene. The working area is yours to observe for the whole run,
not only at step boundaries. The workspace changes constantly while you work
-- the arm moves, and parts are picked, carried and placed -- and those
changes are expected when a step accounts for them. Report any unexpected
event: an unidentified object appears in the scene; an object moves when you
did not move it; someone or something enters the workspace and later leaves."""

P_SCENE_CHANGE_TAIL = """Report each one with record_observation: what it
is, as far as the evidence allows, where it is, and when it appeared -- and
when it left, if it did. An unexpected event is not a violation by itself; it
becomes one only if it breaks another requirement, and then you also record
that requirement's verdict as usual and name the observation in the
explanation."""

P_CLOSEOUT = """Before recording the run verdict, make sure everything unexpected you
noticed has been recorded with record_observation and that each
observation's status is current."""

P_LOOK_EXEC = """look(reason, views=["rgb"], max_width=640)
    Returns the current camera image of the workspace to you: "rgb" and/or
    "depth" (grayscale, near = bright, with its metre range stated). Use it
    whenever seeing the scene would help you act."""

P_WORKSPACE_REQ = """Workspace observation: observe and monitor the working area for the whole
run, not only at step boundaries; its camera images and depth data are
published. Report any unexpected event: an object present that is not
listed under object_ids, an object that moves when the robot did not move
it, or someone or something entering the workspace and later leaving.
Report each one with what it is (as far as the evidence allows), where it
is, and when it appeared -- and when it left, if it did. An unexpected
event is not a violation by itself; it is a violation only if it breaks
another requirement."""


def test_observation_passages_in_every_watchdog_prompt(s1_watchdog, s2_text, s3_text):
    for text in (s1_watchdog, s2_text, s3_text):
        assert _count(text, P_LOOK_WD) == 1
        assert _count(text, P_RECORD_OBS) == 1
        assert _count(text, P_CAPTURE) == 1
        # Deliberately twice: once in Evidence discipline, once attached to
        # the Failure modes threshold rule it qualifies.
        assert _count(text, P_THRESHOLD_EXEMPT) == 2
        assert _count(text, P_SCENE_CHANGE_WD) == 1
        assert _count(text, P_SCENE_CHANGE_TAIL) == 1
        assert _count(text, P_CLOSEOUT) == 1


def test_camera_monitor_sentence_only_in_authored_scenarios(
    s1_watchdog, s2_text, s3_text, s4_text, s1_executor
):
    assert _count(s1_watchdog, pv.BLOCK_I) == 0
    assert _count(s1_executor, pv.BLOCK_I) == 0
    assert _count(s2_text, pv.BLOCK_I) == 1
    assert _count(s3_text, pv.BLOCK_I) == 1
    # The S4 executor authors its own monitors, so it carries it too.
    assert _count(s4_text, pv.BLOCK_I) == 1


def test_s4_executor_carries_the_adapted_observation_passages(
    s4_text, s1_executor
):
    assert _count(s4_text, P_RECORD_OBS) == 1
    assert _count(s4_text, P_THRESHOLD_EXEMPT) == 1
    assert _count(s4_text, P_SCENE_CHANGE_S4) == 1
    assert _count(s4_text, P_SCENE_CHANGE_WD) == 0  # role-adapted, not copied
    assert _count(s4_text, P_SCENE_CHANGE_TAIL) == 1
    assert _count(s4_text, P_CLOSEOUT) == 1
    assert _count(s4_text, P_LOOK_EXEC) == 1
    # The S1-S3 executor gets the look tool and nothing else.
    assert _count(s1_executor, P_LOOK_EXEC) == 1
    for passage in (P_RECORD_OBS, P_THRESHOLD_EXEMPT, P_SCENE_CHANGE_TAIL,
                    P_CLOSEOUT):
        assert _count(s1_executor, passage) == 0


def test_workspace_requirement_in_the_right_task_files():
    carries = [
        "fmb_assembly.watchdog.yaml", "fmb_assembly_2.watchdog.yaml",
        "fmb_assembly_s4.executor.yaml", "fmb_assembly_2_s4.executor.yaml",
    ]
    lacks = ["fmb_assembly.executor.yaml", "fmb_assembly_2.executor.yaml"]
    for name in carries:
        spec = yaml.safe_load(_read(TASKS / name))["instruction_spec"]
        assert _count(spec, P_WORKSPACE_REQ) == 1, name
    for name in lacks:
        spec = yaml.safe_load(_read(TASKS / name))["instruction_spec"]
        assert _count(spec, P_WORKSPACE_REQ) == 0, name
