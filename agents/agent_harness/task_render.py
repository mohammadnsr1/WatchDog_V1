"""Task-template rendering, shared by run_agent.py and the MCP servers.

This logic used to live in run_agent.py, which was fine while the harness was
the only thing that rendered a prompt. Under Claude Code there is no
run_agent.py in the loop: each role's session asks its own MCP server for the
rendered prompt (get_rendered_task), so the same rendering has to be callable
from inside a server process.

It lives here rather than being imported out of run_agent.py because importing
that module has side effects a server process should not inherit: it calls
load_dotenv at import time, mutates sys.path, and lives outside the ROS package
tree. run_agent.py imports FROM here, so there is exactly one implementation
and the API-key path keeps working unchanged.
"""

from __future__ import annotations

import re

_PLACEHOLDER_RE = re.compile(r"\{\{([A-Za-z0-9_]+)\}\}")


def format_steps(steps: list) -> str:
    """Render the task's steps: block as readable text for the EXECUTOR
    prompt only (see render_task/watchdog_task.txt: the watchdog template has
    no {{STEPS}} placeholder, so this text never reaches it).

    The executor is instructed (executor_system.txt) to pass each step's
    grasp_width_m / grasp_center_offset to the motion and gripper tools
    itself; it can only do that if the numbers are actually in its rendered
    prompt, not just present as structured YAML the harness parses. Before
    this, the agent could see the steps: block only as raw prose text buried
    in step_plan (if at all) -- this is why width was passed only
    intermittently on real runs.
    """
    if not steps:
        return "(no structured steps in this task file)"
    lines: list[str] = []
    for entry in steps:
        if not isinstance(entry, dict):
            continue
        lines.append(f"- step_id: {entry.get('step_id', '?')}")
        lines.append(f"  object_id: {entry.get('object_id', '?')}")
        # Render EVERY remaining field rather than an allowlist. An allowlist
        # silently drops whatever a task author adds -- approach_distance_m and
        # target_frame were being dropped this way, so the agent never saw the
        # authored values and substituted its own.
        for key, value in entry.items():
            if key in ("step_id", "object_id", "grasp_center_offset"):
                continue
            lines.append(f"  {key}: {value}")
        offset = entry.get("grasp_center_offset")
        if isinstance(offset, dict):
            pos = offset.get("position", {})
            ori = offset.get("orientation", {})
            lines.append(
                f"  grasp_center_offset: position={pos} orientation={ori}"
            )
    return "\n".join(lines)


def render_task(template_text: str, task: dict) -> str:
    """Substitute {{PLACEHOLDER}} tokens. Raise if any survive rendering.

    A literal placeholder reaching the model produces a confidently wrong run
    that looks clean in the logs, so this is a hard failure, not a warning.
    """
    substitutions = {
        "INSTRUCTION_SPEC": task.get("instruction_spec", ""),
        "STEP_PLAN": task.get("step_plan", ""),
        "OBJECT_IDS": task.get("object_ids", ""),
        "RUN_NOTES": task.get("run_notes", ""),
        # Only the executor template contains {{STEPS}} -- see
        # format_steps' docstring. Building it here unconditionally is
        # harmless for the watchdog template, which never references the key.
        "STEPS": format_steps(task.get("steps", [])),
    }

    def replace(m: re.Match) -> str:
        key = m.group(1)
        if key not in substitutions:
            raise ValueError(f"task file has no value for placeholder {{{{{key}}}}}")
        return str(substitutions[key])

    rendered = _PLACEHOLDER_RE.sub(replace, template_text)

    # An empty run_notes must not leave a dangling "## Run notes" header in
    # the rendered task: a header over nothing reads as missing content, and
    # the field is empty BY DESIGN (a note here renders into every scenario's
    # task, so it must never name a scenario-gated tool -- see the task
    # files). Dropped here, in the one shared renderer, rather than per
    # template.
    if not str(task.get("run_notes", "")).strip():
        rendered = re.sub(r"## Run notes\s*\n\s*\n(?=---|\Z)", "", rendered)

    survivors = _PLACEHOLDER_RE.findall(rendered)
    if survivors:
        raise ValueError(
            f"unrendered placeholder(s) survived template rendering: {survivors}. "
            f"A literal placeholder reaching the model produces a confidently "
            f"wrong run that looks clean in the logs -- refusing to proceed."
        )
    return rendered


__all__ = ["render_task", "format_steps"]
