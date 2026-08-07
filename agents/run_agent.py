#!/usr/bin/env python3
"""Entry point for launching one agent (executor XOR watchdog) against a task.

Each scenario has TWO task files, one per role, so the two agents can diverge
(an executor file may carry extra or faulty steps; the watchdog file states
only what conformant execution looks like) while sharing object-level truth,
which lives in model_registry.yaml, not in either task file:

    python3 run_agent.py --config config/watchdog_agent.yaml --task tasks/two_cube_stack.watchdog.yaml
    python3 run_agent.py --config config/executor_agent.yaml --task tasks/two_cube_stack.executor.yaml
                         [--provider ...] [--model ...] [--dry-run]

Launch watchdog FIRST, then executor: the watchdog process writes the shared
session pointer (see _resolve_and_export_session), and the executor joins it
via that pointer. The two agents are launched as two separate processes. This
script never launches both -- coupling their lifecycles here would blur the
independence the study rests on. See README.md for the launch protocol.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import re
import sys
from pathlib import Path

from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).resolve().parent))

import yaml

from agent_harness.config import load_config
from agent_harness.mcp_client import MCPClientPool

_PLACEHOLDER_RE = re.compile(r"\{\{([A-Za-z0-9_]+)\}\}")


def _resolve_and_export_session(agents_dir: Path):
    """Resolve the session directory, export WATCHDOG_SESSION_DIR for the MCP
    servers this process is about to spawn, and publish the pointer so the
    OTHER agent (launched as a separate process, moments later) joins the
    same session instead of resolving a fresh one of its own.

    Uses the same resolver as the ROS side (robot_backend.watchdog.session), so
    both process trees agree on one directory: WATCHDOG_SESSION_DIR wins, else
    the ~/.watchdog/current_session pointer the launch wrote, else a fresh
    session under agents/transcripts/sessions. The resolved path is written
    back into os.environ as a concrete path -- essential for the "fresh" case,
    where each child re-resolving on its own would otherwise pick a different
    timestamp. mcp_client passes {**os.environ, **server.env} to each server,
    so all four inherit it.

    Publishing the pointer is what lets the second agent join: without it,
    resolve_session_dir() in the second process sees no WATCHDOG_SESSION_DIR
    (each agent is its own process, launched separately -- see module
    docstring) and no valid pointer, so it falls back to rule='fresh' and
    creates a SECOND, different session directory. That breaks
    watchdog/executor correlation silently -- both agents run, both look
    healthy, and their events land in two different directories with no
    error. The two agents launch sequentially (watchdog then executor per the
    documented protocol), so first-writer-wins is sufficient and no locking
    is added here:
      - rule == 'fresh' or 'env': this process is the one establishing the
        session (nothing else has claimed it yet, or an explicit override was
        given) -- write the pointer so the next agent finds it.
      - rule == 'pointer': a pointer already names this exact session (either
        this agent is the second one joining, or a prior run's pointer is
        being reused) -- leave it untouched rather than rewrite it to the
        same value for no reason.
    This governs only the harness-side pointer used by these two Python
    processes; it does not touch the ROS-side cell-bringup pointer logic.

    Returns the session Path, or None if the shared helper cannot be imported
    (install-only / no source tree) -- in which case transcripts fall back to
    config.transcript_dir and the servers keep their legacy log locations.
    """
    src_path = agents_dir.parent / "ros2_ws" / "src" / "robot_backend"
    if src_path.is_dir() and str(src_path) not in sys.path:
        sys.path.insert(0, str(src_path))
    try:
        from robot_backend.watchdog import session as sess
    except Exception as exc:
        print(f"session: shared helper unavailable ({exc}); "
              f"transcripts fall back to config.transcript_dir")
        return None

    session_dir, rule = sess.resolve_session_dir()
    session_dir.mkdir(parents=True, exist_ok=True)
    os.environ["WATCHDOG_SESSION_DIR"] = str(session_dir)
    if rule in ("fresh", "env"):
        sess.write_pointer(session_dir)
    print(f"session: {session_dir} (resolved by rule={rule!r}; "
          f"exported WATCHDOG_SESSION_DIR for MCP servers)")
    return session_dir


def _export_task_path(agent_role: str, task_path: str) -> None:
    """Export the role-matching task-path env var so the MCP servers this
    process spawns can read this agent's own task YAML directly.

    Each agent now runs against its OWN file (tasks/<scenario>.executor.yaml
    or tasks/<scenario>.watchdog.yaml -- see agents/tasks/README or the
    per-file step_id-convention header) so the two roles can diverge: an
    executor file may carry extra or faulty steps (class 2/3), while the
    watchdog file states only what conformant execution looks like.
    Object-level truth (geometry, tolerances, max_clamp_effort) stays out of
    both, in model_registry.yaml, shared by construction.

    Exports ONLY the env var matching agent_role -- never both -- so the
    watchdog process can never read the executor's (possibly faulty) file,
    even on a launch mistake (wrong --config paired with the wrong --task).
    grasp_width_m and grasp_center_offset are instruction-sourced per-step
    conformance/geometry data (see agents/tasks/*.yaml's steps: block); the
    contract loader (get_contract) reads them from WATCHDOG_TASK_PATH, never
    from an agent-supplied tool argument. mcp_client passes
    {**os.environ, **server.env} to each server, so all four inherit
    whichever var this process exports.
    """
    resolved = str(Path(task_path).resolve())
    if agent_role == "watchdog":
        env_name = "WATCHDOG_TASK_PATH"
    elif agent_role == "executor":
        env_name = "EXECUTOR_TASK_PATH"
    else:
        raise ValueError(
            f"unknown agent role {agent_role!r}; expected 'executor' or 'watchdog'"
        )
    os.environ[env_name] = resolved
    print(f"task: {resolved} (exported {env_name} for MCP servers)")


def _format_steps(steps: list) -> str:
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
        if "grasp_width_m" in entry:
            lines.append(f"  grasp_width_m: {entry['grasp_width_m']}")
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
        # _format_steps' docstring. Building it here unconditionally is
        # harmless for the watchdog template, which never references the key.
        "STEPS": _format_steps(task.get("steps", [])),
    }

    def replace(m: re.Match) -> str:
        key = m.group(1)
        if key not in substitutions:
            raise ValueError(f"task file has no value for placeholder {{{{{key}}}}}")
        return str(substitutions[key])

    rendered = _PLACEHOLDER_RE.sub(replace, template_text)

    survivors = _PLACEHOLDER_RE.findall(rendered)
    if survivors:
        raise ValueError(
            f"unrendered placeholder(s) survived template rendering: {survivors}. "
            f"A literal placeholder reaching the model produces a confidently "
            f"wrong run that looks clean in the logs -- refusing to proceed."
        )
    return rendered


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="path to agent config yaml")
    parser.add_argument("--task", required=True, help="path to task yaml")
    parser.add_argument("--provider", default=None, help="override config provider")
    parser.add_argument("--model", default=None, help="override config model")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="open servers, list tools, print rendered prompts, and exit "
        "without calling the provider",
    )
    args = parser.parse_args()

    agents_dir = Path(__file__).resolve().parent
    load_dotenv(agents_dir / ".env")

    # Loaded first (before the session/task exports and MCPClientPool) purely
    # because _export_task_path needs config.agent to pick the role-matching
    # env var.
    config = load_config(args.config)
    if args.provider is not None:
        config.provider = args.provider
    if args.model is not None:
        config.model = args.model

    # Resolve and export the session directory BEFORE MCPClientPool, so the
    # four MCP servers this process spawns inherit WATCHDOG_SESSION_DIR and
    # this run's transcripts land in the session.
    _resolve_and_export_session(agents_dir)
    # Same reasoning for the task path -- export it before the MCP servers
    # spawn so get_contract (watchdog role only) can read the steps: block
    # from this run's own task file.
    _export_task_path(config.agent, args.task)

    with open(config.system_prompt) as f:
        system_prompt = f.read()
    with open(config.task_template) as f:
        task_template_text = f.read()
    with open(args.task) as f:
        task = yaml.safe_load(f)

    task_text = render_task(task_template_text, task)

    if args.dry_run:
        return asyncio.run(_dry_run(config, system_prompt, task_text))

    from agent_harness.runner import run

    result = asyncio.run(run(config, task_text, system_prompt))
    print(f"termination_reason={result.termination_reason}")
    print(f"turn_count={result.turn_count}")
    print(f"transcript={result.transcript_path}")
    if result.verdict_count is not None:
        print(f"verdict_count={result.verdict_count}")

    _render_own_transcript(result.transcript_path)

    return 0


def _render_own_transcript(transcript_path: str) -> None:
    """Render THIS agent's own timeline views, best-effort, at process exit.

    executor and watchdog are two separate processes launched sequentially
    (see module docstring) -- neither can tell from inside itself whether the
    other has finished, so each process renders only its own transcript here.
    Both <agent>_timeline.* files exist once both processes have exited; there
    is no cross-process coordination. Never lets a render failure affect the
    run's exit code -- the raw transcript is already complete and correct
    regardless of whether this derived view could be produced.
    """
    scripts_dir = Path(__file__).resolve().parent.parent / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    try:
        from render_session_log import render_transcript

        transcript = Path(transcript_path)
        render_transcript(transcript, transcript.parent)
    except Exception as exc:
        print(f"WARNING: session-log render failed (non-fatal): {exc}", file=sys.stderr)


async def _dry_run(config, system_prompt: str, task_text: str) -> int:
    print(f"=== agent: {config.agent} | provider: {config.provider} | model: {config.model} ===")
    print()
    print("--- connecting to MCP servers ---")
    async with MCPClientPool(config.mcp_servers, config.mcp_client_timeout_sec) as pool:
        tools = await pool.list_tools()
        print(f"discovered {len(tools)} tool(s):")
        for t in tools:
            print(f"  - {t.name}  (server: {t.server_name})")
            print(f"      {t.description}")
    print()
    print("--- rendered system prompt ---")
    print(system_prompt)
    print()
    print("--- rendered task ---")
    print(task_text)
    print()
    print("--- dry run complete: provider was not called ---")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
