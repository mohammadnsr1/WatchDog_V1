#!/usr/bin/env python3
"""Entry point for launching one agent (executor XOR watchdog) against a task.

    python3 run_agent.py --config config/watchdog_agent.yaml --task tasks/two_cube_stack.yaml
    python3 run_agent.py --config config/executor_agent.yaml --task tasks/two_cube_stack.yaml
                         [--provider ...] [--model ...] [--dry-run]

The two agents are launched as two separate processes against the same task
file. This script never launches both -- coupling their lifecycles here would
blur the independence the study rests on. See README.md for the launch
protocol.
"""

from __future__ import annotations

import argparse
import asyncio
import re
import sys
from pathlib import Path

from dotenv import load_dotenv

sys.path.insert(0, str(Path(__file__).resolve().parent))

import yaml

from agent_harness.config import load_config
from agent_harness.mcp_client import MCPClientPool

_PLACEHOLDER_RE = re.compile(r"\{\{([A-Za-z0-9_]+)\}\}")


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

    config = load_config(args.config)
    if args.provider is not None:
        config.provider = args.provider
    if args.model is not None:
        config.model = args.model

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
    return 0


async def _dry_run(config, system_prompt: str, task_text: str) -> int:
    print(f"=== agent: {config.agent} | provider: {config.provider} | model: {config.model} ===")
    print()
    print("--- connecting to MCP servers ---")
    async with MCPClientPool(config.mcp_servers) as pool:
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
