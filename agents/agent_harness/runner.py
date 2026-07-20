"""The runner: identical for both agents, no agent-specific logic.

It connects to the agent's configured MCP servers, discovers tools, loops the
provider against those tools, and records everything. It does not know about
monitors, verdicts, skills, or steps -- it is a conduit and a recorder only.

The one piece of cross-cutting behavior is the wait_for_event timeout rule,
which exists because it is how a WatchDog run terminates once WatchDog has
run out of things to watch for. The executor has no such tool and simply
never takes that path (see agent_harness.mcp_client.WAIT_FOR_EVENT_TOOL).
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field

from agent_harness.config import AgentConfig
from agent_harness.mcp_client import MCPClientPool, MCPToolError, WAIT_FOR_EVENT_TOOL
from agent_harness.providers import (
    FatalProviderError,
    ProviderResponse,
    TransientProviderError,
    get_provider,
)
from agent_harness.timebase import wall_ns
from agent_harness.transcript import TranscriptWriter

NUDGE_MESSAGE = (
    "You have not taken any action. If your work is complete, say so plainly. "
    "Otherwise, continue."
)

TERMINATION_REASONS = {
    "run_complete",
    "max_turns_exhausted",
    "agent_idle",
    "provider_error",
    "mcp_error",
}


@dataclass
class RunResult:
    termination_reason: str
    turn_count: int
    transcript_path: str
    verdict_count: int | None = None


def _git_commit(cwd: str) -> str | None:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=cwd,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return None


async def run(config: AgentConfig, task_text: str, system_prompt: str) -> RunResult:
    task_name = config.task_template.stem
    transcript = TranscriptWriter(config.transcript_dir, config.agent, task_name)

    raw_config_for_log = {
        "agent": config.agent,
        "provider": config.provider,
        "model": config.model,
        "api_key_env": config.api_key_env,
        "max_tokens": config.max_tokens,
        "mcp_servers": [
            {"name": s.name, "command": s.command, "args": s.args, "cwd": s.cwd, "env": s.env}
            for s in config.mcp_servers
        ],
        "loop": {
            "max_turns": config.loop.max_turns,
            "wait_timeout_sec": config.loop.wait_timeout_sec,
            "stop_after_consecutive_timeouts": config.loop.stop_after_consecutive_timeouts,
        },
    }
    transcript.run_start(
        config=raw_config_for_log,
        agent=config.agent,
        provider=config.provider,
        model=config.model,
        task_path=str(config.task_template),
        git_commit=_git_commit(str(config.config_path.parent)),
    )

    provider = get_provider(config.provider, config.model, config.api_key, config.max_tokens)

    termination_reason = "max_turns_exhausted"
    turn_count = 0
    verdict_count = 0

    try:
        async with MCPClientPool(config.mcp_servers) as pool:
            discovered = await pool.list_tools()
            tool_dicts = [
                {
                    "name": t.name,
                    "description": t.description,
                    "input_schema": t.input_schema,
                }
                for t in discovered
            ]
            transcript.tools_discovered(tool_dicts)

            messages: list[dict] = [{"role": "user", "content": task_text}]
            consecutive_no_tool_calls = 0
            consecutive_timeouts = 0

            while True:
                if turn_count >= config.loop.max_turns:
                    termination_reason = "max_turns_exhausted"
                    break
                turn_count += 1

                transcript.llm_request(turn_count, system_prompt, messages)
                try:
                    response: ProviderResponse = provider.complete(
                        system_prompt, messages, tool_dicts
                    )
                except (FatalProviderError, TransientProviderError):
                    termination_reason = "provider_error"
                    break

                transcript.llm_response(
                    turn_count,
                    response.text,
                    response.tool_calls,
                    response.stop_reason,
                    response.raw,
                    response.usage,
                )

                if not response.tool_calls:
                    consecutive_no_tool_calls += 1
                    messages.append({"role": "assistant", "content": response.text})
                    if consecutive_no_tool_calls >= 2:
                        termination_reason = "agent_idle"
                        break
                    messages.append({"role": "user", "content": NUDGE_MESSAGE})
                    continue

                consecutive_no_tool_calls = 0
                messages.append(
                    {
                        "role": "assistant",
                        "content": response.text,
                        "tool_calls": response.tool_calls,
                    }
                )

                mcp_error_occurred = False
                for tc in response.tool_calls:
                    t0 = wall_ns()
                    transcript.tool_call(turn_count, tc.name, tc.arguments, t0)
                    try:
                        result = await pool.call_tool(tc.name, tc.arguments)
                    except MCPToolError:
                        mcp_error_occurred = True
                        break
                    t1 = wall_ns()
                    transcript.tool_result(turn_count, tc.name, result, t1, t1 - t0)
                    messages.append(
                        {
                            "role": "tool_result",
                            "tool_call_id": tc.id,
                            "name": tc.name,
                            "content": result,
                        }
                    )

                    if tc.name == "record_verdict":
                        verdict_count += 1

                    if tc.name == WAIT_FOR_EVENT_TOOL:
                        wake = result.get("wake") if isinstance(result, dict) else None
                        if wake == "timeout":
                            consecutive_timeouts += 1
                        else:
                            consecutive_timeouts = 0

                if mcp_error_occurred:
                    termination_reason = "mcp_error"
                    break

                if consecutive_timeouts >= config.loop.stop_after_consecutive_timeouts:
                    termination_reason = "run_complete"
                    break
    finally:
        transcript.run_end(termination_reason, turn_count, verdict_count=verdict_count)
        transcript.close()

    return RunResult(
        termination_reason=termination_reason,
        turn_count=turn_count,
        transcript_path=str(transcript.path),
        verdict_count=verdict_count,
    )
