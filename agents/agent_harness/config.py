"""Config loading and validation for an agent run.

A config is agent-agnostic data: the runner does not know or care whether it
is running the executor or WatchDog. Validation here catches structural and
environment problems (missing files, unset keys) plus the one cross-cutting
mistake that would silently break the study's separation guarantee: a config
that lists both the watchdog server and an executor server together.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml

_WATCHDOG_SERVER_NAME = "watchdog"
_EXECUTOR_SERVER_NAMES = {"gripper", "motion", "scene"}
_VALID_REASONING_EFFORTS = {"minimal", "low", "medium", "high"}
_DEFAULT_REASONING_EFFORT = "medium"
_DEFAULT_THINKING_BUDGET_TOKENS = 4096
_DEFAULT_CONTEXT_BUDGET_TOKENS = 150000
_DEFAULT_CONTEXT_KEEP_RECENT_TURNS = 5


@dataclass
class MCPServerConfig:
    name: str
    command: str
    args: list[str]
    cwd: str
    env: dict[str, str] = field(default_factory=dict)


@dataclass
class LoopConfig:
    max_turns: int
    # Meaningful only for an agent that can call wait_for_event (i.e.
    # WatchDog). None for configs (like the executor's) that never take the
    # wait_for_event timeout path at all.
    stop_after_consecutive_timeouts: int | None = None


@dataclass
class AgentConfig:
    agent: str
    provider: str
    model: str
    api_key_env: str
    max_tokens: int
    system_prompt: Path
    task_template: Path
    mcp_servers: list[MCPServerConfig]
    loop: LoopConfig
    transcript_dir: Path
    config_path: Path
    mcp_client_timeout_sec: float
    # Only meaningful for a config whose MCP servers gate on
    # robot_backend.cell.gate (i.e. the executor's scene server). None for
    # configs (like WatchDog's) that never call request_go at all.
    gate_timeout_sec: float | None = None
    # openai: valid values minimal/low/medium/high. Ignored by the anthropic
    # provider.
    reasoning_effort: str = _DEFAULT_REASONING_EFFORT
    # anthropic: extended-thinking token budget. Ignored by the openai
    # provider.
    thinking_budget_tokens: int = _DEFAULT_THINKING_BUDGET_TOKENS
    context_budget_tokens: int = _DEFAULT_CONTEXT_BUDGET_TOKENS
    context_keep_recent_turns: int = _DEFAULT_CONTEXT_KEEP_RECENT_TURNS

    @property
    def api_key(self) -> str:
        return os.environ[self.api_key_env]


def _expand(path_str: str) -> Path:
    return Path(os.path.expanduser(os.path.expandvars(path_str)))


def load_config(config_path: str | Path) -> AgentConfig:
    config_path = Path(config_path).resolve()
    with open(config_path) as f:
        raw = yaml.safe_load(f)

    # Relative paths in the config (system_prompt, task_template,
    # transcript_dir) are written relative to the agents/ directory the
    # tooling is run from (see README), not relative to config/ itself.
    base_dir = Path.cwd()

    def resolve_relative(p: str) -> Path:
        expanded = _expand(p)
        if not expanded.is_absolute():
            expanded = (base_dir / expanded).resolve()
        return expanded

    system_prompt = resolve_relative(raw["system_prompt"])
    task_template = resolve_relative(raw["task_template"])

    if not system_prompt.is_file():
        raise FileNotFoundError(
            f"{config_path}: system_prompt file not found: {system_prompt}"
        )
    if not task_template.is_file():
        raise FileNotFoundError(
            f"{config_path}: task_template file not found: {task_template}"
        )

    api_key_env = raw["api_key_env"]
    if not os.environ.get(api_key_env):
        raise RuntimeError(
            f"{config_path}: api_key_env names '{api_key_env}', but that "
            f"environment variable is not set. Copy .env.example to .env, "
            f"fill it in, and make sure it is loaded before running."
        )

    servers = []
    server_names = set()
    for s in raw["mcp_servers"]:
        cwd = str(_expand(s["cwd"]))
        env = {k: str(_expand(v)) for k, v in s.get("env", {}).items()}
        servers.append(
            MCPServerConfig(
                name=s["name"],
                command=s["command"],
                args=list(s["args"]),
                cwd=cwd,
                env=env,
            )
        )
        server_names.add(s["name"])

    if _WATCHDOG_SERVER_NAME in server_names and (
        server_names & _EXECUTOR_SERVER_NAMES
    ):
        print(
            f"WARNING: {config_path} lists the '{_WATCHDOG_SERVER_NAME}' "
            f"server together with executor server(s) "
            f"{sorted(server_names & _EXECUTOR_SERVER_NAMES)}. This would "
            f"leak monitoring tools to the executor and is almost certainly "
            f"a mistake. Refusing to proceed.",
            file=sys.stderr,
        )
        raise ValueError(
            "config mixes watchdog server with executor servers in one agent"
        )

    # MCP client timeout vs. gate timeout ordering. begin_step/mark_phase can
    # block for as long as the monitoring agent takes to arm (the cell's
    # /cell/request_go gate; see robot_backend/cell/gate.py). If this
    # process's own MCP client timeout is shorter than -- or even just equal
    # to -- that gate timeout, the client aborts the call while the
    # orchestrator is still legitimately holding it: the symptom is an
    # mcp_error in this transcript with the orchestrator log showing a
    # perfectly normal gate_open a few seconds later, which reads like a
    # transport bug and would cost hours to find as anything else. Required
    # ordering: mcp_client_timeout_sec > gate_timeout_sec > expected agent
    # arming turn. Enforced here, at config load, not discovered at runtime.
    mcp_client_timeout_sec = float(raw["mcp_client_timeout_sec"])
    if mcp_client_timeout_sec <= 0:
        raise ValueError(
            f"{config_path}: mcp_client_timeout_sec must be positive, got "
            f"{mcp_client_timeout_sec!r}."
        )

    gate_timeout_sec = raw.get("gate_timeout_sec")
    if gate_timeout_sec is not None:
        gate_timeout_sec = float(gate_timeout_sec)
        if mcp_client_timeout_sec <= gate_timeout_sec:
            raise ValueError(
                f"{config_path}: mcp_client_timeout_sec ({mcp_client_timeout_sec}) "
                f"must be strictly greater than gate_timeout_sec "
                f"({gate_timeout_sec}), or the MCP client will abort a "
                f"begin_step/mark_phase call the orchestrator is still "
                f"legitimately holding open. Increase mcp_client_timeout_sec "
                f"(recommended: comfortably above gate_timeout_sec, e.g. "
                f"gate_timeout_sec + 60)."
            )

        # Cross-check against whatever CELL_GATE_TIMEOUT_SEC the scene
        # server's own env actually carries (robot_backend.cell.gate reads
        # this env var for its default timeout_sec) -- catches the two
        # independently-run processes drifting apart even when each config
        # value looks fine in isolation.
        for s in servers:
            declared = s.env.get("CELL_GATE_TIMEOUT_SEC")
            if declared is None:
                continue
            declared_sec = float(declared)
            if declared_sec != gate_timeout_sec:
                raise ValueError(
                    f"{config_path}: server '{s.name}' sets "
                    f"CELL_GATE_TIMEOUT_SEC={declared!r}, but the config's "
                    f"top-level gate_timeout_sec is {gate_timeout_sec!r}. "
                    f"These must match -- they describe the same timeout "
                    f"from two sides of the gate."
                )

    loop_raw = raw["loop"]
    loop = LoopConfig(
        max_turns=int(loop_raw["max_turns"]),
        stop_after_consecutive_timeouts=(
            int(loop_raw["stop_after_consecutive_timeouts"])
            if "stop_after_consecutive_timeouts" in loop_raw
            else None
        ),
    )

    # Reasoning visibility config. Both fields are read regardless of which
    # provider this config targets -- reasoning_effort is ignored by
    # AnthropicProvider and thinking_budget_tokens by OpenAIProvider (see
    # get_provider) -- so a config can be pointed at either provider without
    # editing this block. Validated here, at load time, rather than left to
    # fail at request time with an opaque provider error.
    max_tokens_val = int(raw["max_tokens"])

    reasoning_effort = str(raw.get("reasoning_effort", _DEFAULT_REASONING_EFFORT))
    if reasoning_effort not in _VALID_REASONING_EFFORTS:
        raise ValueError(
            f"{config_path}: reasoning_effort must be one of "
            f"{sorted(_VALID_REASONING_EFFORTS)}, got {reasoning_effort!r}."
        )

    thinking_budget_tokens = int(
        raw.get("thinking_budget_tokens", _DEFAULT_THINKING_BUDGET_TOKENS)
    )
    if thinking_budget_tokens > max_tokens_val:
        raise ValueError(
            f"{config_path}: thinking_budget_tokens ({thinking_budget_tokens}) must not "
            f"exceed max_tokens ({max_tokens_val}) -- Anthropic's extended-thinking "
            f"budget is part of max_tokens, not additional to it, and this config "
            f"would otherwise fail at request time with an opaque provider error."
        )

    context_budget_tokens = int(
        raw.get("context_budget_tokens", _DEFAULT_CONTEXT_BUDGET_TOKENS)
    )
    if context_budget_tokens <= 0:
        raise ValueError(
            f"{config_path}: context_budget_tokens must be positive, got "
            f"{context_budget_tokens!r}."
        )

    context_keep_recent_turns = int(
        raw.get("context_keep_recent_turns", _DEFAULT_CONTEXT_KEEP_RECENT_TURNS)
    )
    if context_keep_recent_turns < 1:
        raise ValueError(
            f"{config_path}: context_keep_recent_turns must be at least 1, got "
            f"{context_keep_recent_turns!r}."
        )

    transcript_dir = resolve_relative(raw["transcript_dir"])
    transcript_dir.mkdir(parents=True, exist_ok=True)

    return AgentConfig(
        agent=raw["agent"],
        provider=raw["provider"],
        model=raw["model"],
        api_key_env=api_key_env,
        max_tokens=max_tokens_val,
        system_prompt=system_prompt,
        task_template=task_template,
        mcp_servers=servers,
        loop=loop,
        transcript_dir=transcript_dir,
        config_path=config_path,
        mcp_client_timeout_sec=mcp_client_timeout_sec,
        gate_timeout_sec=gate_timeout_sec,
        reasoning_effort=reasoning_effort,
        thinking_budget_tokens=thinking_budget_tokens,
        context_budget_tokens=context_budget_tokens,
        context_keep_recent_turns=context_keep_recent_turns,
    )
