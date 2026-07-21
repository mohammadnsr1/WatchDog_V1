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
    wait_timeout_sec: float
    stop_after_consecutive_timeouts: int


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

    loop_raw = raw["loop"]
    loop = LoopConfig(
        max_turns=int(loop_raw["max_turns"]),
        wait_timeout_sec=float(loop_raw["wait_timeout_sec"]),
        stop_after_consecutive_timeouts=int(
            loop_raw["stop_after_consecutive_timeouts"]
        ),
    )

    transcript_dir = resolve_relative(raw["transcript_dir"])
    transcript_dir.mkdir(parents=True, exist_ok=True)

    return AgentConfig(
        agent=raw["agent"],
        provider=raw["provider"],
        model=raw["model"],
        api_key_env=api_key_env,
        max_tokens=int(raw["max_tokens"]),
        system_prompt=system_prompt,
        task_template=task_template,
        mcp_servers=servers,
        loop=loop,
        transcript_dir=transcript_dir,
        config_path=config_path,
    )
