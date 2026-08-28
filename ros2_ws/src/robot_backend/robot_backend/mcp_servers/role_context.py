"""Role identity, prompt rendering, and tool-call logging for Claude Code runs.

Background. In the run_agent.py path a single harness process rendered each
role's prompt, held the two roles apart by giving each its own MCP server set,
and wrote a per-agent transcript. Under Claude Code there is no such process:
the model is driven by the CLI, two sessions are launched by hand, and nothing
writes a transcript this repo can score. This module rebuilds the three pieces
that were lost, as server-side behaviour:

  * get_rendered_task (Phase 1) -- returns the role's system prompt and its
    rendered task, so a session can load them as opening context.
  * mcp_calls_<role>.jsonl (Phase 2) -- a passive record of every tool call,
    recovering the action trace the transcript used to hold.
  * the session guard -- refuses to run when the two sessions would resolve
    different session directories.

ROLE IS NOT A PARAMETER. It is derived from which task-path env var this
process was given, exactly as run_agent.py derived it from --config:

    WATCHDOG_TASK_PATH set -> this process serves the watchdog session
    EXECUTOR_TASK_PATH set -> this process serves the executor session

Both set, or neither, is a launch error and fails loudly. This matters because
the independence guarantee cannot rest on the agent behaving: a watchdog that
could ask for the executor's rendering would be able to read a task file whose
whole purpose is to differ from its own. Since role comes from the environment
and get_rendered_task is hosted only on servers that role's session loads,
neither role has any tool capable of naming the other's file.

Chain-of-thought is deliberately NOT captured -- only the tool-call surface.
"""

from __future__ import annotations

import functools
import json
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from robot_backend.watchdog import session as session_helper

WATCHDOG_ROLE = "watchdog"
EXECUTOR_ROLE = "executor"

_TASK_PATH_ENV = {
    WATCHDOG_ROLE: "WATCHDOG_TASK_PATH",
    EXECUTOR_ROLE: "EXECUTOR_TASK_PATH",
}
# Optional override for where a role's agent config lives. Defaults to
# agents/config/<role>_agent.yaml under the repo root, which is where
# run_agent.py's --config values point.
_CONFIG_PATH_ENV = {
    WATCHDOG_ROLE: "WATCHDOG_AGENT_CONFIG",
    EXECUTOR_ROLE: "EXECUTOR_AGENT_CONFIG",
}

_log_lock = threading.Lock()


# --------------------------------------------------------------------------
# role + repo resolution
# --------------------------------------------------------------------------


def detect_role() -> str:
    """Return this process's role, derived from the task-path env vars.

    Raises on both-set or neither-set rather than picking a default. A default
    here would silently hand one role the other's rendering on a launch
    mistake, which is the single failure this module exists to prevent.
    """
    present = [
        role for role, env in _TASK_PATH_ENV.items() if os.environ.get(env, "").strip()
    ]
    if len(present) == 1:
        return present[0]
    if not present:
        raise RuntimeError(
            "neither WATCHDOG_TASK_PATH nor EXECUTOR_TASK_PATH is set, so this "
            "server cannot tell which role's session it is serving. Launch via "
            "`claude --mcp-config <role>.mcp.json --strict-mcp-config`, whose "
            "env block sets exactly one of them (see RUN.md)."
        )
    raise RuntimeError(
        f"both {_TASK_PATH_ENV[WATCHDOG_ROLE]} and {_TASK_PATH_ENV[EXECUTOR_ROLE]} "
        f"are set. Exactly one must be, or the two roles are no longer isolated: "
        f"whichever server this is could render either role's task. Check that "
        f"the launching shell has no leftover export from a previous run."
    )


def _agents_dir() -> Path:
    """Path to the repo's agents/ directory, or raise with a usable message."""
    root = session_helper.repo_root()
    if root is None:
        raise RuntimeError(
            "cannot locate the WatchDog_V1 repo root (walked up from "
            f"{__file__} looking for a parent containing both agents/ and "
            "ros2_ws/). get_rendered_task needs the source tree for the "
            "prompts and task files; an install-only deployment cannot render."
        )
    agents = root / "agents"
    if not agents.is_dir():
        raise RuntimeError(f"repo root {root} has no agents/ directory")
    return agents


def _config_path_for(role: str) -> Path:
    """Locate the role's agent config yaml (env override, else convention)."""
    override = os.environ.get(_CONFIG_PATH_ENV[role], "").strip()
    if override:
        path = Path(os.path.expanduser(override))
        if not path.is_file():
            raise RuntimeError(
                f"{_CONFIG_PATH_ENV[role]}={override!r} does not name a file"
            )
        return path
    path = _agents_dir() / "config" / f"{role}_agent.yaml"
    if not path.is_file():
        raise RuntimeError(
            f"no agent config for role {role!r} at {path}. Set "
            f"{_CONFIG_PATH_ENV[role]} to point at it explicitly."
        )
    return path


# --------------------------------------------------------------------------
# session guard
# --------------------------------------------------------------------------


def resolve_session_dir_strict() -> tuple[Path, str]:
    """Resolve the session dir, refusing the 'fresh' rule.

    Under Claude Code the two sessions are separate `claude` processes with no
    parent to hand them a resolved path, so they agree only through the
    ~/.watchdog/current_session pointer the cell bringup writes. If that
    pointer is missing or dangling, resolve_session_dir() falls back to
    rule='fresh' and each session mints its OWN timestamped directory. Both
    roles then run, both look healthy, and their events land in two different
    directories -- the correlation the study rests on is gone, with nothing in
    any log saying so.

    That silent split is the reason this is a hard failure rather than a
    warning. 'fresh' here always means the operator skipped the bringup or is
    pointing at a deleted session, never a legitimate state for a scored run.
    """
    session_dir, rule = session_helper.resolve_session_dir()
    if rule == "fresh":
        pointer = Path.home() / ".watchdog" / "current_session"
        detail = "does not exist"
        try:
            raw = pointer.read_text(encoding="utf-8").strip()
            detail = f"names {raw!r}, which is not a directory"
        except Exception:
            pass
        raise RuntimeError(
            f"session directory would be resolved by rule='fresh' ({session_dir}). "
            f"The pointer {pointer} {detail}. Two Claude Code sessions resolving "
            f"'fresh' independently each create their OWN session directory, and "
            f"watchdog/executor correlation dies silently. Bring up the cell "
            f"(sim_moveit_bringup writes the pointer) before launching either "
            f"session, and do not export WATCHDOG_SESSION_DIR by hand."
        )
    return session_dir, rule


# --------------------------------------------------------------------------
# Phase 1 -- get_rendered_task
# --------------------------------------------------------------------------


def build_rendered_task(role: str) -> dict[str, Any]:
    """Render this role's system prompt + task. Returns them as SEPARATE fields.

    Kept separate rather than concatenated so the session can load the system
    prompt as system-level context and the task as the opening user turn,
    matching how run_agent.py passed them to the provider -- and so this works
    regardless of whether the deployment honours CLAUDE.md.

    Resolution goes through the same agent config run_agent.py loads, so the
    prompt/template paths are never hardcoded here. render_task keeps its hard
    failure on any surviving {{PLACEHOLDER}}: a literal placeholder reaching
    the model is a confidently wrong run that looks clean in the logs.
    """
    import yaml

    task_env = _TASK_PATH_ENV[role]
    task_path_raw = os.environ.get(task_env, "").strip()
    if not task_path_raw:
        raise RuntimeError(f"{task_env} is not set")
    task_path = Path(os.path.expanduser(task_path_raw))
    if not task_path.is_file():
        raise RuntimeError(f"{task_env}={task_path_raw!r} does not name a file")

    agents_dir = _agents_dir()
    if str(agents_dir) not in sys.path:
        sys.path.insert(0, str(agents_dir))
    from agent_harness.config import load_config
    from agent_harness.task_render import render_task

    # require_api_key=False: the Claude Code path has no provider key by
    # construction. base_dir: the config's relative prompt paths are written
    # relative to agents/, and this process's cwd is whatever terminal
    # launched the session.
    config = load_config(
        _config_path_for(role), require_api_key=False, base_dir=agents_dir
    )
    if config.agent != role:
        raise RuntimeError(
            f"config {_config_path_for(role)} declares agent={config.agent!r} "
            f"but was loaded for role {role!r}. Refusing to render a prompt "
            f"whose role does not match this process's task-path env var."
        )

    system_prompt = Path(config.system_prompt).read_text(encoding="utf-8")
    template_text = Path(config.task_template).read_text(encoding="utf-8")
    with open(task_path) as f:
        task = yaml.safe_load(f)

    task_text = render_task(template_text, task)

    session_dir, session_rule = resolve_session_dir_strict()

    return {
        "ok": True,
        "command": "get_rendered_task",
        "role": role,
        "system_prompt": system_prompt,
        "task": task_text,
        "sources": {
            "agent_config": str(_config_path_for(role)),
            "system_prompt_path": str(config.system_prompt),
            "task_template_path": str(config.task_template),
            "task_path": str(task_path),
            "task_path_env": task_env,
        },
        "session_dir": str(session_dir),
        "session_rule": session_rule,
        "instructions": (
            "Load system_prompt as your standing instructions and task as your "
            "opening task, before calling any other tool."
        ),
    }


# --------------------------------------------------------------------------
# Phase 2 -- mcp_calls_<role>.jsonl
# --------------------------------------------------------------------------


def _call_log_path() -> Path | None:
    """Path to this role's tool-call log, or None if it cannot be determined.

    Never raises: logging is an observability side-channel, and a run must not
    die because its log path could not be resolved. A None here degrades to
    "no call log", which the session guard and the ROS-side recording will
    both make obvious anyway.
    """
    try:
        role = detect_role()
    except Exception:
        return None
    try:
        session_dir, _ = session_helper.resolve_session_dir()
    except Exception:
        return None
    return session_dir / f"mcp_calls_{role}.jsonl"


def _redact(value: Any, _depth: int = 0) -> Any:
    """Make tool arguments JSON-safe without dropping their content."""
    if _depth > 6:
        return "<max-depth>"
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _redact(v, _depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_redact(v, _depth + 1) for v in value]
    return repr(value)


def _append_call_record(record: dict[str, Any]) -> None:
    """Append one JSON line to mcp_calls_<role>.jsonl. Never raises.

    Opened per write in append mode rather than held open, so the three
    executor server processes can share one file. Verified safe: concurrent
    O_APPEND writes from separate processes to a local filesystem do not
    interleave, tested up to 100 KB lines. The lock below only serialises
    threads WITHIN one process (FastMCP may serve calls on several).
    """
    path = _call_log_path()
    if path is None:
        return
    try:
        line = json.dumps(record, default=str)
    except Exception:
        return
    try:
        with _log_lock:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "a", encoding="utf-8") as fh:
                fh.write(line + "\n")
    except Exception:
        # Deliberately silent: a failed log write must never surface as a tool
        # error to the agent, which would change the run it is recording.
        pass


def log_tool_call(server_name: str, func: Callable) -> Callable:
    """Wrap one MCP tool so every call is recorded.

    wall_time_iso is the cross-process correlation key -- the same basis the
    ROS-side events_*.jsonl use -- so a call log and an event log from
    different processes align by time with no shared sequence number.
    """

    @functools.wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        started = time.monotonic()
        wall_time_iso = datetime.now(timezone.utc).isoformat()
        record: dict[str, Any] = {
            "wall_time_iso": wall_time_iso,
            "server": server_name,
            "tool": getattr(func, "__name__", "?"),
            "arguments": _redact(kwargs) if kwargs else _redact(list(args)),
            "pid": os.getpid(),
        }
        try:
            result = func(*args, **kwargs)
        except Exception as exc:
            record["elapsed_sec"] = round(time.monotonic() - started, 6)
            record["ok"] = False
            record["error"] = f"{type(exc).__name__}: {exc}"
            _append_call_record(record)
            raise
        record["elapsed_sec"] = round(time.monotonic() - started, 6)
        # These servers return {"ok": bool, ...} envelopes and convert their
        # own exceptions into ok=false payloads, so the envelope -- not the
        # absence of a raise -- is what says whether the call succeeded.
        if isinstance(result, dict) and "ok" in result:
            record["ok"] = bool(result["ok"])
            if not record["ok"]:
                record["error"] = result.get("error")
        else:
            record["ok"] = True
        _append_call_record(record)
        return result

    return wrapper


def install_call_logging(mcp: Any, server_name: str) -> None:
    """Patch mcp.tool() so every tool registered afterwards is logged.

    Done by wrapping the registration decorator rather than by editing ~45
    tool definitions across four servers: one call per server, and a tool
    added later is covered automatically instead of being silently missed.
    functools.wraps preserves __name__/__doc__/__annotations__ and sets
    __wrapped__, so FastMCP still derives the same schema from the signature.

    Must be called BEFORE the tool definitions are registered.
    """
    if mcp is None:
        return
    original_tool = mcp.tool

    def patched_tool(*t_args: Any, **t_kwargs: Any):
        decorator = original_tool(*t_args, **t_kwargs)

        def register(func: Callable):
            return decorator(log_tool_call(server_name, func))

        return register

    mcp.tool = patched_tool  # type: ignore[method-assign]


__all__ = [
    "WATCHDOG_ROLE",
    "EXECUTOR_ROLE",
    "detect_role",
    "resolve_session_dir_strict",
    "build_rendered_task",
    "install_call_logging",
    "log_tool_call",
]
