"""JSONL transcript writer.

One record per line, flushed on write so a crashed or halted run stays
readable. Every record carries wall_ns -- for WatchDog this is the evidence
that the halt path never waited on the agent; for both agents it is what lets
a paired executor/WatchDog run be correlated after the fact.

This is the agent's record of what it decided and when -- a complement to the
cell's /watchdog/events JSONL, not a duplicate of it. It deliberately does not
carry git_commit, task_path, a full config echo, a command line, or the
system prompt text itself (repeated on every turn, 3.6KB at a time, when it
already lives on disk in prompts/) -- only a hash of the system prompt file,
so a run can be tied to the prompt version that produced it without storing
the prompt.
"""

from __future__ import annotations

import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agent_harness.timebase import wall_ns

_REDACT_KEY_RE = re.compile(r"key|token|secret|password", re.IGNORECASE)


def _redact(obj: Any) -> Any:
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if _REDACT_KEY_RE.search(str(k)):
                out[k] = "***REDACTED***"
            else:
                out[k] = _redact(v)
        return out
    if isinstance(obj, list):
        return [_redact(v) for v in obj]
    return obj


def hash_file(path: Path) -> str:
    """sha256 of a file's bytes, as a hex string."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


class TranscriptWriter:
    def __init__(self, transcript_dir: Path, agent: str, task_name: str):
        transcript_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.path = transcript_dir / f"{agent}_{task_name}_{ts}.jsonl"
        self._fh = open(self.path, "a", buffering=1)

    def _write(self, record_type: str, **fields: Any) -> None:
        record = {"type": record_type, "wall_ns": wall_ns(), **fields}
        self._fh.write(json.dumps(record, default=str) + "\n")
        self._fh.flush()

    def run_start(
        self,
        agent: str,
        provider: str,
        model: str,
        task_name: str,
        system_prompt_sha256: str,
    ) -> None:
        self._write(
            "run_start",
            agent=agent,
            provider=provider,
            model=model,
            task_name=task_name,
            system_prompt_sha256=system_prompt_sha256,
        )

    def tools_discovered(self, tools: list[dict]) -> None:
        self._write("tools_discovered", tools=tools)

    def llm_request(self, turn: int, messages: list) -> None:
        self._write("llm_request", turn=turn, messages=_jsonable(messages))

    def llm_response(
        self,
        turn: int,
        text: str | None,
        tool_calls: list,
        stop_reason: str,
        usage: dict,
    ) -> None:
        self._write(
            "llm_response",
            turn=turn,
            text=text,
            tool_calls=_jsonable(tool_calls),
            stop_reason=stop_reason,
            usage=usage,
        )

    def tool_call(self, turn: int, name: str, arguments: dict) -> None:
        self._write("tool_call", turn=turn, name=name, arguments=_redact(arguments))

    def tool_result(self, turn: int, name: str, result: Any, elapsed_ns: int) -> None:
        self._write(
            "tool_result",
            turn=turn,
            name=name,
            result=_redact(result),
            elapsed_ns=elapsed_ns,
        )

    def run_end(
        self,
        termination_reason: str,
        turn_count: int,
        verdict_count: int | None = None,
        error: dict | None = None,
    ) -> None:
        fields: dict[str, Any] = {
            "termination_reason": termination_reason,
            "turn_count": turn_count,
        }
        if verdict_count is not None:
            fields["verdict_count"] = verdict_count
        if error is not None:
            fields["error"] = error
        self._write("run_end", **fields)

    def close(self) -> None:
        self._fh.close()


def _jsonable(obj: Any) -> Any:
    if hasattr(obj, "__dataclass_fields__"):
        from dataclasses import asdict

        return asdict(obj)
    if isinstance(obj, list):
        return [_jsonable(x) for x in obj]
    if isinstance(obj, dict):
        return {k: _jsonable(v) for k, v in obj.items()}
    return obj
