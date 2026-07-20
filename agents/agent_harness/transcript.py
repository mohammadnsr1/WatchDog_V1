"""JSONL transcript writer.

One record per line, flushed on write so a crashed or halted run stays
readable. Every record carries wall_ns -- for WatchDog this is the evidence
that the halt path never waited on the agent; for both agents it is what lets
a paired executor/WatchDog run be correlated after the fact.
"""

from __future__ import annotations

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
        config: dict,
        agent: str,
        provider: str,
        model: str,
        task_path: str,
        git_commit: str | None,
    ) -> None:
        self._write(
            "run_start",
            config=_redact(config),
            agent=agent,
            provider=provider,
            model=model,
            task_path=task_path,
            git_commit=git_commit,
        )

    def tools_discovered(self, tools: list[dict]) -> None:
        self._write("tools_discovered", tools=tools)

    def llm_request(self, turn: int, system: str, messages: list) -> None:
        self._write("llm_request", turn=turn, system=system, messages=_jsonable(messages))

    def llm_response(
        self,
        turn: int,
        text: str | None,
        tool_calls: list,
        stop_reason: str,
        raw: dict,
        usage: dict,
    ) -> None:
        self._write(
            "llm_response",
            turn=turn,
            text=text,
            tool_calls=_jsonable(tool_calls),
            stop_reason=stop_reason,
            raw=raw,
            usage=usage,
        )

    def tool_call(self, turn: int, name: str, arguments: dict, wall_ns_taken: int) -> None:
        self._write(
            "tool_call", turn=turn, name=name, arguments=arguments, wall_ns_taken=wall_ns_taken
        )

    def tool_result(
        self, turn: int, name: str, result: Any, wall_ns_after: int, elapsed_ns: int
    ) -> None:
        self._write(
            "tool_result",
            turn=turn,
            name=name,
            result=result,
            wall_ns_after=wall_ns_after,
            elapsed_ns=elapsed_ns,
        )

    def run_end(
        self, termination_reason: str, turn_count: int, verdict_count: int | None = None
    ) -> None:
        fields: dict[str, Any] = {
            "termination_reason": termination_reason,
            "turn_count": turn_count,
        }
        if verdict_count is not None:
            fields["verdict_count"] = verdict_count
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
