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
import os
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agent_harness.timebase import wall_ns

_REDACT_KEY_RE = re.compile(r"key|token|secret|password", re.IGNORECASE)


def _resolved_transcript_dir(fallback: Path) -> Path:
    """The session dir when one has been resolved, else the configured fallback.

    run_agent.py resolves the session and exports WATCHDOG_SESSION_DIR before
    the runner constructs any TranscriptWriter, so this run's transcripts land
    in the same directory as the cell's bag, events, and manifest. With no
    session resolvable (an agent run with no cell up), falls back to
    config.transcript_dir exactly as before -- that path must keep working.
    """
    session = os.environ.get("WATCHDOG_SESSION_DIR")
    if session and session.strip():
        return Path(os.path.expanduser(session.strip()))
    return fallback


# Opaque provider blobs that must still be sent to the provider on every turn
# (reasoning continuity across tool-call turns depends on them -- see
# providers.py's provider_native threading) but carry no audit value on disk
# and are large enough to dominate a transcript: OpenAI reasoning items'
# encrypted_content, Anthropic's redacted_thinking content blocks' data.
_OPAQUE_BLOB_KEY = "encrypted_content"
_REDACTED_THINKING_TYPE = "redacted_thinking"


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


def _strip_opaque_blobs(obj: Any) -> Any:
    """Recursively drop encrypted/opaque reasoning blobs from a message tree.

    Builds an entirely new nested structure rather than mutating obj in
    place at any level, so this is safe to call even when obj shares
    references with the live `messages` list the runner is about to send to
    the provider on the next turn -- stripping here can never affect what
    goes out on the wire.
    """
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if k == _OPAQUE_BLOB_KEY and v:
                out[k] = f"<stripped: {len(str(v))} chars, not persisted>"
            elif k == "data" and obj.get("type") == _REDACTED_THINKING_TYPE and v:
                out[k] = f"<stripped: {len(str(v))} chars, not persisted>"
            else:
                out[k] = _strip_opaque_blobs(v)
        return out
    if isinstance(obj, list):
        return [_strip_opaque_blobs(v) for v in obj]
    return obj


def hash_file(path: Path) -> str:
    """sha256 of a file's bytes, as a hex string."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


_DEFAULT_SIDECAR_THRESHOLD_BYTES = 8192


def _summarize(payload: Any) -> Any:
    """Collapse a payload to its shape for the on-disk record when the full
    payload has been sidecared: top-level scalars (e.g. ok, command) kept
    as-is, every list or dict value replaced by its length or key count.
    Generic on purpose -- no per-tool summariser to maintain.
    """
    if isinstance(payload, dict):
        return {
            k: (f"<{type(v).__name__} len={len(v)}>" if isinstance(v, (dict, list)) else v)
            for k, v in payload.items()
        }
    if isinstance(payload, list):
        return f"<list len={len(payload)}>"
    return payload


class TranscriptWriter:
    def __init__(
        self,
        transcript_dir: Path,
        agent: str,
        task_name: str,
        sidecar_threshold_bytes: int = _DEFAULT_SIDECAR_THRESHOLD_BYTES,
    ):
        transcript_dir = _resolved_transcript_dir(transcript_dir)
        transcript_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        stem = f"{agent}_{task_name}_{ts}"
        self.path = transcript_dir / f"{stem}.jsonl"
        self._fh = open(self.path, "a", buffering=1)
        self._sidecar_dir = transcript_dir / f"{stem}_results"
        self._sidecar_threshold_bytes = sidecar_threshold_bytes
        self._sidecar_seq = 0
        # content-hash -> {"result_ref", "result_summary"}. A tool result
        # that recurs verbatim -- once in its own tool_result record, again
        # inside the following turn's llm_request.new_messages, since a tool
        # result becomes a message in the history the provider is resent --
        # is sidecared once; both records reference the same file rather
        # than each writing their own copy.
        self._sidecar_cache: dict[str, dict[str, Any]] = {}

    def _write(self, record_type: str, **fields: Any) -> None:
        record = {"type": record_type, "wall_ns": wall_ns(), **fields}
        self._fh.write(json.dumps(record, default=str) + "\n")
        self._fh.flush()

    def _maybe_sidecar(
        self, payload: Any, turn: int, tool_name: str
    ) -> dict[str, Any] | None:
        """Write payload to a sidecar file and return its ref/summary if it
        exceeds the size threshold, else return None (caller keeps payload
        inline). Content-addressed: the same payload seen twice (see the
        cache note in __init__) is written once and both callers get the
        same result_ref.
        """
        try:
            serialized = json.dumps(payload, default=str)
        except Exception:
            return None
        if len(serialized.encode("utf-8")) <= self._sidecar_threshold_bytes:
            return None
        key = hashlib.sha256(serialized.encode("utf-8")).hexdigest()
        cached = self._sidecar_cache.get(key)
        if cached is not None:
            return cached
        self._sidecar_dir.mkdir(parents=True, exist_ok=True)
        self._sidecar_seq += 1
        sidecar_path = self._sidecar_dir / f"{turn}_{self._sidecar_seq}_{tool_name}.json"
        with open(sidecar_path, "w") as f:
            f.write(serialized)
        ref = {
            "result_ref": str(sidecar_path.relative_to(self.path.parent)),
            "result_summary": _summarize(payload),
        }
        self._sidecar_cache[key] = ref
        return ref

    def _sidecar_tool_result_messages(self, messages: list, turn: int) -> list:
        """Shallow-copy new_messages, substituting result_ref/result_summary
        for any tool-result message's content that exceeds the sidecar
        threshold -- the same substitution tool_result() applies, so a large
        payload embedded in this turn's history is not written to disk a
        second time in full. Redacted the same way tool_result() redacts, so
        the content hash used for dedup matches regardless of which call site
        reaches a given payload first.

        Builds a new list/dicts throughout; never mutates a message object
        that runner.py's live `messages` list still holds a reference to, so
        this can never affect what is sent to the provider on the next turn.
        """
        out = []
        for msg in messages:
            if isinstance(msg, dict) and msg.get("role") == "tool_result" and "content" in msg:
                redacted = _redact(msg["content"])
                ref = self._maybe_sidecar(redacted, turn, str(msg.get("name", "unknown")))
                new_msg = {k: v for k, v in msg.items() if k != "content"}
                if ref is not None:
                    new_msg["result_ref"] = ref["result_ref"]
                    new_msg["result_summary"] = ref["result_summary"]
                else:
                    new_msg["content"] = redacted
                out.append(new_msg)
            else:
                out.append(msg)
        return out

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

    def llm_request(self, turn: int, new_messages: list, tool_count: int) -> None:
        """Log only the messages appended to the conversation since the
        previous turn's llm_request, not the full running history.

        Every turn still resends the full messages array to the provider
        (see runner.py/providers.py) -- that repetition is required by the
        stateless API and is unrelated to this. Only the on-disk record
        stops repeating message 1 inside every subsequent turn's record.

        To reconstruct the full context as of any turn N, concatenate
        new_messages from every llm_request record with turn <= N, in
        order. Turn 1's new_messages is the initial task message; turn 2's
        is whatever was appended (assistant response, tool result(s)) after
        turn 1; and so on.

        Reasoning items echoed back for provider continuity (see
        providers.py's provider_native) carry opaque encrypted blobs
        (OpenAI's encrypted_content, Anthropic's redacted_thinking data)
        that must keep going out on the wire every turn but have no reason
        to live on disk -- they are stripped here, not from the outbound
        payload, and never from the reasoning summaries themselves.

        A tool-result message in new_messages carries the exact payload
        already written by a prior tool_result() call (a tool result becomes
        a message in the history, resent every turn) -- large ones are
        sidecared here the same way, so a large get_evidence payload is not
        stored twice in full. See _sidecar_tool_result_messages.
        """
        sidecared = self._sidecar_tool_result_messages(new_messages, turn)
        self._write(
            "llm_request",
            turn=turn,
            new_messages=_strip_opaque_blobs(_jsonable(sidecared)),
            tool_count=tool_count,
        )

    def llm_response(
        self,
        turn: int,
        text: str | None,
        reasoning: Any,
        tool_calls: list,
        stop_reason: str,
        usage: dict,
    ) -> None:
        """reasoning is a providers.ReasoningTrace (summary, tokens), or None.

        Only the extracted summary text and token count are recorded here --
        never the full raw provider payload. summary is None whenever the
        provider returned no reasoning content; that is a normal outcome
        (the API may withhold it), not something this call treats as an
        error.
        """
        self._write(
            "llm_response",
            turn=turn,
            text=text,
            reasoning=_jsonable(reasoning),
            tool_calls=_jsonable(tool_calls),
            stop_reason=stop_reason,
            usage=usage,
        )

    def tool_call(self, turn: int, name: str, arguments: dict) -> None:
        self._write("tool_call", turn=turn, name=name, arguments=_redact(arguments))

    def tool_result(self, turn: int, name: str, result: Any, elapsed_ns: int) -> None:
        """Large results (default over 8192 bytes serialized, e.g.
        get_evidence) are sidecared to a file next to the transcript rather
        than written inline; the record then carries result_ref and a
        generic result_summary instead of result. Never truncated or
        silently dropped -- the full payload is always on disk, just not
        duplicated inline in the transcript. See _maybe_sidecar.
        """
        redacted = _redact(result)
        ref = self._maybe_sidecar(redacted, turn, name)
        fields: dict[str, Any] = {"turn": turn, "name": name, "elapsed_ns": elapsed_ns}
        if ref is not None:
            fields["result_ref"] = ref["result_ref"]
            fields["result_summary"] = ref["result_summary"]
        else:
            fields["result"] = redacted
        self._write("tool_result", **fields)

    def context_elision(
        self,
        turn: int,
        tool: str,
        elided_bytes: int,
        estimated_tokens_before: int,
        estimated_tokens_after: int,
    ) -> None:
        """Record one outbound-history tool-result elision."""
        self._write(
            "context_elision",
            turn=turn,
            tool=tool,
            elided_bytes=elided_bytes,
            estimated_tokens_before=estimated_tokens_before,
            estimated_tokens_after=estimated_tokens_after,
        )

    def context_overflow_recovery(
        self,
        turn: int,
        estimated_tokens_before: int,
        estimated_tokens_after: int,
        elision_count: int,
    ) -> None:
        """Record the single aggressive retry after a context-length error."""
        self._write(
            "context_overflow_recovery",
            turn=turn,
            retry=1,
            estimated_tokens_before=estimated_tokens_before,
            estimated_tokens_after=estimated_tokens_after,
            elision_count=elision_count,
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
