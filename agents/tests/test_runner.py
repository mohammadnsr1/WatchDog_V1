"""Runner return-path test: run() must hand back a RunResult.

Guards the exact regression that the finally-block assert cannot catch: the
run completes its finally (writes run_end, closes the transcript) and then
raises on `return RunResult(...)` if RunResult is undefined. Only a test that
exercises the return closes it.

No live provider, no cell, no API key: get_provider and MCPClientPool are
stubbed on the runner module. Runs under pytest, and also standalone via
`python3 tests/test_runner.py` for environments without pytest installed.
"""

from __future__ import annotations

import asyncio
import copy
import contextlib
import json
from pathlib import Path
from types import SimpleNamespace

from agent_harness import runner
from agent_harness.providers import ProviderResponse, ReasoningTrace, ToolCall


class _StubProvider:
    """Returns text with no tool calls, so run() reaches agent_idle in two turns."""

    def complete(self, system_prompt, messages, tool_dicts):
        return ProviderResponse(
            text="nothing to do",
            reasoning=None,
            tool_calls=[],
            stop_reason="end_turn",
            usage={},
            provider_native=None,
        )


class _StubPool:
    """Async-context-manager stand-in for MCPClientPool with no servers."""

    def __init__(self, servers, timeout_sec):
        self._servers = servers
        self._timeout_sec = timeout_sec

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return None

    async def list_tools(self):
        return []

    async def call_tool(self, name, arguments):  # pragma: no cover - never called
        raise AssertionError("stub pool should not be asked to call a tool")


def _stub_config(tmp_dir: Path) -> SimpleNamespace:
    system_prompt = tmp_dir / "system.txt"
    system_prompt.write_text("stub system prompt", encoding="utf-8")
    return SimpleNamespace(
        agent="watchdog",
        provider="stub",
        model="stub-model",
        api_key="unused",
        max_tokens=256,
        reasoning_effort="medium",
        thinking_budget_tokens=64,
        context_budget_tokens=150000,
        context_keep_recent_turns=5,
        system_prompt=system_prompt,
        task_template=tmp_dir / "task_template.txt",
        transcript_dir=tmp_dir / "transcripts",
        mcp_servers=[],
        mcp_client_timeout_sec=30.0,
        loop=SimpleNamespace(
            max_turns=10,
            wait_timeout_sec=None,
            stop_after_consecutive_timeouts=None,
        ),
    )


@contextlib.contextmanager
def _stubbed_runner():
    orig_provider = runner.get_provider
    orig_pool = runner.MCPClientPool
    runner.get_provider = lambda *a, **k: _StubProvider()
    runner.MCPClientPool = _StubPool
    try:
        yield
    finally:
        runner.get_provider = orig_provider
        runner.MCPClientPool = orig_pool


def _run_once(tmp_dir: Path):
    config = _stub_config(tmp_dir)
    with _stubbed_runner():
        return asyncio.run(runner.run(config, "do the task", "stub system prompt"))


def test_run_returns_runresult(tmp_path):
    result = _run_once(tmp_path)
    assert isinstance(result, runner.RunResult)
    assert result.termination_reason in runner.TERMINATION_REASONS
    # Two tool-callless turns drive the agent to idle; the point of the test
    # is the return path, so assert the concrete reason too.
    assert result.termination_reason == "agent_idle"
    assert result.turn_count >= 1
    assert Path(result.transcript_path).exists()


def test_runresult_is_constructible():
    r = runner.RunResult("run_complete", 3, "/tmp/x.jsonl", 2)
    assert r.termination_reason == "run_complete"
    assert r.verdict_count == 2
    # verdict_count is optional.
    r2 = runner.RunResult("agent_idle", 1, "/tmp/y.jsonl")
    assert r2.verdict_count is None


class _ElisionRecorder:
    def __init__(self):
        self.records = []

    def context_elision(self, **fields):
        self.records.append(fields)


def test_elision_only_replaces_old_tool_results() -> None:
    opaque = {"type": "reasoning", "encrypted_content": "opaque-provider-blob"}
    arguments = {"channels": ["joint_states"], "max_samples": 200}
    recent_result = {"samples": ["r" * 12000]}
    messages = [
        {"role": "user", "content": "task"},
        {
            "role": "assistant",
            "content": None,
            "provider_native": [opaque],
        },
        {
            "role": "tool_result",
            "tool_call_id": "old",
            "name": "get_evidence",
            "content": {"ok": True, "samples": ["x" * 12000]},
        },
        {
            "role": "tool_result",
            "tool_call_id": "recent",
            "name": "get_evidence",
            "content": recent_result,
        },
    ]
    non_tool_before = copy.deepcopy(messages[:2])
    recorder = _ElisionRecorder()

    count = runner._elide_tool_results(
        messages,
        {2: (1, arguments), 3: (6, {"channels": ["frames"]})},
        current_turn=7,
        keep_recent_turns=5,
        context_budget_tokens=1,
        transcript=recorder,
    )

    assert count == 1
    assert messages[:2] == non_tool_before
    stub = messages[2]["content"]
    assert stub["elided"] is True
    assert stub["tool"] == "get_evidence"
    assert stub["arguments"] == arguments
    assert stub["summary"]["ok"] is True
    assert "Call this tool again with the same arguments" in stub["note"]
    assert messages[3]["content"] is recent_result
    assert recorder.records[0]["elided_bytes"] > 10000
    assert recorder.records[0]["estimated_tokens_after"] < recorder.records[0][
        "estimated_tokens_before"
    ]


_EVIDENCE_ARGUMENTS = {
    "start_wall_ns": 100,
    "end_wall_ns": 200,
    "channels": ["joint_states"],
    "joint_names": ["wrist_3_joint"],
}
_FULL_EVIDENCE = {
    "ok": True,
    "command": "get_evidence",
    "evidence": {
        "joint_states": [
            {"wall_ns": 150, "positions": {"wrist_3_joint": 1.234}}
        ],
        "padding": "x" * 20000,
    },
}


class _ContextLengthError(RuntimeError):
    status_code = 400


class _RecoveryProvider:
    def __init__(self):
        self.calls = 0
        self.snapshots = []

    def complete(self, system_prompt, messages, tool_dicts):
        self.calls += 1
        self.snapshots.append(copy.deepcopy(messages))
        if self.calls == 1:
            return ProviderResponse(
                text=None,
                reasoning=ReasoningTrace("initial reasoning", 10),
                tool_calls=[ToolCall("evidence-1", "get_evidence", _EVIDENCE_ARGUMENTS)],
                stop_reason="tool_call",
                usage={},
                provider_native=[
                    {"type": "reasoning", "encrypted_content": "opaque-one"}
                ],
            )
        if self.calls == 2:
            return ProviderResponse(
                text="considering evidence",
                reasoning=ReasoningTrace("second reasoning", 11),
                tool_calls=[],
                stop_reason="end_turn",
                usage={},
                provider_native=[
                    {"type": "reasoning", "encrypted_content": "opaque-two"}
                ],
            )
        if self.calls == 3:
            raise _ContextLengthError(
                "Error code: 400 - Your input exceeds the context window of this model."
            )
        if self.calls == 4:
            old_result = next(
                message
                for message in messages
                if message.get("role") == "tool_result"
                and message.get("tool_call_id") == "evidence-1"
            )
            assert old_result["content"]["elided"] is True
            assert old_result["content"]["arguments"] == _EVIDENCE_ARGUMENTS
            assert any(
                item.get("encrypted_content") == "opaque-one"
                for message in messages
                if message.get("role") == "assistant"
                for item in (message.get("provider_native") or [])
            )
            return ProviderResponse(
                text=None,
                reasoning=ReasoningTrace("recovered reasoning", 12),
                tool_calls=[ToolCall("evidence-2", "get_evidence", _EVIDENCE_ARGUMENTS)],
                stop_reason="tool_call",
                usage={},
                provider_native=[
                    {"type": "reasoning", "encrypted_content": "opaque-three"}
                ],
            )
        if self.calls == 5:
            results = [
                message["content"]
                for message in messages
                if message.get("role") == "tool_result"
            ]
            assert results[0]["elided"] is True
            assert results[-1] == _FULL_EVIDENCE
            return ProviderResponse(
                text="evidence restored",
                reasoning=ReasoningTrace("post-elision reasoning", 13),
                tool_calls=[],
                stop_reason="end_turn",
                usage={},
                provider_native=None,
            )
        return ProviderResponse(
            text="done",
            reasoning=ReasoningTrace("final reasoning", 14),
            tool_calls=[],
            stop_reason="end_turn",
            usage={},
            provider_native=None,
        )


class _RecoveryPool(_StubPool):
    calls = []

    async def call_tool(self, name, arguments):
        self.__class__.calls.append((name, copy.deepcopy(arguments)))
        return copy.deepcopy(_FULL_EVIDENCE)


def test_context_overflow_elides_retries_and_preserves_reasoning(tmp_path) -> None:
    config = _stub_config(tmp_path)
    config.context_budget_tokens = 10_000_000
    config.context_keep_recent_turns = 1
    provider = _RecoveryProvider()
    _RecoveryPool.calls = []
    original_provider = runner.get_provider
    original_pool = runner.MCPClientPool
    runner.get_provider = lambda *a, **k: provider
    runner.MCPClientPool = _RecoveryPool
    try:
        result = asyncio.run(runner.run(config, "do the task", "stub system prompt"))
    finally:
        runner.get_provider = original_provider
        runner.MCPClientPool = original_pool

    assert result.termination_reason == "agent_idle"
    assert [arguments for _, arguments in _RecoveryPool.calls] == [
        _EVIDENCE_ARGUMENTS,
        _EVIDENCE_ARGUMENTS,
    ]
    records = [
        json.loads(line)
        for line in Path(result.transcript_path).read_text(encoding="utf-8").splitlines()
    ]
    assert any(record["type"] == "context_elision" for record in records)
    recovery = next(
        record for record in records if record["type"] == "context_overflow_recovery"
    )
    assert recovery["retry"] == 1
    assert recovery["elision_count"] == 1
    assert any(
        record["type"] == "llm_response"
        and record["turn"] >= recovery["turn"]
        and (record.get("reasoning") or {}).get("summary")
        in {"recovered reasoning", "post-elision reasoning", "final reasoning"}
        for record in records
    )


if __name__ == "__main__":
    import tempfile

    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            if "tmp_path" in fn.__code__.co_varnames[: fn.__code__.co_argcount]:
                with tempfile.TemporaryDirectory() as td:
                    fn(Path(td))
            else:
                fn()
            print(f"PASS {name}")
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"FAIL {name}: {type(exc).__name__}: {exc}")
    raise SystemExit(1 if failures else 0)
