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
import contextlib
from pathlib import Path
from types import SimpleNamespace

from agent_harness import runner
from agent_harness.providers import ProviderResponse


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
