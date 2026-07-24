"""The runner: identical for both agents, no agent-specific logic.

It connects to the agent's configured MCP servers, discovers tools, loops the
provider against those tools, and records everything. It does not know about
monitors, verdicts, skills, or steps -- it is a conduit and a recorder only.

The one piece of cross-cutting behavior is the wait_for_event timeout rule,
which exists because it is how a WatchDog run terminates once WatchDog has
run out of things to watch for. The executor has no such tool and simply
never takes that path (see agent_harness.mcp_client.WAIT_FOR_EVENT_TOOL).

Termination-reason discipline (see the B1 postmortem this file was rewritten
against): every path that can end the run either (a) records the response/
result that was actually received before checking anything else, or (b) is
driven by an explicit exception handler that classifies the failure and
carries the exception type/message into run_end. There is no default value
that a fallthrough can leave in place -- termination_reason starts at None and
the loop condition itself (`while termination_reason is None`) is what stops
it from ever masquerading as something it isn't.
"""

from __future__ import annotations

import traceback
from dataclasses import dataclass
from typing import Any

from agent_harness.config import AgentConfig
from agent_harness.mcp_client import MCPClientPool, WAIT_FOR_EVENT_TOOL
from agent_harness.providers import ProviderResponse, get_provider
from agent_harness.timebase import wall_ns
from agent_harness.transcript import TranscriptWriter, hash_file

NUDGE_MESSAGE = (
    "You have not taken any action. If your work is complete, say so plainly. "
    "Otherwise, continue."
)

TERMINATION_REASONS = {
    "run_complete",
    "no_execution_observed",
    "max_turns_exhausted",
    "agent_idle",
    "provider_error",
    "mcp_error",
    "user_abort",
}


@dataclass
class RunResult:
    termination_reason: str
    turn_count: int
    transcript_path: str
    verdict_count: int | None = None


_TRACEBACK_FRAME_LIMIT = 20


def _exception_detail(e: BaseException) -> dict:
    """Recursively describe an exception, including any nested exception
    group members and __cause__ chain, each with a truncated traceback.

    A KeyboardInterrupt raised inside an anyio/asyncio TaskGroup surfaces
    here as an ExceptionGroup whose sub-exceptions carry the real cause --
    without recursing into `exceptions`, an operator Ctrl-C is
    indistinguishable on the record from a genuine transport failure. See
    _contains_keyboard_interrupt, which walks the same structure to decide
    termination_reason.
    """
    detail: dict[str, Any] = {
        "exception_type": type(e).__name__,
        "exception_message": str(e),
        "traceback": traceback.format_exception(
            type(e), e, e.__traceback__, limit=_TRACEBACK_FRAME_LIMIT
        ),
    }
    subs = getattr(e, "exceptions", None)
    if subs:
        detail["sub_exceptions"] = [_exception_detail(sub) for sub in subs]
    if e.__cause__ is not None:
        detail["cause"] = _exception_detail(e.__cause__)
    return detail


def _contains_keyboard_interrupt(e: BaseException) -> bool:
    """True if e, or anything nested inside it (exception group members or
    the __cause__ chain), is a KeyboardInterrupt.

    Needed because an operator Ctrl-C during an `async with` block (e.g.
    MCPClientPool's TaskGroup) does not arrive here as a bare
    KeyboardInterrupt -- it is wrapped in an ExceptionGroup, whose top-level
    type is just "ExceptionGroup" with no indication of what is inside
    unless this recurses.
    """
    if isinstance(e, KeyboardInterrupt):
        return True
    for sub in getattr(e, "exceptions", None) or []:
        if _contains_keyboard_interrupt(sub):
            return True
    if e.__cause__ is not None and _contains_keyboard_interrupt(e.__cause__):
        return True
    return False


def _drained_begin_step(result: Any) -> bool:
    """True if a wait_for_event result's drain contains a begin_step event.

    Checks both the woken-on event (`wake`) and any events batched into the
    same drain (`additional_events`) -- a begin_step can arrive bundled
    behind another event in the same wake, and would otherwise never be
    counted (see B1: a WatchDog run that never observed a begin_step must
    not be labeled run_complete).
    """
    if not isinstance(result, dict):
        return False
    if result.get("wake") == "begin_step":
        return True
    for ev in result.get("additional_events", None) or []:
        if isinstance(ev, dict) and ev.get("event_type") == "begin_step":
            return True
    return False


async def run(config: AgentConfig, task_text: str, system_prompt: str) -> RunResult:
    task_name = config.task_template.stem
    transcript = TranscriptWriter(config.transcript_dir, config.agent, task_name)
    transcript.run_start(
        agent=config.agent,
        provider=config.provider,
        model=config.model,
        task_name=task_name,
        system_prompt_sha256=hash_file(config.system_prompt),
    )

    provider = get_provider(
        config.provider,
        config.model,
        config.api_key,
        config.max_tokens,
        reasoning_effort=config.reasoning_effort,
        thinking_budget_tokens=config.thinking_budget_tokens,
    )

    # B1: bind the loop bound directly from config, once, and assert it --
    # so a future refactor that introduces a second local copy (the exact
    # shape of the bug this replaced: a stale default masquerading as the
    # real termination reason) cannot silently desync the loop from what the
    # config actually says.
    max_turns = config.loop.max_turns
    assert max_turns == config.loop.max_turns and max_turns > 0, (
        f"runner must use config.loop.max_turns as the loop bound; got {max_turns!r}"
    )

    # No default. If nothing below ever sets this, that is a bug in this
    # function, not a completed-to-cap run -- see the final assert.
    termination_reason: str | None = None
    error_detail: dict | None = None
    turn_count = 0
    verdict_count = 0

    try:
        async with MCPClientPool(config.mcp_servers, config.mcp_client_timeout_sec) as pool:
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
            # B1: whether any wait_for_event drain has ever contained a
            # begin_step -- i.e. whether the executor was ever observed
            # doing anything at all. Meaningless for the executor config
            # (never calls wait_for_event) but required to distinguish a
            # WatchDog run that genuinely monitored a run from one that sat
            # alone and timed itself out; see _drained_begin_step and the
            # run_complete/no_execution_observed split below.
            begin_step_observed = False
            # Index into `messages` of the first entry not yet logged to the
            # transcript. The provider always receives the full, growing
            # `messages` list (see providers.complete below) -- this index
            # only narrows what record_llm_request writes to disk, so the
            # same message isn't re-recorded inside every later turn.
            logged_upto = 0

            while termination_reason is None:
                if turn_count >= max_turns:
                    termination_reason = "max_turns_exhausted"
                    break
                turn_count += 1

                transcript.llm_request(
                    turn_count, messages[logged_upto:], len(tool_dicts)
                )
                logged_upto = len(messages)
                try:
                    response: ProviderResponse = provider.complete(
                        system_prompt, messages, tool_dicts
                    )
                except Exception as e:
                    # Any provider-call failure -- rate limit exhausted after
                    # retries, auth/schema error, an invalid model name, a
                    # timeout, or a bug in this harness's provider code --
                    # ends the run here, labeled for what it is. It must
                    # never fall through and read as turn-cap exhaustion.
                    termination_reason = "provider_error"
                    error_detail = _exception_detail(e)
                    break

                # B1: record the response immediately on return, before any
                # termination check. A response that arrived is evidence
                # regardless of what happens next.
                transcript.llm_response(
                    turn_count,
                    response.text,
                    response.reasoning,
                    response.tool_calls,
                    response.stop_reason,
                    response.usage,
                )

                if not response.tool_calls:
                    consecutive_no_tool_calls += 1
                    # provider_native is opaque here -- only the originating
                    # provider's own *_to_*_messages/input reads it, to
                    # reconstruct this turn (with its reasoning) on a later
                    # call. See providers.py's module docstring.
                    messages.append(
                        {
                            "role": "assistant",
                            "content": response.text,
                            "provider_native": response.provider_native,
                        }
                    )
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
                        "provider_native": response.provider_native,
                    }
                )

                for tc in response.tool_calls:
                    t0 = wall_ns()
                    transcript.tool_call(turn_count, tc.name, tc.arguments)
                    # Deliberately no try/except here: any exception (a
                    # transport failure, or anything else) propagates to the
                    # outer handler below, which classifies it as mcp_error.
                    # tool_call has already been recorded, so the attempt is
                    # on the record even though its result never arrived.
                    result = await pool.call_tool(tc.name, tc.arguments)
                    t1 = wall_ns()
                    transcript.tool_result(turn_count, tc.name, result, t1 - t0)
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
                        if _drained_begin_step(result):
                            begin_step_observed = True
                        wake = result.get("wake") if isinstance(result, dict) else None
                        # B2: increment only on a genuine wait_for_event
                        # timeout. Reset on anything else -- a wake on a real
                        # event, or any other tool call in this turn's list
                        # (e.g. get_execution_state between two waits). A
                        # turn that timed out, checked state, and resumed
                        # waiting is diagnosing correctly and must not be
                        # pushed toward termination; only two genuinely
                        # back-to-back wait_for_event timeouts should be.
                        if wake == "timeout":
                            consecutive_timeouts += 1
                        else:
                            consecutive_timeouts = 0
                    else:
                        consecutive_timeouts = 0

                if (
                    config.loop.stop_after_consecutive_timeouts is not None
                    and consecutive_timeouts
                    >= config.loop.stop_after_consecutive_timeouts
                ):
                    # B1: a run that reached this cap without ever observing
                    # a begin_step monitored nothing -- an executor that
                    # never started is indistinguishable, in its
                    # consequences, from one that ran cleanly with no
                    # violations, unless this is labeled distinctly.
                    termination_reason = (
                        "run_complete" if begin_step_observed else "no_execution_observed"
                    )
                    break
    except (KeyboardInterrupt, BaseExceptionGroup) as e:
        # Two distinct ways an operator Ctrl-C can reach here, neither
        # caught by `except Exception` below: a bare KeyboardInterrupt
        # raised outside the `async with` block, or a BaseExceptionGroup --
        # what anyio's TaskGroup raises when a KeyboardInterrupt is one of
        # its sub-exceptions. BaseExceptionGroup is a BaseException, not an
        # Exception (only ExceptionGroup, its all-Exception-subclass sibling,
        # is), so it silently skips `except Exception` entirely; without this
        # clause it would escape this function altogether and the finally
        # block's assert would fire instead of a clean user_abort.
        termination_reason = "user_abort" if _contains_keyboard_interrupt(e) else "mcp_error"
        error_detail = _exception_detail(e)
    except Exception as e:
        # Anything that escaped the loop above -- MCP pool connect/list_tools
        # failure, a tool-call transport failure, or any other unhandled
        # exception on the MCP/harness side (provider failures are already
        # caught and labeled above and never reach here). Same treatment as
        # B1 requires for provider failures: classify, don't default.
        #
        # An operator Ctrl-C during an `async with` block (MCPClientPool's
        # TaskGroup) surfaces here as an ExceptionGroup, not a bare
        # KeyboardInterrupt -- _contains_keyboard_interrupt recurses into it
        # so this is still labeled user_abort, not a mislabeled mcp_error.
        if _contains_keyboard_interrupt(e):
            termination_reason = "user_abort"
        else:
            termination_reason = "mcp_error"
        error_detail = _exception_detail(e)
    finally:
        # By construction termination_reason is always set by this point:
        # either a break above set it, or the except clause just did.
        assert termination_reason is not None, (
            "runner exited without setting a termination reason -- this is a "
            "bug in runner.py, not a completed run"
        )
        transcript.run_end(
            termination_reason, turn_count, verdict_count=verdict_count, error=error_detail
        )
        transcript.close()

    return RunResult(
        termination_reason=termination_reason,
        turn_count=turn_count,
        transcript_path=str(transcript.path),
        verdict_count=verdict_count,
    )
