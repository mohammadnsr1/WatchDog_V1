"""Provider-neutral LLM loop. runner.py never sees a provider-specific type.

Conversation state between runner and provider is a neutral message list:

    {"role": "user", "content": "..."}
    {"role": "assistant", "content": "...", "tool_calls": [ToolCall, ...],
     "provider_native": <opaque>}
    {"role": "tool_result", "tool_call_id": "...", "name": "...",
     "content": <any>, "is_error": bool}

Each provider converts this neutral history into its own wire format on
every call (both APIs are stateless, so full history is resent each time) and
owns its own tool-result formatting. runner.py only ever reads/writes the
neutral schema and the dataclasses below.

`provider_native` on an assistant message is opaque to runner.py and to the
other provider: it is whatever the *originating* provider needs to
faithfully reconstruct that turn on a later call (OpenAI's raw Responses API
output items including the reasoning item; Anthropic's raw content blocks
including thinking blocks with their signature). runner.py copies it through
without inspecting it -- see ProviderResponse.provider_native and each
provider's *_to_*_messages/input method for why this matters (B3: silently
dropping this degrades reasoning continuity rather than raising).
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: dict


@dataclass
class ReasoningTrace:
    """Provider-neutral view of one turn's reasoning, for the transcript only.

    summary is None whenever the provider returned no reasoning content --
    a normal outcome (the API may withhold it for reasons outside our
    control), never an error.
    """

    summary: str | None
    tokens: int | None


@dataclass
class ProviderResponse:
    text: str | None
    tool_calls: list[ToolCall]
    stop_reason: str
    reasoning: ReasoningTrace = field(default_factory=lambda: ReasoningTrace(None, None))
    # Opaque, provider-specific data needed to reconstruct this exact turn on
    # a later call (see module docstring). Never written to the transcript.
    provider_native: Any = None
    usage: dict = field(default_factory=dict)


class TransientProviderError(RuntimeError):
    """429 / 5xx / timeout -- safe to retry with backoff."""


class FatalProviderError(RuntimeError):
    """4xx auth or schema error -- must surface, never retried."""


class LLMProvider(ABC):
    @abstractmethod
    def name(self) -> str: ...

    @abstractmethod
    def convert_tools(self, mcp_tools: list[dict]) -> list[dict]: ...

    @abstractmethod
    def complete(
        self, system: str, messages: list[dict], tools: list[dict]
    ) -> ProviderResponse: ...


def _retry_with_backoff(fn, max_attempts: int = 6, base_delay: float = 1.0, max_delay: float = 30.0):
    attempt = 0
    while True:
        try:
            return fn()
        except TransientProviderError:
            attempt += 1
            if attempt >= max_attempts:
                raise
            delay = min(max_delay, base_delay * (2 ** (attempt - 1)))
            time.sleep(delay)


class AnthropicProvider(LLMProvider):
    def __init__(
        self,
        model: str,
        api_key: str,
        max_tokens: int,
        thinking_budget_tokens: int | None = None,
    ):
        self.model = model
        self.max_tokens = max_tokens
        self.thinking_budget_tokens = thinking_budget_tokens
        import anthropic

        self._client = anthropic.Anthropic(api_key=api_key)
        self._anthropic = anthropic

    def name(self) -> str:
        return "anthropic"

    def convert_tools(self, mcp_tools: list[dict]) -> list[dict]:
        return [
            {
                "name": t["name"],
                "description": t["description"],
                "input_schema": t["input_schema"],
            }
            for t in mcp_tools
        ]

    def _to_anthropic_messages(self, messages: list[dict]) -> list[dict]:
        out: list[dict] = []
        for m in messages:
            role = m["role"]
            if role == "user":
                out.append({"role": "user", "content": m["content"]})
            elif role == "assistant":
                native = m.get("provider_native")
                if native is not None:
                    # Echo back the exact content blocks Anthropic returned
                    # for this turn, thinking blocks (with signature) and
                    # all -- required to preserve reasoning continuity across
                    # a tool-use turn; see module docstring (B3).
                    out.append({"role": "assistant", "content": native})
                else:
                    content = []
                    if m.get("content"):
                        content.append({"type": "text", "text": m["content"]})
                    for tc in m.get("tool_calls", []):
                        content.append(
                            {
                                "type": "tool_use",
                                "id": tc.id if isinstance(tc, ToolCall) else tc["id"],
                                "name": tc.name if isinstance(tc, ToolCall) else tc["name"],
                                "input": tc.arguments
                                if isinstance(tc, ToolCall)
                                else tc["arguments"],
                            }
                        )
                    out.append({"role": "assistant", "content": content})
            elif role == "tool_result":
                out.append(
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "tool_result",
                                "tool_use_id": m["tool_call_id"],
                                "content": _stringify(m["content"]),
                                "is_error": bool(m.get("is_error", False)),
                            }
                        ],
                    }
                )
            else:
                raise ValueError(f"unknown neutral message role: {role}")
        return out

    def complete(
        self, system: str, messages: list[dict], tools: list[dict]
    ) -> ProviderResponse:
        anthropic_messages = self._to_anthropic_messages(messages)
        anthropic_tools = self.convert_tools(tools)

        def do_call():
            try:
                kwargs: dict[str, Any] = dict(
                    model=self.model,
                    max_tokens=self.max_tokens,
                    system=system,
                    messages=anthropic_messages,
                    tools=anthropic_tools,
                )
                # Extended thinking is this provider's reasoning-visibility
                # knob (see AnthropicProvider vs. OpenAIProvider.reasoning);
                # the config comment in agent configs documents that
                # thinking_budget_tokens is ignored for the openai provider
                # -- this is where that's true: an OpenAIProvider instance
                # never constructs a thinking_budget_tokens value at all.
                if self.thinking_budget_tokens:
                    kwargs["thinking"] = {
                        "type": "enabled",
                        "budget_tokens": self.thinking_budget_tokens,
                    }
                return self._client.messages.create(**kwargs)
            except self._anthropic.RateLimitError as e:
                raise TransientProviderError(str(e)) from e
            except self._anthropic.APIConnectionError as e:
                raise TransientProviderError(str(e)) from e
            except self._anthropic.APIStatusError as e:
                if e.status_code >= 500:
                    raise TransientProviderError(str(e)) from e
                raise FatalProviderError(str(e)) from e

        response = _retry_with_backoff(do_call)

        text_parts = []
        tool_calls = []
        thinking_parts = []
        native_blocks = []
        # Anthropic returns thinking, text, and tool_use blocks interleaved
        # in one content list -- separate by block.type rather than assuming
        # position (e.g. that the first block is always text).
        for block in response.content:
            native_blocks.append(block.model_dump())
            if block.type == "text":
                text_parts.append(block.text)
            elif block.type == "tool_use":
                tool_calls.append(
                    ToolCall(id=block.id, name=block.name, arguments=block.input)
                )
            elif block.type == "thinking":
                thinking_parts.append(block.thinking)
            # redacted_thinking blocks carry only encrypted `data`, no visible
            # text to surface as a summary -- still preserved in
            # native_blocks above so resubmission stays byte-for-byte.

        usage_dict = response.usage.model_dump() if response.usage else {}
        reasoning_tokens = (usage_dict.get("output_tokens_details") or {}).get(
            "thinking_tokens"
        )

        return ProviderResponse(
            text="".join(text_parts) if text_parts else None,
            tool_calls=tool_calls,
            stop_reason=response.stop_reason,
            reasoning=ReasoningTrace(
                summary="\n\n".join(thinking_parts) if thinking_parts else None,
                tokens=reasoning_tokens,
            ),
            provider_native=native_blocks,
            usage=usage_dict,
        )


class OpenAIProvider(LLMProvider):
    """Uses the Responses API, not Chat Completions.

    Migrated for this task: reasoning summaries (reasoning={"summary": ...})
    are supported on the Responses API but not on Chat Completions for
    reasoning models, so generating them at all requires this endpoint.
    Multi-turn reasoning continuity for gpt-5-class models also depends on
    echoing the model's own prior reasoning items back in `input` on
    subsequent turns (see _to_openai_input's provider_native handling, B3)
    -- OpenAI's own guidance is to reuse the previous response's `output`
    array directly for this rather than reconstructing it.
    """

    def __init__(
        self,
        model: str,
        api_key: str,
        max_tokens: int,
        reasoning_effort: str = "medium",
    ):
        self.model = model
        self.max_tokens = max_tokens
        self.reasoning_effort = reasoning_effort
        import openai

        self._client = openai.OpenAI(api_key=api_key)
        self._openai = openai

    def name(self) -> str:
        return "openai"

    def convert_tools(self, mcp_tools: list[dict]) -> list[dict]:
        # Responses API tool shape is flat (name/description/parameters at
        # the top level), unlike Chat Completions' {"function": {...}}
        # nesting.
        return [
            {
                "type": "function",
                "name": t["name"],
                "description": t["description"],
                "parameters": t["input_schema"],
                "strict": False,
            }
            for t in mcp_tools
        ]

    def _to_openai_input(self, messages: list[dict]) -> list[dict]:
        input_items: list[dict] = []
        for m in messages:
            role = m["role"]
            if role == "user":
                input_items.append({"role": "user", "content": m["content"]})
            elif role == "assistant":
                native = m.get("provider_native")
                if native is not None:
                    # native is exactly the output items this provider
                    # returned for this turn (reasoning item, function_call
                    # item(s), optional message item, in original order).
                    # Echoing them back verbatim is what keeps a reasoning
                    # model's chain of thought available across the tool
                    # calls in this run, per OpenAI's own guidance -- losing
                    # this doesn't raise, it just quietly makes the model
                    # reason less effectively turn to turn.
                    input_items.extend(native)
                else:
                    if m.get("content"):
                        input_items.append({"role": "assistant", "content": m["content"]})
                    for tc in m.get("tool_calls", []):
                        input_items.append(
                            {
                                "type": "function_call",
                                "call_id": tc.id if isinstance(tc, ToolCall) else tc["id"],
                                "name": tc.name if isinstance(tc, ToolCall) else tc["name"],
                                "arguments": _json_dumps(
                                    tc.arguments
                                    if isinstance(tc, ToolCall)
                                    else tc["arguments"]
                                ),
                            }
                        )
            elif role == "tool_result":
                input_items.append(
                    {
                        "type": "function_call_output",
                        "call_id": m["tool_call_id"],
                        "output": _stringify(m["content"]),
                    }
                )
            else:
                raise ValueError(f"unknown neutral message role: {role}")
        return input_items

    def complete(
        self, system: str, messages: list[dict], tools: list[dict]
    ) -> ProviderResponse:
        input_items = self._to_openai_input(messages)
        openai_tools = self.convert_tools(tools)

        def do_call():
            try:
                kwargs: dict[str, Any] = dict(
                    model=self.model,
                    instructions=system,
                    input=input_items,
                    max_output_tokens=self.max_tokens,
                    reasoning={"effort": self.reasoning_effort, "summary": "auto"},
                    # Full history is resent every turn (stateless-by-design,
                    # same as Chat Completions was) -- this harness never
                    # uses previous_response_id, so there is nothing to gain
                    # from server-side retention.
                    store=False,
                )
                if openai_tools:
                    kwargs["tools"] = openai_tools
                return self._client.responses.create(**kwargs)
            except self._openai.RateLimitError as e:
                raise TransientProviderError(str(e)) from e
            except self._openai.APIConnectionError as e:
                raise TransientProviderError(str(e)) from e
            except self._openai.APITimeoutError as e:
                raise TransientProviderError(str(e)) from e
            except self._openai.APIStatusError as e:
                if e.status_code >= 500:
                    raise TransientProviderError(str(e)) from e
                raise FatalProviderError(str(e)) from e

        response = _retry_with_backoff(do_call)

        tool_calls = []
        reasoning_summary_parts = []
        native_output = []
        for item in response.output:
            native_output.append(item.model_dump(exclude_none=True))
            if item.type == "function_call":
                tool_calls.append(
                    ToolCall(
                        id=item.call_id,
                        name=item.name,
                        arguments=_json_loads(item.arguments),
                    )
                )
            elif item.type == "reasoning":
                # summary may legitimately be an empty list -- the API can
                # withhold reasoning summaries for reasons outside our
                # control (e.g. safety filtering); that is a normal outcome,
                # not an error, and leaves reasoning_summary_parts empty so
                # ReasoningTrace.summary ends up None below.
                for s in item.summary or []:
                    text = getattr(s, "text", None)
                    if text:
                        reasoning_summary_parts.append(text)
            # "message" items are collected via response.output_text below.

        text = response.output_text or None

        usage_dict = response.usage.model_dump() if response.usage else {}
        reasoning_tokens = (usage_dict.get("output_tokens_details") or {}).get(
            "reasoning_tokens"
        )

        return ProviderResponse(
            text=text,
            tool_calls=tool_calls,
            stop_reason=response.status,
            reasoning=ReasoningTrace(
                summary="\n\n".join(reasoning_summary_parts)
                if reasoning_summary_parts
                else None,
                tokens=reasoning_tokens,
            ),
            provider_native=native_output,
            usage=usage_dict,
        )


def _stringify(content) -> str:
    if isinstance(content, str):
        return content
    return _json_dumps(content)


def _json_dumps(obj) -> str:
    import json

    return json.dumps(obj)


def _json_loads(s: str):
    import json

    return json.loads(s)


_PROVIDERS = {
    "anthropic": AnthropicProvider,
    "openai": OpenAIProvider,
}


def get_provider(
    provider_name: str,
    model: str,
    api_key: str,
    max_tokens: int,
    *,
    reasoning_effort: str = "medium",
    thinking_budget_tokens: int | None = None,
) -> LLMProvider:
    try:
        cls = _PROVIDERS[provider_name]
    except KeyError:
        raise ValueError(
            f"unknown provider '{provider_name}'; available: {sorted(_PROVIDERS)}"
        )
    if provider_name == "openai":
        return cls(
            model=model, api_key=api_key, max_tokens=max_tokens,
            reasoning_effort=reasoning_effort,
        )
    if provider_name == "anthropic":
        return cls(
            model=model, api_key=api_key, max_tokens=max_tokens,
            thinking_budget_tokens=thinking_budget_tokens,
        )
    return cls(model=model, api_key=api_key, max_tokens=max_tokens)
