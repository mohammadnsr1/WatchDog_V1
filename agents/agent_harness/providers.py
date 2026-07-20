"""Provider-neutral LLM loop. runner.py never sees a provider-specific type.

Conversation state between runner and provider is a neutral message list:

    {"role": "user", "content": "..."}
    {"role": "assistant", "content": "...", "tool_calls": [ToolCall, ...]}
    {"role": "tool_result", "tool_call_id": "...", "name": "...",
     "content": <any>, "is_error": bool}

Each provider converts this neutral history into its own wire format on
every call (both APIs are stateless, so full history is resent each time) and
owns its own tool-result formatting. runner.py only ever reads/writes the
neutral schema and the dataclasses below.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: dict


@dataclass
class ProviderResponse:
    text: str | None
    tool_calls: list[ToolCall]
    stop_reason: str
    raw: dict = field(default_factory=dict)
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
    def __init__(self, model: str, api_key: str, max_tokens: int):
        self.model = model
        self.max_tokens = max_tokens
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
                return self._client.messages.create(
                    model=self.model,
                    max_tokens=self.max_tokens,
                    system=system,
                    messages=anthropic_messages,
                    tools=anthropic_tools,
                )
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
        for block in response.content:
            if block.type == "text":
                text_parts.append(block.text)
            elif block.type == "tool_use":
                tool_calls.append(
                    ToolCall(id=block.id, name=block.name, arguments=block.input)
                )

        return ProviderResponse(
            text="".join(text_parts) if text_parts else None,
            tool_calls=tool_calls,
            stop_reason=response.stop_reason,
            raw=response.model_dump(),
            usage=response.usage.model_dump() if response.usage else {},
        )


class OpenAIProvider(LLMProvider):
    def __init__(self, model: str, api_key: str, max_tokens: int):
        self.model = model
        self.max_tokens = max_tokens
        import openai

        self._client = openai.OpenAI(api_key=api_key)
        self._openai = openai

    def name(self) -> str:
        return "openai"

    def convert_tools(self, mcp_tools: list[dict]) -> list[dict]:
        return [
            {
                "type": "function",
                "function": {
                    "name": t["name"],
                    "description": t["description"],
                    "parameters": t["input_schema"],
                },
            }
            for t in mcp_tools
        ]

    def _to_openai_messages(self, system: str, messages: list[dict]) -> list[dict]:
        out: list[dict] = [{"role": "system", "content": system}]
        for m in messages:
            role = m["role"]
            if role == "user":
                out.append({"role": "user", "content": m["content"]})
            elif role == "assistant":
                msg: dict = {"role": "assistant", "content": m.get("content")}
                tool_calls = m.get("tool_calls", [])
                if tool_calls:
                    msg["tool_calls"] = [
                        {
                            "id": tc.id if isinstance(tc, ToolCall) else tc["id"],
                            "type": "function",
                            "function": {
                                "name": tc.name
                                if isinstance(tc, ToolCall)
                                else tc["name"],
                                "arguments": _json_dumps(
                                    tc.arguments
                                    if isinstance(tc, ToolCall)
                                    else tc["arguments"]
                                ),
                            },
                        }
                        for tc in tool_calls
                    ]
                out.append(msg)
            elif role == "tool_result":
                out.append(
                    {
                        "role": "tool",
                        "tool_call_id": m["tool_call_id"],
                        "content": _stringify(m["content"]),
                    }
                )
            else:
                raise ValueError(f"unknown neutral message role: {role}")
        return out

    def complete(
        self, system: str, messages: list[dict], tools: list[dict]
    ) -> ProviderResponse:
        openai_messages = self._to_openai_messages(system, messages)
        openai_tools = self.convert_tools(tools)

        def do_call():
            try:
                kwargs = dict(
                    model=self.model,
                    max_completion_tokens=self.max_tokens,
                    messages=openai_messages,
                )
                if openai_tools:
                    kwargs["tools"] = openai_tools
                return self._client.chat.completions.create(**kwargs)
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
        choice = response.choices[0]
        msg = choice.message

        tool_calls = []
        for tc in msg.tool_calls or []:
            tool_calls.append(
                ToolCall(
                    id=tc.id,
                    name=tc.function.name,
                    arguments=_json_loads(tc.function.arguments),
                )
            )

        return ProviderResponse(
            text=msg.content,
            tool_calls=tool_calls,
            stop_reason=choice.finish_reason,
            raw=response.model_dump(),
            usage=response.usage.model_dump() if response.usage else {},
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


def get_provider(provider_name: str, model: str, api_key: str, max_tokens: int) -> LLMProvider:
    try:
        cls = _PROVIDERS[provider_name]
    except KeyError:
        raise ValueError(
            f"unknown provider '{provider_name}'; available: {sorted(_PROVIDERS)}"
        )
    return cls(model=model, api_key=api_key, max_tokens=max_tokens)
