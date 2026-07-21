"""Thin wrapper around the mcp stdio client, supporting several servers at once.

Everything here is discovered at runtime from the servers themselves. No tool
name, signature, or schema is hardcoded, with exactly one exception: the
runner needs to recognize wait_for_event to detect a timeout-driven
termination path. That coupling is confined to the single constant below.
"""

from __future__ import annotations

import contextlib
import os
from dataclasses import dataclass
from typing import Any

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from agent_harness.config import MCPServerConfig

# The one tool name the runner is allowed to know about. It is used only to
# detect wait_for_event timeouts for the loop-termination rule in runner.py.
# An agent whose servers don't expose this tool (the executor) simply never
# takes that termination path, which is the correct, expected behavior.
WAIT_FOR_EVENT_TOOL = "wait_for_event"


class MCPToolError(RuntimeError):
    """Raised on a transport-level failure, never on a tool-level error."""


@dataclass
class DiscoveredTool:
    name: str  # namespaced name as exposed to the provider/model
    server_name: str
    original_name: str
    description: str
    input_schema: dict[str, Any]


class MCPClientPool(contextlib.AbstractAsyncContextManager):
    """Spawns one stdio_client + ClientSession per configured server."""

    def __init__(self, servers: list[MCPServerConfig]):
        self._server_configs = servers
        self._sessions: dict[str, ClientSession] = {}
        self._tools: dict[str, DiscoveredTool] = {}
        self._stack: contextlib.AsyncExitStack | None = None

    async def __aenter__(self) -> "MCPClientPool":
        self._stack = contextlib.AsyncExitStack()
        await self._stack.__aenter__()
        try:
            for server in self._server_configs:
                params = StdioServerParameters(
                    command=server.command,
                    args=server.args,
                    cwd=server.cwd,
                    # mcp's stdio_client defaults to a minimal, security-safe
                    # environment (HOME/PATH/USER/...) that does NOT inherit
                    # PYTHONPATH, AMENT_PREFIX_PATH, ROS_DISTRO, etc. These
                    # servers are ROS2 nodes that need the sourced overlay, so
                    # we explicitly pass this process's full environment
                    # through, with the config's own env vars layered on top.
                    env={**os.environ, **server.env},
                )
                read, write = await self._stack.enter_async_context(
                    stdio_client(params)
                )
                session = await self._stack.enter_async_context(
                    ClientSession(read, write)
                )
                await session.initialize()
                self._sessions[server.name] = session
        except Exception:
            await self._stack.__aexit__(None, None, None)
            raise
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self._stack is not None:
            await self._stack.__aexit__(exc_type, exc, tb)

    async def list_tools(self) -> list[DiscoveredTool]:
        """Discover and aggregate tools across all servers.

        Namespaces on name collision (server_name__tool_name) but otherwise
        leaves names, descriptions, and schemas exactly as the server reports
        them.
        """
        self._tools = {}
        seen_names: dict[str, str] = {}  # original_name -> server_name
        collisions: set[str] = set()

        per_server: list[tuple[str, Any]] = []
        for server_name, session in self._sessions.items():
            try:
                result = await session.list_tools()
            except Exception as e:
                raise MCPToolError(
                    f"transport failure listing tools on server '{server_name}': {e}"
                ) from e
            for tool in result.tools:
                per_server.append((server_name, tool))
                if tool.name in seen_names and seen_names[tool.name] != server_name:
                    collisions.add(tool.name)
                seen_names[tool.name] = server_name

        discovered: list[DiscoveredTool] = []
        for server_name, tool in per_server:
            exposed_name = (
                f"{server_name}__{tool.name}"
                if tool.name in collisions
                else tool.name
            )
            dt = DiscoveredTool(
                name=exposed_name,
                server_name=server_name,
                original_name=tool.name,
                description=tool.description or "",
                input_schema=tool.inputSchema or {"type": "object", "properties": {}},
            )
            self._tools[exposed_name] = dt
            discovered.append(dt)
        return discovered

    async def call_tool(self, name: str, arguments: dict[str, Any]) -> Any:
        """Route a tool call to its owning server and return the parsed result whole.

        Tool-level failures come back as ordinary content (the servers return
        {"ok": false, "error": ...}) and are returned like any other result --
        they are data for the agent to reason about, not exceptions. Only a
        transport failure raises.
        """
        tool = self._tools.get(name)
        if tool is None:
            raise MCPToolError(f"unknown tool '{name}' (not in last list_tools() call)")
        session = self._sessions[tool.server_name]
        try:
            result = await session.call_tool(tool.original_name, arguments)
        except Exception as e:
            raise MCPToolError(
                f"transport failure calling tool '{name}' on server "
                f"'{tool.server_name}': {e}"
            ) from e
        return _extract_result(result)


def _extract_result(call_tool_result: Any) -> Any:
    """Unwrap an MCP CallToolResult into plain data, passed through whole.

    get_evidence can return up to 1000 samples; this must never truncate or
    summarize -- if a result overflows context, that is a finding for the
    study and must surface as an error, not a silently shortened payload.
    """
    content = getattr(call_tool_result, "content", None)
    is_error = bool(getattr(call_tool_result, "isError", False))

    parts: list[Any] = []
    for block in content or []:
        block_type = getattr(block, "type", None)
        if block_type == "text":
            text = block.text
            try:
                import json

                parts.append(json.loads(text))
            except (ValueError, TypeError):
                parts.append(text)
        else:
            # Preserve non-text content blocks (images, etc.) whole.
            parts.append(getattr(block, "model_dump", lambda: block)())

    if len(parts) == 1:
        payload = parts[0]
    else:
        payload = parts

    if is_error:
        return {"ok": False, "error": payload}
    return payload
