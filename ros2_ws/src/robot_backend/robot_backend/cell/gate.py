"""Executor-side client for the cell's synchronization gate.

The gate is a service pair (/cell/request_go, /cell/signal_ready) served by
the orchestrator node in ros2_ws/src/cell_orchestrator/ -- a plain ROS node,
its own colcon package, with no LLM, no reasoning, and no knowledge of
contracts or monitors. This module is the executor-side half of that
handshake: it blocks the calling MCP server node between announcing a step
(or a mid-step phase) and acting on it, so a monitor has a chance to arm
before the scene is disturbed.

Synchronization runs over a service, not the /watchdog/events topic, on
purpose: the topic already dropped one begin_step to a discovery race (see
EventBus._wait_for_subscriber_once), and a dropped release token would be
indistinguishable from a slow monitor. A service gives a real handshake --
wait_for_service proves the peer exists, and every call has an explicit
success or failure.

This module must never make the cell deadlock. If no orchestrator is running,
or the call errors, or it times out, the caller proceeds. A stalled or absent
monitor is a finding (recorded in the returned dict for the transcript), never
a hang.

Timeout ordering (load-bearing, not just convention): whatever timeout_sec a
caller uses here must sit strictly BELOW the MCP client's own request timeout
(agents/config/*.yaml's mcp_client_timeout_sec) or the MCP client aborts the
call while the orchestrator is still legitimately holding it -- the resulting
symptom is an mcp_error in the executor transcript with the orchestrator log
showing a perfectly normal gate_open a few seconds later, which reads as a
transport bug and is actually a configuration mismatch. DEFAULT_GATE_TIMEOUT_SEC
below is read from CELL_GATE_TIMEOUT_SEC so the same number can be set once,
in the MCP server's env block in agents/config/executor_agent.yaml, and
cross-checked there against mcp_client_timeout_sec at config-load time (see
agent_harness/config.py) instead of drifting between two independently-run
processes.
"""

from __future__ import annotations

import os
import time
from typing import Any

import rclpy

GO_SERVICE = "/cell/request_go"

# Read once at import time so both begin_step and mark_phase use the same
# value without each having to know about the env var individually. See the
# module docstring above for why this must stay below the MCP client timeout.
DEFAULT_GATE_TIMEOUT_SEC = float(os.environ.get("CELL_GATE_TIMEOUT_SEC", "120.0"))

_CLIENT_ATTR = "_cell_gate_request_go_client"


def _get_client(node: Any) -> Any:
    """Create the RequestGo client once per node and cache it on the node."""
    client = getattr(node, _CLIENT_ATTR, None)
    if client is not None:
        return client
    from cell_sync_interfaces.srv import RequestGo

    client = node.create_client(RequestGo, GO_SERVICE)
    setattr(node, _CLIENT_ATTR, client)
    return client


def request_go(
    node: Any,
    step_id: str,
    phase: str = "",
    timeout_sec: float = DEFAULT_GATE_TIMEOUT_SEC,
) -> dict[str, Any]:
    """Block until the orchestrator releases this step/phase, or fail open.

    Returns a dict describing what happened, always including "gated" (was an
    orchestrator present to gate on at all) so the caller can surface an
    unguarded-step finding downstream.
    """
    client = _get_client(node)

    if not client.wait_for_service(timeout_sec=0.2):
        # No orchestrator running. Executor-alone runs must work unchanged --
        # this is a supported configuration, not an error.
        return {"gated": False, "reason": "no_orchestrator"}

    from cell_sync_interfaces.srv import RequestGo

    request = RequestGo.Request()
    request.step_id = step_id
    request.phase = phase
    request.timeout_sec = float(timeout_sec)

    t0 = time.monotonic()
    try:
        future = client.call_async(request)
        # The MCP server nodes do not spin, so blocking inline here (rather
        # than add_done_callback + threading.Event, as WatchDog's node uses
        # since it IS already being spun) is correct and simple.
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec + 5.0)
        measured_wait_sec = time.monotonic() - t0

        if not future.done():
            # spin_until_future_complete's own timeout elapsed without a
            # response -- the orchestrator itself may be stuck. Proceed; a
            # stalled monitor must never deadlock the cell.
            return {
                "gated": True,
                "proceed": True,
                "timed_out": True,
                "waited_sec": measured_wait_sec,
                "reason": "error",
                "error": "orchestrator did not respond within the call timeout",
            }

        response = future.result()
        if response is None:
            return {
                "gated": True,
                "proceed": True,
                "timed_out": True,
                "waited_sec": measured_wait_sec,
                "reason": "error",
                "error": "RequestGo call returned no result",
            }

        return {
            "gated": True,
            "proceed": bool(response.proceed),
            "timed_out": bool(response.timed_out),
            "waited_sec": float(response.waited_sec),
            "reason": response.reason,
        }
    except Exception as exc:
        # Proceed on any error -- a crashed orchestrator must never deadlock
        # the cell. This becomes an unguarded-step finding, recorded here.
        return {
            "gated": True,
            "proceed": True,
            "timed_out": True,
            "waited_sec": time.monotonic() - t0,
            "reason": "error",
            "error": str(exc),
        }
