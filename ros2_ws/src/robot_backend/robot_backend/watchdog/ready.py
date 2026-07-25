"""WatchDog-side client for the cell's /cell/signal_ready service.

Mirrors HaltClient's shape (robot_backend.watchdog.halt) on purpose: WatchDog's
node is already being spun by a background MultiThreadedExecutor (see
watchdog_mcp_server._ensure_watchdog), so rclpy.spin_until_future_complete on
it would deadlock -- spinning it again from inside a callback the executor is
already driving. call_async + add_done_callback + threading.Event is the
pattern that already works for exactly this reason in HaltClient.halt.
"""

from __future__ import annotations

import threading
from typing import Any

from robot_backend.events.schema import WatchDogReadyPayload, make_event


class ReadyClient:
    """Signal readiness to the cell's orchestrator, for a run, step, or phase."""

    def __init__(
        self,
        node: Any,
        event_bus: Any,
        callback_group: Any,
        timeout_sec: float = 5.0,
    ) -> None:
        self.node = node
        self.event_bus = event_bus
        self._callback_group = callback_group
        self.timeout_sec = timeout_sec
        self._client: Any | None = None
        self._SignalReady: Any | None = None
        self._lock = threading.Lock()

    def signal(
        self,
        scope: str,
        step_id: str | None = None,
        phase: str | None = None,
        detail: str | None = None,
    ) -> dict[str, Any]:
        ok = False
        message = "signal not attempted"
        try:
            self._ensure_client()
            if self._client is None or self._SignalReady is None:
                raise RuntimeError("signal_ready client unavailable")
            if not self._client.wait_for_service(timeout_sec=0.2):
                raise RuntimeError("/cell/signal_ready service unavailable")

            request = self._SignalReady.Request()
            request.scope = scope
            request.step_id = step_id or ""
            request.phase = phase or ""
            # detail is only meaningful for scope="abort"; harmless (empty
            # string) for every other scope. Guarded with getattr so a client
            # built against an older SignalReady.srv without the field still
            # works.
            if hasattr(request, "detail"):
                request.detail = detail or ""

            done = threading.Event()
            holder: dict[str, Any] = {}
            future = self._client.call_async(request)

            def _done(fut: Any) -> None:
                holder["future"] = fut
                done.set()

            future.add_done_callback(_done)
            if done.wait(self.timeout_sec):
                result = future.result()
                ok = bool(getattr(result, "ok", False))
                message = getattr(result, "message", "")
            else:
                message = f"signal_ready timed out after {self.timeout_sec:.1f}s"
        except Exception as exc:
            message = str(exc)
        finally:
            self._publish_ready(scope, step_id, phase)
        return {"ok": bool(ok), "message": message}

    def prewarm(self) -> None:
        """Construct the ROS service client before its first signal."""
        self._ensure_client()

    def _ensure_client(self) -> None:
        with self._lock:
            if self._client is not None:
                return
            from cell_sync_interfaces.srv import SignalReady

            self._client = self.node.create_client(
                SignalReady,
                "/cell/signal_ready",
                callback_group=self._callback_group,
            )
            self._SignalReady = SignalReady

    def _publish_ready(
        self, scope: str, step_id: str | None, phase: str | None
    ) -> None:
        try:
            if self.event_bus is not None:
                self.event_bus.publish(make_event(
                    "watchdog_ready",
                    WatchDogReadyPayload(scope=scope, step_id=step_id, phase=phase),
                ))
        except Exception:
            pass
