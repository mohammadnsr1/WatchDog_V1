"""Narrow halt authority for WatchDog."""

from __future__ import annotations

import threading
import time
from typing import Any

from robot_backend.events.schema import HaltPayload, make_event


class HaltClient:
    """Cancel MoveGroup goals without issuing any replacement motion."""

    def __init__(
        self,
        node: Any,
        event_bus: Any,
        callback_group: Any,
        timeout_sec: float = 1.0,
    ) -> None:
        self.node = node
        self.event_bus = event_bus
        self._callback_group = callback_group
        self.timeout_sec = timeout_sec
        self._action_client: Any | None = None
        self._cancel_client: Any | None = None
        self._CancelGoal: Any | None = None
        self._GoalInfo: Any | None = None
        self._lock = threading.Lock()

    def halt(self, step_id: str, monitor_id: str, detail: str) -> dict[str, Any]:
        ok = False
        cancel_detail = "cancel not attempted"
        # None until a response is actually received: 0 must mean "answered,
        # and nothing was cancelling", never "never answered". See
        # goals_canceling in action_msgs/srv/CancelGoal.srv -- the return_code
        # alone cannot tell "cancel succeeded, N goals stopped" apart from
        # "cancel succeeded, nothing was active to stop".
        goals_cancelled: int | None = None
        elapsed_sec: float | None = None
        call_started: float | None = None
        try:
            self._ensure_clients()
            if self._cancel_client is None or self._CancelGoal is None or self._GoalInfo is None:
                raise RuntimeError("cancel client unavailable")
            if not self._cancel_client.wait_for_service(timeout_sec=0.2):
                raise RuntimeError("/move_action/_action/cancel_goal service unavailable")
            request = self._CancelGoal.Request()
            request.goal_info = self._GoalInfo()
            request.goal_info.goal_id.uuid = [0] * 16
            done = threading.Event()
            holder: dict[str, Any] = {}
            call_started = time.monotonic()
            future = self._cancel_client.call_async(request)

            def _done(fut: Any) -> None:
                holder["future"] = fut
                holder["elapsed_sec"] = time.monotonic() - call_started
                done.set()

            future.add_done_callback(_done)
            if done.wait(self.timeout_sec):
                elapsed_sec = holder["elapsed_sec"]
                result = future.result()
                return_code = getattr(result, "return_code", None)
                canceling = list(getattr(result, "goals_canceling", []) or [])
                goals_cancelled = len(canceling)
                ok = return_code in (None, 0)
                cancel_detail = (
                    f"cancel-all requested; return_code={return_code}; "
                    f"goals_canceling={goals_cancelled}"
                )
            else:
                elapsed_sec = time.monotonic() - call_started
                cancel_detail = f"cancel-all timed out after {self.timeout_sec:.1f}s"
        except Exception as exc:
            if call_started is not None and elapsed_sec is None:
                elapsed_sec = time.monotonic() - call_started
            cancel_detail = str(exc)
        finally:
            self._publish_halt(step_id, monitor_id, detail)
        return {
            "ok": bool(ok),
            "detail": cancel_detail,
            "goals_cancelled": goals_cancelled,
            "elapsed_sec": elapsed_sec,
        }

    def _ensure_clients(self) -> None:
        with self._lock:
            if self._cancel_client is not None:
                return
            from action_msgs.msg import GoalInfo
            from action_msgs.srv import CancelGoal
            from moveit_msgs.action import MoveGroup
            from rclpy.action import ActionClient

            # rclpy ActionClient has per-goal cancel APIs, not cancel-all, so WatchDog
            # uses the action cancel service with an all-zero GoalInfo goal_id.
            self._action_client = ActionClient(self.node, MoveGroup, "/move_action")
            self._cancel_client = self.node.create_client(
                CancelGoal,
                "/move_action/_action/cancel_goal",
                callback_group=self._callback_group,
            )
            self._CancelGoal = CancelGoal
            self._GoalInfo = GoalInfo

    def _publish_halt(self, step_id: str, monitor_id: str, detail: str) -> None:
        try:
            if self.event_bus is not None:
                self.event_bus.publish(make_event(
                    "halt",
                    HaltPayload(step_id=step_id, monitor_id=monitor_id, detail=detail),
                ))
        except Exception:
            pass
