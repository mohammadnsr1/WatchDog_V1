"""WatchDog monitor engine."""

from __future__ import annotations

import collections
import threading
from typing import Any

from robot_backend.events.schema import (
    MonitorArmedPayload,
    MonitorDisarmedPayload,
    MonitorFirePayload,
    make_event,
)

from .monitors import MonitorPrimitive
from .timebase import wall_ns


class MonitorEngine:
    """Evaluate armed monitor primitives from recorder callbacks."""

    def __init__(
        self,
        event_bus: Any,
        halt_client: Any | None = None,
        wake_event: threading.Event | None = None,
        logger: Any | None = None,
    ) -> None:
        self.event_bus = event_bus
        self.halt_client = halt_client
        self.wake_event = wake_event or threading.Event()
        self.logger = logger
        self.lock = threading.Lock()
        self.monitors: dict[str, MonitorPrimitive] = {}
        self._fire_queue: collections.deque[dict[str, Any]] = collections.deque(maxlen=1000)

    def arm(self, monitor: MonitorPrimitive) -> dict[str, Any]:
        with self.lock:
            if monitor.monitor_id in self.monitors:
                raise ValueError(f"monitor_id {monitor.monitor_id!r} is already armed.")
            if not monitor.armed_wall_ns:
                monitor.armed_wall_ns = wall_ns()
            self.monitors[monitor.monitor_id] = monitor
        self._publish(make_event(
            "monitor_armed",
            MonitorArmedPayload(
                monitor_id=monitor.monitor_id,
                monitor_type=monitor.monitor_type,
                step_id=monitor.step_id,
                params=monitor.params,
                clause=monitor.clause,
                halt_on_fire=monitor.halt_on_fire,
            ),
        ))
        return {
            "monitor_id": monitor.monitor_id,
            "monitor_type": monitor.monitor_type,
            "step_id": monitor.step_id,
            "reference_values": dict(monitor.reference_values),
        }

    def disarm(self, monitor_id: str, reason: str) -> bool:
        with self.lock:
            monitor = self.monitors.pop(monitor_id, None)
        if monitor is None:
            return False
        self._emit_disarmed(monitor, reason)
        return True

    def disarm_all(self, reason: str) -> list[str]:
        with self.lock:
            monitors = list(self.monitors.values())
            self.monitors.clear()
        for monitor in monitors:
            self._emit_disarmed(monitor, reason)
        return [monitor.monitor_id for monitor in monitors]

    def on_tf_tick(self, recorder: Any) -> None:
        self._evaluate_feed("tf", recorder)

    def on_joint_state(self, msg: Any) -> None:
        self._evaluate_feed("joint_states", msg)

    def pop_fires(self) -> list[dict[str, Any]]:
        with self.lock:
            fires = list(self._fire_queue)
            self._fire_queue.clear()
        return fires

    def snapshot(self) -> list[dict[str, Any]]:
        with self.lock:
            monitors = list(self.monitors.values())
        return [
            {
                "monitor_id": monitor.monitor_id,
                "monitor_type": monitor.monitor_type,
                "step_id": monitor.step_id,
                "params": dict(monitor.params),
                "clause": monitor.clause,
                "halt_on_fire": monitor.halt_on_fire,
                "armed_wall_ns": monitor.armed_wall_ns,
            }
            for monitor in monitors
        ]

    def _evaluate_feed(self, feed: str, sample: Any) -> None:
        with self.lock:
            monitors = [m for m in self.monitors.values() if m.feed == feed]
        for monitor in monitors:
            try:
                triggering_value = monitor.evaluate(sample)
            except Exception as exc:
                self._log_warn(f"monitor {monitor.monitor_id} raised: {exc}")
                self.disarm(monitor.monitor_id, "internal_error")
                continue
            if triggering_value is not None:
                self._handle_fire(monitor, float(triggering_value))

    def _handle_fire(self, monitor: MonitorPrimitive, triggering_value: float) -> None:
        trigger_stamp = wall_ns()
        action_taken = "flag"
        halt_result: dict[str, Any] | None = None
        if monitor.halt_on_fire and self.halt_client is not None:
            try:
                halt_result = self.halt_client.halt(
                    monitor.step_id,
                    monitor.monitor_id,
                    f"monitor {monitor.monitor_id} fired",
                )
            except Exception as exc:
                self._log_warn(f"halt client raised for {monitor.monitor_id}: {exc}")
                halt_result = {"ok": False, "detail": str(exc), "goals_cancelled": None}
            # action_taken reports what the halt actually achieved, not merely
            # that it was attempted: goals_cancelled distinguishes "cancelled
            # something active" from "the cancel landed but nothing was
            # active to stop" from "the cancel itself failed or timed out".
            if not halt_result.get("ok"):
                action_taken = "halt_failed"
            elif halt_result.get("goals_cancelled"):
                action_taken = "halt"
            else:
                action_taken = "halt_no_active_goal"
        payload = MonitorFirePayload(
            monitor_id=monitor.monitor_id,
            monitor_type=monitor.monitor_type,
            step_id=monitor.step_id,
            trigger_wall_ns=trigger_stamp,
            triggering_value=triggering_value,
            threshold=float(monitor.threshold),
            comparison=monitor.comparison,
            clause=monitor.clause,
            action_taken=action_taken,
            halt_result=halt_result,
        )
        fire_dict = payload.model_dump(mode="json")
        with self.lock:
            current = self.monitors.pop(monitor.monitor_id, None)
            if current is None:
                return
            self._fire_queue.append(fire_dict)
        self._publish(make_event("monitor_fire", payload))
        self._emit_disarmed(monitor, "fired")
        self.wake_event.set()

    def _emit_disarmed(self, monitor: MonitorPrimitive, reason: str) -> None:
        self._publish(make_event(
            "monitor_disarmed",
            MonitorDisarmedPayload(
                monitor_id=monitor.monitor_id,
                step_id=monitor.step_id,
                reason=reason,
            ),
        ))

    def _publish(self, event: Any) -> None:
        try:
            if self.event_bus is not None:
                self.event_bus.publish(event)
        except Exception as exc:
            self._log_warn(f"WatchDog event publish failed: {exc}")

    def _log_warn(self, message: str) -> None:
        logger = self.logger
        if logger is not None:
            try:
                logger.warning(message)
            except Exception:
                pass

