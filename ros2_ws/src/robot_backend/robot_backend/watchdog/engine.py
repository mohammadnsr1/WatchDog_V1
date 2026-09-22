"""WatchDog monitor engine."""

from __future__ import annotations

import collections
import math
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


def _finite_or_none(value: Any) -> float | None:
    """Float for JSON, or None for a non-finite/absent threshold.

    The base MonitorPrimitive initialises threshold to NaN, and NaN is not
    representable in strict JSON -- it would reach the scorer as the literal
    NaN token or as a parse error depending on the reader. None says "this
    family has no scalar threshold" unambiguously.
    """
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


_PROVENANCE_FIELDS = (
    "threshold_source",
    "threshold_verified",
    "threshold_registry_field",
    "threshold_registry_value",
)


def _fire_provenance(monitor: Any) -> dict[str, Any]:
    """The arm-time provenance fields, for restating on the fire payload.

    Only the four provenance keys: provenance also carries the arm-time
    object_id, which must not overwrite the fire's own object_id (for
    effort_event the two are resolved at different moments and can differ).
    """
    provenance = getattr(monitor, "provenance", None) or {}
    return {k: provenance[k] for k in _PROVENANCE_FIELDS if k in provenance}


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
        # The part the gripper is holding, tracked from the executor's event
        # stream. The watchdog is a separate process and has no view of
        # MoveIt's attach state, so this is the only way to know. The span
        # opens at grasp_skill's skill_start (so a clamp-effort fire DURING
        # the close still attributes to the part being closed on), is
        # confirmed at skill_end ok=True, and closes at release_skill's
        # skill_start or a failed grasp.
        self._held_object_id: str | None = None
        self._held_confirmed: bool = False

    def arm(self, monitor: MonitorPrimitive) -> dict[str, Any]:
        with self.lock:
            if monitor.monitor_id in self.monitors:
                raise ValueError(f"monitor_id {monitor.monitor_id!r} is already armed.")
            if not monitor.armed_wall_ns:
                monitor.armed_wall_ns = wall_ns()
            self.monitors[monitor.monitor_id] = monitor
        provenance = dict(getattr(monitor, "provenance", None) or {})
        # provenance["object_id"] is the arm-time resolution done by the
        # server (which owns the registry and the held-part question); fall
        # back to the monitor's own subject so an arm that bypassed the
        # server still attributes as well as it can.
        object_id = provenance.pop("object_id", None) or monitor.subject_object_id
        payload = MonitorArmedPayload(
            monitor_id=monitor.monitor_id,
            monitor_type=monitor.monitor_type,
            params=monitor.params,
            clause=monitor.clause,
            halt_on_fire=monitor.halt_on_fire,
            object_id=object_id,
            threshold=_finite_or_none(monitor.threshold),
            comparison=monitor.comparison or "",
            reference_values=dict(monitor.reference_values),
            **provenance,
        )
        self._publish(make_event("monitor_armed", payload))
        # The tool result mirrors the event so mcp_calls_watchdog.jsonl is
        # self-sufficient for the arming sub-scores -- object, threshold and
        # provenance all readable without joining to events_watchdog.jsonl.
        return {
            "monitor_id": monitor.monitor_id,
            "monitor_type": monitor.monitor_type,
            "object_id": object_id,
            "threshold": _finite_or_none(monitor.threshold),
            "comparison": monitor.comparison or "",
            "reference_values": dict(monitor.reference_values),
            "threshold_source": payload.threshold_source,
            "threshold_verified": payload.threshold_verified,
            "threshold_registry_field": payload.threshold_registry_field,
            "threshold_registry_value": payload.threshold_registry_value,
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

    def on_skill_event(
        self,
        event_type: str,
        skill_name: str | None,
        params: dict[str, Any] | None,
        ok: Any = None,
    ) -> None:
        """React to an executor skill event observed on the shared event bus.

        Two mechanical consequences, both keyed on the event alone -- no
        cross-process call and nothing the agent has to be told to do:

          * grasp/release bracket the held span, so effort fires (which are
            joint-scoped) can be attributed to the part in the gripper.
          * a release ends the held span by definition, so the slip monitor
            watching that span comes down with it, before the opening jaws can
            register as a slip. release_skill emits skill_start before
            backend.gripper.open(), which is the margin this relies on.

        A reorient set-down is a release like any other and needs no special
        case; the re-pick arms a fresh slip monitor as usual.
        """
        if event_type == "skill_start" and skill_name == "grasp_skill":
            object_id = (params or {}).get("object_id")
            if isinstance(object_id, str):
                self._held_object_id = object_id
                self._held_confirmed = False
            return

        if event_type == "skill_end" and skill_name == "grasp_skill":
            if ok is True:
                self._held_confirmed = True
            else:
                # The grasp did not take: nothing is held.
                self._held_object_id = None
                self._held_confirmed = False
            return

        if event_type == "skill_start" and skill_name == "release_skill":
            object_id = (params or {}).get("object_id")
            if isinstance(object_id, str):
                self._release_disarm_slip_monitors(object_id)
                if self._held_object_id == object_id:
                    self._held_object_id = None
                    self._held_confirmed = False
            return

    def held_object_id(self) -> str | None:
        """Return the part currently in the gripper, or None."""
        return self._held_object_id

    def _release_disarm_slip_monitors(self, object_id: str) -> list[str]:
        """Disarm the released part's slip monitors, and only those.

        Scoped by subject: a stationarity monitor on some other part, an
        effort monitor, or a slip monitor on a different part all stay armed.
        The disarm is emitted with its own reason so the record distinguishes
        "torn down because the part was released" from an agent-initiated
        disarm and from never having been armed.
        """
        with self.lock:
            doomed = [
                monitor
                for monitor in self.monitors.values()
                if monitor.monitor_type == "relative_frame"
                and monitor.subject_object_id == object_id
            ]
            for monitor in doomed:
                self.monitors.pop(monitor.monitor_id, None)
        for monitor in doomed:
            self._emit_disarmed(monitor, "auto_disarm_release")
        if doomed:
            self._log_warn(
                "released %s: auto-disarmed %d slip monitor(s) %s"
                % (object_id, len(doomed), [m.monitor_id for m in doomed])
            )
        return [monitor.monitor_id for monitor in doomed]

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
        object_id = self._resolve_fire_object_id(monitor)
        payload = MonitorFirePayload(
            monitor_id=monitor.monitor_id,
            monitor_type=monitor.monitor_type,
            trigger_wall_ns=trigger_stamp,
            triggering_value=triggering_value,
            threshold=float(monitor.threshold),
            comparison=monitor.comparison,
            clause=self._clause_with_object_id(monitor.clause, object_id),
            object_id=object_id,
            # Carried from the arm, not re-derived: the audit question is
            # where the ARMED value came from.
            **_fire_provenance(monitor),
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

    def _resolve_fire_object_id(self, monitor: MonitorPrimitive) -> str | None:
        """Attribute a fire to an object structurally, not from the clause.

        One path, two sources. A monitor that is intrinsically about an object
        (relative_frame's child_frame, stationarity's and region's object_id)
        answers for itself. effort_event watches a joint and has no intrinsic
        object, so the question "which part was being grasped when the clamp
        bound was exceeded" is answered from cell state at fire time -- the
        held span tracked from the event stream. None is a real answer there:
        an effort fire with an empty gripper.
        """
        subject = monitor.subject_object_id
        if subject is not None:
            return subject
        return self._held_object_id

    @staticmethod
    def _clause_with_object_id(
        clause: dict[str, Any] | None, object_id: str | None
    ) -> dict[str, Any] | None:
        """Backfill object_id into the clause without mutating the monitor's."""
        if clause is None or object_id is None:
            return clause
        if clause.get("object_id") is not None:
            return clause
        enriched = dict(clause)
        enriched["object_id"] = object_id
        return enriched

    def _emit_disarmed(self, monitor: MonitorPrimitive, reason: str) -> None:
        self._publish(make_event(
            "monitor_disarmed",
            MonitorDisarmedPayload(
                monitor_id=monitor.monitor_id,
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
