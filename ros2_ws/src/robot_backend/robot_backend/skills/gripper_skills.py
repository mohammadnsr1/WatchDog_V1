"""Thin gripper skills that only depend on the backend contract."""

from __future__ import annotations

from typing import Any, Sequence

import rclpy
from rclpy.action import ActionClient

from robot_backend.events.decorator import emit_skill_events
from robot_backend.interfaces.robot_backend import RobotBackend
from isaac_trajectory_bridge_interfaces.action import GripperGuardedMove


# All RG2 links that physically contact or surround a grasped object.
RG2_TOUCH_LINKS: list[str] = [
    "onrobot_rg2_base_link",
    "onrobot_rg2_left_outer_knuckle",
    "onrobot_rg2_left_inner_knuckle",
    "onrobot_rg2_left_inner_finger",
    "onrobot_rg2_right_outer_knuckle",
    "onrobot_rg2_right_inner_knuckle",
    "onrobot_rg2_right_inner_finger",
    "tcp",
]

GRASP_WIDTH_TOLERANCE_M: float = 0.010   # ±10 mm post-grasp width check
# Clearance applied to the safety-floor target so the floor sits inside the
# object, guaranteeing mechanical contact before the floor clamp stops progress.
GRASP_CLEARANCE_M: float = 0.003
DEFAULT_FORCE_THRESHOLD: float = 0.45
DEFAULT_WIDTH_TOLERANCE_M: float = 0.003
GUARDED_MOVE_ACTION = "/rg2_controller/guarded_move"
ACTION_SERVER_TIMEOUT_SEC: float = 5.0
ACTION_EXECUTE_TIMEOUT_SEC: float = 30.0


def _get_grasp_spec(
    registry: dict[str, Any] | None,
    object_id: str,
) -> dict[str, Any] | None:
    if not registry:
        return None
    entry = registry.get("objects", {}).get(object_id)
    if not isinstance(entry, dict):
        return None
    return entry.get("spec")


# ---------------------------------------------------------------------------
# Primitive skills
# ---------------------------------------------------------------------------


def open_gripper_skill(backend: RobotBackend) -> Any:
    """Open the gripper via the backend only."""
    return backend.gripper.open()


def close_gripper_skill(backend: RobotBackend) -> Any:
    """Close the gripper via the backend only."""
    return backend.gripper.close()


def set_gripper_width_skill(backend: RobotBackend, width_m: float) -> Any:
    """Set the target gripper width in meters via the backend only."""
    return backend.gripper.set_width(width_m)


# ---------------------------------------------------------------------------
# Action client helper
# ---------------------------------------------------------------------------


def _resolve_guarded_move_action_name(backend: RobotBackend) -> str:
    """Resolve the guarded-move endpoint from the backend when available."""

    action_name = getattr(backend.gripper, "guarded_move_action_name", None)
    if isinstance(action_name, str) and action_name.strip():
        return action_name.strip()
    return GUARDED_MOVE_ACTION


def _call_guarded_move(
    node,
    *,
    action_name: str,
    target_width_m: float,
    force_threshold: float,
    mode: str = "torque",
    width_tolerance_m: float = DEFAULT_WIDTH_TOLERANCE_M,
) -> GripperGuardedMove.Result:
    """Send a GripperGuardedMove goal and block until a result arrives."""
    action_client = ActionClient(node, GripperGuardedMove, action_name)

    if not action_client.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
        raise RuntimeError(
            f"GripperGuardedMove action server not available at {action_name} "
            f"within {ACTION_SERVER_TIMEOUT_SEC}s."
        )

    goal = GripperGuardedMove.Goal()
    goal.target_width_m = float(target_width_m)
    goal.force_threshold = float(force_threshold)
    goal.mode = mode
    goal.width_tolerance_m = float(width_tolerance_m)

    send_future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=ACTION_SERVER_TIMEOUT_SEC)

    if not send_future.done():
        raise TimeoutError("Timed out sending GripperGuardedMove goal.")

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        raise RuntimeError("GripperGuardedMove goal was rejected by the action server.")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=ACTION_EXECUTE_TIMEOUT_SEC)

    if not result_future.done():
        raise TimeoutError("GripperGuardedMove action timed out waiting for result.")

    return result_future.result().result


# ---------------------------------------------------------------------------
# grasp_skill — uses GripperGuardedMove action
# ---------------------------------------------------------------------------


@emit_skill_events
def grasp_skill(
    backend: RobotBackend,
    *,
    object_id: str,
    registry: dict[str, Any],
    attach_link_name: str = "tcp",
    touch_links: Sequence[str] | None = None,
    # Legacy params kept for call-site compatibility; not used in action path
    joint_name: str = "finger_joint",
    n_steps: int = 20,
    step_pause_sec: float = 0.10,
    plateau_window: int = 4,
    plateau_effort_delta: float = 0.02,
    plateau_position_delta_rad: float = 0.002,
    air_close_floor: float = 0.05,
    grasp_width_tolerance_m: float = GRASP_WIDTH_TOLERANCE_M,
) -> dict[str, Any]:
    """Torque-probed grasp via the GripperGuardedMove action server (mode='torque').

    A successful grasp stops on 'force_threshold' (effort-based contact).
    The final width is checked against the spec's expected grasp_width_m
    (±grasp_width_tolerance_m) as a sanity bound, not a stop condition.
    """
    steps_log: list[dict[str, Any]] = []

    spec = _get_grasp_spec(registry, object_id)
    grasp_spec = spec.get("grasp", {}) if spec else {}
    expected_width_m: float = float(grasp_spec.get("grasp_width_m", 0.0))
    force_threshold: float = float(grasp_spec.get("force_threshold_n", DEFAULT_FORCE_THRESHOLD))

    # --- Open gripper before approaching ---
    open_result = backend.gripper.open()
    steps_log.append({"step": "open_gripper", "result": open_result})

    # --- Call guarded-move action ---
    try:
        node = backend.gripper.node
    except AttributeError:
        return {
            "ok": False,
            "command": "grasp",
            "object_id": object_id,
            "reason": "backend.gripper.node not available — ROSGripperBackend required for grasp_skill",
            "provenance": {},
            "steps": steps_log,
        }

    action_name = _resolve_guarded_move_action_name(backend)

    try:
        action_result = _call_guarded_move(
            node,
            action_name=action_name,
            target_width_m=max(0.0, expected_width_m - GRASP_CLEARANCE_M),
            force_threshold=force_threshold,
            mode="torque",
            width_tolerance_m=DEFAULT_WIDTH_TOLERANCE_M,
        )
        steps_log.append({
            "step": "guarded_move_action",
            "result": {
                "success": action_result.success,
                "stop_reason": action_result.stop_reason,
                "final_width_m": action_result.final_width_m,
                "final_effort": action_result.final_effort,
                "asymmetry_detected": action_result.asymmetry_detected,
                "message": action_result.message,
            },
        })
    except Exception as exc:
        steps_log.append({"step": "guarded_move_action", "error": str(exc)})
        return {
            "ok": False,
            "command": "grasp",
            "object_id": object_id,
            "reason": f"GripperGuardedMove action failed: {exc}",
            "provenance": {"action_name": action_name, "expected_width_m": expected_width_m},
            "steps": steps_log,
        }

    final_width = action_result.final_width_m
    final_effort = action_result.final_effort
    stop_reason = action_result.stop_reason

    # --- Success criteria ---
    # torque mode: contact is confirmed only by effort-threshold; width is a sanity bound only.
    criterion_contact = stop_reason == "force_threshold"
    criterion_width_plausible = abs(final_width - expected_width_m) <= grasp_width_tolerance_m

    provenance: dict[str, Any] = {
        "force_mode": "guarded_move_action",
        "action_name": action_name,
        "stop_reason": stop_reason,
        "final_width_m": final_width,
        "final_effort": final_effort,
        "expected_width_m": expected_width_m,
        "force_threshold": force_threshold,
        "asymmetry_detected": action_result.asymmetry_detected,
        "criterion_contact": criterion_contact,
        "criterion_width_plausible": criterion_width_plausible,
    }

    grasp_ok = criterion_contact and criterion_width_plausible

    if not grasp_ok:
        reason_parts = []
        if not criterion_contact:
            reason_parts.append(
                f"stop_reason '{stop_reason}' indicates no confirmed contact "
                f"(expected 'force_threshold')"
            )
        if not criterion_width_plausible:
            reason_parts.append(
                f"final width {final_width:.4f}m deviates from expected "
                f"{expected_width_m:.4f}m by more than {grasp_width_tolerance_m:.4f}m tolerance"
            )
        return {
            "ok": False,
            "command": "grasp",
            "object_id": object_id,
            "reason": "; ".join(reason_parts),
            "provenance": provenance,
            "steps": steps_log,
        }

    # --- Attach object ---
    resolved_touch_links = list(touch_links) if touch_links is not None else list(RG2_TOUCH_LINKS)
    attach_result = backend.scene.attach_object(
        object_id,
        attach_link_name,
        touch_links=resolved_touch_links,
    )
    steps_log.append({"step": "attach_object", "result": attach_result})

    return {
        "ok": attach_result.get("ok", False),
        "command": "grasp",
        "object_id": object_id,
        "attach_link_name": attach_link_name,
        "provenance": provenance,
        "steps": steps_log,
    }


@emit_skill_events
def release_skill(
    backend: RobotBackend,
    *,
    object_id: str,
) -> dict[str, Any]:
    """Open the gripper, detach the object, and sync its TF pose into the world scene."""
    steps_log: list[dict[str, Any]] = []

    open_result = backend.gripper.open()
    steps_log.append({"step": "open_gripper", "result": open_result})

    detach_result = backend.scene.detach_object(object_id)
    steps_log.append({"step": "detach_object", "result": detach_result})

    sync_result = backend.scene.sync_collision_objects([object_id])
    steps_log.append({"step": "sync_collision_objects", "result": sync_result})

    return {
        "ok": detach_result.get("ok", False) and sync_result.get("ok", False),
        "command": "release",
        "object_id": object_id,
        "steps": steps_log,
    }
