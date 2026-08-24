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

# Post-grasp width sanity check: how far final_width_m may deviate from the
# caller-supplied expected width (sourced from the task instruction's step,
# not the registry) before a "successful" contact is rejected as implausible.
# Distinct from the GripperGuardedMove goal's own position tolerance, which we
# don't override — see NO_WIDTH_TOLERANCE_OVERRIDE below.
GRASP_WIDTH_TOLERANCE_M: float = 0.005   # ±5 mm post-grasp width check
# Physical impossibility floor, independent of expected_width_m: below this,
# the fingers are effectively closed on nothing (jam or air close), not
# holding an object, regardless of what any caller expected or whether they
# passed a width at all. RG2Calibration.width_closed_m is exactly 0.0 m
# (utils/rg2_geometry.py), so a jam/air close that drives to the mechanical
# stop reads at or near 0.0 m; 3 mm gives headroom over that stop plus sensor/
# backlash noise while sitting far below the smallest known graspable object
# (u_green, 21 mm) minus its width tolerance (5 mm) -- 16 mm of margin, no
# real grasp can be mistaken for a jam. A true grasp of ANY registered object
# plateaus at its real width, well above this floor.
GRASP_MIN_PLAUSIBLE_WIDTH_M: float = 0.003   # 3 mm; see rationale above
# Clearance applied to the safety-floor target so the floor sits inside the
# object, guaranteeing mechanical contact before the floor clamp stops progress.
# Shared across EVERY object -- this is not a per-object registry field, so
# raising it raises the settled-effort ceiling (see the ceiling-law ~1.15
# effort-units/mm relationship in model_registry.yaml's header comment) for
# every grasp, not just the one you're tuning. Was 0.003 (3mm), chosen
# because its ~3.43 measured ceiling sat under red_cube/green_cube's
# max_clamp_effort=4.0; at 0.005 (5mm) the ceiling measures ~5.54 on
# green_cube, ABOVE that 4.0 bound -- red_cube/green_cube grasps will now
# violate their own max_clamp_effort by design until those registry values
# are raised too (u_green's was raised to 50.0, comfortably clear of 5.54).
GRASP_CLEARANCE_M: float = 0.003
# Fallback only, used when the registry entry lacks force_threshold_n.
# Matches the controller's hard-guard default (DEFAULT_HARD_FORCE_THRESHOLD in
# rg2_controller.py); per-object tuned overrides belong in the registry's
# spec.grasp.force_threshold_n, not here.
DEFAULT_FORCE_THRESHOLD: float = 0.5
# Mirrors rg2_controller.CONTACT_STOP_REASONS (isaac_trajectory_bridge). Not
# imported directly: robot_backend only depends on
# isaac_trajectory_bridge_interfaces, not the isaac_trajectory_bridge package
# itself, and importing rg2_controller would pull in its full ROS Node stack.
# Source of truth is rg2_controller.py — keep this set in manual sync with it.
# Both reasons indicate confirmed physical contact, differing only in HOW it
# was detected (threshold crossed vs effort stalled above noise floor).
CONTACT_STOP_REASONS: frozenset[str] = frozenset({"force_threshold", "effort_plateau"})
# Sentinel meaning "no override" for goal.width_tolerance_m: the controller
# treats <= 0.0 as "use my own verify_tolerance_m parameter" (rg2_controller.py
# DEFAULT_VERIFY_TOLERANCE_M). The tolerance is defined once, on the controller
# side, so it stays in sync with whatever open/close/set_position use.
NO_WIDTH_TOLERANCE_OVERRIDE: float = 0.0
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


@emit_skill_events
def open_gripper_skill(backend: RobotBackend) -> Any:
    """Open the gripper via the backend only."""
    return backend.gripper.open()


@emit_skill_events
def close_gripper_skill(backend: RobotBackend) -> Any:
    """Close the gripper via the backend only."""
    return backend.gripper.close()


@emit_skill_events
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
    width_tolerance_m: float = NO_WIDTH_TOLERANCE_OVERRIDE,
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
    expected_width_m: float | None = None,
    grasp_width_tolerance_m: float = GRASP_WIDTH_TOLERANCE_M,
) -> dict[str, Any]:
    """Torque-probed grasp via the GripperGuardedMove action server (mode='torque').

    A successful grasp stops on 'force_threshold' or 'effort_plateau'
    (both are effort-based confirmed contact — see CONTACT_STOP_REASONS).

    expected_width_m is the caller-supplied expected RG2 opening at contact,
    sourced from the task instruction's step (agents/tasks/*.yaml's steps:
    block), never from the registry. When provided, the final width is also
    checked against it (±grasp_width_tolerance_m) as a sanity bound, not a
    stop condition. When omitted, only the plausibility band is skipped.

    Independent of expected_width_m, a fully-closed or air-closed gripper
    (observed final_width_m at or below GRASP_MIN_PLAUSIBLE_WIDTH_M) is never
    reported as a success: that is a physical impossibility check against the
    observed width alone, so a forgotten or absent expected width degrades to
    an honest failure instead of a false success on a jam.
    """
    steps_log: list[dict[str, Any]] = []

    spec = _get_grasp_spec(registry, object_id)
    grasp_spec = spec.get("grasp", {}) if spec else {}
    force_threshold: float = float(grasp_spec.get("force_threshold_n", DEFAULT_FORCE_THRESHOLD))
    target_width_m = (
        max(0.0, expected_width_m - GRASP_CLEARANCE_M)
        if expected_width_m is not None
        else 0.0
    )

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
            target_width_m=target_width_m,
            force_threshold=force_threshold,
            mode="torque",
            width_tolerance_m=NO_WIDTH_TOLERANCE_OVERRIDE,
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
    # torque mode: contact is confirmed by either effort-based stop reason.
    # Width has two independent checks:
    #   - criterion_min_width_floor: a PHYSICAL impossibility check against
    #     the observed final_width_m only (never a contract/registry/
    #     instruction value). A gripper at or below this floor is closed on
    #     nothing -- jammed or air-closed -- regardless of what any caller
    #     expected or whether a width was passed at all. Applies always.
    #   - criterion_width_band: a plausibility band against the caller-
    #     supplied expected_width_m, only checked when it was provided.
    criterion_contact = stop_reason in CONTACT_STOP_REASONS
    criterion_min_width_floor = final_width > GRASP_MIN_PLAUSIBLE_WIDTH_M
    criterion_width_band = (
        True
        if expected_width_m is None
        else abs(final_width - expected_width_m) <= grasp_width_tolerance_m
    )
    criterion_width_plausible = criterion_min_width_floor and criterion_width_band

    provenance: dict[str, Any] = {
        "force_mode": "guarded_move_action",
        "action_name": action_name,
        "stop_reason": stop_reason,
        "final_width_m": final_width,
        "final_effort": final_effort,
        "expected_width_m": expected_width_m,
        "grasp_min_plausible_width_m": GRASP_MIN_PLAUSIBLE_WIDTH_M,
        "force_threshold": force_threshold,
        "asymmetry_detected": action_result.asymmetry_detected,
        "criterion_contact": criterion_contact,
        "criterion_min_width_floor": criterion_min_width_floor,
        "criterion_width_band": criterion_width_band,
        "criterion_width_plausible": criterion_width_plausible,
    }

    grasp_ok = criterion_contact and criterion_width_plausible

    if not grasp_ok:
        reason_parts = []
        if not criterion_contact:
            reason_parts.append(
                f"stop_reason '{stop_reason}' indicates no confirmed contact "
                f"(expected one of {sorted(CONTACT_STOP_REASONS)})"
            )
        if not criterion_min_width_floor:
            reason_parts.append(
                f"observed width below plausible floor: object not held "
                f"(jam/air close) -- final width {final_width:.4f}m <= "
                f"{GRASP_MIN_PLAUSIBLE_WIDTH_M:.4f}m floor"
            )
        if criterion_min_width_floor and not criterion_width_band:
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
