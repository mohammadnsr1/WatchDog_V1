"""Thin gripper skills that only depend on the backend contract."""

from __future__ import annotations

import time
from typing import Any, Sequence

from robot_backend.events.decorator import emit_skill_events
from robot_backend.interfaces.robot_backend import RobotBackend
from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION, joint_position_to_width


# All RG2 links that physically contact or surround a grasped object.
# Passed as touch_links to attach_object so MoveIt does not treat
# gripper-object contact as a collision during planning.
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

# ---------------------------------------------------------------------------
# Empirical constants for force-controlled guarded close (Isaac Sim / RG2)
# ---------------------------------------------------------------------------

# Observed solid-object grasp plateau: ~0.48 sim effort units (finger_joint).
# Measured by commanding close on a rigid cube at correct approach width.
# Air-close floor: effort when fingers close on nothing.
#   - In simulation physics, unloaded finger_joint effort sits at ~0.0 when
#     the fingers move freely and rises only when they contact a rigid surface.
#   - The value below (0.05) is a conservative floor: a true contact plateau
#     must exceed this to be considered a successful grasp.
#   - IMPORTANT: characterize this on your sim by running close() with no
#     object present and reading peak finger_joint effort via read_joint_state.
#     The observed air-close effort should be substantially below 0.05 if the
#     controller is well-tuned; adjust this constant if it is not.
AIR_CLOSE_FLOOR_EFFORT: float = 0.05

# Plateau detection: effort and position are considered "stable" when their
# range across the last PLATEAU_WINDOW readings stays below these deltas.
PLATEAU_WINDOW: int = 4
PLATEAU_EFFORT_DELTA: float = 0.02   # sim effort units
PLATEAU_POSITION_DELTA_RAD: float = 0.002  # radians

# Measured width tolerance for the "plausible object" check.
# If the actual gripper width at plateau deviates from spec.grasp.grasp_width_m
# by more than this, the grasp is treated as an air close or badly-positioned close.
GRASP_WIDTH_TOLERANCE_M: float = 0.020  # ±20 mm


def _get_grasp_spec(
    registry: dict[str, Any] | None,
    object_id: str,
) -> dict[str, Any] | None:
    """Return the 'spec' block for object_id from the registry, or None."""
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


@emit_skill_events
def grasp_skill(
    backend: RobotBackend,
    *,
    object_id: str,
    registry: dict[str, Any],
    joint_name: str = "finger_joint",
    attach_link_name: str = "tcp",
    touch_links: Sequence[str] | None = None,
    n_steps: int = 20,
    step_pause_sec: float = 0.10,
    plateau_window: int = PLATEAU_WINDOW,
    plateau_effort_delta: float = PLATEAU_EFFORT_DELTA,
    plateau_position_delta_rad: float = PLATEAU_POSITION_DELTA_RAD,
    air_close_floor: float = AIR_CLOSE_FLOOR_EFFORT,
    grasp_width_tolerance_m: float = GRASP_WIDTH_TOLERANCE_M,
) -> dict[str, Any]:
    """Force-controlled guarded close that attaches object_id on confirmed contact.

    Algorithm
    ---------
    1. Open the gripper fully.
    2. Step the commanded width linearly from open to closed in n_steps increments.
       After each step, pause step_pause_sec and read finger_joint effort + position.
    3. Plateau detection: when the range of effort AND position across the last
       plateau_window readings both fall below their respective deltas while
       effort is above air_close_floor, contact is confirmed and the loop stops.
    4. A force cap from spec.grasp.force_threshold_n is enforced: if effort
       exceeds the cap the loop stops immediately (over-force guard).
    5. Success criteria (both must pass to attach):
       - criterion_plateau_above_floor: final plateau effort > air_close_floor.
       - criterion_width_plausible: |measured_width - expected_width| <=
         grasp_width_tolerance_m (guards against near-zero air-grasp width).
    6. On success: attach the object via scene.attach_object.
       On failure: do NOT attach; return structured provenance.

    Provenance fields in the result
    --------------------------------
    force_mode            : "effort_plateau" (always, for future swappability)
    plateau_effort        : mean effort over the plateau window [sim units]
    measured_width_m      : width derived from final joint position
    expected_width_m      : spec.grasp.grasp_width_m
    force_threshold        : effective effort cap used
    criterion_plateau_above_floor : bool
    criterion_width_plausible     : bool
    steps_taken           : how many incremental steps were executed
    """
    steps_log: list[dict[str, Any]] = []
    calibration = DEFAULT_RG2_CALIBRATION

    # --- Spec lookup ---
    spec = _get_grasp_spec(registry, object_id)
    grasp_spec = spec.get("grasp", {}) if spec else {}
    expected_width_m: float = float(grasp_spec.get("grasp_width_m", 0.0))
    # force_threshold_n lives under spec.grasp (added in Item 4).
    # Fallback to a permissive cap so grasps work before Item 4 is deployed.
    force_threshold: float = float(grasp_spec.get("force_threshold_n", 1.0))

    # --- Open gripper ---
    open_result = backend.gripper.open()
    steps_log.append({"step": "open_gripper", "result": open_result})

    # --- Incremental guarded close ---
    width_open = calibration.width_open_m
    effort_history: list[float] = []
    position_history: list[float] = []
    stop_reason: str = "exhausted_steps"
    steps_taken: int = 0

    for i in range(n_steps):
        steps_taken = i + 1
        # Linear interpolation from open width to 0 (fully closed)
        t = steps_taken / n_steps
        target_width = width_open * (1.0 - t)
        backend.gripper.set_width(target_width)

        if step_pause_sec > 0:
            time.sleep(step_pause_sec)

        js = backend.gripper.read_joint_state(joint_name)
        steps_log.append({"step": f"read_joint_state[{i}]", "result": js})

        if not js["ok"]:
            # Can't read joint state — skip plateau logic this step.
            continue

        effort = js["effort"]
        position = js["position"]
        effort_history.append(effort)
        position_history.append(position)

        # Over-force guard
        if effort > force_threshold:
            stop_reason = "force_cap_exceeded"
            break

        # Plateau detection: need at least plateau_window readings
        if len(effort_history) >= plateau_window:
            w_efforts = effort_history[-plateau_window:]
            w_positions = position_history[-plateau_window:]
            effort_range = max(w_efforts) - min(w_efforts)
            pos_range = max(w_positions) - min(w_positions)
            if (
                effort_range <= plateau_effort_delta
                and pos_range <= plateau_position_delta_rad
                and w_efforts[-1] > air_close_floor
            ):
                stop_reason = "effort_plateau_detected"
                break

    # --- Final joint state reading ---
    final_js = backend.gripper.read_joint_state(joint_name)
    steps_log.append({"step": "final_joint_state", "result": final_js})

    measured_effort: float | None = None
    measured_width_m: float | None = None
    if final_js["ok"]:
        measured_effort = final_js["effort"]
        measured_width_m = joint_position_to_width(final_js["position"], calibration)

    # Plateau effort = mean over last window (or the single final value)
    if effort_history:
        window = effort_history[-plateau_window:]
        plateau_effort = sum(window) / len(window)
    else:
        plateau_effort = measured_effort or 0.0

    # --- Success criteria ---
    criterion_plateau_above_floor: bool = (
        plateau_effort > air_close_floor
    )
    criterion_width_plausible: bool = (
        measured_width_m is not None
        and abs(measured_width_m - expected_width_m) <= grasp_width_tolerance_m
    )
    grasp_ok: bool = criterion_plateau_above_floor and criterion_width_plausible

    provenance: dict[str, Any] = {
        "force_mode": "effort_plateau",
        "plateau_effort": plateau_effort,
        "measured_width_m": measured_width_m,
        "expected_width_m": expected_width_m,
        "force_threshold": force_threshold,
        "air_close_floor": air_close_floor,
        "stop_reason": stop_reason,
        "steps_taken": steps_taken,
        "criterion_plateau_above_floor": criterion_plateau_above_floor,
        "criterion_width_plausible": criterion_width_plausible,
    }

    if not grasp_ok:
        reason_parts = []
        if not criterion_plateau_above_floor:
            reason_parts.append(
                f"effort plateau {plateau_effort:.3f} did not exceed "
                f"air-close floor {air_close_floor:.3f} (air grasp or no contact)"
            )
        if not criterion_width_plausible:
            reason_parts.append(
                f"measured width {measured_width_m:.4f} m deviates from "
                f"expected {expected_width_m:.4f} m by more than "
                f"{grasp_width_tolerance_m:.4f} m tolerance"
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
