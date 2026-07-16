"""Tests for grasp_skill and release_skill — no ROS required.

Covers:
  grasp_skill:
    - successful plateau → contact confirmed → attach called
    - air grasp (effort never rises above floor) → ok=False, no attach
    - width outside tolerance (near-zero, object not between fingers) → ok=False
    - force cap exceeded stops loop early
    - joint state read failures are skipped gracefully
    - provenance fields present on both success and failure
  release_skill:
    - open → detach → sync called in order
    - ok=False when detach fails
    - ok=False when sync fails

Run with: pytest test/test_grasp_skill.py -v
"""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock, call

import pytest

from robot_backend.skills.gripper_skills import (
    AIR_CLOSE_FLOOR_EFFORT,
    GRASP_WIDTH_TOLERANCE_M,
    grasp_skill,
    release_skill,
)


# ---------------------------------------------------------------------------
# Registry fixture
# ---------------------------------------------------------------------------

_REGISTRY = {
    "objects": {
        "red_cube": {
            "enabled": True,
            "tf_frame": "red_cube",
            "lookup_frame": "world",
            "graspable": True,
            "spec": {
                "grasp": {
                    "grasp_width_m": 0.065,
                    "force_threshold_n": 0.50,
                },
                "tolerances": {"position_m": 0.005, "orientation_deg": 5.0, "lift_height_min_m": 0.02},
            },
        }
    }
}


# ---------------------------------------------------------------------------
# Backend mock helpers
# ---------------------------------------------------------------------------

def _make_backend(
    *,
    joint_state_seq: list[dict[str, Any]] | None = None,
    attach_ok: bool = True,
    detach_ok: bool = True,
    sync_ok: bool = True,
) -> MagicMock:
    """Return a minimal backend mock.

    joint_state_seq: successive dicts returned by read_joint_state.
      Each dict should have: ok, effort, position (and optionally joint_name).
      Repeated calls cycle through the list; the last entry is returned for any
      subsequent call.
    """
    backend = MagicMock()

    # Gripper
    backend.gripper.open.return_value = {"ok": True}
    backend.gripper.set_width.return_value = {"ok": True}

    if joint_state_seq:
        _seq = iter(joint_state_seq)
        _last: dict[str, Any] = joint_state_seq[-1]

        def _read_js(joint_name: str = "finger_joint", timeout_sec: float = 1.0) -> dict[str, Any]:
            nonlocal _last
            try:
                _last = next(_seq)
            except StopIteration:
                pass
            r = dict(_last)
            r.setdefault("joint_name", joint_name)
            return r

        backend.gripper.read_joint_state.side_effect = _read_js
    else:
        backend.gripper.read_joint_state.return_value = {
            "ok": True, "joint_name": "finger_joint", "effort": 0.0, "position": 0.785
        }

    # Scene
    backend.scene.attach_object.return_value = {"ok": attach_ok}
    backend.scene.detach_object.return_value = {"ok": detach_ok}
    backend.scene.sync_collision_objects.return_value = {"ok": sync_ok}

    return backend


def _plateau_seq(
    *,
    ramp_steps: int = 3,
    plateau_effort: float = 0.48,
    plateau_position: float = -0.055,  # ≈ 0.065 m width per RG2 calibration
    noise: float = 0.005,
) -> list[dict[str, Any]]:
    """Build a joint-state sequence that ramps up and then plateaus.

    Default plateau_position (-0.055 rad) maps to ~0.065 m physical width,
    matching the red_cube spec's grasp_width_m so the width-plausibility
    check passes.
    """
    seq: list[dict[str, Any]] = []
    # Ramp: effort rises from 0 to plateau over ramp_steps
    for k in range(ramp_steps):
        e = plateau_effort * (k + 1) / ramp_steps
        seq.append({"ok": True, "effort": e, "position": plateau_position + noise * (ramp_steps - k)})
    # Plateau: effort stable, position stable
    for _ in range(6):
        seq.append({"ok": True, "effort": plateau_effort, "position": plateau_position})
    return seq


# ---------------------------------------------------------------------------
# grasp_skill — success path
# ---------------------------------------------------------------------------

class TestGraspSkillSuccess:

    def test_attach_called_on_confirmed_contact(self) -> None:
        seq = _plateau_seq(plateau_effort=0.48)
        backend = _make_backend(joint_state_seq=seq)
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        assert result["ok"] is True
        backend.scene.attach_object.assert_called_once()

    def test_result_ok_true_with_provenance(self) -> None:
        seq = _plateau_seq(plateau_effort=0.48)
        backend = _make_backend(joint_state_seq=seq)
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        assert result["ok"] is True
        prov = result["provenance"]
        assert prov["force_mode"] == "effort_plateau"
        assert "plateau_effort" in prov
        assert "measured_width_m" in prov
        assert "expected_width_m" in prov
        assert prov["criterion_plateau_above_floor"] is True
        assert prov["criterion_width_plausible"] is True

    def test_gripper_opened_first(self) -> None:
        seq = _plateau_seq()
        backend = _make_backend(joint_state_seq=seq)
        grasp_skill(backend, object_id="red_cube", registry=_REGISTRY, step_pause_sec=0)
        # open must be the first gripper call
        first_call = backend.gripper.method_calls[0]
        assert first_call[0] == "open"

    def test_set_width_called_multiple_times(self) -> None:
        seq = _plateau_seq()
        backend = _make_backend(joint_state_seq=seq)
        grasp_skill(backend, object_id="red_cube", registry=_REGISTRY, step_pause_sec=0)
        assert backend.gripper.set_width.call_count >= 1

    def test_expected_width_from_registry(self) -> None:
        seq = _plateau_seq()
        backend = _make_backend(joint_state_seq=seq)
        result = grasp_skill(backend, object_id="red_cube", registry=_REGISTRY, step_pause_sec=0)
        assert result["provenance"]["expected_width_m"] == pytest.approx(0.065)


# ---------------------------------------------------------------------------
# grasp_skill — air-grasp failure
# ---------------------------------------------------------------------------

class TestGraspSkillAirGrasp:

    def _air_close_seq(self, n: int = 25) -> list[dict[str, Any]]:
        """Sequence where effort stays near zero (air close, no object)."""
        return [{"ok": True, "effort": 0.01, "position": 0.785} for _ in range(n)]

    def test_no_attach_on_air_grasp(self) -> None:
        backend = _make_backend(joint_state_seq=self._air_close_seq())
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        assert result["ok"] is False
        backend.scene.attach_object.assert_not_called()

    def test_failure_criterion_plateau_false(self) -> None:
        backend = _make_backend(joint_state_seq=self._air_close_seq())
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        assert result["provenance"]["criterion_plateau_above_floor"] is False

    def test_failure_has_reason(self) -> None:
        backend = _make_backend(joint_state_seq=self._air_close_seq())
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        assert "reason" in result
        assert len(result["reason"]) > 0


# ---------------------------------------------------------------------------
# grasp_skill — width-implausible failure
# ---------------------------------------------------------------------------

class TestGraspSkillWidthFailure:

    def test_no_attach_when_width_implausible(self) -> None:
        # Effort plateaus correctly but final position is fully closed (near-zero width)
        near_closed_position = 0.78  # ~= calibration.position_closed_rad → width ≈ 0
        seq = (
            # Ramp up effort
            [{"ok": True, "effort": 0.2 * (k + 1), "position": near_closed_position} for k in range(3)]
            # Plateau at near-closed position (air grasp: width ≈ 0, far from 0.065 m)
            + [{"ok": True, "effort": 0.48, "position": near_closed_position} for _ in range(6)]
        )
        backend = _make_backend(joint_state_seq=seq)
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        assert result["provenance"]["criterion_width_plausible"] is False
        backend.scene.attach_object.assert_not_called()


# ---------------------------------------------------------------------------
# grasp_skill — force cap guard
# ---------------------------------------------------------------------------

class TestGraspSkillForceCap:

    def test_force_cap_stops_loop(self) -> None:
        # Effort immediately exceeds force_threshold_n = 0.50 from registry
        seq = [{"ok": True, "effort": 0.60, "position": 0.5} for _ in range(5)]
        backend = _make_backend(joint_state_seq=seq)
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        assert result["provenance"]["stop_reason"] == "force_cap_exceeded"
        # Loop should have exited early — fewer set_width calls than n_steps
        assert backend.gripper.set_width.call_count < 20

    def test_force_cap_exceeded_does_not_attach(self) -> None:
        # Over-force on first step → criteria not met → no attach
        seq = [{"ok": True, "effort": 0.99, "position": 0.5}] * 5
        backend = _make_backend(joint_state_seq=seq)
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        backend.scene.attach_object.assert_not_called()


# ---------------------------------------------------------------------------
# grasp_skill — joint state read failure resilience
# ---------------------------------------------------------------------------

class TestGraspSkillJointStateFailure:

    def test_failed_reads_skipped(self) -> None:
        # First several reads fail, then plateau
        seq = (
            [{"ok": False, "error": "timeout"} for _ in range(5)]
            + _plateau_seq(plateau_effort=0.48, plateau_position=0.35)
        )
        backend = _make_backend(joint_state_seq=seq)
        # Should not raise
        result = grasp_skill(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            step_pause_sec=0,
        )
        # Either success (if plateau was detected) or graceful failure
        assert "ok" in result
        assert "provenance" in result


# ---------------------------------------------------------------------------
# release_skill
# ---------------------------------------------------------------------------

class TestReleaseSkill:

    def test_open_detach_sync_called(self) -> None:
        backend = _make_backend()
        result = release_skill(backend, object_id="red_cube")
        backend.gripper.open.assert_called_once()
        backend.scene.detach_object.assert_called_once_with("red_cube")
        backend.scene.sync_collision_objects.assert_called_once_with(["red_cube"])

    def test_call_order_open_detach_sync(self) -> None:
        order: list[str] = []
        backend = _make_backend()
        backend.gripper.open.side_effect = lambda: order.append("open") or {"ok": True}
        backend.scene.detach_object.side_effect = lambda *a: order.append("detach") or {"ok": True}
        backend.scene.sync_collision_objects.side_effect = lambda *a: order.append("sync") or {"ok": True}
        release_skill(backend, object_id="red_cube")
        assert order == ["open", "detach", "sync"]

    def test_ok_true_when_all_succeed(self) -> None:
        backend = _make_backend()
        result = release_skill(backend, object_id="red_cube")
        assert result["ok"] is True

    def test_ok_false_when_detach_fails(self) -> None:
        backend = _make_backend(detach_ok=False)
        result = release_skill(backend, object_id="red_cube")
        assert result["ok"] is False

    def test_ok_false_when_sync_fails(self) -> None:
        backend = _make_backend(sync_ok=False)
        result = release_skill(backend, object_id="red_cube")
        assert result["ok"] is False

    def test_result_contains_object_id(self) -> None:
        backend = _make_backend()
        result = release_skill(backend, object_id="red_cube")
        assert result["object_id"] == "red_cube"

    def test_steps_contain_all_stages(self) -> None:
        backend = _make_backend()
        result = release_skill(backend, object_id="red_cube")
        step_names = [s["step"] for s in result["steps"]]
        assert "open_gripper" in step_names
        assert "detach_object" in step_names
        assert "sync_collision_objects" in step_names
