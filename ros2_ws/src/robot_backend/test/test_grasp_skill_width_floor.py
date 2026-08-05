"""Tests for grasp_skill's GRASP_MIN_PLAUSIBLE_WIDTH_M floor.

grasp_skill's own guarded-move call goes straight through rclpy's
ActionClient inside _call_guarded_move, so it needs a live action server to
exercise end-to-end. The floor logic itself does not: it only runs on the
result _call_guarded_move returns, so these tests patch that one function
with a fake result and drive grasp_skill's success-criteria decision — no
ROS action server, no live sim.

Run with: pytest test/test_grasp_skill_width_floor.py -v
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from robot_backend.skills.gripper_skills import (
    GRASP_MIN_PLAUSIBLE_WIDTH_M,
    grasp_skill,
)


def _fake_guarded_move_result(
    *,
    final_width_m: float,
    stop_reason: str = "force_threshold",
    final_effort: float = 0.5,
    asymmetry_detected: bool = False,
    success: bool = True,
    message: str = "",
) -> SimpleNamespace:
    return SimpleNamespace(
        success=success,
        stop_reason=stop_reason,
        final_width_m=final_width_m,
        final_effort=final_effort,
        asymmetry_detected=asymmetry_detected,
        message=message,
    )


def _make_backend() -> MagicMock:
    backend = MagicMock()
    backend.gripper.open.return_value = {"ok": True}
    backend.gripper.node = MagicMock()
    backend.scene.attach_object.return_value = {"ok": True}
    return backend


_PATCH_TARGET = "robot_backend.skills.gripper_skills._call_guarded_move"


class TestGraspMinPlausibleWidthFloor:

    def test_zero_width_jam_fails_and_does_not_attach(self) -> None:
        """A guarded move that jams fully closed (final_width_m=0.0) must
        never read as success, matching the u_green false-grasp: expected
        width omitted, fingers drove into the table, guarded move jammed
        closed at force_threshold."""
        backend = _make_backend()
        with patch(_PATCH_TARGET, return_value=_fake_guarded_move_result(final_width_m=0.0)):
            result = grasp_skill(
                backend,
                object_id="u_green",
                registry={},
                expected_width_m=None,
            )
        assert result["ok"] is False
        assert result["provenance"]["criterion_min_width_floor"] is False
        assert "floor" in result["reason"]
        backend.scene.attach_object.assert_not_called()

    def test_zero_width_jam_fails_even_with_expected_width_supplied(self) -> None:
        """The floor is independent of expected_width_m -- it must fire even
        when a width WAS supplied, since a jam is a physical impossibility,
        not a conformance question."""
        backend = _make_backend()
        with patch(_PATCH_TARGET, return_value=_fake_guarded_move_result(final_width_m=0.0)):
            result = grasp_skill(
                backend,
                object_id="u_green",
                registry={},
                expected_width_m=0.021,
            )
        assert result["ok"] is False
        assert result["provenance"]["criterion_min_width_floor"] is False
        backend.scene.attach_object.assert_not_called()

    def test_width_at_floor_fails(self) -> None:
        """Exactly at the floor is not plausible (strict '>' required)."""
        backend = _make_backend()
        with patch(
            _PATCH_TARGET,
            return_value=_fake_guarded_move_result(final_width_m=GRASP_MIN_PLAUSIBLE_WIDTH_M),
        ):
            result = grasp_skill(backend, object_id="u_green", registry={}, expected_width_m=None)
        assert result["ok"] is False
        assert result["provenance"]["criterion_min_width_floor"] is False

    def test_real_grasp_with_matching_expected_width_succeeds(self) -> None:
        """u_green's true width (0.021 m), well above the floor, with a
        matching expected_width_m: both the floor and the band pass."""
        backend = _make_backend()
        with patch(_PATCH_TARGET, return_value=_fake_guarded_move_result(final_width_m=0.021)):
            result = grasp_skill(
                backend,
                object_id="u_green",
                registry={},
                expected_width_m=0.021,
            )
        assert result["ok"] is True
        assert result["provenance"]["criterion_min_width_floor"] is True
        assert result["provenance"]["criterion_width_band"] is True
        backend.scene.attach_object.assert_called_once()

    def test_green_cube_sanity_real_grasp_no_expected_width(self) -> None:
        """Class-1 sanity: a real green_cube grasp (~0.040 m, far above the
        floor) still succeeds when no expected width is passed -- the floor
        only rejects near-zero widths, never a genuine object width."""
        backend = _make_backend()
        with patch(_PATCH_TARGET, return_value=_fake_guarded_move_result(final_width_m=0.040)):
            result = grasp_skill(
                backend,
                object_id="green_cube",
                registry={},
                expected_width_m=None,
            )
        assert result["ok"] is True
        assert result["provenance"]["criterion_min_width_floor"] is True
        backend.scene.attach_object.assert_called_once()

    def test_real_grasp_width_mismatch_fails_band_not_floor(self) -> None:
        """Above the floor but outside the expected-width band: fails on the
        band criterion specifically, not the floor -- provenance must
        distinguish the two failure modes."""
        backend = _make_backend()
        with patch(_PATCH_TARGET, return_value=_fake_guarded_move_result(final_width_m=0.021)):
            result = grasp_skill(
                backend,
                object_id="u_green",
                registry={},
                expected_width_m=0.040,
            )
        assert result["ok"] is False
        assert result["provenance"]["criterion_min_width_floor"] is True
        assert result["provenance"]["criterion_width_band"] is False
        backend.scene.attach_object.assert_not_called()

    def test_no_contact_air_grasp_still_fails_regardless_of_width(self) -> None:
        """Class-2 (deliberate air-grasp testing) must still fail honestly:
        no confirmed contact stop_reason fails even if width happened to
        read plausible."""
        backend = _make_backend()
        with patch(
            _PATCH_TARGET,
            return_value=_fake_guarded_move_result(final_width_m=0.021, stop_reason="timeout"),
        ):
            result = grasp_skill(
                backend,
                object_id="u_green",
                registry={},
                expected_width_m=0.021,
            )
        assert result["ok"] is False
        assert result["provenance"]["criterion_contact"] is False
        backend.scene.attach_object.assert_not_called()
