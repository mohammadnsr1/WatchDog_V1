"""Tests for pose-based grasp verification in pick_object — no ROS required.

Covers:
  - _get_grasp_spec helper
  - _compute_distance_3d helper
  - _make_verification_lift_pose helper
  - _verify_grasp_by_pose: success, displacement failure, TCP proximity failure,
    post-lift TF lookup failure, no TCP frame available, fallback thresholds
  - pick_object integration:
    - tf_buffer=None → unconditional attach (backward-compatible)
    - reference pose capture failure → unconditional attach
    - successful grasp verification → attach + retreat
    - failed grasp (Z displacement) → detach + sync + structured failure result
    - failed grasp (TCP proximity) → detach + sync + structured failure result
    - retreat_pose used as verification lift (no second move)
    - minimal lift when no retreat, then retreat after successful verify
    - no grasp_pose and no retreat → verification skipped, attach unconditionally
  - _sanitize_params excludes registry and tf_buffer keys

Run with: pytest test/test_grasp_verification.py -v
"""

from __future__ import annotations

import math
from typing import Any
from unittest.mock import MagicMock, call

import pytest

from robot_backend.events.decorator import _sanitize_params
from robot_backend.skills.pick_place_skills import (
    _compute_distance_3d,
    _get_grasp_spec,
    _make_verification_lift_pose,
    _verify_grasp_by_pose,
    pick_object,
)

# Import the TransformException that pose_query_skills (and get_object_pose) catches.
# pose_query_skills defines its own stub when tf2_ros is absent; using that same class
# ensures _TFBufferStub exceptions are caught by the skill rather than propagating.
try:
    from tf2_ros import TransformException
except ImportError:
    from robot_backend.skills.pose_query_skills import TransformException  # type: ignore[no-redef,attr-defined]


# ---------------------------------------------------------------------------
# Shared test fixtures
# ---------------------------------------------------------------------------

_REGISTRY = {
    "objects": {
        "red_cube": {
            "tf_frame": "red_cube",
            "lookup_frame": "world",
            "spec": {
                "tolerances": {
                    "lift_height_min_m": 0.02,
                    "position_m": 0.005,
                    "orientation_deg": 5.0,
                },
            },
        },
        "green_cube": {
            "tf_frame": "green_cube",
            "lookup_frame": "world",
            # no spec block
        },
    }
}

_GRASP_POSE = {
    "frame_id": "world",
    "position": {"x": 0.5, "y": 0.0, "z": 0.05},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
}

_RETREAT_POSE = {
    "frame_id": "world",
    "position": {"x": 0.5, "y": 0.0, "z": 0.20},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
}


def _make_backend() -> MagicMock:
    """Return a backend mock with all required scene/motion/gripper stubs."""
    backend = MagicMock()
    backend.scene.sync_collision_objects.return_value = {"ok": True}
    backend.scene.attach_object.return_value = {"ok": True}
    backend.scene.detach_object.return_value = {"ok": True}
    backend.motion.move_to_pose.return_value = {"ok": True}
    backend.gripper.open.return_value = {"ok": True}
    backend.gripper.close.return_value = {"ok": True}
    return backend


def _make_transform(x: float, y: float, z: float) -> MagicMock:
    """Build a mock TransformStamped at position (x, y, z) with identity rotation."""
    t = MagicMock()
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    return t


class _TFBufferStub:
    """Minimal TF buffer stub for grasp verification tests.

    object_positions: successive (x, y, z) tuples returned for the object frame.
    tcp_position:     (x, y, z) returned for the TCP frame; None means TransformException.
    fail_object_after_lift: if True, the second object lookup raises TransformException.
    """

    def __init__(
        self,
        object_positions: list[tuple[float, float, float]],
        tcp_position: tuple[float, float, float] | None = (0.5, 0.0, 0.20),
        fail_object_after_lift: bool = False,
    ) -> None:
        self._object_iter = iter(object_positions)
        self._tcp_position = tcp_position
        self._fail_after_lift = fail_object_after_lift
        self._object_call_count = 0

    def lookup_transform(
        self, target_frame: str, source_frame: str, time: Any, timeout: Any = None
    ) -> MagicMock:
        if source_frame == "red_cube" or source_frame == "green_cube":
            self._object_call_count += 1
            if self._fail_after_lift and self._object_call_count > 1:
                raise TransformException("forced failure on second lookup")
            try:
                pos = next(self._object_iter)
            except StopIteration:
                raise TransformException("no more transforms in stub")
            return _make_transform(*pos)
        if source_frame == "tcp":
            if self._tcp_position is None:
                raise TransformException("no tcp frame in stub")
            return _make_transform(*self._tcp_position)
        raise TransformException(f"Unknown frame: {source_frame}")


# ---------------------------------------------------------------------------
# Unit tests for helpers
# ---------------------------------------------------------------------------

class TestGetGraspSpec:

    def test_returns_spec_when_present(self) -> None:
        spec = _get_grasp_spec(_REGISTRY, "red_cube")
        assert spec is not None
        assert "tolerances" in spec

    def test_returns_none_for_missing_object(self) -> None:
        assert _get_grasp_spec(_REGISTRY, "unknown_cube") is None

    def test_returns_none_when_no_spec_block(self) -> None:
        assert _get_grasp_spec(_REGISTRY, "green_cube") is None

    def test_returns_none_for_none_registry(self) -> None:
        assert _get_grasp_spec(None, "red_cube") is None

    def test_returns_none_for_empty_registry(self) -> None:
        assert _get_grasp_spec({}, "red_cube") is None


class TestComputeDistance3d:

    def test_same_position_is_zero(self) -> None:
        pose = {"position": {"x": 1.0, "y": 2.0, "z": 3.0}}
        assert _compute_distance_3d(pose, pose) == pytest.approx(0.0)

    def test_unit_x_offset(self) -> None:
        p1 = {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}
        p2 = {"position": {"x": 1.0, "y": 0.0, "z": 0.0}}
        assert _compute_distance_3d(p1, p2) == pytest.approx(1.0)

    def test_3d_pythagorean(self) -> None:
        p1 = {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}
        p2 = {"position": {"x": 1.0, "y": 1.0, "z": 1.0}}
        assert _compute_distance_3d(p1, p2) == pytest.approx(math.sqrt(3))


class TestMakeVerificationLiftPose:

    def test_z_increases_by_lift(self) -> None:
        base = {
            "frame_id": "world",
            "position": {"x": 0.5, "y": 0.0, "z": 0.05},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
        lifted = _make_verification_lift_pose(base, 0.05)
        assert lifted["position"]["z"] == pytest.approx(0.10)

    def test_x_y_unchanged(self) -> None:
        base = {
            "frame_id": "world",
            "position": {"x": 0.3, "y": -0.1, "z": 0.05},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
        lifted = _make_verification_lift_pose(base, 0.10)
        assert lifted["position"]["x"] == pytest.approx(0.3)
        assert lifted["position"]["y"] == pytest.approx(-0.1)

    def test_does_not_mutate_original(self) -> None:
        base = {
            "frame_id": "world",
            "position": {"x": 0.5, "y": 0.0, "z": 0.05},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
        original_z = base["position"]["z"]
        _make_verification_lift_pose(base, 0.10)
        assert base["position"]["z"] == original_z


# ---------------------------------------------------------------------------
# Unit tests for _verify_grasp_by_pose
# ---------------------------------------------------------------------------

class TestVerifyGraspByPose:

    _REF_POSE = {
        "position": {"x": 0.5, "y": 0.0, "z": 0.05},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }

    def test_success_sufficient_displacement_and_tcp_nearby(self) -> None:
        # Object lifted 0.15 m (> 0.02 spec minimum); TCP at same position.
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.20)],  # post-lift
            tcp_position=(0.5, 0.0, 0.20),
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="red_cube",
            reference_pose=self._REF_POSE,
            grasp_width_m=0.065,
        )
        assert result["ok"] is True
        assert result["measured_displacement_m"] == pytest.approx(0.15)
        assert result["required_displacement_m"] == pytest.approx(0.02)

    def test_failure_insufficient_displacement(self) -> None:
        # Object only moved 0.005 m (below 0.02 threshold).
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.055)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="red_cube",
            reference_pose=self._REF_POSE,
            grasp_width_m=0.065,
        )
        assert result["ok"] is False
        assert "did not follow" in result["reason"]
        assert result["measured_displacement_m"] == pytest.approx(0.005)
        assert result["required_displacement_m"] == pytest.approx(0.02)

    def test_failure_tcp_proximity_exceeded(self) -> None:
        # Object lifted enough but is 1 m away from TCP.
        buf = _TFBufferStub(
            object_positions=[(1.5, 0.0, 0.20)],  # 1 m away in X from TCP
            tcp_position=(0.5, 0.0, 0.20),
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="red_cube",
            reference_pose=self._REF_POSE,
            grasp_width_m=0.065,
        )
        assert result["ok"] is False
        assert "too far from TCP" in result["reason"]
        assert result["tcp_offset_3d_m"] == pytest.approx(1.0)

    def test_no_tcp_frame_skips_proximity_check(self) -> None:
        # TCP frame unavailable → proximity check is skipped, only displacement checked.
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.20)],
            tcp_position=None,  # TCP lookup will raise TransformException
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="red_cube",
            reference_pose=self._REF_POSE,
            grasp_width_m=0.065,
        )
        assert result["ok"] is True  # passes on displacement alone
        assert result["tcp_pose_after_lift"] is None
        assert result["tcp_offset_3d_m"] is None

    def test_post_lift_tf_lookup_failure_returns_unverified_failure(self) -> None:
        buf = _TFBufferStub(
            object_positions=[],  # no transforms available
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="red_cube",
            reference_pose=self._REF_POSE,
        )
        assert result["ok"] is False
        assert "Cannot verify" in result["reason"]
        assert result["measured_displacement_m"] is None

    def test_fallback_thresholds_when_no_spec(self) -> None:
        # green_cube has no spec → fallback lift_height_min_m=0.01
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.065)],  # 0.015 m lift > 0.01 fallback
            tcp_position=None,
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="green_cube",
            reference_pose=self._REF_POSE,
        )
        assert result["ok"] is True
        assert result["required_displacement_m"] == pytest.approx(0.01)

    def test_zero_displacement_fails(self) -> None:
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05)],  # same as reference
            tcp_position=(0.5, 0.0, 0.05),
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="red_cube",
            reference_pose=self._REF_POSE,
        )
        assert result["ok"] is False
        assert result["measured_displacement_m"] == pytest.approx(0.0)

    def test_negative_displacement_fails(self) -> None:
        # Object fell below its resting position.
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.01)],
        )
        result = _verify_grasp_by_pose(
            tf_buffer=buf,
            registry=_REGISTRY,
            object_id="red_cube",
            reference_pose=self._REF_POSE,
        )
        assert result["ok"] is False
        assert result["measured_displacement_m"] < 0


# ---------------------------------------------------------------------------
# Integration tests for pick_object
# ---------------------------------------------------------------------------

class TestPickObjectBackwardCompat:
    """When tf_buffer=None the skill must behave identically to the original."""

    def test_attaches_unconditionally_no_tf_buffer(self) -> None:
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        assert result["ok"] is True
        backend.scene.attach_object.assert_called_once()
        backend.scene.detach_object.assert_not_called()

    def test_steps_include_attach_and_retreat(self) -> None:
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        step_names = [s["step"] for s in result["steps"]]
        assert "attach_object" in step_names
        assert "move_to_retreat" in step_names

    def test_no_retreat_no_move_after_attach(self) -> None:
        backend = _make_backend()
        pick_object(backend, object_id="red_cube", grasp_pose=_GRASP_POSE)
        # move_to_pose called exactly once for the grasp; no retreat call
        backend.motion.move_to_pose.assert_called_once_with(_GRASP_POSE)


class TestPickObjectVerificationSuccess:

    def test_successful_grasp_stays_attached(self) -> None:
        # reference z=0.05, after lift z=0.20 → displacement 0.15 m ≥ 0.02
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.20)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        assert result["ok"] is True
        backend.scene.attach_object.assert_called_once()
        backend.scene.detach_object.assert_not_called()

    def test_steps_include_capture_verify_attach(self) -> None:
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.20)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        step_names = [s["step"] for s in result["steps"]]
        assert "capture_reference_pose" in step_names
        assert "attach_object" in step_names
        assert "move_to_verification_lift" in step_names
        assert "verify_grasp" in step_names

    def test_retreat_used_as_verification_lift_no_extra_move(self) -> None:
        """When retreat_pose is provided it is used as the lift — only one move_to_pose
        for the lift (not two: lift then separate retreat)."""
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.20)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        lift_calls = [
            c for c in backend.motion.move_to_pose.call_args_list
            if c == call(_RETREAT_POSE)
        ]
        # retreat_pose should appear exactly once (as the verification lift)
        assert len(lift_calls) == 1

    def test_minimal_lift_then_retreat_when_no_retreat_pose(self) -> None:
        """With no retreat_pose a minimal lift is synthesised; after passing verify
        the skill does NOT move again (no explicit retreat was requested)."""
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.10)],
            tcp_position=(0.5, 0.0, 0.10),
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            verification_lift_m=0.05,
            # no retreat_pose
        )
        assert result["ok"] is True
        # move_to_pose called twice: once for grasp, once for the minimal lift
        assert backend.motion.move_to_pose.call_count == 2

    def test_minimal_lift_then_retreat_when_retreat_provided(self) -> None:
        """With a retreat_pose but no grasp pose matching case: retreat is the lift.
        After successful verify no extra move needed."""
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.20)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        assert result["ok"] is True
        # grasp move + retreat (=lift) = 2 calls; no third call
        assert backend.motion.move_to_pose.call_count == 2


class TestPickObjectVerificationFailure:

    def test_grasp_failure_returns_ok_false(self) -> None:
        # Object stays at resting height after lift → displacement ≈ 0.
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.05)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        assert result["ok"] is False
        assert result.get("grasp_failed") is True

    def test_grasp_failure_detaches_and_syncs(self) -> None:
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.05)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        backend.scene.detach_object.assert_called_once_with("red_cube")
        # sync called: once at start + once on failure recovery
        assert backend.scene.sync_collision_objects.call_count == 2

    def test_grasp_failure_steps_include_detach_and_sync(self) -> None:
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.05)],
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        step_names = [s["step"] for s in result["steps"]]
        assert "detach_on_verify_failure" in step_names
        assert "sync_on_verify_failure" in step_names

    def test_grasp_failure_result_contains_measurements(self) -> None:
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (0.5, 0.0, 0.055)],  # 5 mm < 20 mm
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
            grasp_width_m=0.065,
        )
        assert result["ok"] is False
        assert result["measured_displacement_m"] == pytest.approx(0.005)
        assert result["required_displacement_m"] == pytest.approx(0.02)
        assert "reason" in result
        assert "object_pose_after_lift" in result

    def test_tcp_proximity_failure_detaches(self) -> None:
        # Object lifts correctly but ends up 1 m from TCP.
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05), (1.5, 0.0, 0.20)],
            tcp_position=(0.5, 0.0, 0.20),
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            grasp_width_m=0.065,
            retreat_pose=_RETREAT_POSE,
        )
        assert result["ok"] is False
        backend.scene.detach_object.assert_called_once()

    def test_reference_pose_failure_falls_back_to_unconditional_attach(self) -> None:
        """If the reference TF lookup fails before the pick, we cannot verify —
        the skill should attach unconditionally (safe degradation) rather than
        always reporting failure."""
        # TFBufferStub with no object positions → first lookup raises TransformException
        buf = _TFBufferStub(object_positions=[])
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            grasp_pose=_GRASP_POSE,
            retreat_pose=_RETREAT_POSE,
        )
        # Fallback: attach succeeded, no verification performed
        assert result["ok"] is True
        backend.scene.attach_object.assert_called_once()
        backend.scene.detach_object.assert_not_called()
        step_names = [s["step"] for s in result["steps"]]
        assert "capture_reference_pose_failed" in step_names

    def test_no_grasp_pose_and_no_retreat_skips_verification(self) -> None:
        """Without any pose to lift to, verification is skipped and attach is
        unconditional (the robot cannot provide a conclusive displacement check)."""
        buf = _TFBufferStub(
            object_positions=[(0.5, 0.0, 0.05)],
        )
        backend = _make_backend()
        result = pick_object(
            backend,
            object_id="red_cube",
            registry=_REGISTRY,
            tf_buffer=buf,
            # no grasp_pose, no retreat_pose
        )
        assert result["ok"] is True
        step_names = [s["step"] for s in result["steps"]]
        assert "grasp_verification_skipped" in step_names
        backend.scene.attach_object.assert_called_once()
        backend.scene.detach_object.assert_not_called()


# ---------------------------------------------------------------------------
# _sanitize_params excludes registry and tf_buffer
# ---------------------------------------------------------------------------

class TestSanitizeParamsExclusions:

    def test_registry_excluded(self) -> None:
        sanitized = _sanitize_params({
            "object_id": "red_cube",
            "registry": {"objects": {"red_cube": {}}},
            "grasp_width_m": 0.065,
        })
        assert "registry" not in sanitized
        assert sanitized["object_id"] == "red_cube"

    def test_tf_buffer_excluded(self) -> None:
        buf = _TFBufferStub(object_positions=[])
        sanitized = _sanitize_params({
            "tf_buffer": buf,
            "object_id": "red_cube",
        })
        assert "tf_buffer" not in sanitized
        assert sanitized["object_id"] == "red_cube"

    def test_other_keys_still_serialized(self) -> None:
        sanitized = _sanitize_params({
            "registry": {},
            "tf_buffer": MagicMock(),
            "object_id": "red_cube",
            "grasp_width_m": 0.065,
            "verification_lift_m": 0.05,
        })
        assert sanitized["object_id"] == "red_cube"
        assert sanitized["grasp_width_m"] == pytest.approx(0.065)
        assert sanitized["verification_lift_m"] == pytest.approx(0.05)
