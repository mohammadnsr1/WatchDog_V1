"""Tests for the pick_object contract schema and registry loader.

Runs without ROS — pure Python, no rclpy import required.
Run with: pytest test/test_pick_contract.py -v
"""

from __future__ import annotations

import textwrap
from pathlib import Path

import pytest
import yaml

from robot_backend.contracts.loader import load_pick_contract
from robot_backend.contracts.schema import (
    Clause,
    Comparison,
    PickContract,
    from_json,
    from_yaml,
    to_json,
    to_yaml,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def red_cube_contract() -> PickContract:
    return load_pick_contract("red_cube")


@pytest.fixture(scope="module")
def green_cube_contract() -> PickContract:
    return load_pick_contract("green_cube")


@pytest.fixture()
def minimal_registry(tmp_path: Path) -> Path:
    """A minimal registry with a full spec block, used for unit-level tests."""
    content = textwrap.dedent("""\
        objects:
          test_cube:
            enabled: true
            object_id: test_cube
            tf_frame: test_cube
            lookup_frame: world
            graspable: true
            collision:
              type: box
              dimensions: [0.05, 0.05, 0.05]
              pose_offset:
                position: {x: 0.0, y: 0.0, z: 0.0}
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            spec:
              grasp:
                grasp_center_offset:
                  position: {x: 0.0, y: 0.0, z: 0.0}
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                grasp_width_m: 0.055
                force_threshold_n: 0.42
              tolerances:
                position_m: 0.004
                orientation_deg: 3.0
                lift_height_min_m: 0.015
              limits:
                max_contact_force_n: 10.0
    """)
    registry = tmp_path / "model_registry.yaml"
    registry.write_text(content, encoding="utf-8")
    return registry


# ---------------------------------------------------------------------------
# Load-from-registry tests
# ---------------------------------------------------------------------------

class TestLoadFromRegistry:

    def test_loads_red_cube(self, red_cube_contract: PickContract) -> None:
        assert red_cube_contract.object_id == "red_cube"
        assert red_cube_contract.schema_name == "pick_object_v1"
        assert red_cube_contract.version == "1.0"

    def test_red_cube_grasp_width(self, red_cube_contract: PickContract) -> None:
        # From registry spec: grasp_width_m: 0.065
        assert red_cube_contract.grasp_width_m == pytest.approx(0.065)

    def test_green_cube_grasp_width(self, green_cube_contract: PickContract) -> None:
        # From registry spec: grasp_width_m: 0.045
        assert green_cube_contract.grasp_width_m == pytest.approx(0.045)

    def test_precondition_count(self, red_cube_contract: PickContract) -> None:
        assert len(red_cube_contract.preconditions) == 3

    def test_effect_count(self, red_cube_contract: PickContract) -> None:
        # pose effects (lift height, TCP offset) + grasp effects (effort plateau, width error)
        assert len(red_cube_contract.effects) == 4

    def test_invariant_count(self, red_cube_contract: PickContract) -> None:
        # hardware force limit + software effort cap
        assert len(red_cube_contract.invariants) == 2

    def test_precondition_thresholds_from_registry(self, red_cube_contract: PickContract) -> None:
        pos_clause = next(
            c for c in red_cube_contract.preconditions
            if c.field == "object_position_error_m"
        )
        ori_clause = next(
            c for c in red_cube_contract.preconditions
            if c.field == "object_orientation_error_deg"
        )
        # Both come from registry spec.tolerances (position_m=0.005, orientation_deg=5.0)
        assert pos_clause.threshold == pytest.approx(0.005)
        assert pos_clause.comparison == Comparison.LE
        assert ori_clause.threshold == pytest.approx(5.0)
        assert ori_clause.comparison == Comparison.LE

    def test_effect_lift_height_from_registry(self, red_cube_contract: PickContract) -> None:
        lift_clause = next(
            c for c in red_cube_contract.effects
            if c.field == "object_lift_height_m"
        )
        # From registry spec.tolerances.lift_height_min_m: 0.02
        assert lift_clause.threshold == pytest.approx(0.02)
        assert lift_clause.comparison == Comparison.GE

    def test_invariant_force_limit_from_registry(self, red_cube_contract: PickContract) -> None:
        force_clause = red_cube_contract.invariants[0]
        assert force_clause.field == "contact_force_n"
        # From registry spec.limits.max_contact_force_n: 20.0
        assert force_clause.threshold == pytest.approx(20.0)
        assert force_clause.comparison == Comparison.LE

    def test_green_cube_has_lower_force_limit(
        self,
        red_cube_contract: PickContract,
        green_cube_contract: PickContract,
    ) -> None:
        red_force = red_cube_contract.invariants[0].threshold
        green_force = green_cube_contract.invariants[0].threshold
        assert green_force < red_force

    # ------------------------------------------------------------------
    # force_threshold_n and new clauses
    # ------------------------------------------------------------------

    def test_red_cube_force_threshold_n(self, red_cube_contract: PickContract) -> None:
        # From registry spec.grasp.force_threshold_n: 0.50
        assert red_cube_contract.force_threshold_n == pytest.approx(0.50)

    def test_green_cube_force_threshold_n(self, green_cube_contract: PickContract) -> None:
        # From registry spec.grasp.force_threshold_n: 0.45
        assert green_cube_contract.force_threshold_n == pytest.approx(0.45)

    def test_green_cube_force_threshold_lower_than_red(
        self,
        red_cube_contract: PickContract,
        green_cube_contract: PickContract,
    ) -> None:
        assert green_cube_contract.force_threshold_n < red_cube_contract.force_threshold_n

    def test_grasp_effort_plateau_effect_clause(self, red_cube_contract: PickContract) -> None:
        clause = next(
            c for c in red_cube_contract.effects if c.field == "grasp_effort_plateau"
        )
        assert clause.comparison == Comparison.GE
        # Threshold = _GRASP_EFFORT_FLOOR = 0.05
        assert clause.threshold == pytest.approx(0.05)
        assert clause.unit == "sim_effort"

    def test_grasp_width_error_effect_clause(self, red_cube_contract: PickContract) -> None:
        clause = next(
            c for c in red_cube_contract.effects if c.field == "grasp_width_error_m"
        )
        assert clause.comparison == Comparison.LE
        # Threshold = _GRASP_WIDTH_TOL_M = 0.020
        assert clause.threshold == pytest.approx(0.020)
        assert clause.unit == "m"

    def test_grasp_effort_instantaneous_invariant(self, red_cube_contract: PickContract) -> None:
        clause = next(
            c for c in red_cube_contract.invariants
            if c.field == "grasp_effort_instantaneous"
        )
        assert clause.comparison == Comparison.LE
        # Threshold sourced from spec.grasp.force_threshold_n = 0.50
        assert clause.threshold == pytest.approx(0.50)
        assert clause.unit == "sim_effort"

    def test_grasp_effort_cap_differs_per_object(
        self,
        red_cube_contract: PickContract,
        green_cube_contract: PickContract,
    ) -> None:
        red_cap = next(
            c.threshold for c in red_cube_contract.invariants
            if c.field == "grasp_effort_instantaneous"
        )
        green_cap = next(
            c.threshold for c in green_cube_contract.invariants
            if c.field == "grasp_effort_instantaneous"
        )
        assert green_cap < red_cap

    def test_missing_object_raises(self) -> None:
        with pytest.raises(KeyError, match="not found in registry"):
            load_pick_contract("no_such_object")

    def test_no_spec_block_raises(self, minimal_registry: Path) -> None:
        # Temporarily write a registry without a spec block
        content = textwrap.dedent("""\
            objects:
              bare_cube:
                enabled: true
                object_id: bare_cube
                tf_frame: bare_cube
                lookup_frame: world
                graspable: true
                collision:
                  type: box
                  dimensions: [0.05, 0.05, 0.05]
                  pose_offset:
                    position: {x: 0.0, y: 0.0, z: 0.0}
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        """)
        registry = minimal_registry.parent / "no_spec.yaml"
        registry.write_text(content, encoding="utf-8")
        with pytest.raises(ValueError, match="no 'spec' block"):
            load_pick_contract("bare_cube", registry_path=registry)

    def test_custom_registry_path(self, minimal_registry: Path) -> None:
        contract = load_pick_contract("test_cube", registry_path=minimal_registry)
        assert contract.object_id == "test_cube"
        assert contract.grasp_width_m == pytest.approx(0.055)
        assert contract.force_threshold_n == pytest.approx(0.42)
        assert contract.invariants[0].threshold == pytest.approx(10.0)


# ---------------------------------------------------------------------------
# Serialization round-trip tests
# ---------------------------------------------------------------------------

class TestSerialization:

    def test_yaml_roundtrip(self, red_cube_contract: PickContract) -> None:
        serialized = to_yaml(red_cube_contract)
        recovered = from_yaml(serialized)
        assert recovered == red_cube_contract

    def test_json_roundtrip(self, red_cube_contract: PickContract) -> None:
        serialized = to_json(red_cube_contract)
        recovered = from_json(serialized)
        assert recovered == red_cube_contract

    def test_yaml_is_human_readable(self, red_cube_contract: PickContract) -> None:
        serialized = to_yaml(red_cube_contract)
        # Should contain field names and the object id as plain text
        assert "object_id" in serialized
        assert "red_cube" in serialized
        assert "preconditions" in serialized
        assert "effects" in serialized
        assert "invariants" in serialized

    def test_yaml_roundtrip_preserves_thresholds(self, red_cube_contract: PickContract) -> None:
        serialized = to_yaml(red_cube_contract)
        recovered = from_yaml(serialized)
        original_thresholds = [c.threshold for c in red_cube_contract.preconditions]
        recovered_thresholds = [c.threshold for c in recovered.preconditions]
        assert original_thresholds == pytest.approx(recovered_thresholds)

    def test_json_roundtrip_preserves_comparisons(self, red_cube_contract: PickContract) -> None:
        serialized = to_json(red_cube_contract)
        recovered = from_json(serialized)
        for orig, rec in zip(
            red_cube_contract.preconditions + red_cube_contract.effects + red_cube_contract.invariants,
            recovered.preconditions + recovered.effects + recovered.invariants,
        ):
            assert orig.comparison == rec.comparison

    def test_both_objects_roundtrip(
        self,
        red_cube_contract: PickContract,
        green_cube_contract: PickContract,
    ) -> None:
        for contract in (red_cube_contract, green_cube_contract):
            assert from_yaml(to_yaml(contract)) == contract
            assert from_json(to_json(contract)) == contract

    def test_frozen_contract_is_immutable(self, red_cube_contract: PickContract) -> None:
        with pytest.raises(Exception):
            red_cube_contract.object_id = "tampered"  # type: ignore[misc]
