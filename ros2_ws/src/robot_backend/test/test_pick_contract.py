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

# grasp_width_m is instruction-sourced (task YAML steps: block), not
# registry-sourced -- these step_spec dicts stand in for that per-object,
# matching what used to be the registry's spec.grasp.grasp_width_m values.
RED_CUBE_STEP_SPEC = {"step_id": "pick_red_cube", "object_id": "red_cube", "grasp_width_m": 0.06}
GREEN_CUBE_STEP_SPEC = {"step_id": "pick_green_cube", "object_id": "green_cube", "grasp_width_m": 0.04}

_REGISTRY_FILE = Path(__file__).resolve().parents[1] / "config" / "model_registry.yaml"


def _registry_spec(object_id: str) -> dict:
    """The object's live spec block. The registry-backed tests below compare
    the CONTRACT against THIS, not against hardcoded numbers: the cube values
    were deliberately retuned once already (0.18 -> 70/95/130) and hardcoded
    expectations turned five loader tests into permanent known-failures that
    masked real regressions. What the loader owes is a faithful mapping."""
    data = yaml.safe_load(_REGISTRY_FILE.read_text(encoding="utf-8"))
    return data["objects"][object_id]["spec"]


@pytest.fixture(scope="module")
def red_cube_contract() -> PickContract:
    return load_pick_contract("red_cube", RED_CUBE_STEP_SPEC)


@pytest.fixture(scope="module")
def green_cube_contract() -> PickContract:
    return load_pick_contract("green_cube", GREEN_CUBE_STEP_SPEC)


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
            class_name: cube
            source: test
            graspable: true
            collision:
              type: box
              dimensions: [0.05, 0.05, 0.05]
              pose_offset:
                position: {x: 0.0, y: 0.0, z: 0.0}
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            spec:
              grasp:
                force_threshold_n: 0.42
              tolerances:
                position_m: 0.004
                orientation_deg: 3.0
                lift_height_min_m: 0.015
                grasp_width_tolerance_m: 0.005
              limits:
                max_clamp_effort: 10.0
                grasp_effort_floor: 0.05
    """)
    registry = tmp_path / "model_registry.yaml"
    registry.write_text(content, encoding="utf-8")
    return registry


TEST_CUBE_STEP_SPEC = {"step_id": "pick_test_cube", "object_id": "test_cube", "grasp_width_m": 0.055}


# ---------------------------------------------------------------------------
# Load-from-registry tests
# ---------------------------------------------------------------------------

class TestLoadFromRegistry:

    def test_loads_red_cube(self, red_cube_contract: PickContract) -> None:
        assert red_cube_contract.object_id == "red_cube"
        assert red_cube_contract.schema_name == "pick_object_v1"
        assert red_cube_contract.version == "1.0"

    def test_red_cube_grasp_width(self, red_cube_contract: PickContract) -> None:
        # From RED_CUBE_STEP_SPEC (task instruction step), not the registry
        assert red_cube_contract.grasp_width_m == pytest.approx(0.06)

    def test_green_cube_grasp_width(self, green_cube_contract: PickContract) -> None:
        # From GREEN_CUBE_STEP_SPEC (task instruction step), not the registry
        assert green_cube_contract.grasp_width_m == pytest.approx(0.04)

    def test_precondition_count(self, red_cube_contract: PickContract) -> None:
        assert len(red_cube_contract.preconditions) == 3

    def test_effect_count(self, red_cube_contract: PickContract) -> None:
        # pose effects (lift height, TCP offset) + grasp effects (effort plateau, width error)
        assert len(red_cube_contract.effects) == 4

    def test_invariant_count(self, red_cube_contract: PickContract) -> None:
        # software effort cap only — there is no monitorable hardware force limit
        assert len(red_cube_contract.invariants) == 1

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

    def test_grasp_effort_invariant_matches_each_cubes_registry_limit(
        self,
        red_cube_contract: PickContract,
        green_cube_contract: PickContract,
    ) -> None:
        # invariants[0] is grasp_effort_instantaneous, sourced from
        # spec.limits.max_clamp_effort. The old assertion (green < red) pinned
        # an ordering of values that were later retuned; the loader's actual
        # obligation is that each contract carries ITS object's registry limit.
        for object_id, contract in (("red_cube", red_cube_contract),
                                    ("green_cube", green_cube_contract)):
            assert contract.invariants[0].threshold == pytest.approx(
                _registry_spec(object_id)["limits"]["max_clamp_effort"]
            ), object_id

    # ------------------------------------------------------------------
    # force_threshold_n and new clauses
    # ------------------------------------------------------------------

    def test_red_cube_force_threshold_n(self, red_cube_contract: PickContract) -> None:
        # From registry spec.grasp.force_threshold_n, read live.
        assert red_cube_contract.force_threshold_n == pytest.approx(
            _registry_spec("red_cube")["grasp"]["force_threshold_n"]
        )

    def test_green_cube_force_threshold_n(self, green_cube_contract: PickContract) -> None:
        # From registry spec.grasp.force_threshold_n, read live.
        assert green_cube_contract.force_threshold_n == pytest.approx(
            _registry_spec("green_cube")["grasp"]["force_threshold_n"]
        )

    def test_green_cube_force_threshold_matches_red(
        self,
        red_cube_contract: PickContract,
        green_cube_contract: PickContract,
    ) -> None:
        assert green_cube_contract.force_threshold_n == red_cube_contract.force_threshold_n

    def test_grasp_effort_plateau_effect_clause(self, red_cube_contract: PickContract) -> None:
        clause = next(
            c for c in red_cube_contract.effects if c.field == "grasp_effort_plateau"
        )
        assert clause.comparison == Comparison.GE
        # Threshold sourced from registry spec.limits.grasp_effort_floor: 0.05
        assert clause.threshold == pytest.approx(0.05)
        assert clause.unit == "sim_effort"

    def test_grasp_width_error_effect_clause(self, red_cube_contract: PickContract) -> None:
        clause = next(
            c for c in red_cube_contract.effects if c.field == "grasp_width_error_m"
        )
        assert clause.comparison == Comparison.LE
        # Threshold sourced from registry spec.tolerances.grasp_width_tolerance_m: 0.005
        assert clause.threshold == pytest.approx(0.005)
        assert clause.unit == "m"

    def test_grasp_effort_floor_and_width_tolerance_come_from_registry(
        self, minimal_registry: Path
    ) -> None:
        """Both thresholds are read from the registry, not a Python constant --
        changing the registry value changes the loaded contract's threshold.
        """
        content = textwrap.dedent("""\
            objects:
              test_cube:
                enabled: true
                object_id: test_cube
                tf_frame: test_cube
                lookup_frame: world
                class_name: cube
                source: test
                graspable: true
                collision:
                  type: box
                  dimensions: [0.05, 0.05, 0.05]
                  pose_offset:
                    position: {x: 0.0, y: 0.0, z: 0.0}
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                spec:
                  grasp:
                    force_threshold_n: 0.42
                  tolerances:
                    position_m: 0.004
                    orientation_deg: 3.0
                    lift_height_min_m: 0.015
                    grasp_width_tolerance_m: 0.011
                  limits:
                    max_clamp_effort: 10.0
                    grasp_effort_floor: 0.09
        """)
        registry = minimal_registry.parent / "custom_thresholds.yaml"
        registry.write_text(content, encoding="utf-8")
        contract = load_pick_contract("test_cube", TEST_CUBE_STEP_SPEC, registry_path=registry)
        plateau = next(c for c in contract.effects if c.field == "grasp_effort_plateau")
        width_err = next(c for c in contract.effects if c.field == "grasp_width_error_m")
        assert plateau.threshold == pytest.approx(0.09)
        assert width_err.threshold == pytest.approx(0.011)

    def test_missing_grasp_effort_floor_raises(self, tmp_path: Path) -> None:
        content = textwrap.dedent("""\
            objects:
              test_cube:
                enabled: true
                object_id: test_cube
                tf_frame: test_cube
                lookup_frame: world
                class_name: cube
                source: test
                graspable: true
                collision:
                  type: box
                  dimensions: [0.05, 0.05, 0.05]
                  pose_offset:
                    position: {x: 0.0, y: 0.0, z: 0.0}
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                spec:
                  grasp:
                    force_threshold_n: 0.42
                  tolerances:
                    position_m: 0.004
                    orientation_deg: 3.0
                    lift_height_min_m: 0.015
                    grasp_width_tolerance_m: 0.005
                  limits:
                    max_clamp_effort: 10.0
        """)
        registry = tmp_path / "no_effort_floor.yaml"
        registry.write_text(content, encoding="utf-8")
        with pytest.raises(KeyError):
            load_pick_contract("test_cube", TEST_CUBE_STEP_SPEC, registry_path=registry)

    def test_missing_grasp_width_tolerance_raises(self, tmp_path: Path) -> None:
        content = textwrap.dedent("""\
            objects:
              test_cube:
                enabled: true
                object_id: test_cube
                tf_frame: test_cube
                lookup_frame: world
                class_name: cube
                source: test
                graspable: true
                collision:
                  type: box
                  dimensions: [0.05, 0.05, 0.05]
                  pose_offset:
                    position: {x: 0.0, y: 0.0, z: 0.0}
                    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
                spec:
                  grasp:
                    force_threshold_n: 0.42
                  tolerances:
                    position_m: 0.004
                    orientation_deg: 3.0
                    lift_height_min_m: 0.015
                  limits:
                    max_clamp_effort: 10.0
                    grasp_effort_floor: 0.05
        """)
        registry = tmp_path / "no_width_tol.yaml"
        registry.write_text(content, encoding="utf-8")
        with pytest.raises(KeyError):
            load_pick_contract("test_cube", TEST_CUBE_STEP_SPEC, registry_path=registry)

    def test_grasp_effort_instantaneous_invariant(self, red_cube_contract: PickContract) -> None:
        clause = next(
            c for c in red_cube_contract.invariants
            if c.field == "grasp_effort_instantaneous"
        )
        assert clause.comparison == Comparison.LE
        # Threshold sourced from spec.limits.max_clamp_effort, read live.
        assert clause.threshold == pytest.approx(
            _registry_spec("red_cube")["limits"]["max_clamp_effort"]
        )
        assert clause.unit == "sim_effort"

    def test_grasp_effort_cap_matches_per_object_registry_values(
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
        # Per-object values, read live (the two cubes deliberately differ).
        assert red_cap == pytest.approx(
            _registry_spec("red_cube")["limits"]["max_clamp_effort"]
        )
        assert green_cap == pytest.approx(
            _registry_spec("green_cube")["limits"]["max_clamp_effort"]
        )

    def test_missing_object_raises(self) -> None:
        step_spec = {"step_id": "x", "object_id": "no_such_object", "grasp_width_m": 0.05}
        with pytest.raises(KeyError, match="not found in registry"):
            load_pick_contract("no_such_object", step_spec)

    def test_missing_step_spec_grasp_width_raises(self) -> None:
        with pytest.raises(ValueError, match="grasp_width_m"):
            load_pick_contract("red_cube", {"step_id": "x", "object_id": "red_cube"})

    def test_no_spec_block_raises(self, minimal_registry: Path) -> None:
        # Temporarily write a registry without a spec block
        content = textwrap.dedent("""\
            objects:
              bare_cube:
                enabled: true
                object_id: bare_cube
                tf_frame: bare_cube
                lookup_frame: world
                class_name: cube
                source: test
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
        step_spec = {"step_id": "x", "object_id": "bare_cube", "grasp_width_m": 0.05}
        with pytest.raises(ValueError, match="no 'spec' block"):
            load_pick_contract("bare_cube", step_spec, registry_path=registry)

    # ------------------------------------------------------------------
    # contract_clause_groups (optional step_spec field)
    # ------------------------------------------------------------------

    def test_no_clause_groups_key_yields_all_three(
        self, red_cube_contract: PickContract
    ) -> None:
        # RED_CUBE_STEP_SPEC carries no contract_clause_groups -- default is
        # all three, unchanged from before this field existed.
        assert len(red_cube_contract.preconditions) == 3
        assert len(red_cube_contract.effects) == 4
        assert len(red_cube_contract.invariants) == 1

    def test_clause_groups_subset_omits_preconditions(self) -> None:
        step_spec = {
            **RED_CUBE_STEP_SPEC,
            "contract_clause_groups": ["effects", "invariants"],
        }
        contract = load_pick_contract("red_cube", step_spec)
        assert contract.preconditions == []
        assert len(contract.effects) == 4
        assert len(contract.invariants) == 1

    def test_clause_groups_does_not_change_thresholds(self) -> None:
        # Same clause_id/threshold/comparison as the full contract -- only
        # which groups are INCLUDED changes, never the clause definitions.
        full = load_pick_contract("red_cube", RED_CUBE_STEP_SPEC)
        step_spec = {
            **RED_CUBE_STEP_SPEC,
            "contract_clause_groups": ["effects", "invariants"],
        }
        subset = load_pick_contract("red_cube", step_spec)
        assert subset.effects == full.effects
        assert subset.invariants == full.invariants

    def test_clause_groups_empty_list_yields_no_clauses_and_warns(self) -> None:
        step_spec = {**RED_CUBE_STEP_SPEC, "contract_clause_groups": []}
        with pytest.warns(UserWarning, match="no clauses at all"):
            contract = load_pick_contract("red_cube", step_spec)
        assert contract.preconditions == []
        assert contract.effects == []
        assert contract.invariants == []

    def test_clause_groups_not_a_list_raises(self) -> None:
        step_spec = {**RED_CUBE_STEP_SPEC, "contract_clause_groups": "effects"}
        with pytest.raises(ValueError, match="red_cube"):
            load_pick_contract("red_cube", step_spec)

    def test_clause_groups_unknown_value_raises(self) -> None:
        step_spec = {
            **RED_CUBE_STEP_SPEC,
            "contract_clause_groups": ["effects", "bogus"],
        }
        with pytest.raises(ValueError, match="bogus"):
            load_pick_contract("red_cube", step_spec)

    def test_custom_registry_path(self, minimal_registry: Path) -> None:
        contract = load_pick_contract("test_cube", TEST_CUBE_STEP_SPEC, registry_path=minimal_registry)
        assert contract.object_id == "test_cube"
        assert contract.grasp_width_m == pytest.approx(0.055)
        assert contract.force_threshold_n == pytest.approx(0.42)
        assert contract.max_clamp_effort == pytest.approx(10.0)
        # invariants[0] is grasp_effort_instantaneous, sourced from
        # spec.limits.max_clamp_effort (10.0), not force_threshold_n.
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
            red_cube_contract.preconditions
            + red_cube_contract.effects
            + red_cube_contract.invariants,
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
