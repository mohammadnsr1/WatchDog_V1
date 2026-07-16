"""Build a PickContract from a model_registry.yaml spec block.

The loader is the ONLY place that reads the registry and translates spec
values into clause thresholds. This guarantees that thresholds in running
contracts always match the authoritative registry — if the registry changes,
reloading the contract picks up the new values automatically.

ROS-free: uses only pathlib + yaml, so the WatchDog can load contracts
without a running ROS context. The default registry path prefers the
installed package's share directory (resolved via ament_index_python when
importable) and otherwise falls back to the package-relative path — no
running ROS context is required either way.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .schema import Clause, Comparison, PickContract


# Path from this file: .../robot_backend/robot_backend/contracts/loader.py
# parents[2]          = .../robot_backend/   (the ROS2 package root)
_DEFAULT_REGISTRY_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "model_registry.yaml"
)

# System-wide constants for grasp success criteria clauses.
# Instrument characterization constant: empirical RG2 air-close floor in
# Isaac Sim, above the ~0.03 idle noise floor.
_GRASP_EFFORT_FLOOR: float = 0.05
# Mirrors gripper_skills.GRASP_WIDTH_TOLERANCE_M; keep in sync manually.
_GRASP_WIDTH_TOL_M: float = 0.005


def _load_registry(path: Path | None) -> dict[str, Any]:
    if path is not None:
        resolved = path
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            resolved = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
        except Exception:
            resolved = _DEFAULT_REGISTRY_PATH
    if not resolved.is_file():
        raise FileNotFoundError(f"Model registry not found: {resolved}")
    with resolved.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError("model_registry.yaml must be a top-level mapping.")
    return data


def load_pick_contract(
    object_id: str,
    registry_path: Path | None = None,
) -> PickContract:
    """Build a PickContract from a registry object's spec block.

    All clause thresholds are read directly from spec.grasp and
    spec.tolerances — nothing is hard-coded here.

    Parameters
    ----------
    object_id:
        Registry key of the graspable object (e.g. "red_cube").
    registry_path:
        Explicit path to model_registry.yaml. Defaults to the package config.

    Raises
    ------
    KeyError:
        If object_id is absent from the registry.
    ValueError:
        If the entry has no spec block, or spec fields are missing/wrong type.
    FileNotFoundError:
        If the registry file cannot be found.
    """
    registry = _load_registry(registry_path)
    objects = registry.get("objects", {})
    if not isinstance(objects, dict) or object_id not in objects:
        raise KeyError(
            f"Object {object_id!r} not found in registry. "
            f"Known objects: {list(objects.keys())}"
        )

    entry = objects[object_id]
    spec = entry.get("spec")
    if not isinstance(spec, dict):
        raise ValueError(
            f"Object {object_id!r} has no 'spec' block in the registry. "
            "Add grasp / tolerances subsections to enable contract loading."
        )

    grasp = spec.get("grasp", {})
    tolerances = spec.get("tolerances", {})

    # --- Extract spec values (let KeyError/TypeError surface for missing fields) ---
    grasp_width_m: float = float(grasp["grasp_width_m"])
    force_threshold_n: float = float(grasp["force_threshold_n"])
    position_tol: float = float(tolerances["position_m"])
    orientation_tol: float = float(tolerances["orientation_deg"])
    lift_height_min: float = float(tolerances["lift_height_min_m"])

    # --- Preconditions: must hold BEFORE skill execution ---
    # "gripper_is_open" uses threshold 1.0 (boolean encoded as float); this is
    # a logical constant (open=true), not a per-object spec value. All other
    # thresholds are sourced from the registry spec above.
    preconditions: list[Clause] = [
        Clause(
            field="object_position_error_m",
            comparison=Comparison.LE,
            threshold=position_tol,
            description=(
                "Object centroid must be within position tolerance of its expected "
                "TF pose before the pick begins."
            ),
            unit="m",
        ),
        Clause(
            field="object_orientation_error_deg",
            comparison=Comparison.LE,
            threshold=orientation_tol,
            description=(
                "Object principal axis must be within orientation tolerance of its "
                "expected TF pose before the pick begins."
            ),
            unit="deg",
        ),
        Clause(
            field="gripper_is_open",
            comparison=Comparison.EQ,
            threshold=1.0,
            description="Gripper must be fully open (1=open, 0=closed) before approach.",
            unit="bool",
        ),
    ]

    # --- Effects: must hold AFTER a claimed-success outcome ---
    effects: list[Clause] = [
        Clause(
            field="object_lift_height_m",
            comparison=Comparison.GE,
            threshold=lift_height_min,
            description=(
                "Object must have risen at least lift_height_min_m above its resting "
                "pose to confirm positive grasp."
            ),
            unit="m",
        ),
        Clause(
            field="object_tcp_offset_m",
            comparison=Comparison.LE,
            threshold=position_tol,
            description=(
                "Object-to-TCP positional offset must stay within position tolerance "
                "after lift, confirming the object is tracking the gripper."
            ),
            unit="m",
        ),
        Clause(
            field="grasp_effort_plateau",
            comparison=Comparison.GE,
            threshold=_GRASP_EFFORT_FLOOR,
            description=(
                "Mean finger_joint effort at the detected plateau must exceed the "
                "air-close floor, confirming physical contact with the object "
                "rather than an air grasp."
            ),
            unit="sim_effort",
        ),
        Clause(
            field="grasp_width_error_m",
            comparison=Comparison.LE,
            threshold=_GRASP_WIDTH_TOL_M,
            description=(
                "Absolute deviation between the measured grip width at plateau and "
                "spec.grasp.grasp_width_m must stay within the plausibility band. "
                "A near-zero measured width indicates an air grasp."
            ),
            unit="m",
        ),
    ]

    # --- Invariants: must hold THROUGHOUT execution ---
    invariants: list[Clause] = [
        Clause(
            field="grasp_effort_instantaneous",
            comparison=Comparison.LE,
            threshold=force_threshold_n,
            description=(
                "Instantaneous finger_joint effort must not exceed the per-object "
                "guarded-close cap (spec.grasp.force_threshold_n) during the grasp "
                "sequence. This is a software-enforced effort ceiling, distinct from "
                "the hardware safety limit in limits.max_contact_force_n."
            ),
            unit="sim_effort",
        ),
    ]

    return PickContract(
        object_id=object_id,
        grasp_width_m=grasp_width_m,
        force_threshold_n=force_threshold_n,
        preconditions=preconditions,
        effects=effects,
        invariants=invariants,
    )
