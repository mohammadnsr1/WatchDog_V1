"""Pydantic contract schema for pick_object skill conformance checking.

Design notes:
- All models are frozen (immutable after construction) to match the frozen-dataclass
  style used by RobotBackend, RG2Calibration, and the backend interfaces.
- Clause.threshold must always come from the registry spec; the loader is the only
  place that reads the registry and constructs clause instances.
- Comparison is a str-enum so YAML/JSON round-trips preserve human-readable strings.
- to_yaml / from_yaml and to_json / from_json are module-level functions (not class
  methods) so the schema stays a plain data model with no I/O logic embedded.
"""

from __future__ import annotations

import json
from enum import Enum
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict


class Comparison(str, Enum):
    """Relational operator for a contract clause: field <op> threshold."""

    LT = "lt"
    LE = "le"
    GE = "ge"
    GT = "gt"
    EQ = "eq"


class Clause(BaseModel):
    """A single checkable assertion: field <comparison> threshold.

    field:       sensor / computed field name that the WatchDog will evaluate.
    comparison:  relational operator (see Comparison enum).
    threshold:   the bound, always sourced from the registry spec block.
    description: human-readable explanation of what is being checked.
    unit:        physical unit of field and threshold (for display / logging).
    joint_name:  structured joint id for clauses whose unit is "sim_effort",
                 sourced from cell.gripper.effort_joint_name in the registry
                 (see loader.py). None for clauses that concern no joint.
                 Exists so effort_event's required joint_name parameter can be
                 read from the contract rather than parsed out of description.
    """

    model_config = ConfigDict(frozen=True)

    field: str
    comparison: Comparison
    threshold: float
    description: str = ""
    unit: str = ""
    joint_name: str | None = None


class PickContract(BaseModel):
    """Serializable contract for a pick_object skill invocation.

    Preconditions are evaluated BEFORE the skill runs.
    Effects       are evaluated AFTER a claimed-success outcome.
    Invariants    are evaluated THROUGHOUT execution (continuous constraints).

    grasp_width_m is carried here so the WatchDog can compare it against
    what the executor actually reported in the event log (provenance check).

    force_threshold_n and max_clamp_effort are different quantities carried
    for different reasons. force_threshold_n is the stop TRIGGER the executor
    was configured with — a provenance value the WatchDog can compare against
    what the guarded close actually used, not a conformance bound. It can only
    stop the close early; it never limits achievable effort. max_clamp_effort
    is the conformance BOUND — what the object tolerates — and is the
    threshold actually used by the grasp_effort_instantaneous invariant.
    """

    model_config = ConfigDict(frozen=True)

    schema_name: str = "pick_object_v1"
    version: str = "1.0"
    object_id: str
    grasp_width_m: float
    force_threshold_n: float
    max_clamp_effort: float
    preconditions: list[Clause]
    effects: list[Clause]
    invariants: list[Clause]


# ---------------------------------------------------------------------------
# Serialization helpers
# ---------------------------------------------------------------------------


def to_yaml(contract: PickContract) -> str:
    """Serialize a contract to a YAML string.

    Uses mode="json" so Comparison enums are emitted as plain strings
    ("le", "ge", etc.) rather than Python-specific YAML tags that
    safe_load cannot read back.
    """
    return yaml.dump(
        contract.model_dump(mode="json"), sort_keys=False, allow_unicode=True
    )


def from_yaml(data: str) -> PickContract:
    """Deserialize a contract from a YAML string."""
    return PickContract.model_validate(yaml.safe_load(data))


def to_json(contract: PickContract) -> str:
    """Serialize a contract to a pretty-printed JSON string."""
    return contract.model_dump_json(indent=2)


def from_json(data: str) -> PickContract:
    """Deserialize a contract from a JSON string."""
    return PickContract.model_validate_json(data)
