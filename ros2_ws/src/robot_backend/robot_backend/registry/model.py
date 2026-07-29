"""Frozen typed models for the authoritative object registry."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_backend.utils.geometry import PoseDict


@dataclass(frozen=True)
class CollisionSpec:
    """Validated box collision geometry for Phase 0.5."""

    type: str
    dimensions: tuple[float, float, float]


@dataclass(frozen=True)
class RegisteredObject:
    """One validated object and its lazily resolved collision geometry."""

    object_id: str
    tf_frame: str
    lookup_frame: str
    class_name: str
    source: str
    graspable: bool
    enabled: bool
    spec: dict[str, Any]
    collision: CollisionSpec
    pose_offset: PoseDict

    def resolve_aabb(self) -> tuple[dict[str, float], dict[str, float]]:
        """Return collision AABB ``(center, half_extents)`` in object space."""
        from .geometry import resolve_box_aabb

        return resolve_box_aabb(self.collision.dimensions, self.pose_offset)


@dataclass(frozen=True)
class Registry:
    """Validated registry plus its exact source and pass-through YAML mapping."""

    raw_text: str
    raw_dict: dict[str, Any]
    asset_root: Path
    objects: dict[str, RegisteredObject]

    @property
    def raw_sha256(self) -> str:
        """SHA-256 of the same UTF-8 registry bytes represented by raw_text."""
        return hashlib.sha256(self.raw_text.encode("utf-8")).hexdigest()
