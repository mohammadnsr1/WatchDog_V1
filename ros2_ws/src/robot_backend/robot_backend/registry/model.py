"""Frozen typed models for the authoritative object registry."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock
from typing import Any, Literal

from robot_backend.utils.geometry import PoseDict


Vector3Tuple = tuple[float, float, float]
Triangle = tuple[int, int, int]


@dataclass(frozen=True)
class CollisionSpec:
    """Validated box collision geometry; retained as the Phase 0.5 API."""

    type: Literal["box"]
    dimensions: Vector3Tuple


BoxCollisionSpec = CollisionSpec


@dataclass(frozen=True)
class MeshCollisionSpec:
    """Validated authored and resolved mesh collision geometry."""

    type: Literal["mesh"]
    mesh_path: str
    resolved_mesh_path: Path
    scale: Vector3Tuple


CollisionVariant = CollisionSpec | MeshCollisionSpec


@dataclass(frozen=True)
class MeshGeometry:
    """Post-scale triangle mesh and its collision-frame bounds."""

    vertices: tuple[Vector3Tuple, ...]
    faces: tuple[Triangle, ...]
    aabb_center: Vector3Tuple
    aabb_half_extents: Vector3Tuple
    file_sha256: str


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
    kind: str
    spec: dict[str, Any]
    collision: CollisionVariant
    pose_offset: PoseDict

    @property
    def is_frame(self) -> bool:
        """True for a pose-only landmark that must never become collision geometry.

        `enabled` and `kind` are two independent axes and conflating them was a
        real bug (see the ACM/warm-up notes in model_registry.yaml). `enabled`
        means "the stage publishes this, so its pose is queryable and the
        recorder should buffer it". `kind` means "does this entry denote a
        physical body". A placement/hole/reorient frame is enabled and queryable
        yet has no body, so it is excluded from the planning scene and from the
        ACM, while still being available to the seating check.
        """
        return self.kind == "frame"

    @property
    def is_object(self) -> bool:
        """True for an entry denoting a physical body with collision geometry."""
        return self.kind == "object"
    _mesh_geometry: MeshGeometry | None = field(
        default=None, init=False, repr=False, compare=False
    )
    _mesh_lock: Lock = field(default_factory=Lock, init=False, repr=False, compare=False)

    def resolve_mesh_geometry(self) -> MeshGeometry:
        """Load and cache the post-scale mesh, or raise for a non-mesh object."""
        if not isinstance(self.collision, MeshCollisionSpec):
            raise ValueError(
                f"Object {self.object_id!r}: collision.type is not mesh."
            )
        cached = self._mesh_geometry
        if cached is not None:
            return cached
        with self._mesh_lock:
            cached = self._mesh_geometry
            if cached is None:
                from .geometry import load_mesh_geometry

                cached = load_mesh_geometry(
                    self.collision.resolved_mesh_path,
                    self.collision.scale,
                    object_id=self.object_id,
                )
                object.__setattr__(self, "_mesh_geometry", cached)
        return cached

    def resolve_aabb(self) -> tuple[dict[str, float], dict[str, float]]:
        """Return collision AABB ``(center, half_extents)`` in object space."""
        from .geometry import resolve_box_aabb, transform_aabb

        if isinstance(self.collision, CollisionSpec):
            return resolve_box_aabb(self.collision.dimensions, self.pose_offset)
        if isinstance(self.collision, MeshCollisionSpec):
            mesh = self.resolve_mesh_geometry()
            return transform_aabb(
                mesh.aabb_center,
                mesh.aabb_half_extents,
                self.pose_offset,
            )
        raise ValueError(
            f"Object {self.object_id!r}: unsupported collision.type "
            f"{self.collision.type!r}."
        )


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

    def mesh_provenance(self) -> dict[str, dict[str, Any]]:
        """Resolve provenance for mesh objects only, loading each mesh lazily."""
        provenance: dict[str, dict[str, Any]] = {}
        for obj in self.objects.values():
            if not isinstance(obj.collision, MeshCollisionSpec):
                continue
            geometry = obj.resolve_mesh_geometry()
            center, half_extents = obj.resolve_aabb()
            provenance[obj.object_id] = {
                "mesh_path": str(obj.collision.resolved_mesh_path),
                "scale": list(obj.collision.scale),
                "sha256": geometry.file_sha256,
                "aabb": {
                    "frame": "object",
                    "center": center,
                    "half_extents": half_extents,
                },
            }
        return provenance
