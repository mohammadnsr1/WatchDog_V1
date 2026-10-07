"""Typed, ROS-free model-registry loading and geometry resolution."""

from .loader import REGISTRY_OVERRIDE_ENV, executor_registry_override, load_registry
from .model import (
    BoxCollisionSpec,
    CollisionSpec,
    MeshCollisionSpec,
    MeshGeometry,
    RegisteredObject,
    Registry,
)

__all__ = [
    "BoxCollisionSpec",
    "CollisionSpec",
    "MeshCollisionSpec",
    "MeshGeometry",
    "RegisteredObject",
    "Registry",
    "REGISTRY_OVERRIDE_ENV",
    "executor_registry_override",
    "load_registry",
]
