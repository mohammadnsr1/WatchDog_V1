"""Typed, ROS-free model-registry loading and geometry resolution."""

from .loader import load_registry
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
    "load_registry",
]
