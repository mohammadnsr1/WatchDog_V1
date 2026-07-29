"""Typed, ROS-free model-registry loading and geometry resolution."""

from .loader import load_registry
from .model import CollisionSpec, RegisteredObject, Registry

__all__ = ["CollisionSpec", "RegisteredObject", "Registry", "load_registry"]
