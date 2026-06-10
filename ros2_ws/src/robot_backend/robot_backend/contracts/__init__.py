"""Versioned, serializable skill-invocation contracts for WatchDog conformance checking.

A contract represents the specification for a single skill call as three
tagged clause lists — preconditions, effects, invariants — each clause
sourced from the model_registry.yaml spec block, never from inline constants.

This package is intentionally ROS-free so the WatchDog supervisor can import
it without a running ROS context.
"""
