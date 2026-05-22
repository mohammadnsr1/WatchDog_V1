"""Manual smoke test helper for the MoveIt motion backend."""

from __future__ import annotations

import argparse
import json
from typing import Sequence

import rclpy
from rclpy.node import Node

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.ros_moveit_backend.motion import ARM_JOINT_NAMES


SAFE_TEST_JOINTS = [0.0, -1.2, 1.2, -1.57, -1.57, 0.0]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manual helper for exercising robot_backend motion calls.",
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)

    subparsers.add_parser("state", help="Print backend.motion.get_robot_state().")

    joints_parser = subparsers.add_parser(
        "joints",
        help="Send a joint target through backend.motion.move_to_joint_positions().",
    )
    joints_parser.add_argument(
        "values",
        nargs="*",
        type=float,
        help=(
            "Six joint values in UR5e order: "
            f"{ARM_JOINT_NAMES}. If omitted, uses the conservative example {SAFE_TEST_JOINTS}."
        ),
    )
    return parser


def _resolve_joint_values(values: Sequence[float]) -> list[float]:
    if not values:
        return list(SAFE_TEST_JOINTS)
    if len(values) != len(ARM_JOINT_NAMES):
        raise ValueError(
            f"Expected {len(ARM_JOINT_NAMES)} joint values in order {ARM_JOINT_NAMES}, "
            f"got {len(values)}."
        )
    return [float(value) for value in values]


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    rclpy.init()
    node: Node | None = None
    try:
        node = Node("motion_backend_smoke_test")
        backend = create_ros_moveit_backend(node=node)

        if args.mode == "state":
            result = backend.motion.get_robot_state()
        elif args.mode == "joints":
            result = backend.motion.move_to_joint_positions(
                _resolve_joint_values(args.values)
            )
        else:  # pragma: no cover - argparse enforces valid modes
            raise ValueError(f"Unsupported mode: {args.mode}")

        print(json.dumps(result, indent=2, sort_keys=True))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
