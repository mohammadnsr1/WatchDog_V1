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
SAFE_TEST_POSE = {
    "frame_id": "world",
    "position": {"x": 0.45, "y": 0.0, "z": 0.35},
    "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0},
}


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

    pose_parser = subparsers.add_parser(
        "pose",
        help="Send a Cartesian pose target through backend.motion.move_to_pose().",
    )
    pose_parser.add_argument("--frame-id", default=SAFE_TEST_POSE["frame_id"])
    pose_parser.add_argument("--x", type=float, default=SAFE_TEST_POSE["position"]["x"])
    pose_parser.add_argument("--y", type=float, default=SAFE_TEST_POSE["position"]["y"])
    pose_parser.add_argument("--z", type=float, default=SAFE_TEST_POSE["position"]["z"])
    pose_parser.add_argument("--qx", type=float, default=SAFE_TEST_POSE["orientation"]["x"])
    pose_parser.add_argument("--qy", type=float, default=SAFE_TEST_POSE["orientation"]["y"])
    pose_parser.add_argument("--qz", type=float, default=SAFE_TEST_POSE["orientation"]["z"])
    pose_parser.add_argument("--qw", type=float, default=SAFE_TEST_POSE["orientation"]["w"])
    pose_parser.add_argument("--ik-timeout-sec", type=float, default=5.0)
    pose_parser.add_argument(
        "--avoid-collisions",
        dest="avoid_collisions",
        action="store_true",
        default=True,
        help="Ask MoveIt IK to avoid collisions while solving.",
    )
    pose_parser.add_argument(
        "--allow-collisions",
        dest="avoid_collisions",
        action="store_false",
        help="Disable collision checking for IK to see whether the timeout is collision-related.",
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


def _resolve_pose(args: argparse.Namespace) -> dict[str, object]:
    return {
        "frame_id": args.frame_id,
        "position": {
            "x": float(args.x),
            "y": float(args.y),
            "z": float(args.z),
        },
        "orientation": {
            "x": float(args.qx),
            "y": float(args.qy),
            "z": float(args.qz),
            "w": float(args.qw),
        },
    }


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
        elif args.mode == "pose":
            result = backend.motion.move_to_pose(
                _resolve_pose(args),
                ik_timeout_sec=args.ik_timeout_sec,
                avoid_collisions=args.avoid_collisions,
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
