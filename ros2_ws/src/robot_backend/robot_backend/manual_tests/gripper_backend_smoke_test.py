"""Manual smoke test for the RG2 gripper backend.

This helper is intentionally not exposed as a console script yet.
Run it manually later in a ROS environment after the relevant workspace is
built and sourced.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.skills.gripper_skills import close_gripper_skill
from robot_backend.skills.gripper_skills import open_gripper_skill
from robot_backend.skills.gripper_skills import set_gripper_width_skill


def build_argument_parser() -> argparse.ArgumentParser:
    """Create the CLI parser for the manual gripper smoke test."""

    parser = argparse.ArgumentParser(
        description="Manual smoke test for the robot_backend RG2 gripper backend.",
    )
    subparsers = parser.add_subparsers(dest="mode")

    subparsers.add_parser("open", help="Open the gripper.")
    subparsers.add_parser("close", help="Close the gripper.")

    width_parser = subparsers.add_parser(
        "width",
        help="Set the gripper to a specific physical opening width in meters.",
    )
    width_parser.add_argument(
        "--value",
        type=float,
        required=True,
        help="Target physical gripper opening in meters.",
    )

    subparsers.add_parser(
        "sequence",
        help="Run open -> half-open -> close with pauses for visual checking.",
    )

    return parser


def run_sequence(backend) -> None:
    """Run a safer visual test sequence with pauses between commands."""

    print("[sequence] Calling open_gripper_skill()...")
    open_response = open_gripper_skill(backend)
    print(f"[sequence] open_gripper_skill response: {open_response}")

    print("[sequence] Sleeping for 2 seconds...")
    time.sleep(2.0)

    print("[sequence] Calling set_gripper_width_skill(backend, 0.052)...")
    width_response = set_gripper_width_skill(backend, 0.052)
    print(f"[sequence] set_gripper_width_skill response: {width_response}")

    print("[sequence] Sleeping for 2 seconds...")
    time.sleep(2.0)

    print("[sequence] Calling close_gripper_skill()...")
    close_response = close_gripper_skill(backend)
    print(f"[sequence] close_gripper_skill response: {close_response}")


def main() -> None:
    """Create the backend and exercise the gripper skill wrappers."""

    parser = build_argument_parser()
    args = parser.parse_args()
    mode = args.mode or "sequence"

    rclpy.init(args=None)
    node = Node("gripper_backend_smoke_test")
    try:
        backend = create_ros_moveit_backend(node=node)

        if mode == "open":
            print("[open] Calling open_gripper_skill()...")
            response = open_gripper_skill(backend)
            print(f"[open] open_gripper_skill response: {response}")
        elif mode == "close":
            print("[close] Calling close_gripper_skill()...")
            response = close_gripper_skill(backend)
            print(f"[close] close_gripper_skill response: {response}")
        elif mode == "width":
            print(f"[width] Calling set_gripper_width_skill(backend, {args.value})...")
            response = set_gripper_width_skill(backend, args.value)
            print(f"[width] set_gripper_width_skill response: {response}")
        elif mode == "sequence":
            run_sequence(backend)
        else:
            raise ValueError(f"Unsupported mode: {mode}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
