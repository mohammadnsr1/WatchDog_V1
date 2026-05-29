"""Manual smoke test helper for the MoveIt planning scene backend.

This helper is intentionally not exposed as a console script yet.
Run it manually later in a ROS environment after the relevant workspace is
built and sourced.
"""

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node

from robot_backend.ros_moveit_backend import create_ros_moveit_backend


DEFAULT_BOX_POSE = {
    "frame_id": "world",
    "position": {
        "x": 0.2309480458498001,
        "y": -0.27473875880241394,
        "z": 0.03525618091225624,
    },
    "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0,
    },
}
DEFAULT_BOX_DIMENSIONS = [0.05, 0.05, 0.05]
DEFAULT_TOUCH_LINKS = ["tcp"]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manual helper for exercising robot_backend scene calls.",
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)

    add_box_parser = subparsers.add_parser(
        "add-box",
        help="Add or update a box collision object in the planning scene.",
    )
    add_box_parser.add_argument("--object-id", default="red_cube")
    add_box_parser.add_argument("--frame-id", default=DEFAULT_BOX_POSE["frame_id"])
    add_box_parser.add_argument("--x", type=float, default=DEFAULT_BOX_POSE["position"]["x"])
    add_box_parser.add_argument("--y", type=float, default=DEFAULT_BOX_POSE["position"]["y"])
    add_box_parser.add_argument("--z", type=float, default=DEFAULT_BOX_POSE["position"]["z"])
    add_box_parser.add_argument("--qx", type=float, default=DEFAULT_BOX_POSE["orientation"]["x"])
    add_box_parser.add_argument("--qy", type=float, default=DEFAULT_BOX_POSE["orientation"]["y"])
    add_box_parser.add_argument("--qz", type=float, default=DEFAULT_BOX_POSE["orientation"]["z"])
    add_box_parser.add_argument("--qw", type=float, default=DEFAULT_BOX_POSE["orientation"]["w"])
    add_box_parser.add_argument("--size-x", type=float, default=DEFAULT_BOX_DIMENSIONS[0])
    add_box_parser.add_argument("--size-y", type=float, default=DEFAULT_BOX_DIMENSIONS[1])
    add_box_parser.add_argument("--size-z", type=float, default=DEFAULT_BOX_DIMENSIONS[2])

    remove_parser = subparsers.add_parser(
        "remove",
        help="Remove a collision object from the planning scene.",
    )
    remove_parser.add_argument("--object-id", default="red_cube")

    subparsers.add_parser(
        "list",
        help="List world and attached planning-scene objects.",
    )

    subparsers.add_parser(
        "clear-octomap",
        help="Call the MoveIt clear_octomap service.",
    )

    allow_touch_parser = subparsers.add_parser(
        "allow-touch",
        help="Cache touch links locally for a later attach operation.",
    )
    allow_touch_parser.add_argument("--object-id", default="red_cube")
    allow_touch_parser.add_argument(
        "--link-name",
        dest="link_names",
        nargs="+",
        default=list(DEFAULT_TOUCH_LINKS),
        help=(
            "Touch links to cache for the object. Defaults to ['tcp'] because "
            "that is a safe generic fallback when RG2-specific links are not pinned down yet."
        ),
    )

    attach_parser = subparsers.add_parser(
        "attach",
        help="Attach a known collision object to a robot link.",
    )
    attach_parser.add_argument("--object-id", default="red_cube")
    attach_parser.add_argument("--link-name", default="tcp")
    attach_parser.add_argument(
        "--touch-link",
        dest="touch_links",
        nargs="+",
        help="Optional explicit touch links. If omitted, uses cached allow-touch links.",
    )

    detach_parser = subparsers.add_parser(
        "detach",
        help="Detach an attached collision object from the robot.",
    )
    detach_parser.add_argument("--object-id", default="red_cube")

    subparsers.add_parser(
        "sequence-attach",
        help="Run add -> list -> attach -> list -> detach -> list and clean up if needed.",
    )

    return parser


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


def _default_box_pose() -> dict[str, object]:
    return {
        "frame_id": DEFAULT_BOX_POSE["frame_id"],
        "position": dict(DEFAULT_BOX_POSE["position"]),
        "orientation": dict(DEFAULT_BOX_POSE["orientation"]),
    }


def _run_sequence_attach(backend) -> dict[str, object]:
    object_id = "red_cube"
    touch_links = list(DEFAULT_TOUCH_LINKS)
    steps: list[dict[str, object]] = []

    steps.append(
        {
            "step": "add-box",
            "result": backend.scene.add_collision_object(
                object_id=object_id,
                pose=_default_box_pose(),
                shape={"type": "box", "dimensions": list(DEFAULT_BOX_DIMENSIONS)},
            ),
        }
    )
    steps.append({"step": "list-after-add", "result": backend.scene.get_objects()})
    steps.append(
        {
            "step": "attach",
            "result": backend.scene.attach_object(
                object_id=object_id,
                link_name="tcp",
                touch_links=touch_links,
            ),
        }
    )
    steps.append({"step": "list-after-attach", "result": backend.scene.get_objects()})
    steps.append(
        {
            "step": "detach",
            "result": backend.scene.detach_object(object_id),
        }
    )
    post_detach = backend.scene.get_objects()
    steps.append({"step": "list-after-detach", "result": post_detach})

    world_ids = {
        item.get("id")
        for item in post_detach.get("world_collision_objects", [])
        if isinstance(item, dict)
    }
    if object_id in world_ids:
        steps.append(
            {
                "step": "remove-world-object",
                "result": backend.scene.remove_collision_object(object_id),
            }
        )

    return {
        "ok": True,
        "command": "sequence-attach",
        "object_id": object_id,
        "steps": steps,
    }


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    rclpy.init(args=None)
    node: Node | None = None
    try:
        node = Node("scene_backend_smoke_test")
        backend = create_ros_moveit_backend(node=node)

        if args.mode == "add-box":
            result = backend.scene.add_collision_object(
                object_id=args.object_id,
                pose=_resolve_pose(args),
                shape={
                    "type": "box",
                    "dimensions": [
                        float(args.size_x),
                        float(args.size_y),
                        float(args.size_z),
                    ],
                },
            )
        elif args.mode == "remove":
            result = backend.scene.remove_collision_object(args.object_id)
        elif args.mode == "list":
            result = backend.scene.get_objects()
        elif args.mode == "clear-octomap":
            result = backend.scene.clear_octomap()
        elif args.mode == "allow-touch":
            result = backend.scene.allow_touch(args.object_id, args.link_names)
        elif args.mode == "attach":
            result = backend.scene.attach_object(
                object_id=args.object_id,
                link_name=args.link_name,
                touch_links=args.touch_links,
            )
        elif args.mode == "detach":
            result = backend.scene.detach_object(args.object_id)
        elif args.mode == "sequence-attach":
            result = _run_sequence_attach(backend)
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
