"""Manual smoke test helper for the MoveIt planning scene backend.

Run after building and sourcing the workspace:

    scene_backend_smoke_test <mode> [options]

Or without installing:

    python3 -m robot_backend.manual_tests.scene_backend_smoke_test <mode>

Modes added for the service-driven scene sync refactor:

    sync                  -- call sync_collision_objects() for all registry objects
    sync-objects <ids>    -- call sync_collision_objects() for specific object IDs
    sequence-sync         -- sync-all -> list -> verify registry objects appeared
    sequence-sync-restore -- remove one object, sync, verify it comes back from TF
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

# Objects expected in the model registry for sync verification.
REGISTRY_OBJECT_IDS = ["red_cube", "green_cube"]


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

    # --- Sync service modes ---

    subparsers.add_parser(
        "sync",
        help=(
            "Call sync_collision_objects() with no filter — syncs all enabled "
            "registry objects from TF into MoveIt."
        ),
    )

    sync_objects_parser = subparsers.add_parser(
        "sync-objects",
        help=(
            "Call sync_collision_objects(object_ids=[...]) for specific IDs only. "
            "Example: sync-objects red_cube green_cube"
        ),
    )
    sync_objects_parser.add_argument(
        "object_ids",
        nargs="+",
        help="One or more object IDs to sync from the model registry.",
    )

    subparsers.add_parser(
        "sequence-sync",
        help=(
            "Full roundtrip: sync all registry objects -> list -> verify expected "
            "objects appeared in the planning scene."
        ),
    )

    sequence_sync_restore_parser = subparsers.add_parser(
        "sequence-sync-restore",
        help=(
            "Remove an object from the scene, then call sync to verify it is "
            "restored from TF. Demonstrates TF as the authoritative pose source."
        ),
    )
    sequence_sync_restore_parser.add_argument(
        "--object-id",
        default="red_cube",
        help="Object to remove and then restore via sync.",
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


def _world_object_ids(get_objects_result: dict) -> set[str]:
    """Return the set of world-collision-object IDs from a get_objects() result."""
    return {
        item.get("id")
        for item in get_objects_result.get("world_collision_objects", [])
        if isinstance(item, dict) and item.get("id")
    }


def _run_sequence_attach(backend) -> dict[str, object]:
    object_id = "red_cube"
    touch_links = list(DEFAULT_TOUCH_LINKS)
    steps: list[dict[str, object]] = []

    add_result = backend.scene.add_collision_object(
        object_id=object_id,
        pose=_default_box_pose(),
        shape={"type": "box", "dimensions": list(DEFAULT_BOX_DIMENSIONS)},
    )
    steps.append({"step": "add-box", "result": add_result})

    list_after_add = backend.scene.get_objects()
    steps.append({"step": "list-after-add", "result": list_after_add})

    attach_result = backend.scene.attach_object(
        object_id=object_id,
        link_name="tcp",
        touch_links=touch_links,
    )
    steps.append({"step": "attach", "result": attach_result})

    list_after_attach = backend.scene.get_objects()
    steps.append({"step": "list-after-attach", "result": list_after_attach})

    detach_result = backend.scene.detach_object(object_id)
    steps.append({"step": "detach", "result": detach_result})

    post_detach = backend.scene.get_objects()
    steps.append({"step": "list-after-detach", "result": post_detach})

    cleanup_result = None
    if object_id in _world_object_ids(post_detach):
        cleanup_result = backend.scene.remove_collision_object(object_id)
        steps.append({"step": "remove-world-object", "result": cleanup_result})

    return {
        "ok": True,
        "command": "sequence-attach",
        "object_id": object_id,
        "verification_summary": {
            "add_verified_present": add_result.get("verified_present"),
            "attach_apply_response_success": attach_result.get("apply_response_success"),
            "attach_verified_attached": attach_result.get("verified_attached"),
            "attach_verified_removed_from_world": attach_result.get("verified_removed_from_world"),
            "detach_verified_detached": detach_result.get("verified_detached"),
            "world_object_present_after_detach": detach_result.get("world_object_present_after_detach"),
            "cleanup_ok": None if cleanup_result is None else cleanup_result.get("ok"),
        },
        "steps": steps,
    }


def _run_sequence_sync(backend) -> dict[str, object]:
    """Sync all registry objects and verify they appear in the planning scene."""
    steps: list[dict[str, object]] = []

    initial_state = backend.scene.get_objects()
    steps.append({"step": "list-before-sync", "result": initial_state})
    ids_before = _world_object_ids(initial_state)

    sync_result = backend.scene.sync_collision_objects()
    steps.append({"step": "sync-all", "result": sync_result})

    post_sync_state = backend.scene.get_objects()
    steps.append({"step": "list-after-sync", "result": post_sync_state})
    ids_after = _world_object_ids(post_sync_state)

    expected = set(REGISTRY_OBJECT_IDS)
    synced_ids = set(sync_result.get("synced_object_ids", []))
    failed_ids = set(sync_result.get("failed_object_ids", []))
    skipped_ids = set(sync_result.get("skipped_object_ids", []))
    appeared = ids_after - ids_before
    missing_from_scene = synced_ids - ids_after
    expected_present = expected & ids_after

    overall_ok = (
        sync_result.get("ok", False)
        and not failed_ids
        and expected_present == expected
    )

    return {
        "ok": overall_ok,
        "command": "sequence-sync",
        "verification_summary": {
            "service_ok": sync_result.get("ok"),
            "service_message": sync_result.get("message"),
            "synced_ids": sorted(synced_ids),
            "failed_ids": sorted(failed_ids),
            "skipped_ids": sorted(skipped_ids),
            "world_ids_before_sync": sorted(ids_before),
            "world_ids_after_sync": sorted(ids_after),
            "newly_appeared_in_scene": sorted(appeared),
            "synced_but_missing_from_scene": sorted(missing_from_scene),
            "expected_registry_objects": sorted(expected),
            "expected_objects_present_in_scene": sorted(expected_present),
            "all_expected_present": expected_present == expected,
        },
        "steps": steps,
    }


def _run_sequence_sync_restore(backend, object_id: str) -> dict[str, object]:
    """Remove object_id, call sync, verify it is restored from TF."""
    steps: list[dict[str, object]] = []

    # 1. Sync first so there is a known baseline.
    initial_sync = backend.scene.sync_collision_objects()
    steps.append({"step": "initial-sync", "result": initial_sync})

    state_after_initial_sync = backend.scene.get_objects()
    steps.append({"step": "list-after-initial-sync", "result": state_after_initial_sync})
    present_before_remove = object_id in _world_object_ids(state_after_initial_sync)

    # 2. Remove the object manually to simulate a stale or cleared scene.
    remove_result = backend.scene.remove_collision_object(object_id)
    steps.append({"step": "remove", "result": remove_result})

    state_after_remove = backend.scene.get_objects()
    steps.append({"step": "list-after-remove", "result": state_after_remove})
    absent_after_remove = object_id not in _world_object_ids(state_after_remove)

    # 3. Sync only this object to verify TF restores it.
    restore_sync = backend.scene.sync_collision_objects(object_ids=[object_id])
    steps.append({"step": "sync-restore", "result": restore_sync})

    state_after_restore = backend.scene.get_objects()
    steps.append({"step": "list-after-restore", "result": state_after_restore})
    restored = object_id in _world_object_ids(state_after_restore)

    overall_ok = (
        present_before_remove
        and absent_after_remove
        and restore_sync.get("ok", False)
        and restored
    )

    return {
        "ok": overall_ok,
        "command": "sequence-sync-restore",
        "object_id": object_id,
        "verification_summary": {
            "present_before_remove": present_before_remove,
            "absent_after_remove": absent_after_remove,
            "restore_sync_ok": restore_sync.get("ok"),
            "restore_sync_message": restore_sync.get("message"),
            "restored_in_scene": restored,
            "overall_pass": overall_ok,
        },
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
        elif args.mode == "sync":
            result = backend.scene.sync_collision_objects()
        elif args.mode == "sync-objects":
            result = backend.scene.sync_collision_objects(object_ids=args.object_ids)
        elif args.mode == "sequence-sync":
            result = _run_sequence_sync(backend)
        elif args.mode == "sequence-sync-restore":
            result = _run_sequence_sync_restore(backend, args.object_id)
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
