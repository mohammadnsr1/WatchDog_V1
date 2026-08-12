#!/usr/bin/env python3
"""Resync the running cell after an Isaac Sim stop/replay -- no relaunch.

Isaac's /clock restarts at zero on replay. Every use_sim_time node is then
holding TF stamped in what is now the future, and tf2 rejects each incoming
transform as TF_OLD_DATA:

    TF_OLD_DATA ignoring data from the past for frame u_green at time 1.383333

This does not heal itself. tf2 evicts stale entries only as newer ones are
inserted, and the newer ones are precisely what is being rejected -- the buffer
stays poisoned until sim time climbs back past the old high-water mark (i.e.
as long as the previous run lasted) or the buffer is recreated.

This script recreates the buffers that expose a service for it, drops the
octomap, and re-pushes object geometry, in the order those steps depend on
each other.

    python3 scripts/reset_after_replay.py

WHAT THIS CANNOT DO
-------------------
move_group and rviz2 own TF buffers with no reset service -- move_group's
capability list (apply_planning_scene, clear_octomap, get_planning_scene,
kinematics, move_action, ...) has no such entry, and neither is a lifecycle
node. If they misbehave after a replay they must be restarted:

    pkill -f "moveit_ros_move_group/move_group"; pkill -f "rviz2 -d"
    ros2 launch sim_moveit_bringup restart_moveit.launch.py

In practice rviz2 is the one that visibly breaks, and it is only a viewer --
restarting it alone disturbs nothing functional. Run this script first and
restart rviz2 only if it is still unhappy.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


DEFAULT_TF_RESET_SERVICES = [
    "/reset_tf",                                # scene_tf_sync_node
    "/pointcloud_workspace_filter/reset_tf",    # workspace filter
]
CLEAR_OCTOMAP_SERVICE = "/clear_octomap"
SYNC_SERVICE = "/sync_scene_objects"


def call_trigger(node: Node, service_name: str, timeout_sec: float) -> bool:
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        print(f"  SKIP {service_name} (not available)")
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        print(f"  FAIL {service_name} (timed out)")
        return False
    response = future.result()
    status = "OK  " if response.success else "FAIL"
    print(f"  {status} {service_name}: {response.message}")
    return bool(response.success)


def clear_octomap(node: Node, timeout_sec: float) -> bool:
    from std_srvs.srv import Empty

    client = node.create_client(Empty, CLEAR_OCTOMAP_SERVICE)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        print(f"  SKIP {CLEAR_OCTOMAP_SERVICE} (not available)")
        return False
    future = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        print(f"  FAIL {CLEAR_OCTOMAP_SERVICE} (timed out)")
        return False
    print(f"  OK   {CLEAR_OCTOMAP_SERVICE}: octomap dropped")
    return True


def sync_objects(node: Node, object_ids: list[str], timeout_sec: float) -> bool:
    from robot_backend_interfaces.srv import SyncSceneObjects

    client = node.create_client(SyncSceneObjects, SYNC_SERVICE)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        print(f"  SKIP {SYNC_SERVICE} (not available)")
        return False
    request = SyncSceneObjects.Request()
    request.object_ids = object_ids
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        print(f"  FAIL {SYNC_SERVICE} (timed out)")
        return False
    response = future.result()
    status = "OK  " if response.ok else "FAIL"
    print(f"  {status} {SYNC_SERVICE}: {response.message}")
    return bool(response.ok)


def registry_object_ids() -> list[str]:
    from pathlib import Path

    import yaml
    from ament_index_python.packages import get_package_share_directory

    path = (
        Path(get_package_share_directory("robot_backend"))
        / "config"
        / "model_registry.yaml"
    )
    data = yaml.safe_load(path.read_text())
    objects = data.get("objects", {})
    return [
        name for name, spec in objects.items() if (spec or {}).get("enabled", True)
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument(
        "--objects",
        nargs="*",
        default=None,
        help="Object ids to resync. Default: every enabled object in the registry.",
    )
    parser.add_argument(
        "--no-sync",
        action="store_true",
        help="Flush TF and octomap only; skip re-pushing object geometry.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("reset_after_replay")
    try:
        # 1. TF buffers first: everything downstream reads through them, so a
        #    sync done before this would push pre-reset poses.
        print("Flushing TF buffers:")
        for service_name in DEFAULT_TF_RESET_SERVICES:
            call_trigger(node, service_name, args.timeout_sec)

        # 2. Octomap: voxels carved before the replay describe a world that no
        #    longer exists, and nothing clears them implicitly.
        print("\nClearing octomap:")
        clear_octomap(node, args.timeout_sec)

        # 3. Objects last, now that TF reflects the replayed scene.
        if not args.no_sync:
            object_ids = args.objects
            if object_ids is None:
                try:
                    object_ids = registry_object_ids()
                except Exception as exc:
                    print(f"\n  could not read registry ({exc}); skipping sync")
                    object_ids = []
            if object_ids:
                print(f"\nResyncing objects {object_ids}:")
                sync_objects(node, object_ids, args.timeout_sec)

        print(
            "\nDone. move_group and rviz2 have no TF-reset service; if rviz2 is "
            "still throwing extrapolation errors, restart it:\n"
            '  pkill -f "rviz2 -d" && ros2 launch sim_moveit_bringup '
            "restart_moveit.launch.py launch_rviz:=true"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
