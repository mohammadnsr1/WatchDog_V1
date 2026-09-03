"""Service-driven sync of model-registry TF objects into MoveIt's planning scene.

Exposes three services:
  /sync_scene_objects  (SyncSceneObjects) — push TF-derived poses into MoveIt.
  /get_object_pose     (GetObjectPose)    — look up one object's TF pose.
  /get_object_poses    (GetObjectPoses)   — look up all matching objects' poses.
  /reset_tf            (std_srvs/Trigger) — flush and recreate the TF buffer
                                            after a sim reset.

This node is the sole executor-side owner of a tf2_ros.Buffer. All other
executor components (including the scene MCP server) query TF by calling
these services rather than creating their own buffers.

Requires MultiThreadedExecutor because the sync handler needs to call
/apply_planning_scene (a nested service call) and wait for its response.
A SingleThreadedExecutor would deadlock: the handler thread would be waiting
for a response callback that can never run because the only thread is occupied
by the handler itself.
"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

# This node is launched by sim_moveit_bringup through a console script whose
# shebang is /usr/bin/python3, so it does NOT inherit an activated venv. It
# reaches robot_backend.events (pydantic) via the imports below, so it needs the
# same workspace-venv re-exec the MCP servers perform. Must run before any
# import that pulls in the event bus.
from robot_backend.mcp_servers import ensure_mcp_server_environment

ensure_mcp_server_environment("robot_backend.nodes.scene_tf_sync_node")

import rclpy
import rclpy.executors
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from robot_backend_interfaces.srv import GetObjectPose, GetObjectPoses, SyncSceneObjects
from shape_msgs.msg import Mesh
from shape_msgs.msg import MeshTriangle
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from robot_backend.registry import MeshCollisionSpec, RegisteredObject, Registry, load_registry
from robot_backend.skills.pose_query_skills import get_object_pose, get_object_poses
from robot_backend.utils.geometry import compose_poses
from robot_backend.utils.geometry import make_pose


_DEFAULT_TF_LOOKUP_TIMEOUT_SEC = 0.1
_DEFAULT_MAX_TF_AGE_SEC = 0.5
_DEFAULT_APPLY_PLANNING_SCENE_SERVICE = '/apply_planning_scene'
_DEFAULT_SYNC_SCENE_OBJECTS_SERVICE = '/sync_scene_objects'
_DEFAULT_GET_OBJECT_POSE_SERVICE = '/get_object_pose'
_DEFAULT_GET_OBJECT_POSES_SERVICE = '/get_object_poses'
_DEFAULT_RESET_TF_SERVICE = '/reset_tf'
_DEFAULT_APPLY_TIMEOUT_SEC = 10.0


def build_collision_object(
    resolved_object: RegisteredObject,
    world_object_pose: dict[str, Any],
) -> CollisionObject:
    """Build one CollisionObject from validated geometry and a world pose."""
    collision_pose = compose_poses(world_object_pose, resolved_object.pose_offset)
    collision = resolved_object.collision
    shape_type = str(collision.type).strip().lower()

    obj = CollisionObject()
    obj.header.frame_id = resolved_object.lookup_frame
    fixed_stamp = world_object_pose.get('_header_stamp')
    if fixed_stamp is not None:
        obj.header.stamp = fixed_stamp
    obj.id = resolved_object.object_id
    obj.operation = CollisionObject.ADD

    if shape_type == 'box':
        primitive = SolidPrimitive()
        dims = list(collision.dimensions)
        if not isinstance(dims, list) or len(dims) != 3:
            raise ValueError('Box collision requires dimensions [x, y, z].')
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(v) for v in dims]
        obj.primitives = [primitive]
        obj.primitive_poses = [_pose_to_msg(collision_pose)]
    elif shape_type == 'mesh' and isinstance(collision, MeshCollisionSpec):
        geometry = resolved_object.resolve_mesh_geometry()
        mesh = Mesh()
        mesh.vertices = [_point_to_msg(vertex) for vertex in geometry.vertices]
        mesh.triangles = [_triangle_to_msg(face) for face in geometry.faces]
        obj.meshes = [mesh]
        obj.mesh_poses = [_pose_to_msg(collision_pose)]
    else:
        raise ValueError(
            f"Unsupported collision type {shape_type!r}. Supported: box, mesh."
        )
    return obj


def _pose_to_msg(pose_dict: dict[str, Any]) -> Pose:
    pose = Pose()
    pose.position.x = float(pose_dict['position']['x'])
    pose.position.y = float(pose_dict['position']['y'])
    pose.position.z = float(pose_dict['position']['z'])
    pose.orientation.x = float(pose_dict['orientation']['x'])
    pose.orientation.y = float(pose_dict['orientation']['y'])
    pose.orientation.z = float(pose_dict['orientation']['z'])
    pose.orientation.w = float(pose_dict['orientation']['w'])
    return pose


def _point_to_msg(vertex: tuple[float, float, float]) -> Point:
    point = Point()
    point.x, point.y, point.z = vertex
    return point


def _triangle_to_msg(face: tuple[int, int, int]) -> MeshTriangle:
    triangle = MeshTriangle()
    triangle.vertex_indices = list(face)
    return triangle


class SceneTFSyncNode(Node):
    """Single executor-side TF buffer owner; serves sync, pose query, and TF reset services."""

    def __init__(self) -> None:
        super().__init__('scene_tf_sync_node')

        self.declare_parameter('model_registry_path', '')
        self.declare_parameter('tf_lookup_timeout_sec', _DEFAULT_TF_LOOKUP_TIMEOUT_SEC)
        self.declare_parameter('max_tf_age_sec', _DEFAULT_MAX_TF_AGE_SEC)
        self.declare_parameter('add_only_enabled', True)
        self.declare_parameter(
            'apply_planning_scene_service', _DEFAULT_APPLY_PLANNING_SCENE_SERVICE
        )
        self.declare_parameter(
            'sync_scene_objects_service', _DEFAULT_SYNC_SCENE_OBJECTS_SERVICE
        )
        self.declare_parameter(
            'get_object_pose_service', _DEFAULT_GET_OBJECT_POSE_SERVICE
        )
        self.declare_parameter(
            'get_object_poses_service', _DEFAULT_GET_OBJECT_POSES_SERVICE
        )
        self.declare_parameter('reset_tf_service', _DEFAULT_RESET_TF_SERVICE)
        self.declare_parameter('apply_timeout_sec', _DEFAULT_APPLY_TIMEOUT_SEC)

        self._tf_lookup_timeout_sec = float(
            self.get_parameter('tf_lookup_timeout_sec').value
        )
        self._max_tf_age_sec = float(self.get_parameter('max_tf_age_sec').value)
        self._add_only_enabled = bool(self.get_parameter('add_only_enabled').value)
        self._apply_planning_scene_service = str(
            self.get_parameter('apply_planning_scene_service').value
        )
        self._sync_scene_objects_service = str(
            self.get_parameter('sync_scene_objects_service').value
        )
        self._get_object_pose_service = str(
            self.get_parameter('get_object_pose_service').value
        )
        self._get_object_poses_service = str(
            self.get_parameter('get_object_poses_service').value
        )
        self._reset_tf_service = str(self.get_parameter('reset_tf_service').value)
        self._apply_timeout_sec = float(self.get_parameter('apply_timeout_sec').value)

        self._registry = self._load_registry()

        # ReentrantCallbackGroup lets the MultiThreadedExecutor run both
        # the service handler and the ApplyPlanningScene response callback
        # concurrently, which is required to avoid a deadlock.
        cb_group = ReentrantCallbackGroup()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        # Lock protects _tf_buffer / _tf_listener reassignment during reset so
        # concurrent lookups read a consistent buffer reference.
        self._tf_lock = threading.Lock()

        self._apply_client = self.create_client(
            ApplyPlanningScene,
            self._apply_planning_scene_service,
            callback_group=cb_group,
        )

        self.create_service(
            SyncSceneObjects,
            self._sync_scene_objects_service,
            self._handle_sync_service,
            callback_group=cb_group,
        )
        self.create_service(
            GetObjectPose,
            self._get_object_pose_service,
            self._handle_get_object_pose,
            callback_group=cb_group,
        )
        self.create_service(
            GetObjectPoses,
            self._get_object_poses_service,
            self._handle_get_object_poses,
            callback_group=cb_group,
        )
        self.create_service(
            Trigger,
            self._reset_tf_service,
            self._handle_reset_tf,
            callback_group=cb_group,
        )

        configured = list(self._registry.objects.keys())
        self.get_logger().info(f'Registry objects: {configured}')
        self.get_logger().info(
            f'Serving {self._sync_scene_objects_service}, '
            f'{self._get_object_pose_service}, '
            f'{self._get_object_poses_service}, '
            f'{self._reset_tf_service}'
        )

    # ------------------------------------------------------------------
    # /sync_scene_objects handler
    # ------------------------------------------------------------------

    def _handle_sync_service(
        self,
        request: SyncSceneObjects.Request,
        response: SyncSceneObjects.Response,
    ) -> SyncSceneObjects.Response:
        requested_ids: list[str] | None = (
            list(request.object_ids) if request.object_ids else None
        )

        collision_objects, synced_ids, failed_ids, skipped_ids = self._build_objects(
            requested_ids
        )

        if not collision_objects:
            response.success = len(failed_ids) == 0
            response.synced_object_ids = synced_ids
            response.failed_object_ids = failed_ids
            response.skipped_object_ids = skipped_ids
            response.message = (
                f'Synced {len(synced_ids)}, failed {len(failed_ids)}, '
                f'skipped {len(skipped_ids)}.'
            )
            return response

        if not self._apply_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'ApplyPlanningScene not available: {self._apply_planning_scene_service}'
            )
            response.success = False
            response.message = 'ApplyPlanningScene service not available.'
            response.synced_object_ids = []
            response.failed_object_ids = synced_ids + failed_ids
            response.skipped_object_ids = skipped_ids
            return response

        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = PlanningScene()
        apply_request.scene.is_diff = True
        apply_request.scene.world.collision_objects = collision_objects

        apply_future = self._apply_client.call_async(apply_request)

        # threading.Event lets this handler thread block while the
        # MultiThreadedExecutor's pool dispatches the response on another thread.
        done_event = threading.Event()
        apply_future.add_done_callback(lambda _: done_event.set())
        timed_out = not done_event.wait(timeout=self._apply_timeout_sec)

        apply_success = False
        if timed_out:
            self.get_logger().warning(
                f'ApplyPlanningScene timed out after {self._apply_timeout_sec:.1f}s.'
            )
        elif apply_future.result() is not None:
            apply_success = bool(apply_future.result().success)
        else:
            self.get_logger().warning('ApplyPlanningScene returned no result.')

        if not apply_success:
            failed_ids = synced_ids + failed_ids
            synced_ids = []

        response.success = apply_success
        response.synced_object_ids = synced_ids
        response.failed_object_ids = failed_ids
        response.skipped_object_ids = skipped_ids
        response.message = (
            f'Synced {len(synced_ids)}, failed {len(failed_ids)}, '
            f'skipped {len(skipped_ids)}.'
        )
        return response

    # ------------------------------------------------------------------
    # /get_object_pose handler
    # ------------------------------------------------------------------

    def _handle_get_object_pose(
        self,
        request: GetObjectPose.Request,
        response: GetObjectPose.Response,
    ) -> GetObjectPose.Response:
        with self._tf_lock:
            buf = self._tf_buffer
        result = get_object_pose(
            buf,
            self._registry.raw_dict,
            request.object_id,
            tf_timeout_sec=self._tf_lookup_timeout_sec,
            clock=self.get_clock(),
            max_tf_age_sec=self._max_tf_age_sec,
        )
        response.ok = bool(result['ok'])
        response.object_id = str(result.get('object_id', request.object_id))
        response.error = str(result.get('error', ''))
        if result['ok']:
            response.frame_id = str(result['frame_id'])
            response.tf_frame = str(result['tf_frame'])
            p = result['pose']
            response.pose.position.x = float(p['position']['x'])
            response.pose.position.y = float(p['position']['y'])
            response.pose.position.z = float(p['position']['z'])
            response.pose.orientation.x = float(p['orientation']['x'])
            response.pose.orientation.y = float(p['orientation']['y'])
            response.pose.orientation.z = float(p['orientation']['z'])
            response.pose.orientation.w = float(p['orientation']['w'])
        return response

    # ------------------------------------------------------------------
    # /get_object_poses handler
    # ------------------------------------------------------------------

    def _handle_get_object_poses(
        self,
        request: GetObjectPoses.Request,
        response: GetObjectPoses.Response,
    ) -> GetObjectPoses.Response:
        with self._tf_lock:
            buf = self._tf_buffer
        result = get_object_poses(
            buf,
            self._registry.raw_dict,
            enabled_only=bool(request.enabled_only),
            graspable_only=bool(request.graspable_only),
            tf_timeout_sec=self._tf_lookup_timeout_sec,
            clock=self.get_clock(),
            max_tf_age_sec=self._max_tf_age_sec,
        )
        response.ok = bool(result['ok'])
        response.poses_json = json.dumps(result.get('poses', []))
        response.failed_json = json.dumps(result.get('failed', []))
        response.error = ''
        return response

    # ------------------------------------------------------------------
    # /reset_tf handler
    # ------------------------------------------------------------------

    def _handle_reset_tf(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Destroy and recreate the TF buffer to flush pre-reset stale transforms."""
        new_buffer = Buffer()
        new_listener = TransformListener(new_buffer, self, spin_thread=False)
        with self._tf_lock:
            self._tf_buffer = new_buffer
            self._tf_listener = new_listener
        self.get_logger().info('TF buffer flushed and recreated.')
        response.success = True
        response.message = 'TF buffer flushed and recreated.'
        return response

    # ------------------------------------------------------------------
    # Object building (sync path)
    # ------------------------------------------------------------------

    def _build_objects(
        self,
        object_ids: list[str] | None,
    ) -> tuple[list[CollisionObject], list[str], list[str], list[str]]:
        """Build CollisionObjects for all (or requested) registry objects.

        Returns (collision_objects, synced_ids, failed_ids, skipped_ids).
        synced_ids contains only IDs whose CollisionObject was built successfully.
        failed_ids contains IDs that could not be built (TF not available, bad config, etc.).
        skipped_ids contains IDs that are disabled in the registry.
        """
        objects_section = self._registry.objects

        collision_objects: list[CollisionObject] = []
        synced_ids: list[str] = []
        failed_ids: list[str] = []
        skipped_ids: list[str] = []

        for registry_key, entry in objects_section.items():
            object_id = entry.object_id

            if object_ids is not None:
                if object_id not in object_ids and registry_key not in object_ids:
                    continue

            if self._add_only_enabled and not entry.enabled:
                skipped_ids.append(object_id)
                continue

            try:
                world_object_pose = self._lookup_tf_pose(
                    lookup_frame=entry.lookup_frame,
                    tf_frame=entry.tf_frame,
                )
                collision_object = build_collision_object(entry, world_object_pose)
                collision_object.header.stamp = self.get_clock().now().to_msg()
                collision_objects.append(collision_object)
                synced_ids.append(object_id)
            except Exception as exc:
                failed_ids.append(object_id)
                self.get_logger().warning(
                    f'Failed to build collision object for {object_id!r}: {exc}'
                )

        return collision_objects, synced_ids, failed_ids, skipped_ids

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _lookup_tf_pose(self, *, lookup_frame: str, tf_frame: str) -> dict[str, Any]:
        """Look up the latest TF transform with a freshness guard.

        Raises RuntimeError on lookup failure or stale transform.
        """
        with self._tf_lock:
            buf = self._tf_buffer
        try:
            transform = buf.lookup_transform(
                lookup_frame,
                tf_frame,
                Time(),
                timeout=Duration(seconds=self._tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            raise RuntimeError(
                f'TF lookup failed {lookup_frame} -> {tf_frame}: {exc}'
            ) from exc

        stamp_time = Time.from_msg(transform.header.stamp)
        age_ns = (self.get_clock().now() - stamp_time).nanoseconds
        if age_ns > 0 and age_ns / 1e9 > self._max_tf_age_sec:
            raise RuntimeError(
                f'stale TF: transform age {age_ns / 1e9:.3f}s '
                f'exceeds max_tf_age_sec={self._max_tf_age_sec} '
                f'({lookup_frame} -> {tf_frame})'
            )

        t = transform.transform.translation
        r = transform.transform.rotation
        return make_pose(
            {'x': float(t.x), 'y': float(t.y), 'z': float(t.z)},
            {'x': float(r.x), 'y': float(r.y), 'z': float(r.z), 'w': float(r.w)},
            frame_id=lookup_frame,
        )

    def _load_registry(self) -> Registry:
        configured = str(self.get_parameter('model_registry_path').value or '').strip()
        if configured:
            path = Path(configured)
        else:
            share_dir = Path(get_package_share_directory('robot_backend'))
            path = share_dir / 'config' / 'model_registry.yaml'

        if not path.is_file():
            raise FileNotFoundError(f'Model registry not found: {path}')

        registry = load_registry(path)
        self.get_logger().info(f'Loaded model registry: {path}')
        return registry


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SceneTFSyncNode | None = None
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        node = SceneTFSyncNode()
        executor.add_node(node)
        executor.spin()
    finally:
        if node is not None:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
