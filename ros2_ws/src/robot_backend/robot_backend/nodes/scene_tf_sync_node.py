"""Continuously sync model-registry TF objects into MoveIt's planning scene."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from robot_backend.utils.geometry import compose_poses
from robot_backend.utils.geometry import make_pose


_DEFAULT_SYNC_RATE_HZ = 1.0
_DEFAULT_TF_LOOKUP_TIMEOUT_SEC = 0.1
_DEFAULT_WARNING_THROTTLE_SEC = 5.0
_DEFAULT_SUMMARY_LOG_INTERVAL_SEC = 10.0
_DEFAULT_APPLY_PLANNING_SCENE_SERVICE = '/apply_planning_scene'


class SceneTFSyncNode(Node):
    """Read the model registry, resolve TF poses, and update MoveIt's scene."""

    def __init__(self) -> None:
        super().__init__('scene_tf_sync_node')
        self.declare_parameter('model_registry_path', '')
        self.declare_parameter('sync_rate_hz', _DEFAULT_SYNC_RATE_HZ)
        self.declare_parameter('tf_lookup_timeout_sec', _DEFAULT_TF_LOOKUP_TIMEOUT_SEC)
        self.declare_parameter('add_only_enabled', True)
        self.declare_parameter(
            'apply_planning_scene_service',
            _DEFAULT_APPLY_PLANNING_SCENE_SERVICE,
        )
        self.declare_parameter('warning_throttle_sec', _DEFAULT_WARNING_THROTTLE_SEC)
        self.declare_parameter('summary_log_interval_sec', _DEFAULT_SUMMARY_LOG_INTERVAL_SEC)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._sync_in_progress = False
        self._apply_future: Future | None = None
        self._pending_object_ids: list[str] = []
        self._warning_last_logged_at: dict[str, float] = {}
        self._last_summary_signature: tuple[int, int, int, int] | None = None
        self._last_summary_log_time = 0.0
        self._last_apply_success_signature: tuple[str, ...] | None = None
        self._last_apply_success_log_time = 0.0

        self._model_registry_path = self._resolve_registry_path()
        self._registry = self._load_registry(self._model_registry_path)
        self._sync_rate_hz = float(self.get_parameter('sync_rate_hz').value)
        self._tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self._add_only_enabled = bool(self.get_parameter('add_only_enabled').value)
        self._apply_planning_scene_service = str(
            self.get_parameter('apply_planning_scene_service').value
        )
        self._warning_throttle_sec = float(self.get_parameter('warning_throttle_sec').value)
        self._summary_log_interval_sec = float(
            self.get_parameter('summary_log_interval_sec').value
        )

        if self._sync_rate_hz <= 0.0:
            raise ValueError('sync_rate_hz must be > 0.')
        if self._tf_lookup_timeout_sec < 0.0:
            raise ValueError('tf_lookup_timeout_sec must be >= 0.')
        if self._warning_throttle_sec < 0.0:
            raise ValueError('warning_throttle_sec must be >= 0.')
        if self._summary_log_interval_sec < 0.0:
            raise ValueError('summary_log_interval_sec must be >= 0.')

        self._apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            self._apply_planning_scene_service,
        )

        configured_objects = list(self._registry.get('objects', {}).keys())
        self.get_logger().info(f'Loaded model registry: {self._model_registry_path}')
        self.get_logger().info(f'Configured registry objects: {configured_objects}')
        self.get_logger().info(f'Scene sync rate: {self._sync_rate_hz} Hz')

        self._timer = self.create_timer(1.0 / self._sync_rate_hz, self._on_timer)

    def sync_once(self) -> dict[str, Any]:
        """Build one planning-scene diff request from the current TF state."""

        objects_section = self._registry.get('objects', {})
        if not isinstance(objects_section, dict):
            raise ValueError('model_registry.yaml must contain an objects mapping.')

        results: dict[str, Any] = {}
        collision_objects: list[CollisionObject] = []
        object_ids: list[str] = []
        success_count = 0
        failure_count = 0
        skipped_count = 0

        for registry_key, entry in objects_section.items():
            result = self._build_collision_object_result(registry_key, entry)
            results[registry_key] = result

            if result.get('status') == 'skipped':
                skipped_count += 1
                continue
            if result.get('ok'):
                success_count += 1
                collision_objects.append(result['collision_object'])
                object_ids.append(result['object_id'])
                continue

            failure_count += 1
            self._log_warning_throttled(
                key=f"sync_failure:{result['object_id']}",
                message=f"Scene sync failed for {result['object_id']}: {result.get('error')}",
            )

        return {
            'ok': success_count > 0 and failure_count == 0,
            'registry_path': self._model_registry_path,
            'success_count': success_count,
            'failure_count': failure_count,
            'skipped_count': skipped_count,
            'results': results,
            'collision_objects': collision_objects,
            'object_ids': object_ids,
        }

    def _on_timer(self) -> None:
        if self._sync_in_progress:
            self.get_logger().debug('Scene sync already in progress; skipping timer tick.')
            return

        self._sync_in_progress = True
        try:
            if self._apply_future is not None and not self._apply_future.done():
                self.get_logger().debug('ApplyPlanningScene request still pending; skipping timer tick.')
                return

            if not self._apply_planning_scene_client.wait_for_service(timeout_sec=0.0):
                self._log_warning_throttled(
                    key='apply_service_unavailable',
                    message=(
                        'ApplyPlanningScene service is not available yet: '
                        f'{self._apply_planning_scene_service}'
                    ),
                )
                return

            result = self.sync_once()
            self._log_cycle_summary(result)

            collision_objects = result['collision_objects']
            if not collision_objects:
                self._log_warning_throttled(
                    key='no_buildable_objects',
                    message='No scene objects were buildable this cycle; waiting for TF and registry-valid objects.',
                )
                return

            request = ApplyPlanningScene.Request()
            request.scene = PlanningScene()
            request.scene.is_diff = True
            request.scene.world.collision_objects = collision_objects

            self._pending_object_ids = list(result['object_ids'])
            self._apply_future = self._apply_planning_scene_client.call_async(request)
            self._apply_future.add_done_callback(self._on_apply_done)
        except Exception as exc:
            self.get_logger().error(f'Scene sync cycle crashed unexpectedly: {exc}')
        finally:
            self._sync_in_progress = False

    def _on_apply_done(self, future: Future) -> None:
        pending_ids = list(self._pending_object_ids)
        try:
            exception = future.exception()
            if exception is not None:
                self.get_logger().warning(
                    'ApplyPlanningScene request failed for '
                    f"{', '.join(pending_ids) or 'no objects'}: {exception}"
                )
                return

            response = future.result()
            if response is None:
                self.get_logger().warning('ApplyPlanningScene returned no response.')
                return

            if response.success:
                self._log_apply_success(pending_ids)
            else:
                self.get_logger().warning(
                    'ApplyPlanningScene reported success=false for '
                    f"{', '.join(pending_ids) or 'no objects'}"
                )
        except Exception as exc:
            self.get_logger().error(f'ApplyPlanningScene completion handling failed: {exc}')
        finally:
            self._apply_future = None
            self._pending_object_ids = []

    def _build_collision_object_result(self, registry_key: str, entry: Any) -> dict[str, Any]:
        if not isinstance(entry, dict):
            return self._failure_result(
                registry_key,
                registry_key,
                f'Registry entry {registry_key!r} must be a mapping.',
                status='invalid_registry',
            )

        enabled = bool(entry.get('enabled', True))
        object_id = str(entry.get('object_id', registry_key)).strip()
        if not object_id:
            return self._failure_result(
                registry_key,
                registry_key,
                f'Registry entry {registry_key!r} has an empty object_id.',
                status='invalid_registry',
            )

        if self._add_only_enabled and not enabled:
            return {
                'ok': True,
                'object_id': object_id,
                'status': 'skipped',
                'reason': 'disabled',
            }

        tf_frame = str(entry.get('tf_frame', '')).strip()
        if not tf_frame:
            return self._failure_result(
                registry_key,
                object_id,
                f'Registry entry {registry_key!r} must define tf_frame.',
                status='invalid_registry',
            )

        lookup_frame = str(entry.get('lookup_frame', 'world')).strip() or 'world'
        collision = entry.get('collision')
        if not isinstance(collision, dict):
            return self._failure_result(
                registry_key,
                object_id,
                f'Registry entry {registry_key!r} must define collision.',
                status='invalid_registry',
            )

        try:
            tf_pose = self._lookup_tf_pose(lookup_frame=lookup_frame, tf_frame=tf_frame)
            pose_offset = self._normalize_pose_offset(collision.get('pose_offset'), lookup_frame)
            collision_pose = compose_poses(tf_pose, pose_offset)
            primitive = self._build_primitive(collision)
            collision_object = self._build_collision_object_message(
                object_id=object_id,
                frame_id=lookup_frame,
                collision_pose=collision_pose,
                primitive=primitive,
            )
        except Exception as exc:
            return self._failure_result(
                registry_key,
                object_id,
                str(exc),
                status='build_failed',
                lookup_frame=lookup_frame,
                tf_frame=tf_frame,
            )

        return {
            'ok': True,
            'object_id': object_id,
            'lookup_frame': lookup_frame,
            'tf_frame': tf_frame,
            'collision_pose': collision_pose,
            'collision_object': collision_object,
        }

    def _build_collision_object_message(
        self,
        *,
        object_id: str,
        frame_id: str,
        collision_pose: dict[str, Any],
        primitive: SolidPrimitive,
    ) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [self._pose_dict_to_msg(collision_pose)]
        return collision_object

    def _build_primitive(self, collision: dict[str, Any]) -> SolidPrimitive:
        shape_type = str(collision.get('type', '')).strip().lower()
        primitive = SolidPrimitive()

        if shape_type == 'box':
            dimensions = collision.get('dimensions')
            if not isinstance(dimensions, list) or len(dimensions) != 3:
                raise ValueError('Box collision requires dimensions [x, y, z].')
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(value) for value in dimensions]
            return primitive

        if shape_type == 'sphere':
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [float(collision['radius'])]
            return primitive

        if shape_type == 'cylinder':
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [float(collision['height']), float(collision['radius'])]
            return primitive

        if shape_type == 'mesh':
            raise ValueError(
                'Mesh collision is not supported yet. Scene sync only supports box, sphere, and cylinder.'
            )

        raise ValueError(
            f"Unsupported collision type {shape_type!r}. Supported types now: box, sphere, cylinder, mesh."
        )

    def _pose_dict_to_msg(self, pose_dict: dict[str, Any]) -> Pose:
        pose = Pose()
        pose.position.x = float(pose_dict['position']['x'])
        pose.position.y = float(pose_dict['position']['y'])
        pose.position.z = float(pose_dict['position']['z'])
        pose.orientation.x = float(pose_dict['orientation']['x'])
        pose.orientation.y = float(pose_dict['orientation']['y'])
        pose.orientation.z = float(pose_dict['orientation']['z'])
        pose.orientation.w = float(pose_dict['orientation']['w'])
        return pose

    def _failure_result(
        self,
        registry_key: str,
        object_id: str,
        error: str,
        *,
        status: str,
        lookup_frame: str | None = None,
        tf_frame: str | None = None,
    ) -> dict[str, Any]:
        result: dict[str, Any] = {
            'ok': False,
            'object_id': object_id,
            'registry_key': registry_key,
            'status': status,
            'error': error,
        }
        if lookup_frame is not None:
            result['lookup_frame'] = lookup_frame
        if tf_frame is not None:
            result['tf_frame'] = tf_frame
        return result

    def _log_warning_throttled(self, *, key: str, message: str) -> None:
        now = time.monotonic()
        last_logged_at = self._warning_last_logged_at.get(key, 0.0)
        if now - last_logged_at >= self._warning_throttle_sec:
            self.get_logger().warning(message)
            self._warning_last_logged_at[key] = now

    def _log_cycle_summary(self, result: dict[str, Any]) -> None:
        signature = (
            int(result['success_count']),
            int(result['failure_count']),
            int(result['skipped_count']),
            len(result['object_ids']),
        )
        now = time.monotonic()
        should_log = (
            signature != self._last_summary_signature
            or now - self._last_summary_log_time >= self._summary_log_interval_sec
        )
        if not should_log:
            return

        self.get_logger().info(
            'Scene sync summary: '
            f"buildable={len(result['object_ids'])} "
            f"success={result['success_count']} "
            f"failure={result['failure_count']} "
            f"skipped={result['skipped_count']}"
        )
        self._last_summary_signature = signature
        self._last_summary_log_time = now

    def _log_apply_success(self, object_ids: list[str]) -> None:
        signature = tuple(object_ids)
        now = time.monotonic()
        should_log = (
            signature != self._last_apply_success_signature
            or now - self._last_apply_success_log_time >= self._summary_log_interval_sec
        )
        if not should_log:
            self.get_logger().debug(
                'Applied planning scene objects: ' + ', '.join(object_ids)
            )
            return

        self.get_logger().info(
            'Applied planning scene objects: ' + ', '.join(object_ids)
        )
        self._last_apply_success_signature = signature
        self._last_apply_success_log_time = now

    def _lookup_tf_pose(self, *, lookup_frame: str, tf_frame: str) -> dict[str, Any]:
        try:
            transform = self._tf_buffer.lookup_transform(
                lookup_frame,
                tf_frame,
                Time(),
                timeout=Duration(seconds=self._tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            raise RuntimeError(
                f"TF lookup failed for {lookup_frame} -> {tf_frame}: {exc}"
            ) from exc

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return make_pose(
            {
                'x': float(translation.x),
                'y': float(translation.y),
                'z': float(translation.z),
            },
            {
                'x': float(rotation.x),
                'y': float(rotation.y),
                'z': float(rotation.z),
                'w': float(rotation.w),
            },
            frame_id=lookup_frame,
        )

    def _normalize_pose_offset(self, pose_offset: Any, lookup_frame: str) -> dict[str, Any]:
        if pose_offset is None:
            return make_pose(
                {'x': 0.0, 'y': 0.0, 'z': 0.0},
                {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                frame_id=lookup_frame,
            )
        if not isinstance(pose_offset, dict):
            raise ValueError('collision.pose_offset must be a mapping when provided.')
        position = pose_offset.get('position', {})
        orientation = pose_offset.get('orientation', {})
        return make_pose(
            {
                'x': float(position.get('x', 0.0)),
                'y': float(position.get('y', 0.0)),
                'z': float(position.get('z', 0.0)),
            },
            {
                'x': float(orientation.get('x', 0.0)),
                'y': float(orientation.get('y', 0.0)),
                'z': float(orientation.get('z', 0.0)),
                'w': float(orientation.get('w', 1.0)),
            },
            frame_id=lookup_frame,
        )

    def _resolve_registry_path(self) -> str:
        configured = str(self.get_parameter('model_registry_path').value or '').strip()
        if configured:
            return configured
        share_dir = Path(get_package_share_directory('robot_backend'))
        return str(share_dir / 'config' / 'model_registry.yaml')

    def _load_registry(self, registry_path: str) -> dict[str, Any]:
        registry_file = Path(registry_path)
        if not registry_file.is_file():
            raise FileNotFoundError(f'Model registry file not found: {registry_path}')
        with registry_file.open('r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise ValueError('model_registry.yaml must parse to a top-level mapping.')
        return data


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SceneTFSyncNode | None = None
    try:
        node = SceneTFSyncNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
