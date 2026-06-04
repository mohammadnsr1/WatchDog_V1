"""Continuously sync object TF poses and model-registry collision shapes into MoveIt's planning scene."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener

from robot_backend.ros_moveit_backend import create_ros_moveit_backend
from robot_backend.utils.geometry import compose_poses
from robot_backend.utils.geometry import make_pose


_DEFAULT_SYNC_RATE_HZ = 1.0
_DEFAULT_TF_LOOKUP_TIMEOUT_SEC = 2.0
_DEFAULT_SUMMARY_LOG_INTERVAL_SEC = 10.0
_DEFAULT_WARNING_THROTTLE_SEC = 5.0


class SceneTFSyncNode(Node):
    """Read the model registry, resolve TF poses, and update MoveIt's scene."""

    def __init__(self) -> None:
        super().__init__('scene_tf_sync_node')
        self.declare_parameter('model_registry_path', '')
        self.declare_parameter('sync_rate_hz', _DEFAULT_SYNC_RATE_HZ)
        self.declare_parameter('remove_missing_objects', False)
        self.declare_parameter('tf_lookup_timeout_sec', _DEFAULT_TF_LOOKUP_TIMEOUT_SEC)
        self.declare_parameter('add_only_enabled', True)

        self._backend = create_ros_moveit_backend(node=self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._last_successful_sync_by_object: dict[str, dict[str, Any]] = {}
        self._sync_in_progress = False
        self._warning_last_logged_at: dict[str, float] = {}
        self._last_summary_signature: tuple[int, int, int] | None = None
        self._last_summary_log_time = 0.0

        self._model_registry_path = self._resolve_registry_path()
        self._registry = self._load_registry(self._model_registry_path)
        self._sync_rate_hz = float(self.get_parameter('sync_rate_hz').value)
        self._remove_missing_objects = bool(self.get_parameter('remove_missing_objects').value)
        self._tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self._add_only_enabled = bool(self.get_parameter('add_only_enabled').value)

        if self._sync_rate_hz <= 0.0:
            raise ValueError('sync_rate_hz must be > 0.')

        configured_objects = list(self._registry.get('objects', {}).keys())
        self.get_logger().info(f'Loaded model registry: {self._model_registry_path}')
        self.get_logger().info(f'Configured registry objects: {configured_objects}')
        self.get_logger().info(f'Scene sync rate: {self._sync_rate_hz} Hz')

        self._timer = self.create_timer(1.0 / self._sync_rate_hz, self._on_timer)

    def sync_once(self) -> dict[str, Any]:
        """Perform one registry -> TF -> planning scene update pass."""

        objects_section = self._registry.get('objects', {})
        if not isinstance(objects_section, dict):
            raise ValueError('model_registry.yaml must contain an objects mapping.')

        results: dict[str, Any] = {}
        success_count = 0
        failure_count = 0
        skipped_count = 0

        for registry_key, entry in objects_section.items():
            result = self._sync_registry_entry(registry_key, entry)
            results[registry_key] = result

            if result.get('status') == 'skipped':
                skipped_count += 1
            elif result.get('ok'):
                success_count += 1
                self.get_logger().debug(
                    f"Scene sync succeeded for {result['object_id']}"
                )
            else:
                failure_count += 1
                self._log_warning_throttled(
                    key=f"sync_failure:{result['object_id']}",
                    message=f"Scene sync failed for {result['object_id']}: {result.get('error')}",
                )

        return {
            'ok': failure_count == 0,
            'command': 'sync_once',
            'registry_path': self._model_registry_path,
            'success_count': success_count,
            'failure_count': failure_count,
            'skipped_count': skipped_count,
            'results': results,
        }

    def _on_timer(self) -> None:
        if self._sync_in_progress:
            self.get_logger().debug('Scene sync already in progress; skipping timer tick.')
            return

        self._sync_in_progress = True
        try:
            result = self.sync_once()
            self._log_cycle_summary(result)
        except Exception as exc:
            self.get_logger().error(f'Scene sync cycle crashed unexpectedly: {exc}')
        finally:
            self._sync_in_progress = False

    def _sync_registry_entry(self, registry_key: str, entry: Any) -> dict[str, Any]:
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

        shape_type = str(collision.get('type', '')).strip().lower()
        if shape_type == 'mesh':
            warning = (
                f"Mesh collision for {object_id} is not supported yet. "
                'Scene backend mesh loading is still a TODO.'
            )
            return self._failure_result(
                registry_key,
                object_id,
                warning,
                status='unsupported',
                lookup_frame=lookup_frame,
                tf_frame=tf_frame,
            )

        try:
            tf_pose = self._lookup_tf_pose(lookup_frame=lookup_frame, tf_frame=tf_frame)
        except RuntimeError as exc:
            if self._remove_missing_objects:
                try:
                    remove_result = self._backend.scene.remove_collision_object(object_id)
                    self.get_logger().debug(
                        f"Removed stale world object for {object_id} after TF lookup failure: {remove_result}"
                    )
                except Exception as remove_exc:
                    self._log_warning_throttled(
                        key=f"remove_failure:{object_id}",
                        message=(
                            f'Removing stale collision object for {object_id} failed after TF loss: '
                            f'{remove_exc}'
                        ),
                    )
            return self._failure_result(
                registry_key,
                object_id,
                str(exc),
                status='tf_unavailable',
                lookup_frame=lookup_frame,
                tf_frame=tf_frame,
            )

        try:
            pose_offset = self._normalize_pose_offset(collision.get('pose_offset'), lookup_frame)
            collision_pose = compose_poses(tf_pose, pose_offset)
            shape = self._build_shape_spec(collision)
            backend_result = self._backend.scene.add_collision_object(
                object_id=object_id,
                pose=collision_pose,
                shape=shape,
            )
        except Exception as exc:
            return self._failure_result(
                registry_key,
                object_id,
                str(exc),
                status='backend_error',
                lookup_frame=lookup_frame,
                tf_frame=tf_frame,
            )

        result = {
            'ok': bool(backend_result.get('ok', False)),
            'object_id': object_id,
            'lookup_frame': lookup_frame,
            'tf_frame': tf_frame,
            'collision_pose': collision_pose,
            'shape': shape,
            'backend_result': backend_result,
        }
        if result['ok']:
            self._last_successful_sync_by_object[object_id] = result
            return result

        return self._failure_result(
            registry_key,
            object_id,
            str(backend_result.get('error', 'add_collision_object returned ok=false')),
            status='backend_error',
            lookup_frame=lookup_frame,
            tf_frame=tf_frame,
            backend_result=backend_result,
        )

    def _failure_result(
        self,
        registry_key: str,
        object_id: str,
        error: str,
        *,
        status: str,
        lookup_frame: str | None = None,
        tf_frame: str | None = None,
        backend_result: dict[str, Any] | None = None,
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
        if backend_result is not None:
            result['backend_result'] = backend_result
        return result

    def _log_warning_throttled(self, *, key: str, message: str) -> None:
        now = time.monotonic()
        last_logged_at = self._warning_last_logged_at.get(key, 0.0)
        if now - last_logged_at >= _DEFAULT_WARNING_THROTTLE_SEC:
            self.get_logger().warning(message)
            self._warning_last_logged_at[key] = now

    def _log_cycle_summary(self, result: dict[str, Any]) -> None:
        signature = (
            int(result['success_count']),
            int(result['failure_count']),
            int(result['skipped_count']),
        )
        now = time.monotonic()
        should_log = (
            signature != self._last_summary_signature
            or now - self._last_summary_log_time >= _DEFAULT_SUMMARY_LOG_INTERVAL_SEC
        )
        if not should_log:
            return

        self.get_logger().info(
            'Scene sync summary: '
            f"success={result['success_count']} "
            f"failure={result['failure_count']} "
            f"skipped={result['skipped_count']}"
        )
        self._last_summary_signature = signature
        self._last_summary_log_time = now

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

    def _build_shape_spec(self, collision: dict[str, Any]) -> dict[str, Any]:
        shape_type = str(collision.get('type', '')).strip().lower()
        if shape_type == 'box':
            dimensions = collision.get('dimensions')
            if not isinstance(dimensions, list) or len(dimensions) != 3:
                raise ValueError('Box collision requires dimensions [x, y, z].')
            return {
                'type': 'box',
                'dimensions': [float(value) for value in dimensions],
            }
        if shape_type == 'sphere':
            return {
                'type': 'sphere',
                'radius': float(collision['radius']),
            }
        if shape_type == 'cylinder':
            return {
                'type': 'cylinder',
                'height': float(collision['height']),
                'radius': float(collision['radius']),
            }
        if shape_type == 'mesh':
            return {
                'type': 'mesh',
                'mesh_resource': collision.get('mesh_resource', ''),
                'scale': [float(value) for value in collision.get('scale', [1.0, 1.0, 1.0])],
            }
        raise ValueError(
            f"Unsupported collision type {shape_type!r}. Supported types now: box, sphere, cylinder, mesh."
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
