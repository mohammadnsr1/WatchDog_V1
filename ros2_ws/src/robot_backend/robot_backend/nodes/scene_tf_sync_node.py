"""Sync object TF poses and model-registry collision shapes into MoveIt's planning scene.

Manual usage later:
- Prerequisites:
  - Isaac Sim publishing TF frames such as World -> red_cube and World -> green_cube.
  - Bringup publishing the static identity transform world -> World.
  - MoveIt services /apply_planning_scene and /get_planning_scene available.
- Then run this node once or on a timer to push registry-defined objects into MoveIt.
- After a sync, use the scene backend smoke test list mode to confirm red_cube and
  green_cube appear in the planning scene.
"""

from __future__ import annotations

import json
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
_DEFAULT_TF_LOOKUP_TIMEOUT_SEC = 0.5


class SceneTFSyncNode(Node):
    """Read the model registry, resolve TF poses, and update MoveIt's scene."""

    def __init__(self) -> None:
        super().__init__('scene_tf_sync_node')
        self.declare_parameter('model_registry_path', '')
        self.declare_parameter('sync_rate_hz', _DEFAULT_SYNC_RATE_HZ)
        self.declare_parameter('once', False)
        self.declare_parameter('remove_missing_objects', False)
        self.declare_parameter('tf_lookup_timeout_sec', _DEFAULT_TF_LOOKUP_TIMEOUT_SEC)
        self.declare_parameter('add_only_enabled', True)

        self._backend = create_ros_moveit_backend(node=self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._last_successful_sync_by_object: dict[str, dict[str, Any]] = {}

        self._model_registry_path = self._resolve_registry_path()
        self._registry = self._load_registry(self._model_registry_path)
        self._sync_rate_hz = float(self.get_parameter('sync_rate_hz').value)
        self._once = bool(self.get_parameter('once').value)
        self._remove_missing_objects = bool(self.get_parameter('remove_missing_objects').value)
        self._tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)
        self._add_only_enabled = bool(self.get_parameter('add_only_enabled').value)
        self._timer = None

        configured_objects = list(self._registry.get('objects', {}).keys())
        self.get_logger().info(f'Loaded model registry: {self._model_registry_path}')
        self.get_logger().info(f'Configured registry objects: {configured_objects}')

        if not self._once:
            if self._sync_rate_hz <= 0.0:
                raise ValueError('sync_rate_hz must be > 0 when once=false.')
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
            try:
                result = self._sync_registry_entry(registry_key, entry)
            except Exception as exc:
                failure_count += 1
                result = {
                    'ok': False,
                    'object_id': registry_key,
                    'status': 'exception',
                    'error': str(exc),
                }
                self.get_logger().error(
                    f"Scene sync failed for {registry_key}: {exc}"
                )
            else:
                if result.get('status') == 'skipped':
                    skipped_count += 1
                elif result.get('ok'):
                    success_count += 1
                else:
                    failure_count += 1

                if result.get('ok'):
                    self.get_logger().info(
                        f"Scene sync succeeded for {result['object_id']}"
                    )
                elif result.get('status') == 'skipped':
                    self.get_logger().info(
                        f"Scene sync skipped for {result['object_id']}: {result.get('reason')}"
                    )
                else:
                    self.get_logger().warning(
                        f"Scene sync failed for {result['object_id']}: {result.get('error')}"
                    )

            results[registry_key] = result

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
        result = self.sync_once()
        self.get_logger().info(
            'Scene sync summary: '
            + json.dumps(
                {
                    'success_count': result['success_count'],
                    'failure_count': result['failure_count'],
                    'skipped_count': result['skipped_count'],
                },
                sort_keys=True,
            )
        )

    def _sync_registry_entry(self, registry_key: str, entry: Any) -> dict[str, Any]:
        if not isinstance(entry, dict):
            raise ValueError(f'Registry entry {registry_key!r} must be a mapping.')

        enabled = bool(entry.get('enabled', True))
        object_id = str(entry.get('object_id', registry_key)).strip()
        if not object_id:
            raise ValueError(f'Registry entry {registry_key!r} has an empty object_id.')

        if self._add_only_enabled and not enabled:
            return {
                'ok': True,
                'object_id': object_id,
                'status': 'skipped',
                'reason': 'disabled',
            }

        tf_frame = str(entry.get('tf_frame', '')).strip()
        if not tf_frame:
            raise ValueError(f'Registry entry {registry_key!r} must define tf_frame.')
        lookup_frame = str(entry.get('lookup_frame', 'world')).strip() or 'world'

        collision = entry.get('collision')
        if not isinstance(collision, dict):
            raise ValueError(f'Registry entry {registry_key!r} must define collision.')

        shape_type = str(collision.get('type', '')).strip().lower()
        if shape_type == 'mesh':
            warning = (
                f"Mesh collision for {object_id} is not supported yet. "
                'Scene backend mesh loading is still a TODO.'
            )
            self.get_logger().warning(warning)
            return {
                'ok': False,
                'object_id': object_id,
                'status': 'unsupported',
                'error': warning,
            }

        try:
            tf_pose = self._lookup_tf_pose(lookup_frame=lookup_frame, tf_frame=tf_frame)
        except RuntimeError as exc:
            if self._remove_missing_objects:
                remove_result = self._backend.scene.remove_collision_object(object_id)
                self.get_logger().warning(
                    f"Removed stale world object for {object_id} after TF lookup failure: {remove_result}"
                )
            raise
        pose_offset = self._normalize_pose_offset(collision.get('pose_offset'), lookup_frame)
        collision_pose = compose_poses(tf_pose, pose_offset)
        shape = self._build_shape_spec(collision)
        backend_result = self._backend.scene.add_collision_object(
            object_id=object_id,
            pose=collision_pose,
            shape=shape,
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
        if node.get_parameter('once').value:
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)
            result = node.sync_once()
            node.get_logger().info(json.dumps(result, sort_keys=True))
        else:
            rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
