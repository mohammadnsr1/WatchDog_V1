#!/usr/bin/env python3
import math
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class PointCloudWorkspaceFilter(Node):
    def __init__(self):
        super().__init__("pointcloud_workspace_filter")

        self.declare_parameter("input_topic", "/camera/depth/points")
        self.declare_parameter("output_topic", "/camera/depth/points_filtered")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("output_stamp_mode", "input")
        self.declare_parameter("max_publish_rate", 2.0)
        self.declare_parameter("use_crop_box", True)
        self.declare_parameter("crop_min_x", -0.2)
        self.declare_parameter("crop_max_x", 1.3)
        self.declare_parameter("crop_min_y", -0.6)
        self.declare_parameter("crop_max_y", 0.8)
        self.declare_parameter("crop_min_z", 0.0)
        self.declare_parameter("crop_max_z", 1.4)
        self.declare_parameter("use_voxel_filter", True)
        self.declare_parameter("voxel_leaf_size", 0.03)
        self.declare_parameter("reliable_output_qos", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = str(self.get_parameter("target_frame").value).strip()
        self.output_stamp_mode = str(self.get_parameter("output_stamp_mode").value).strip().lower()
        self.max_publish_rate = float(self.get_parameter("max_publish_rate").value)
        self.use_crop_box = bool(self.get_parameter("use_crop_box").value)
        self.crop_min_x = float(self.get_parameter("crop_min_x").value)
        self.crop_max_x = float(self.get_parameter("crop_max_x").value)
        self.crop_min_y = float(self.get_parameter("crop_min_y").value)
        self.crop_max_y = float(self.get_parameter("crop_max_y").value)
        self.crop_min_z = float(self.get_parameter("crop_min_z").value)
        self.crop_max_z = float(self.get_parameter("crop_max_z").value)
        self.use_voxel_filter = bool(self.get_parameter("use_voxel_filter").value)
        self.voxel_leaf_size = float(self.get_parameter("voxel_leaf_size").value)
        self.reliable_output_qos = bool(self.get_parameter("reliable_output_qos").value)

        if self.output_stamp_mode not in {"now", "input"}:
            raise ValueError("output_stamp_mode must be one of: input, now. Use now only as an emergency/debug workaround.")

        if self.use_crop_box:
            self._validate_crop_bounds()
        if self.use_voxel_filter and self.voxel_leaf_size <= 0.0:
            raise ValueError("voxel_leaf_size must be > 0 when use_voxel_filter is true.")

        self._min_publish_period_sec = (
            1.0 / self.max_publish_rate if self.max_publish_rate > 0.0 else 0.0
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        output_qos = QoSProfile(
            reliability=(
                ReliabilityPolicy.RELIABLE
                if self.reliable_output_qos
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Isaac's /clock restarts at zero on replay, leaving this buffer holding
        # transforms stamped in what is now the future. tf2 then rejects every
        # incoming transform as TF_OLD_DATA and never recovers on its own: the
        # stale entries only age out as newer ones arrive, and those are exactly
        # what is being rejected. Recreating the buffer is the way out, so
        # expose it as a service instead of requiring this node be restarted.
        # Mirrors scene_tf_sync_node's /reset_tf.
        self.reset_tf_service = self.create_service(
            Trigger, "~/reset_tf", self._handle_reset_tf
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, output_qos)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._pointcloud_callback,
            sensor_qos,
        )

        self._last_publish_time_sec = float("-inf")
        self._last_log_times: Dict[str, float] = {}

        self._log_startup_configuration()

    def _handle_reset_tf(self, request: Trigger.Request, response: Trigger.Response):
        """Destroy and recreate the TF buffer to flush pre-reset transforms."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_publish_time_sec = float("-inf")
        self.get_logger().info("TF buffer flushed and recreated.")
        response.success = True
        response.message = "TF buffer flushed and recreated."
        return response

    def _validate_crop_bounds(self):
        crop_bounds = [
            ("crop_min_x", self.crop_min_x, "crop_max_x", self.crop_max_x),
            ("crop_min_y", self.crop_min_y, "crop_max_y", self.crop_max_y),
            ("crop_min_z", self.crop_min_z, "crop_max_z", self.crop_max_z),
        ]
        for min_name, min_value, max_name, max_value in crop_bounds:
            if min_value > max_value:
                raise ValueError(f"{min_name} must be <= {max_name}.")

    def _log_startup_configuration(self):
        self.get_logger().info(f"Point cloud filter input topic: {self.input_topic}")
        self.get_logger().info(f"Point cloud filter output topic: {self.output_topic}")
        self.get_logger().info(f"Point cloud filter target frame: {self.target_frame}")
        self.get_logger().info(
            f"Point cloud filter output stamp mode: {self.output_stamp_mode} "
            "(default: input; now is an emergency/debug workaround that can create robot ghost points from stale clouds during motion)."
        )
        self.get_logger().info(
            "Point cloud filter crop limits: "
            f"x=[{self.crop_min_x}, {self.crop_max_x}] "
            f"y=[{self.crop_min_y}, {self.crop_max_y}] "
            f"z=[{self.crop_min_z}, {self.crop_max_z}] "
            f"enabled={self.use_crop_box}"
        )
        self.get_logger().info(
            f"Point cloud filter voxel leaf size: {self.voxel_leaf_size} enabled={self.use_voxel_filter}"
        )
        self.get_logger().info(f"Point cloud filter max publish rate: {self.max_publish_rate} Hz")
        self.get_logger().info(
            "Point cloud filter output reliability: "
            f"{'RELIABLE' if self.reliable_output_qos else 'BEST_EFFORT'}"
        )

    def _pointcloud_callback(self, msg: PointCloud2):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._min_publish_period_sec > 0.0:
            elapsed = now_sec - self._last_publish_time_sec
            if elapsed < self._min_publish_period_sec:
                return

        if not msg.header.frame_id:
            if self._should_log("missing_frame", 5.0):
                self.get_logger().warn("Skipping point cloud with empty frame_id.")
            return

        try:
            transform = self._lookup_transform(msg)
        except TransformException as exc:
            if self._should_log("tf_lookup", 5.0):
                self.get_logger().warn(
                    f"Skipping point cloud because TF to '{self.target_frame}' is unavailable: {exc}"
                )
            return

        raw_count = 0
        cropped_count = 0
        filtered_points = []
        voxel_accumulators: Dict[Tuple[int, int, int], Tuple[float, float, float, int]] = {}

        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
            raw_count += 1
            x, y, z = point
            if not self._is_finite_point(x, y, z):
                continue

            world_x, world_y, world_z = self._transform_xyz(transform, x, y, z)

            if self.use_crop_box and not self._point_in_crop_box(world_x, world_y, world_z):
                continue

            cropped_count += 1

            if self.use_voxel_filter:
                key = (
                    math.floor(world_x / self.voxel_leaf_size),
                    math.floor(world_y / self.voxel_leaf_size),
                    math.floor(world_z / self.voxel_leaf_size),
                )
                sum_x, sum_y, sum_z, count = voxel_accumulators.get(key, (0.0, 0.0, 0.0, 0))
                voxel_accumulators[key] = (
                    sum_x + world_x,
                    sum_y + world_y,
                    sum_z + world_z,
                    count + 1,
                )
            else:
                filtered_points.append((world_x, world_y, world_z))

        if self.use_voxel_filter:
            filtered_points = [
                (sum_x / count, sum_y / count, sum_z / count)
                for sum_x, sum_y, sum_z, count in voxel_accumulators.values()
                if count > 0
            ]

        output_header = Header()
        output_header.frame_id = self.target_frame.strip()
        if self.output_stamp_mode == "now":
            output_header.stamp = self.get_clock().now().to_msg()
        else:
            output_header.stamp = msg.header.stamp

        filtered_cloud = point_cloud2.create_cloud_xyz32(output_header, filtered_points)
        self.publisher.publish(filtered_cloud)
        self._last_publish_time_sec = now_sec

        if self._should_log("stats", 10.0):
            node_now = self.get_clock().now()
            self.get_logger().info(
                "Point cloud filter stats: "
                f"input_stamp={self._format_stamp(msg.header.stamp)} "
                f"output_stamp={self._format_stamp(filtered_cloud.header.stamp)} "
                f"node_now={self._format_stamp(node_now.to_msg())} "
                f"input_age_sec={self._format_age_sec(node_now, msg.header.stamp)} "
                f"output_age_sec={self._format_age_sec(node_now, filtered_cloud.header.stamp)} "
                f"output_stamp_mode={self.output_stamp_mode} "
                "(expected: input_age_sec should stay small after Isaac simulation time is reset; large input_age_sec usually means Isaac time was not reset or camera stamps are out of sync with /clock) "
                f"raw={raw_count} cropped={cropped_count} filtered={len(filtered_points)}"
            )

    def _lookup_transform(self, msg: PointCloud2):
        if msg.header.frame_id == self.target_frame:
            return None

        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            stamp = Time()
        else:
            stamp = Time.from_msg(msg.header.stamp)

        return self.tf_buffer.lookup_transform(
            self.target_frame,
            msg.header.frame_id,
            stamp,
            timeout=Duration(seconds=0.05),
        )

    def _transform_xyz(self, transform, x: float, y: float, z: float):
        if transform is None:
            return x, y, z

        rotation = transform.transform.rotation
        translation = transform.transform.translation

        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rot_x = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z
        rot_y = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z
        rot_z = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z

        return (
            rot_x + translation.x,
            rot_y + translation.y,
            rot_z + translation.z,
        )
    
    
    @staticmethod
    def _format_stamp(stamp) -> str:
        return f"{stamp.sec}.{stamp.nanosec:09d}"
    
    
    @staticmethod
    def _stamp_to_sec(stamp) -> Optional[float]:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        return stamp.sec + (stamp.nanosec / 1e9)

    def _format_age_sec(self, now: Time, stamp) -> str:
        stamp_sec = self._stamp_to_sec(stamp)
        if stamp_sec is None:
            return "n/a"
        return f"{(now.nanoseconds / 1e9) - stamp_sec:.3f}"

    def _point_in_crop_box(self, x: float, y: float, z: float) -> bool:
        return (
            self.crop_min_x <= x <= self.crop_max_x
            and self.crop_min_y <= y <= self.crop_max_y
            and self.crop_min_z <= z <= self.crop_max_z
        )

    @staticmethod
    def _is_finite_point(x: float, y: float, z: float) -> bool:
        return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)

    def _should_log(self, key: str, interval_sec: float) -> bool:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        last_time = self._last_log_times.get(key, float("-inf"))
        if now_sec - last_time >= interval_sec:
            self._last_log_times[key] = now_sec
            return True
        return False


def main():
    rclpy.init()
    node = PointCloudWorkspaceFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
