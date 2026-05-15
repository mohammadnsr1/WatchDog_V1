#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
import yaml


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


class StaticCameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__("static_camera_tf_broadcaster")
        self.declare_parameter("camera_config", "")
        self.broadcaster = StaticTransformBroadcaster(self)
        self.transforms = []
        self._load_and_publish()
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, _params):
        return SetParametersResult(successful=True)

    def _load_and_publish(self):
        config_path = self.get_parameter("camera_config").get_parameter_value().string_value
        if not config_path:
            raise RuntimeError("camera_config parameter is empty.")

        config_file = Path(config_path)
        if not config_file.exists():
            raise FileNotFoundError(f"Camera config file not found: {config_file}")

        with config_file.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        cameras = data.get("cameras", [])
        if isinstance(cameras, dict):
            camera_entries = [
                {"name": camera_name, **camera_config}
                for camera_name, camera_config in cameras.items()
            ]
        elif isinstance(cameras, list):
            camera_entries = cameras
        else:
            raise TypeError("The 'cameras' entry must be a list or dictionary.")

        transforms = []
        enabled_count = 0

        for index, camera in enumerate(camera_entries):
            camera_name = camera.get("name", f"camera_{index}")
            if not camera.get("enabled", True):
                self.get_logger().info(f"Skipping disabled camera '{camera_name}'.")
                continue

            parent_frame = camera["parent_frame"]
            child_frame = camera["child_frame"]
            xyz = camera["xyz"]
            quat_xyzw = camera.get("quat_xyzw")
            rpy = camera.get("rpy")
            pointcloud_topic = camera.get("pointcloud_topic", "<not set>")

            if quat_xyzw is not None:
                qx, qy, qz, qw = (
                    float(quat_xyzw[0]),
                    float(quat_xyzw[1]),
                    float(quat_xyzw[2]),
                    float(quat_xyzw[3]),
                )
            elif rpy is not None:
                qx, qy, qz, qw = quaternion_from_euler(
                    float(rpy[0]), float(rpy[1]), float(rpy[2])
                )
            else:
                raise KeyError(
                    f"Camera '{camera_name}' must define either 'quat_xyzw' or 'rpy'."
                )

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = parent_frame
            transform.child_frame_id = child_frame
            transform.transform.translation.x = float(xyz[0])
            transform.transform.translation.y = float(xyz[1])
            transform.transform.translation.z = float(xyz[2])
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            transforms.append(transform)
            enabled_count += 1

            self.get_logger().info(
                f"Publishing static camera TF {parent_frame} -> {child_frame}\n"
                f"xyz={xyz}\n"
                f"quat_xyzw={[qx, qy, qz, qw]}\n"
                f"pointcloud_topic={pointcloud_topic}"
            )

        if not transforms:
            self.get_logger().warn("No enabled camera transforms found in the camera config.")
            return

        self.transforms = transforms
        self.broadcaster.sendTransform(transforms)
        self.get_logger().info(f"Published {enabled_count} static camera transform(s).")


def main():
    rclpy.init()
    node = StaticCameraTFBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
