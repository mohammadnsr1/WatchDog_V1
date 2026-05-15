from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                description="Path to the YAML file describing fixed camera extrinsics.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time from Isaac /clock.",
            ),
            Node(
                package="sim_moveit_bringup",
                executable="static_camera_tf_broadcaster.py",
                name="static_camera_tf_broadcaster",
                output="screen",
                parameters=[
                    {
                        "camera_config": LaunchConfiguration("camera_config"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
