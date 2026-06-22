from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    package_share = Path(get_package_share_directory("robot_description"))
    urdf_path = package_share / "urdf" / "ur5e_ros.urdf"
    rviz_config = package_share / "rviz" / "ur5e.rviz"
    robot_description = urdf_path.read_text()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_publisher_gui",
                default_value="false",
                description="Start joint_state_publisher_gui for manual joint control.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=UnlessCondition(use_joint_state_publisher_gui),
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_joint_state_publisher_gui),
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
            ),
        ]
    )
