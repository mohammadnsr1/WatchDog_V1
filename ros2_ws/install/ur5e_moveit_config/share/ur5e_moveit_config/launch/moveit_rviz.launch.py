import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_planning_context_module():
    module_path = Path(__file__).with_name("planning_context.launch.py")
    spec = importlib.util.spec_from_file_location("ur5e_planning_context", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description():
    planning_context = _load_planning_context_module()
    moveit_parameters = planning_context.get_moveit_parameters()

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = Path(get_package_share_directory("ur5e_moveit_config")) / "rviz" / "moveit.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if Isaac publishes /clock.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[
                    moveit_parameters,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
