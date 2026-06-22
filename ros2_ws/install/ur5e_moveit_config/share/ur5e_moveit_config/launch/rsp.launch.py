import importlib.util
from pathlib import Path

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
    description_parameters = planning_context.get_robot_description_parameters()

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if Isaac publishes /clock.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    description_parameters,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
