import importlib.util
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_planning_context_module():
    module_path = Path(__file__).with_name("planning_context.launch.py")
    spec = importlib.util.spec_from_file_location("ur5e_planning_context", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_yaml(relative_path: str):
    path = Path(__file__).resolve().parent.parent / relative_path
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _launch_setup(context):
    planning_context = _load_planning_context_module()
    moveit_parameters = planning_context.get_moveit_parameters()

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_octomap = LaunchConfiguration("use_octomap")

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    move_group_capabilities = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "use_sim_time": use_sim_time,
    }

    parameters = [
        moveit_parameters,
        planning_scene_monitor_parameters,
        move_group_capabilities,
    ]

    if use_octomap.perform(context).lower() in {"true", "1", "yes", "on"}:
        parameters.append(_load_yaml("config/sensors_3d.yaml"))

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if Isaac publishes /clock.",
            ),
            DeclareLaunchArgument(
                "use_octomap",
                default_value="true",
                description="Enable occupancy map updates from config/sensors_3d.yaml.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
