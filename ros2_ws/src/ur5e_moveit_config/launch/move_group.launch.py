import importlib.util
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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


def _load_octomap_config(context):
    octomap_config = _load_yaml("config/sensors_3d.yaml")
    octomap_frame = LaunchConfiguration("octomap_frame").perform(context).strip()
    octomap_config["octomap_frame"] = octomap_frame
    octomap_config["octomap_resolution"] = float(
        LaunchConfiguration("octomap_resolution").perform(context)
    )

    sensor_names = octomap_config.get("sensors", [])
    if sensor_names:
        updater_name = sensor_names[0]
        updater_config = octomap_config.setdefault(updater_name, {})
        updater_config["point_cloud_topic"] = LaunchConfiguration("pointcloud_topic").perform(
            context
        )
        updater_config["max_range"] = float(
            LaunchConfiguration("octomap_max_range").perform(context)
        )
        updater_config["max_update_rate"] = float(
            LaunchConfiguration("max_update_rate").perform(context)
        )
        updater_config["filtered_cloud_topic"] = LaunchConfiguration(
            "filtered_cloud_topic"
        ).perform(context)

    return octomap_config


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
        parameters.append(_load_octomap_config(context))
        octomap_frame = parameters[-1]["octomap_frame"]
        return [
            LogInfo(msg=f"MoveIt octomap_frame: '{octomap_frame}'"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=parameters,
            )
        ]

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
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/depth/points_filtered",
                description="Point cloud topic consumed by MoveIt occupancy updates.",
            ),
            DeclareLaunchArgument(
                "octomap_frame",
                default_value="world",
                description="Frame used for MoveIt octomap updates.",
            ),
            DeclareLaunchArgument(
                "octomap_resolution",
                default_value="0.025",
                description=(
                    "MoveIt octomap resolution in meters. Keep in step with "
                    "sensors_3d.yaml and sim_moveit_bringup's octomap_resolution."
                ),
            ),
            DeclareLaunchArgument(
                "max_update_rate",
                default_value="2.0",
                description="Maximum MoveIt octomap update rate in Hz.",
            ),
            DeclareLaunchArgument(
                "octomap_max_range",
                default_value="2.0",
                description="Maximum point cloud range used by MoveIt octomap updates.",
            ),
            DeclareLaunchArgument(
                "filtered_cloud_topic",
                default_value="/moveit/filtered_cloud",
                description="MoveIt debug topic for the occupancy updater's filtered cloud.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
