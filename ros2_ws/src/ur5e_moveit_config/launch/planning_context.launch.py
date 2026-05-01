from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def load_file(package_name: str, relative_path: str) -> str:
    package_share = Path(get_package_share_directory(package_name))
    return (package_share / relative_path).read_text()


def load_yaml(package_name: str, relative_path: str):
    package_share = Path(get_package_share_directory(package_name))
    with (package_share / relative_path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def get_robot_description_parameters():
    robot_description = load_file("robot_description", "urdf/ur5e_ros.urdf")
    robot_description_semantic = load_file("ur5e_moveit_config", "config/ur5e.srdf")

    return {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
    }


def get_moveit_parameters():
    package_name = "ur5e_moveit_config"
    params = get_robot_description_parameters()
    params["robot_description_kinematics"] = load_yaml(package_name, "config/kinematics.yaml")
    params["robot_description_planning"] = load_yaml(package_name, "config/joint_limits.yaml")
    params["planning_pipelines"] = ["ompl"]
    params["default_planning_pipeline"] = "ompl"
    params["ompl"] = load_yaml(package_name, "config/ompl_planning.yaml")
    params.update(load_yaml(package_name, "config/moveit_controllers.yaml"))
    params.update(load_yaml(package_name, "config/moveit_cpp.yaml"))
    return params


def generate_launch_description():
    # This file is primarily used as a shared source of robot and MoveIt parameters
    # for the other launch files in this package.
    return LaunchDescription()
