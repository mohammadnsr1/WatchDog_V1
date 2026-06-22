from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if Isaac publishes /clock.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "rsp.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "move_group.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "moveit_rviz.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
