"""Restart ONLY move_group and RViz, leaving the rest of the cell running.

Isaac Sim's /clock jumps back to zero on stop/replay. Every node running with
use_sim_time then holds TF and message-filter data stamped in what is now the
future, which is what makes RViz spew extrapolation errors -- RViz's Reset
button clears its own display state but not that poisoned buffer, so it does
not help. The cheapest reliable fix is a fresh process.

Nothing in sim_moveit_bringup.launch.py registers an on_exit or Shutdown
handler, so killing move_group and rviz2 leaves the bridge, the RG2
controller, scene_tf_sync_node, the orchestrator and the bag recorder alive:

    pkill -f "moveit_ros_move_group/move_group"
    pkill -f "rviz2 -d"
    ros2 launch sim_moveit_bringup restart_moveit.launch.py

Defaults below MUST stay in step with the values sim_moveit_bringup.launch.py
passes into move_group.launch.py; a mismatch means the restarted move_group
plans against a different configuration than the original.

Other nodes keep their own stale TF buffers after a sim-time reset. Flush the
scene sync node's separately -- it exposes a service for exactly this:

    ros2 service call /reset_tf std_srvs/srv/Trigger {}
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur5e_launch_dir = PathJoinSubstitution(
        [FindPackageShare("ur5e_moveit_config"), "launch"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Set false to restart move_group only.",
            ),
            DeclareLaunchArgument("use_octomap", default_value="true"),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/depth/points_filtered",
                description=(
                    "Matches the bringup's filtered_pointcloud_topic. Change only "
                    "if the bringup was started with filter_pointcloud:=false."
                ),
            ),
            DeclareLaunchArgument("octomap_resolution", default_value="0.01"),
            DeclareLaunchArgument(
                "max_update_rate",
                default_value="10.0",
                description="Keep equal to the bringup's pointcloud_filter_rate.",
            ),
            DeclareLaunchArgument("octomap_max_range", default_value="2.0"),
            DeclareLaunchArgument(
                "filtered_cloud_topic", default_value="/moveit/filtered_cloud"
            ),
            DeclareLaunchArgument("octomap_frame", default_value="world"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ur5e_launch_dir, "move_group.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_octomap": LaunchConfiguration("use_octomap"),
                    "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    "octomap_resolution": LaunchConfiguration("octomap_resolution"),
                    "max_update_rate": LaunchConfiguration("max_update_rate"),
                    "octomap_max_range": LaunchConfiguration("octomap_max_range"),
                    "filtered_cloud_topic": LaunchConfiguration("filtered_cloud_topic"),
                    "octomap_frame": LaunchConfiguration("octomap_frame"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ur5e_launch_dir, "moveit_rviz.launch.py"])
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
