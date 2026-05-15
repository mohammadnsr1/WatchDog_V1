from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_octomap = LaunchConfiguration("use_octomap")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    publish_camera_tf = LaunchConfiguration("publish_camera_tf")
    camera_config = LaunchConfiguration("camera_config")

    ur5e_launch_dir = PathJoinSubstitution([FindPackageShare("ur5e_moveit_config"), "launch"])
    bridge_launch_dir = PathJoinSubstitution([FindPackageShare("isaac_trajectory_bridge"), "launch"])
    sim_bringup_launch_dir = PathJoinSubstitution([FindPackageShare("sim_moveit_bringup"), "launch"])
    default_camera_config = PathJoinSubstitution(
        [FindPackageShare("sim_moveit_bringup"), "config", "cameras.yaml"]
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ur5e_launch_dir, "rsp.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    move_group_launch = GroupAction(
        [
            SetRemap("/camera/depth/points", pointcloud_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([ur5e_launch_dir, "move_group.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_octomap": use_octomap,
                }.items(),
            ),
        ]
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ur5e_launch_dir, "moveit_rviz.launch.py"])),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(launch_rviz),
    )

    trajectory_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bridge_launch_dir, "trajectory_bridge.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "publish_full_joint_command": "true",
        }.items(),
    )

    camera_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_bringup_launch_dir, "camera_tf.launch.py"])
        ),
        launch_arguments={
            "camera_config": camera_config,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(publish_camera_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time from Isaac /clock.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch MoveIt RViz alongside move_group.",
            ),
            DeclareLaunchArgument(
                "use_octomap",
                default_value="true",
                description="Enable MoveIt occupancy map updates from the configured point cloud.",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/depth/points",
                description="Point cloud topic published by Isaac for occupancy updates.",
            ),
            DeclareLaunchArgument(
                "publish_camera_tf",
                default_value="true",
                description="Publish fixed world-to-camera static transforms from cameras.yaml.",
            ),
            DeclareLaunchArgument(
                "camera_config",
                default_value=default_camera_config,
                description="Path to the YAML file describing fixed camera extrinsics.",
            ),
            LogInfo(msg="Isaac Sim must already be running and simulation must be playing."),
            LogInfo(msg=["Expected topics: /joint_states, /clock, ", pointcloud_topic]),
            LogInfo(msg=["Camera TF config: ", camera_config]),
            LogInfo(
                msg=(
                    "The bringup launch passes publish_full_joint_command:=true to the "
                    "trajectory bridge because that argument is supported today."
                )
            ),
            rsp_launch,
            move_group_launch,
            moveit_rviz_launch,
            trajectory_bridge_launch,
            camera_tf_launch,
        ]
    )
