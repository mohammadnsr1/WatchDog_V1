from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def _is_truthy(value: str) -> bool:
    return value.lower() in {"true", "1", "yes", "on"}


def _launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_octomap = LaunchConfiguration("use_octomap")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    publish_camera_tf = LaunchConfiguration("publish_camera_tf")
    camera_config = LaunchConfiguration("camera_config")
    filter_pointcloud = LaunchConfiguration("filter_pointcloud")
    raw_pointcloud_topic = LaunchConfiguration("raw_pointcloud_topic")
    filtered_pointcloud_topic = LaunchConfiguration("filtered_pointcloud_topic")
    pointcloud_filter_rate = LaunchConfiguration("pointcloud_filter_rate")
    voxel_leaf_size = LaunchConfiguration("voxel_leaf_size")
    use_crop_box = LaunchConfiguration("use_crop_box")
    use_voxel_filter = LaunchConfiguration("use_voxel_filter")
    reliable_output_qos = LaunchConfiguration("reliable_output_qos")
    crop_min_x = LaunchConfiguration("crop_min_x")
    crop_max_x = LaunchConfiguration("crop_max_x")
    crop_min_y = LaunchConfiguration("crop_min_y")
    crop_max_y = LaunchConfiguration("crop_max_y")
    crop_min_z = LaunchConfiguration("crop_min_z")
    crop_max_z = LaunchConfiguration("crop_max_z")
    target_frame = LaunchConfiguration("target_frame")
    output_stamp_mode = LaunchConfiguration("output_stamp_mode")

    ur5e_launch_dir = PathJoinSubstitution([FindPackageShare("ur5e_moveit_config"), "launch"])
    bridge_launch_dir = PathJoinSubstitution([FindPackageShare("isaac_trajectory_bridge"), "launch"])
    sim_bringup_launch_dir = PathJoinSubstitution([FindPackageShare("sim_moveit_bringup"), "launch"])
    filter_enabled = _is_truthy(filter_pointcloud.perform(context))
    raw_topic_value = raw_pointcloud_topic.perform(context)
    filtered_topic_value = filtered_pointcloud_topic.perform(context)
    direct_topic_value = pointcloud_topic.perform(context)
    target_frame_value = target_frame.perform(context).strip()
    moveit_pointcloud_topic = filtered_topic_value if filter_enabled else direct_topic_value
    actions = [
        LogInfo(msg="Isaac Sim must already be running and simulation must be playing."),
        LogInfo(
            msg=(
                "Reset Isaac Sim simulation time before starting this bringup. "
                "/clock, /joint_states, TF, and camera point cloud stamps must share the same simulation-time base. "
                "The bringup uses point cloud input stamps by default."
            )
        ),
        LogInfo(msg=["Raw point cloud topic: ", raw_pointcloud_topic]),
        LogInfo(msg=["MoveIt point cloud topic: ", moveit_pointcloud_topic]),
        LogInfo(msg=["Camera TF config: ", camera_config]),
        LogInfo(msg=f"MoveIt octomap_frame: '{target_frame_value}'"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([ur5e_launch_dir, "rsp.launch.py"])),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([ur5e_launch_dir, "move_group.launch.py"])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_octomap": use_octomap,
                "pointcloud_topic": moveit_pointcloud_topic,
                "octomap_resolution": "0.05",
                "max_update_rate": pointcloud_filter_rate,
                "octomap_max_range": "2.0",
                "filtered_cloud_topic": "/moveit/filtered_cloud",
                "octomap_frame": target_frame_value,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ur5e_launch_dir, "moveit_rviz.launch.py"])
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
            condition=IfCondition(launch_rviz),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bridge_launch_dir, "trajectory_bridge.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "publish_full_joint_command": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim_bringup_launch_dir, "camera_tf.launch.py"])
            ),
            launch_arguments={
                "camera_config": camera_config,
                "use_sim_time": use_sim_time,
            }.items(),
            condition=IfCondition(publish_camera_tf),
        ),
        LogInfo(
            msg=(
                "The bringup launch uses arm-only /joint_command publishing by "
                "default to avoid commanding RG2 gripper joints during MoveIt arm "
                "execution."
            )
        ),
    ]

    if filter_enabled:
        actions.insert(
            4,
            Node(
                package="sim_moveit_bringup",
                executable="pointcloud_workspace_filter",
                name="pointcloud_workspace_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": raw_topic_value,
                        "output_topic": filtered_topic_value,
                        "target_frame": target_frame_value,
                        "output_stamp_mode": output_stamp_mode,
                        "max_publish_rate": float(pointcloud_filter_rate.perform(context)),
                        "use_crop_box": _is_truthy(use_crop_box.perform(context)),
                        "crop_min_x": float(crop_min_x.perform(context)),
                        "crop_max_x": float(crop_max_x.perform(context)),
                        "crop_min_y": float(crop_min_y.perform(context)),
                        "crop_max_y": float(crop_max_y.perform(context)),
                        "crop_min_z": float(crop_min_z.perform(context)),
                        "crop_max_z": float(crop_max_z.perform(context)),
                        "use_voxel_filter": _is_truthy(use_voxel_filter.perform(context)),
                        "voxel_leaf_size": float(voxel_leaf_size.perform(context)),
                        "reliable_output_qos": _is_truthy(reliable_output_qos.perform(context)),
                    }
                ],
            ),
        )
        actions.insert(
            4,
            LogInfo(msg=["Point cloud filtering enabled. Publishing filtered cloud on: ", filtered_pointcloud_topic]),
        )
    else:
        actions.insert(
            4,
            LogInfo(msg="Point cloud filtering disabled. MoveIt will consume the configured pointcloud_topic directly."),
        )

    return actions


def generate_launch_description():
    default_camera_config = PathJoinSubstitution(
        [FindPackageShare("sim_moveit_bringup"), "config", "cameras.yaml"]
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
                default_value="/camera/depth/points_filtered",
                description="Point cloud topic consumed directly by MoveIt when filtering is disabled.",
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
            DeclareLaunchArgument(
                "filter_pointcloud",
                default_value="true",
                description="Launch the workspace point cloud filter and feed MoveIt the filtered cloud.",
            ),
            DeclareLaunchArgument(
                "raw_pointcloud_topic",
                default_value="/camera/depth/points",
                description="Raw Isaac point cloud topic consumed by the workspace filter.",
            ),
            DeclareLaunchArgument(
                "filtered_pointcloud_topic",
                default_value="/camera/depth/points_filtered",
                description="Filtered point cloud topic published for MoveIt occupancy updates.",
            ),
            DeclareLaunchArgument(
                "pointcloud_filter_rate",
                default_value="2.0",
                description="Maximum publish rate for the filtered point cloud in Hz.",
            ),
            DeclareLaunchArgument(
                "output_stamp_mode",
                default_value="input",
                description=(
                    "Point cloud output stamp mode: use input by default after resetting Isaac simulation time "
                    "so cloud, TF, and robot state stay time-consistent; use now only as an emergency/debug "
                    "workaround for stale sensor stamps, because restamping old clouds as now can create "
                    "robot ghost points in the MoveIt octomap during motion."
                ),
            ),
            DeclareLaunchArgument(
                "voxel_leaf_size",
                default_value="0.03",
                description="Voxel leaf size in meters for the workspace point cloud filter.",
            ),
            DeclareLaunchArgument(
                "use_crop_box",
                default_value="true",
                description="Enable workspace crop-box filtering.",
            ),
            DeclareLaunchArgument(
                "use_voxel_filter",
                default_value="true",
                description="Enable voxel-grid downsampling for the filtered cloud.",
            ),
            DeclareLaunchArgument(
                "reliable_output_qos",
                default_value="true",
                description="Publish the filtered cloud with RELIABLE QoS instead of BEST_EFFORT.",
            ),
            DeclareLaunchArgument("crop_min_x", default_value="-1.0", description="Workspace crop minimum X in meters."),
            DeclareLaunchArgument("crop_max_x", default_value="2.0", description="Workspace crop maximum X in meters."),
            DeclareLaunchArgument("crop_min_y", default_value="-1.5", description="Workspace crop minimum Y in meters."),
            DeclareLaunchArgument("crop_max_y", default_value="1.5", description="Workspace crop maximum Y in meters."),
            DeclareLaunchArgument("crop_min_z", default_value="-0.1", description="Workspace crop minimum Z in meters."),
            DeclareLaunchArgument("crop_max_z", default_value="1.8", description="Workspace crop maximum Z in meters."),
            DeclareLaunchArgument(
                "target_frame",
                default_value="world",
                description="Target frame used by the workspace point cloud filter.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
