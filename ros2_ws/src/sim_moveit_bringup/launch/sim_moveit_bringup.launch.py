from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


RG2_PRESET_MULTIPLIERS = {
    "preset_1_current_like": {
        "onrobot_rg2_body_to_finger_1_origin_joint": 1.0,
        "onrobot_rg2_body_to_finger_2_origin_joint": 1.0,
        "onrobot_rg2_finger_1_origin_to_moment_arm_joint": -1.0,
        "onrobot_rg2_finger_1_origin_to_truss_arm_joint": -1.0,
        "onrobot_rg2_finger_1_truss_arm_to_finger_tip_joint": -1.0,
        "onrobot_rg2_finger_2_origin_to_moment_arm_joint": -1.0,
        "onrobot_rg2_finger_2_origin_to_truss_arm_joint": -1.0,
        "onrobot_rg2_finger_2_truss_arm_to_finger_tip_joint": -1.0,
    },
    "preset_2_mirrored_signs": {
        "onrobot_rg2_body_to_finger_1_origin_joint": 1.0,
        "onrobot_rg2_body_to_finger_2_origin_joint": -1.0,
        "onrobot_rg2_finger_1_origin_to_moment_arm_joint": -1.0,
        "onrobot_rg2_finger_1_origin_to_truss_arm_joint": -1.0,
        "onrobot_rg2_finger_1_truss_arm_to_finger_tip_joint": -1.0,
        "onrobot_rg2_finger_2_origin_to_moment_arm_joint": 1.0,
        "onrobot_rg2_finger_2_origin_to_truss_arm_joint": 1.0,
        "onrobot_rg2_finger_2_truss_arm_to_finger_tip_joint": 1.0,
    },
    "preset_3_all_same_sign": {
        "onrobot_rg2_body_to_finger_1_origin_joint": 1.0,
        "onrobot_rg2_body_to_finger_2_origin_joint": 1.0,
        "onrobot_rg2_finger_1_origin_to_moment_arm_joint": 1.0,
        "onrobot_rg2_finger_1_origin_to_truss_arm_joint": 1.0,
        "onrobot_rg2_finger_1_truss_arm_to_finger_tip_joint": 1.0,
        "onrobot_rg2_finger_2_origin_to_moment_arm_joint": 1.0,
        "onrobot_rg2_finger_2_origin_to_truss_arm_joint": 1.0,
        "onrobot_rg2_finger_2_truss_arm_to_finger_tip_joint": 1.0,
    },
}
DEFAULT_RG2_PRESET = "preset_2_mirrored_signs"


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
    gripper_enabled = LaunchConfiguration("gripper_enabled")
    gripper_command_topic = LaunchConfiguration("gripper_command_topic")
    gripper_open_width = LaunchConfiguration("gripper_open_width")
    gripper_closed_width = LaunchConfiguration("gripper_closed_width")
    invert_gripper_command = LaunchConfiguration("invert_gripper_command")
    gripper_articulation_path = LaunchConfiguration("gripper_articulation_path")
    gripper_joint_command_topic = LaunchConfiguration("gripper_joint_command_topic")
    gripper_motion_duration_sec = LaunchConfiguration("gripper_motion_duration_sec")
    gripper_command_rate_hz = LaunchConfiguration("gripper_command_rate_hz")
    gripper_smooth_motion = LaunchConfiguration("gripper_smooth_motion")
    gripper_multiplier_preset = LaunchConfiguration("gripper_multiplier_preset")

    ur5e_launch_dir = PathJoinSubstitution([FindPackageShare("ur5e_moveit_config"), "launch"])
    bridge_launch_dir = PathJoinSubstitution([FindPackageShare("isaac_trajectory_bridge"), "launch"])
    sim_bringup_launch_dir = PathJoinSubstitution([FindPackageShare("sim_moveit_bringup"), "launch"])
    gripper_config = PathJoinSubstitution([FindPackageShare("sim_moveit_bringup"), "config", "rg2_gripper_bridge.yaml"])

    filter_enabled = _is_truthy(filter_pointcloud.perform(context))
    raw_topic_value = raw_pointcloud_topic.perform(context)
    filtered_topic_value = filtered_pointcloud_topic.perform(context)
    direct_topic_value = pointcloud_topic.perform(context)
    moveit_pointcloud_topic = filtered_topic_value if filter_enabled else direct_topic_value
    gripper_preset_name = gripper_multiplier_preset.perform(context)
    preset_multipliers = RG2_PRESET_MULTIPLIERS.get(
        gripper_preset_name,
        RG2_PRESET_MULTIPLIERS[DEFAULT_RG2_PRESET],
    )
    gripper_preset_parameters = {
        "joint_multiplier_preset": gripper_preset_name,
    }
    for joint_name, multiplier in preset_multipliers.items():
        gripper_preset_parameters[
            f"rg2_joint_targets.{joint_name}.multiplier"
        ] = multiplier

    actions = [
        LogInfo(msg="Isaac Sim must already be running and simulation must be playing."),
        LogInfo(msg=["Raw point cloud topic: ", raw_pointcloud_topic]),
        LogInfo(msg=["MoveIt point cloud topic: ", moveit_pointcloud_topic]),
        LogInfo(msg=["Camera TF config: ", camera_config]),
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
                "octomap_frame": target_frame,
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
                "publish_full_joint_command": "true",
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
        Node(
            package="sim_moveit_bringup",
            executable="rg2_gripper_bridge",
            name="rg2_gripper_bridge",
            output="screen",
            condition=IfCondition(gripper_enabled),
            parameters=[
                gripper_config,
                {
                    "use_sim_time": use_sim_time,
                    "articulation_path": gripper_articulation_path,
                    "joint_command_topic": gripper_joint_command_topic,
                    "gripper_command_topic": gripper_command_topic,
                    "open_width_m": float(gripper_open_width.perform(context)),
                    "closed_width_m": float(gripper_closed_width.perform(context)),
                    "invert_command": _is_truthy(invert_gripper_command.perform(context)),
                    "smooth_motion": _is_truthy(gripper_smooth_motion.perform(context)),
                    "motion_duration_sec": float(gripper_motion_duration_sec.perform(context)),
                    "command_rate_hz": float(gripper_command_rate_hz.perform(context)),
                    **gripper_preset_parameters,
                }
            ],
        ),
        LogInfo(msg=["RG2 gripper command topic: ", gripper_command_topic]),
        LogInfo(
            msg=(
                "The bringup launch passes publish_full_joint_command:=true to the "
                "trajectory bridge because that argument is supported today."
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
                        "target_frame": target_frame.perform(context),
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
            DeclareLaunchArgument(
                "gripper_enabled",
                default_value="true",
                description="Launch the simple RG2 width command bridge for Isaac.",
            ),
            DeclareLaunchArgument(
                "gripper_command_topic",
                default_value="/rg2/command_width",
                description="Width command topic for the RG2 bridge.",
            ),
            DeclareLaunchArgument(
                "gripper_open_width",
                default_value="0.085",
                description="Requested width in meters that corresponds to the fully open RG2 pose.",
            ),
            DeclareLaunchArgument(
                "gripper_closed_width",
                default_value="0.0",
                description="Requested width in meters that corresponds to the fully closed RG2 pose.",
            ),
            DeclareLaunchArgument(
                "invert_gripper_command",
                default_value="false",
                description="Invert the width-to-joint mapping if visual testing shows open/close reversed.",
            ),
            DeclareLaunchArgument(
                "gripper_articulation_path",
                default_value="/World/onrobot_rg2",
                description="Isaac articulation path logged by the RG2 bridge for the target gripper articulation.",
            ),
            DeclareLaunchArgument(
                "gripper_joint_command_topic",
                default_value="/joint_command",
                description="JointState command topic used to send RG2 linkage targets into Isaac.",
            ),
            DeclareLaunchArgument(
                "gripper_smooth_motion",
                default_value="true",
                description="Interpolate RG2 joint targets over time instead of stepping instantly.",
            ),
            DeclareLaunchArgument(
                "gripper_motion_duration_sec",
                default_value="0.5",
                description="Duration in seconds for each RG2 open/close interpolation.",
            ),
            DeclareLaunchArgument(
                "gripper_command_rate_hz",
                default_value="60.0",
                description="Publish rate for RG2 JointState commands while moving.",
            ),
            DeclareLaunchArgument(
                "gripper_multiplier_preset",
                default_value="preset_2_mirrored_signs",
                description="Default RG2 joint multiplier preset for the direct-control linkage map.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
