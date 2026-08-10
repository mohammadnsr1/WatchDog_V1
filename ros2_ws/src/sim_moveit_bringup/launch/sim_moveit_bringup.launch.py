import os
import time
from datetime import datetime, timezone
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Core topics recorded on every session. Small (a few MB for a whole run) and
# the shared correlation axis for the whole record.
_CORE_BAG_TOPICS = ["/joint_states", "/tf", "/tf_static", "/watchdog/events"]
# RGB image topics, recorded only when record_images is true. No throttle layer
# is added -- the source already publishes slowly (~4.6 Hz); recording the
# topic directly is correct here. There is no depth *image* topic on this cell.
_IMAGE_BAG_TOPICS = ["/camera/depth/rgb", "/camera_info"]
# Point cloud, recorded only when record_pointcloud is true. ~4x the cost of
# RGB for data a frame-inspecting analyst does not use, so it is off by default.
_POINTCLOUD_BAG_TOPICS = ["/camera/depth/points"]


def _is_truthy(value: str) -> bool:
    return value.lower() in {"true", "1", "yes", "on"}


def _shared_session_root() -> Path:
    """The shared session root (agents/transcripts/sessions), via the one helper.

    Falls back to ~/.watchdog/sessions only if robot_backend cannot be imported.
    Using the same helper the harness uses is what keeps both sides' fresh
    default under one root.
    """
    try:
        from robot_backend.watchdog import session as sess
        return sess.session_root()
    except Exception:
        return Path.home() / ".watchdog" / "sessions"


def _resolve_session_dir(context):
    """Resolve THIS bringup's session dir. Returns (Path, rule).

    The launch is the pointer's writer, not a reader: a new bringup is a new
    run, so it never reuses ~/.watchdog/current_session (that would merge two
    runs). Order: explicit session_dir arg, else WATCHDOG_SESSION_DIR already in
    the environment, else a fresh session under the shared root. run_agent.py
    reads the pointer this bringup writes.
    """
    explicit = LaunchConfiguration("session_dir").perform(context).strip()
    if explicit:
        return Path(os.path.expanduser(explicit)), "arg"
    from_env = os.environ.get("WATCHDOG_SESSION_DIR", "").strip()
    if from_env:
        return Path(os.path.expanduser(from_env)), "env"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return _shared_session_root() / stamp, "fresh"


def _write_manifest(session_dir: Path, context, recorded_topics, images_recorded, record_pointcloud):
    """Write manifest.json for this session (best effort; never fails bringup).

    Imports robot_backend.watchdog.session so the atomic/idempotent write and
    the registry embedding live in one place. If robot_backend is not on the
    path, logs and skips -- the recording still happens; only the manifest is
    absent.
    """
    try:
        from robot_backend.registry import load_registry
        from robot_backend.watchdog import session as sess
    except Exception as exc:  # pragma: no cover - depends on overlay
        return LogInfo(msg=f"Session manifest skipped (robot_backend import failed: {exc}).")

    try:
        from ament_index_python.packages import get_package_share_directory
        registry_path = (
            Path(get_package_share_directory("robot_backend"))
            / "config"
            / "model_registry.yaml"
        )
        registry = load_registry(registry_path)
    except Exception as exc:  # pragma: no cover
        registry = {}
        registry_path = None
        registry_sha = None
        _ = exc
    else:
        registry_sha = registry.raw_sha256

    # SOURCE registry (the file an operator edits to inject a fault), located
    # via the shared helper. When its hash differs from the install-space hash
    # above, the cell is running a DIFFERENT registry than the one on disk in
    # src/ -- the "edited the source, forgot to rebuild" trap. Recorded in the
    # manifest and warned about below; never fatal (a divergent install space
    # can be legitimate).
    src_registry_path = sess.source_registry_path()
    src_registry_sha = sess.sha256_file(src_registry_path) if src_registry_path else None

    image_hz = float(LaunchConfiguration("image_buffer_hz").perform(context) or 5.0)
    try:
        sess.write_manifest(
            session_dir,
            registry=registry,
            registry_path=registry_path,
            registry_sha256=registry_sha,
            started_wall_ns=time.time_ns(),
            recorded_topics=recorded_topics,
            images_recorded=images_recorded,
            image_throttle_hz=image_hz,
            record_pointcloud=record_pointcloud,
            git_commit_value=sess.git_commit(),
            registry_source_path=src_registry_path,
            registry_source_sha256=src_registry_sha,
        )
    except Exception as exc:  # pragma: no cover
        return LogInfo(msg=f"Session manifest write failed: {exc}")

    if (
        src_registry_sha is not None
        and registry_sha is not None
        and src_registry_sha != registry_sha
    ):
        return LogInfo(msg=(
            "WARNING: the installed model_registry.yaml differs from the source "
            f"copy. Cell loaded install={registry_path} (sha256 {registry_sha[:12]}), "
            f"but source={src_registry_path} (sha256 {src_registry_sha[:12]}). "
            "The workspace needs rebuilding: the cell is running the OLD registry. "
            "If a fault was injected into the source, this run executed the CLEAN "
            "registry. `colcon build` and re-launch. (manifest: "
            "registry_install_matches_source=false)"
        ))
    return LogInfo(msg=f"Session manifest written to {session_dir / 'manifest.json'}")


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
    launch_scene_sync = LaunchConfiguration("launch_scene_sync")
    start_orchestrator = LaunchConfiguration("start_orchestrator")
    guard_mode = LaunchConfiguration("guard_mode")
    gate_timeout_sec = LaunchConfiguration("gate_timeout_sec")
    start_rg2_controller = LaunchConfiguration("start_rg2_controller")
    rg2_open_position = LaunchConfiguration("rg2_open_position")
    rg2_close_position = LaunchConfiguration("rg2_close_position")
    rg2_command_duration_sec = LaunchConfiguration("rg2_command_duration_sec")
    rg2_command_rate_hz = LaunchConfiguration("rg2_command_rate_hz")

    ur5e_launch_dir = PathJoinSubstitution([FindPackageShare("ur5e_moveit_config"), "launch"])
    bridge_launch_dir = PathJoinSubstitution([FindPackageShare("isaac_trajectory_bridge"), "launch"])
    sim_bringup_launch_dir = PathJoinSubstitution([FindPackageShare("sim_moveit_bringup"), "launch"])
    orchestrator_launch_dir = PathJoinSubstitution([FindPackageShare("cell_orchestrator"), "launch"])
    filter_enabled = _is_truthy(filter_pointcloud.perform(context))
    raw_topic_value = raw_pointcloud_topic.perform(context)
    filtered_topic_value = filtered_pointcloud_topic.perform(context)
    direct_topic_value = pointcloud_topic.perform(context)
    target_frame_value = target_frame.perform(context).strip()
    moveit_pointcloud_topic = filtered_topic_value if filter_enabled else direct_topic_value

    # -- Session recording (Part B/C) ------------------------------------
    session_dir, session_rule = _resolve_session_dir(context)
    session_dir.mkdir(parents=True, exist_ok=True)
    # Publish the session path so the separately-launched agents find it with no
    # manual export. Atomic write; best effort (never fails bringup).
    pointer_note = "current_session pointer not written (robot_backend unavailable)"
    try:
        from robot_backend.watchdog import session as _sess
        pointer_path = _sess.write_pointer(session_dir)
        pointer_note = f"wrote {pointer_path} -> {session_dir}"
    except Exception as exc:  # pragma: no cover - depends on overlay
        pointer_note = f"current_session pointer skipped ({exc})"
    record_session = _is_truthy(LaunchConfiguration("record_session").perform(context))
    record_images = _is_truthy(LaunchConfiguration("record_images").perform(context))
    record_pointcloud = _is_truthy(LaunchConfiguration("record_pointcloud").perform(context))

    bag_topics = list(_CORE_BAG_TOPICS)
    if record_images:
        bag_topics += _IMAGE_BAG_TOPICS
    if record_pointcloud:
        bag_topics += _POINTCLOUD_BAG_TOPICS

    # The orchestrator log lives under the session dir unless the operator
    # overrode orchestrator_log_dir explicitly (non-empty).
    orch_log_override = LaunchConfiguration("orchestrator_log_dir").perform(context).strip()
    orchestrator_log_dir_value = orch_log_override or str(session_dir / "orchestrator")

    actions = [
        # Every child process launched here inherits WATCHDOG_SESSION_DIR, so
        # the scene_tf_sync node, the orchestrator, and (when started from the
        # same shell) the agents' MCP servers all root their artifacts in one
        # session. Export the printed value into the shell that launches the
        # agents so their MCP servers join the same session.
        SetEnvironmentVariable("WATCHDOG_SESSION_DIR", str(session_dir)),
        LogInfo(msg=f"Session dir: {session_dir} (resolved by rule={session_rule!r})"),
        LogInfo(msg=f"Session pointer: {pointer_note}"),
        LogInfo(
            msg=(
                "Agents launched from any shell will find this session via "
                "~/.watchdog/current_session -- no export needed. To force a "
                "different session, export WATCHDOG_SESSION_DIR before run_agent.py."
            )
        ),
        _write_manifest(session_dir, context, bag_topics, record_images, record_pointcloud),
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
                "octomap_resolution": "0.01",
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
                "start_rg2_controller": start_rg2_controller,
                "rg2_open_position": rg2_open_position,
                "rg2_close_position": rg2_close_position,
                "rg2_command_duration_sec": rg2_command_duration_sec,
                "rg2_command_rate_hz": rg2_command_rate_hz,
            }.items(),
        ),
        # MoveIt planning frame is lowercase `world`, while Isaac Sim's default
        # frame is uppercase `World`. This static identity transform connects
        # the TF trees and assumes both frames represent the same physical
        # coordinate frame.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_isaac_world_static_tf",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "1",
                "world",
                "World",
            ],
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
        # Service-driven scene sync node: exposes /sync_scene_objects so skills
        # can pull fresh TF-based collision objects into MoveIt on demand.
        Node(
            package="robot_backend",
            executable="scene_tf_sync_node",
            name="scene_tf_sync_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(launch_scene_sync),
        ),
        # The cell's timing interlock. Cell infrastructure, not agent
        # infrastructure -- it comes up with the cell so the monitoring agent
        # and executor can be started independently afterward without either
        # one owning the gate. See cell_orchestrator/node.py for why it is a
        # third process and cell_orchestrator/launch/orchestrator.launch.py
        # for its own argument documentation (guard_mode in particular).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([orchestrator_launch_dir, "orchestrator.launch.py"])
            ),
            launch_arguments={
                "guard_mode": guard_mode,
                "gate_timeout_sec": gate_timeout_sec,
                "log_dir": orchestrator_log_dir_value,
            }.items(),
            condition=IfCondition(start_orchestrator),
        ),
        # Session recording. rosbag2 stamps recv_timestamp from the SYSTEM
        # clock even under use_sim_time -- so this recorder is deliberately NOT
        # told use_sim_time, keeping it on system time (recv_timestamp) while
        # the rest of the graph runs on sim time. Correlation uses
        # recv_timestamp only; message header.stamp is sim time and resets to
        # zero each run (see manifest clock block). Core topics are always
        # recorded; images/pointcloud are opt-in.
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", str(session_dir / "bag"), *bag_topics],
            output="screen",
            condition=IfCondition(LaunchConfiguration("record_session")),
        ),
        LogInfo(
            msg=(
                f"Session bag: recording {bag_topics} to {session_dir / 'bag'} "
                f"(record_session={record_session}, record_images={record_images}, "
                f"record_pointcloud={record_pointcloud})."
            )
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
                default_value="10.0",
                description=(
                    "Maximum publish rate for the filtered point cloud in Hz. Also "
                    "used directly as MoveIt's octomap max_update_rate (see "
                    "sensors_3d.yaml's isaac_point_cloud sensor plugin) -- one "
                    "argument controls both, since a faster republish with a "
                    "slower octomap consumer would just be wasted work. Was 2.0; "
                    "raised for faster scene catch-up after an object moves/is "
                    "attached/is released."
                ),
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
                "launch_scene_sync",
                default_value="true",
                description=(
                    "Launch scene_tf_sync_node, which exposes the /sync_scene_objects service "
                    "so pick/place skills can pull TF-based collision objects into MoveIt on demand."
                ),
            ),
            DeclareLaunchArgument(
                "start_orchestrator",
                default_value="true",
                description=(
                    "Launch cell_orchestrator (the /cell/request_go and "
                    "/cell/signal_ready timing interlock) as part of cell "
                    "bringup. Cell infrastructure, not agent infrastructure -- "
                    "the monitoring agent and executor are started separately "
                    "afterward. See operator sequence in agents/README.md."
                ),
            ),
            DeclareLaunchArgument(
                "guard_mode",
                default_value="guarded",
                description=(
                    "Passed through to cell_orchestrator: \"guarded\" (default) "
                    "or \"unguarded\". Set to unguarded for executor-only runs "
                    "-- leaving it guarded with no monitor started produces an "
                    "unguarded_latched run that completes but monitors "
                    "nothing. Changeable at runtime without a restart: "
                    "`ros2 param set /cell_orchestrator guard_mode "
                    "<guarded|unguarded>`."
                ),
            ),
            DeclareLaunchArgument(
                "gate_timeout_sec",
                default_value="90.0",
                description=(
                    "Passed through to cell_orchestrator as its fallback gate "
                    "timeout. Keep in sync with CELL_GATE_TIMEOUT_SEC in "
                    "agents/config/executor_agent.yaml and comfortably below "
                    "mcp_client_timeout_sec there -- see robot_backend/cell/gate.py."
                ),
            ),
            DeclareLaunchArgument(
                "orchestrator_log_dir",
                default_value="",
                description=(
                    "Orchestrator JSONL transition log directory. Empty (default) "
                    "routes it under the session directory (<session>/orchestrator); "
                    "set it to override with an explicit path."
                ),
            ),
            DeclareLaunchArgument(
                "session_dir",
                default_value="",
                description=(
                    "Session directory for this run's recorded artifacts (bag, "
                    "manifest, per-process event logs, orchestrator log). Empty "
                    "(default) uses WATCHDOG_SESSION_DIR from the environment if "
                    "set, else a fresh ~/.watchdog/sessions/<UTC> path. The chosen "
                    "value is exported as WATCHDOG_SESSION_DIR for child processes; "
                    "export the same value in the shell that launches the agents."
                ),
            ),
            DeclareLaunchArgument(
                "record_session",
                default_value="true",
                description=(
                    "Record the core session bag (/joint_states, /tf, /tf_static, "
                    "/watchdog/events) to <session>/bag. A few MB for a whole run."
                ),
            ),
            DeclareLaunchArgument(
                "record_images",
                default_value="false",
                description=(
                    "Also record RGB camera topics (/camera/depth/rgb, /camera_info) "
                    "to the session bag. ~180 MB/min. Off by default."
                ),
            ),
            DeclareLaunchArgument(
                "record_pointcloud",
                default_value="false",
                description=(
                    "Also record /camera/depth/points to the session bag. ~4x the "
                    "cost of RGB for data a frame-inspecting analyst does not use. "
                    "Off by default."
                ),
            ),
            DeclareLaunchArgument(
                "image_buffer_hz",
                default_value="5.0",
                description=(
                    "WatchDog image ring-buffer throttle rate, recorded in the "
                    "manifest. Informational at the launch level; the WatchDog MCP "
                    "server reads WATCHDOG_IMAGE_BUFFER_HZ at runtime."
                ),
            ),
            DeclareLaunchArgument(
                "start_rg2_controller",
                default_value="true",
                description="Start the gripper-only RG2 controller node.",
            ),
            DeclareLaunchArgument(
                "rg2_open_position",
                default_value="-0.558505",
                description="Default RG2 open target in master joint radians for finger_joint.",
            ),
            DeclareLaunchArgument(
                "rg2_close_position",
                default_value="0.785398",
                description="Default RG2 close target in master joint radians for finger_joint.",
            ),
            DeclareLaunchArgument(
                "rg2_command_duration_sec",
                default_value="1.0",
                description="Duration in seconds for RG2 gripper interpolation commands.",
            ),
            DeclareLaunchArgument(
                "rg2_command_rate_hz",
                default_value="50.0",
                description="Publish rate in Hz for RG2 gripper interpolation commands.",
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
            DeclareLaunchArgument("crop_min_y", default_value="-1.0", description="Workspace crop minimum Y in meters."),
            DeclareLaunchArgument("crop_max_y", default_value="1.0", description="Workspace crop maximum Y in meters."),
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
