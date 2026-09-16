from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_full_joint_command = LaunchConfiguration("publish_full_joint_command")
    start_rg2_controller = LaunchConfiguration("start_rg2_controller")
    rg2_open_position = LaunchConfiguration("rg2_open_position")
    rg2_close_position = LaunchConfiguration("rg2_close_position")
    rg2_command_duration_sec = LaunchConfiguration("rg2_command_duration_sec")
    rg2_command_rate_hz = LaunchConfiguration("rg2_command_rate_hz")
    goal_tolerance = LaunchConfiguration("goal_tolerance")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if Isaac publishes /clock.",
            ),
            DeclareLaunchArgument(
                "publish_full_joint_command",
                default_value="false",
                description=(
                    "false = publish only the configured arm joints to "
                    "/joint_command; true = publish all joints from /joint_states "
                    "while overwriting the arm joints, which should only be used if "
                    "Isaac requires full-articulation commands and gripper "
                    "preservation is known safe."
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
                "goal_tolerance",
                default_value="0.0001",
                description=(
                    "Per-joint acceptance band in radians for the final "
                    "trajectory point. This is the gate that decides the "
                    "precision actually delivered, so it is held at the cell's "
                    "0.1 mm / 0.1 deg spec (~1e-4 rad is ~0.085 mm at the TCP). "
                    "Raise it only to diagnose a drive that cannot settle."
                ),
            ),
            Node(
                package="isaac_trajectory_bridge",
                executable="follow_joint_trajectory_server",
                name="isaac_follow_joint_trajectory_bridge",
                output="screen",
                parameters=[
                    {
                        "joint_names": [
                            "shoulder_pan_joint",
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint",
                        ],
                        "joint_command_topic": "/joint_command",
                        "joint_state_topic": "/joint_states",
                        "action_name": "/arm_controller/follow_joint_trajectory",
                        # Matches physxScene:timeStepsPerSecond in the Isaac
                        # stage; a mismatch beats against the physics loop.
                        "publish_rate_hz": 60.0,
                        "goal_tolerance": goal_tolerance,
                        "goal_time_tolerance_sec": 2.0,
                        "require_joint_state_before_execute": True,
                        "publish_full_joint_command": publish_full_joint_command,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="isaac_trajectory_bridge",
                executable="rg2_controller",
                name="rg2_controller",
                output="screen",
                condition=IfCondition(start_rg2_controller),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "joint_state_topic": "/joint_states",
                        "joint_command_topic": "/joint_command",
                        "service_namespace": "/rg2_controller",
                        "open_position": rg2_open_position,
                        "close_position": rg2_close_position,
                        "command_duration_sec": rg2_command_duration_sec,
                        "command_rate_hz": rg2_command_rate_hz,
                    }
                ],
            ),
        ]
    )
