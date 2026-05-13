from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if Isaac publishes /clock.",
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
                        "publish_rate_hz": 50.0,
                        "goal_tolerance": 0.03,
                        "goal_time_tolerance_sec": 2.0,
                        "require_joint_state_before_execute": True,
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        ]
    )
