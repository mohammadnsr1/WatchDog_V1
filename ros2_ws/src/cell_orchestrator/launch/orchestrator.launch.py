from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gate_timeout_sec",
                default_value="90.0",
                description=(
                    "Fallback gate timeout (seconds) used only when a "
                    "RequestGo caller sends timeout_sec <= 0. Callers "
                    "normally send their own timeout_sec; keep this in sync "
                    "with the value gripper/motion/scene callers actually "
                    "use, and keep the MCP client timeout in "
                    "agents/config/*.yaml comfortably above whichever value "
                    "is in effect -- see robot_backend/cell/gate.py."
                ),
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="~/.watchdog/orchestrator",
                description="Directory for the orchestrator's own JSONL transition log.",
            ),
            Node(
                package="cell_orchestrator",
                executable="orchestrator_node",
                name="cell_orchestrator",
                output="screen",
                parameters=[
                    {
                        "gate_timeout_sec": LaunchConfiguration("gate_timeout_sec"),
                        "log_dir": LaunchConfiguration("log_dir"),
                    }
                ],
            ),
        ]
    )
