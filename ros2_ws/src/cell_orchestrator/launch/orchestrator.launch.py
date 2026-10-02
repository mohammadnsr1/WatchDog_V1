from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gate_timeout_sec",
                default_value="120.0",
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
            DeclareLaunchArgument(
                "guard_mode",
                default_value="guarded",
                description=(
                    "\"guarded\" (default) or \"unguarded\". Declared, not "
                    "inferred: whether a run is monitored is experimental "
                    "metadata. unguarded skips every gate immediately "
                    "(reason=unguarded_by_config) -- use it for "
                    "executor-only runs. guarded gates normally, and "
                    "self-latches unguarded (reason=unguarded_latched, "
                    "logged at warning level) if no monitor ever signals on "
                    "the run's first gate, so a guarded run with no monitor "
                    "started fails loudly instead of burning the full gate "
                    "timeout on every step. Change at runtime with "
                    "`ros2 param set /cell_orchestrator guard_mode "
                    "<guarded|unguarded>` -- takes effect on the next gate, "
                    "no restart needed."
                ),
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
                        "guard_mode": LaunchConfiguration("guard_mode"),
                    }
                ],
            ),
        ]
    )
