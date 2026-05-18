#!/usr/bin/env python3
import math
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


RG2_JOINT_ORDER = [
    "onrobot_rg2_body_to_finger_1_origin_joint",
    "onrobot_rg2_body_to_finger_2_origin_joint",
    "onrobot_rg2_finger_1_origin_to_moment_arm_joint",
    "onrobot_rg2_finger_1_origin_to_truss_arm_joint",
    "onrobot_rg2_finger_1_truss_arm_to_finger_tip_joint",
    "onrobot_rg2_finger_2_origin_to_moment_arm_joint",
    "onrobot_rg2_finger_2_origin_to_truss_arm_joint",
    "onrobot_rg2_finger_2_truss_arm_to_finger_tip_joint",
]

PRESET_MULTIPLIERS = {
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
        joint_name: 1.0 for joint_name in RG2_JOINT_ORDER
    },
}
DEFAULT_PRESET = "preset_2_mirrored_signs"


@dataclass
class JointTarget:
    multiplier: float
    offset: float


class RG2GripperBridge(Node):
    def __init__(self) -> None:
        super().__init__("rg2_gripper_bridge")

        self.declare_parameter("use_sim_time", False)
        self.declare_parameter("articulation_path", "/World/onrobot_rg2")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_command_topic", "/joint_command")
        self.declare_parameter("gripper_command_topic", "/rg2/command_width")
        self.declare_parameter("open_width_m", 0.085)
        self.declare_parameter("closed_width_m", 0.0)
        self.declare_parameter("max_q", 1.18)
        self.declare_parameter("invert_command", False)
        self.declare_parameter("smooth_motion", True)
        self.declare_parameter("motion_duration_sec", 0.5)
        self.declare_parameter("command_rate_hz", 60.0)
        self.declare_parameter("command_tolerance", 1e-3)
        self.declare_parameter("joint_multiplier_preset", DEFAULT_PRESET)
        self.declare_parameter("rg2_joint_names", RG2_JOINT_ORDER)

        for joint_name, target in PRESET_MULTIPLIERS[DEFAULT_PRESET].items():
            self.declare_parameter(
                f"rg2_joint_targets.{joint_name}.multiplier",
                target,
                ParameterDescriptor(description=f"Multiplier for {joint_name}."),
            )
            self.declare_parameter(
                f"rg2_joint_targets.{joint_name}.offset",
                0.0,
                ParameterDescriptor(description=f"Offset for {joint_name}."),
            )

        self.articulation_path = str(self.get_parameter("articulation_path").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self.gripper_command_topic = str(self.get_parameter("gripper_command_topic").value)
        self.open_width_m = float(self.get_parameter("open_width_m").value)
        self.closed_width_m = float(self.get_parameter("closed_width_m").value)
        self.max_q = float(self.get_parameter("max_q").value)
        self.invert_command = bool(self.get_parameter("invert_command").value)
        self.smooth_motion = bool(self.get_parameter("smooth_motion").value)
        self.motion_duration_sec = max(0.0, float(self.get_parameter("motion_duration_sec").value))
        self.command_rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.command_tolerance = max(1e-6, float(self.get_parameter("command_tolerance").value))
        self.joint_multiplier_preset = str(self.get_parameter("joint_multiplier_preset").value)
        self.rg2_joint_names = [str(name) for name in self.get_parameter("rg2_joint_names").value]

        if self.open_width_m <= self.closed_width_m:
            raise ValueError("open_width_m must be greater than closed_width_m.")
        if self.max_q <= 0.0:
            raise ValueError("max_q must be greater than zero.")

        self._preset_targets = self._preset_target_map(self.joint_multiplier_preset)
        self._joint_targets = self._load_joint_targets()
        self._lock = threading.Lock()
        self._latest_joint_state: Optional[Tuple[List[str], List[float], List[float]]] = None
        self._motion_start_positions: Dict[str, float] = {}
        self._motion_target_positions: Dict[str, float] = {}
        self._active_motion_positions: Dict[str, float] = {}
        self._motion_start_time_sec = 0.0
        self._motion_end_time_sec = 0.0
        self._latest_requested_width_m = self.open_width_m
        self._discovery_logged = False
        self._last_logged_times: Dict[str, float] = {}

        self._publisher = self.create_publisher(JointState, self.joint_command_topic, 10)
        self._joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            50,
        )
        self._width_sub = self.create_subscription(
            Float64,
            self.gripper_command_topic,
            self._width_callback,
            10,
        )
        self._timer = self.create_timer(1.0 / self.command_rate_hz, self._publish_step)

        self._log_startup_configuration()

    def _preset_target_map(self, preset_name: str) -> Dict[str, JointTarget]:
        preset = PRESET_MULTIPLIERS.get(preset_name)
        if preset is None:
            self.get_logger().warn(
                f"Unknown joint_multiplier_preset '{preset_name}', falling back to {DEFAULT_PRESET}."
            )
            preset = PRESET_MULTIPLIERS[DEFAULT_PRESET]
            self.joint_multiplier_preset = DEFAULT_PRESET
        return {joint_name: JointTarget(multiplier=value, offset=0.0) for joint_name, value in preset.items()}

    def _load_joint_targets(self) -> Dict[str, JointTarget]:
        targets: Dict[str, JointTarget] = {}
        for joint_name in self.rg2_joint_names:
            preset_target = self._preset_targets.get(joint_name, JointTarget(1.0, 0.0))
            multiplier = float(self.get_parameter(f"rg2_joint_targets.{joint_name}.multiplier").value)
            offset = float(self.get_parameter(f"rg2_joint_targets.{joint_name}.offset").value)
            if math.isclose(multiplier, PRESET_MULTIPLIERS[DEFAULT_PRESET].get(joint_name, 1.0), abs_tol=1e-9):
                multiplier = preset_target.multiplier
            targets[joint_name] = JointTarget(multiplier=multiplier, offset=offset if not math.isclose(offset, 0.0, abs_tol=1e-12) else preset_target.offset)
        return targets

    def _log_startup_configuration(self) -> None:
        self.get_logger().info(f"RG2 articulation path: {self.articulation_path}")
        self.get_logger().info(f"RG2 command topic: {self.gripper_command_topic}")
        self.get_logger().info(f"RG2 joint command topic: {self.joint_command_topic}")
        self.get_logger().info(f"RG2 joint state topic: {self.joint_state_topic}")
        self.get_logger().info(
            f"RG2 width mapping: open_width={self.open_width_m:.3f} m closed_width={self.closed_width_m:.3f} m max_q={self.max_q:.3f} invert_command={self.invert_command}"
        )
        self.get_logger().info(
            f"RG2 motion: smooth_motion={self.smooth_motion} motion_duration_sec={self.motion_duration_sec:.3f} command_rate_hz={self.command_rate_hz:.1f}"
        )
        self.get_logger().info(f"RG2 joint multiplier preset: {self.joint_multiplier_preset}")
        self.get_logger().info(
            "RG2 joint multiplier map: "
            + str({joint_name: {"multiplier": target.multiplier, "offset": target.offset} for joint_name, target in self._joint_targets.items()})
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        velocities = list(msg.velocity) if len(msg.velocity) == len(msg.name) else []
        with self._lock:
            self._latest_joint_state = (list(msg.name), list(msg.position), velocities)

        if not self._discovery_logged:
            self._discovery_logged = True
            available = list(msg.name)
            found = [name for name in self.rg2_joint_names if name in available]
            missing = [name for name in self.rg2_joint_names if name not in available]
            self.get_logger().info(f"All DOF names: {available}")
            self.get_logger().info(f"RG2 joints found: {found}")
            self.get_logger().info(f"RG2 joints missing: {missing}")

    def _width_callback(self, msg: Float64) -> None:
        width_m = min(max(float(msg.data), self.closed_width_m), self.open_width_m)
        q = self._width_to_q(width_m)

        with self._lock:
            latest = self._latest_joint_state
            current_positions = self._current_rg2_positions_locked(latest)
            targets = {
                joint_name: self._joint_targets[joint_name].multiplier * q + self._joint_targets[joint_name].offset
                for joint_name in self.rg2_joint_names
            }
            if self.smooth_motion and self.motion_duration_sec > 0.0:
                start_positions = {joint_name: current_positions.get(joint_name, targets[joint_name]) for joint_name in self.rg2_joint_names}
                self._motion_start_positions = start_positions
                self._motion_target_positions = targets
                self._active_motion_positions = dict(start_positions)
                self._motion_start_time_sec = self._now_sec()
                self._motion_end_time_sec = self._motion_start_time_sec + self.motion_duration_sec
            else:
                self._motion_start_positions = dict(targets)
                self._motion_target_positions = dict(targets)
                self._active_motion_positions = dict(targets)
                now_sec = self._now_sec()
                self._motion_start_time_sec = now_sec
                self._motion_end_time_sec = now_sec
            self._latest_requested_width_m = width_m

        self.get_logger().info(
            f"Received /rg2/command_width={float(msg.data):.4f} m, clamped={width_m:.4f} m, q={q:.4f}."
        )

    def _width_to_q(self, width_m: float) -> float:
        q = (self.open_width_m - width_m) / self.open_width_m * self.max_q
        q = min(max(q, 0.0), self.max_q)
        if self.invert_command:
            q = self.max_q - q
        return min(max(q, 0.0), self.max_q)

    def _current_rg2_positions_locked(self, latest: Optional[Tuple[List[str], List[float], List[float]]]) -> Dict[str, float]:
        if latest is None:
            return dict(self._active_motion_positions)
        joint_names, joint_positions, _ = latest
        joint_map = {name: idx for idx, name in enumerate(joint_names)}
        return {
            joint_name: joint_positions[joint_map[joint_name]]
            for joint_name in self.rg2_joint_names
            if joint_name in joint_map and joint_map[joint_name] < len(joint_positions)
        }

    def _publish_step(self) -> None:
        with self._lock:
            latest = self._latest_joint_state
            if latest is None or not self._motion_target_positions:
                return

            joint_names, joint_positions, joint_velocities = latest
            joint_map = {name: idx for idx, name in enumerate(joint_names)}
            found = [name for name in self.rg2_joint_names if name in joint_map]
            missing = [name for name in self.rg2_joint_names if name not in joint_map]
            if missing and self._should_log("missing_rg2_joints", 5.0):
                self.get_logger().warn(f"RG2 joints missing from {self.joint_state_topic}: {missing}")
            if not found:
                return

            current_time = self._now_sec()
            commanded, done = self._sample_motion_locked(current_time, found)
            self._active_motion_positions.update(commanded)

            output_positions = list(joint_positions)
            output_velocities = list(joint_velocities) if joint_velocities else []

            for joint_name in found:
                idx = joint_map[joint_name]
                target_position = commanded[joint_name]
                previous_position = joint_positions[idx] if idx < len(joint_positions) else target_position
                output_positions[idx] = target_position
                if output_velocities and idx < len(output_velocities):
                    output_velocities[idx] = (target_position - previous_position) * self.command_rate_hz

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = output_positions
        if output_velocities:
            msg.velocity = output_velocities
        self._publisher.publish(msg)

        if done and self._should_log("target_reached", 1.0):
            self.get_logger().info("RG2 gripper target reached within tolerance.")

    def _sample_motion_locked(self, current_time: float, found: List[str]) -> Tuple[Dict[str, float], bool]:
        if not self.smooth_motion or self.motion_duration_sec <= 0.0 or self._motion_end_time_sec <= self._motion_start_time_sec:
            return ({joint_name: self._motion_target_positions[joint_name] for joint_name in found}, True)

        alpha = (current_time - self._motion_start_time_sec) / (self._motion_end_time_sec - self._motion_start_time_sec)
        alpha = min(max(alpha, 0.0), 1.0)
        commanded = {}
        done = alpha >= 1.0
        for joint_name in found:
            start = self._motion_start_positions.get(joint_name, self._motion_target_positions[joint_name])
            target = self._motion_target_positions[joint_name]
            value = start + alpha * (target - start)
            if abs(target - value) <= self.command_tolerance:
                value = target
            else:
                done = False
            commanded[joint_name] = value
        return commanded, done

    def _should_log(self, key: str, interval_sec: float) -> bool:
        now_sec = self._now_sec()
        last_time = self._last_logged_times.get(key, float("-inf"))
        if now_sec - last_time >= interval_sec:
            self._last_logged_times[key] = now_sec
            return True
        return False

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main() -> None:
    rclpy.init()
    node = RG2GripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
