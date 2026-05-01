import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

ONCE_POSE = [0.0, -1.0, 1.2, -1.5, -1.57, 0.0]
HOME_POSE = [0.0, -1.2, 1.2, -1.57, -1.57, 0.0]


class JointCommandTestNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_command_test")

        self.declare_parameter("mode", "sine")
        self.declare_parameter("joint_index", 1)
        self.declare_parameter("amplitude", 0.2)
        self.declare_parameter("frequency", 0.2)
        self.declare_parameter("publish_rate", 20.0)

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.joint_index = self.get_parameter("joint_index").get_parameter_value().integer_value
        self.amplitude = self.get_parameter("amplitude").get_parameter_value().double_value
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value

        if self.mode not in ("once", "sine"):
            raise ValueError("Parameter 'mode' must be 'once' or 'sine'.")
        if not 0 <= self.joint_index < len(JOINT_NAMES):
            raise ValueError("Parameter 'joint_index' must be between 0 and 5.")
        if self.publish_rate <= 0.0:
            raise ValueError("Parameter 'publish_rate' must be greater than zero.")

        self.publisher = self.create_publisher(JointState, "/joint_command", 10)
        self.start_time = self.get_clock().now()

        if self.mode == "once":
            self.get_logger().info(
                f"Publishing one joint command to /joint_command: {ONCE_POSE}"
            )
            self.publish_positions(ONCE_POSE)
            # Shut down shortly after publishing once so the process exits cleanly.
            self.create_timer(0.25, self.shutdown_after_once)
        else:
            period = 1.0 / self.publish_rate
            self.get_logger().info(
                "Publishing sinusoidal joint commands to /joint_command "
                f"at {self.publish_rate:.1f} Hz on '{JOINT_NAMES[self.joint_index]}' "
                f"with amplitude {self.amplitude:.3f} rad and frequency {self.frequency:.3f} Hz."
            )
            self.create_timer(period, self.publish_sine_command)

    def publish_positions(self, positions) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(positions)
        self.publisher.publish(msg)

    def publish_sine_command(self) -> None:
        # Start from a fixed home pose and only oscillate one selected joint.
        positions = list(HOME_POSE)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        positions[self.joint_index] += self.amplitude * math.sin(
            2.0 * math.pi * self.frequency * elapsed
        )
        self.publish_positions(positions)

    def shutdown_after_once(self) -> None:
        self.get_logger().info("One-shot command published. Shutting down.")
        raise SystemExit


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = JointCommandTestNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
