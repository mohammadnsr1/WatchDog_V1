"""ROS-backed RG2 gripper backend.

The workspace confirms these default RG2 controller services:
- `/rg2_controller/open` using `std_srvs.srv.Trigger`
- `/rg2_controller/close` using `std_srvs.srv.Trigger`
- `/rg2_controller/set_position` using
  `isaac_trajectory_bridge_interfaces.srv.SetGripperPosition`
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Callable

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from isaac_trajectory_bridge_interfaces.srv import SetGripperPosition
from robot_backend.interfaces.gripper_backend import GripperBackend
from robot_backend.utils.rg2_geometry import DEFAULT_RG2_CALIBRATION
from robot_backend.utils.rg2_geometry import RG2Calibration
from robot_backend.utils.rg2_geometry import width_to_joint_position


RequestBuilder = Callable[..., Any]


@dataclass(frozen=True)
class ServiceBinding:
    """Describe one RG2 service endpoint and how to build its request."""

    name: str
    srv_type: type | None = None
    request_builder: RequestBuilder | None = None


class ROSGripperBackend(GripperBackend):
    """Gripper backend that talks to RG2 controller services through rclpy.

    The concrete service bindings are taken from
    `isaac_trajectory_bridge.rg2_controller.RG2Controller`.
    """

    def __init__(
        self,
        node: Node | None = None,
        *,
        calibration: RG2Calibration = DEFAULT_RG2_CALIBRATION,
        open_binding: ServiceBinding | None = None,
        close_binding: ServiceBinding | None = None,
        set_position_binding: ServiceBinding | None = None,
        joint_states_topic: str = "/joint_states",
    ) -> None:
        self._owns_node = node is None
        self._node = node or self._create_node()
        self._calibration = calibration
        self._open_binding = open_binding or ServiceBinding(
            name="/rg2_controller/open",
            srv_type=Trigger,
            request_builder=self._build_trigger_request,
        )
        self._close_binding = close_binding or ServiceBinding(
            name="/rg2_controller/close",
            srv_type=Trigger,
            request_builder=self._build_trigger_request,
        )
        self._set_position_binding = set_position_binding or ServiceBinding(
            name="/rg2_controller/set_position",
            srv_type=SetGripperPosition,
            request_builder=self._build_set_position_request,
        )
        self._joint_states_topic = joint_states_topic
        self._joint_state_sub = None
        self._latest_joint_state: Any = None

    @property
    def node(self) -> Node:
        """Return the shared ROS node used for service calls."""

        return self._node

    def open(self) -> Trigger.Response:
        """Call the RG2 open service."""

        return self._call_service(self._open_binding)

    def close(self) -> Trigger.Response:
        """Call the RG2 close service."""

        return self._call_service(self._close_binding)

    def set_width(self, width_m: float) -> SetGripperPosition.Response:
        """Command a physical RG2 opening width in meters.

        This converts the requested physical width to the RG2 master joint
        target in radians using the configured calibration, then calls the
        controller's `/rg2_controller/set_position` service.
        """

        position_rad = width_to_joint_position(width_m, self._calibration)
        return self.set_joint_position(position_rad)

    def set_joint_position(self, position_rad: float) -> SetGripperPosition.Response:
        """Command the raw RG2 master joint target in radians.

        This is backend-specific and bypasses the higher-level physical width
        semantics exposed by `GripperBackend.set_width(width_m)`.
        """

        return self._call_service(
            self._set_position_binding,
            position_rad=position_rad,
        )

    def get_state(self) -> dict[str, Any]:
        """Return backend metadata and the active RG2 calibration."""

        return {
            "service_names": {
                "open": self._open_binding.name,
                "close": self._close_binding.name,
                "set_position": self._set_position_binding.name,
            },
            "service_types": {
                "open": "std_srvs/srv/Trigger",
                "close": "std_srvs/srv/Trigger",
                "set_position": (
                    "isaac_trajectory_bridge_interfaces/srv/"
                    "SetGripperPosition"
                ),
            },
            "calibration": {
                "width_open_m": self._calibration.width_open_m,
                "width_closed_m": self._calibration.width_closed_m,
                "position_open_rad": self._calibration.position_open_rad,
                "position_closed_rad": self._calibration.position_closed_rad,
                "tcp_to_grasp_center_open": self._calibration.tcp_to_grasp_center_open,
                "tcp_to_grasp_center_closed": self._calibration.tcp_to_grasp_center_closed,
            },
        }

    def read_joint_state(
        self,
        joint_name: str,
        timeout_sec: float = 1.0,
    ) -> dict[str, Any]:
        """Return effort and position for joint_name from /joint_states.

        Creates the subscription once on first call (lazy init) and caches
        the latest message. Spins the node with spin_once until a message
        containing the named joint arrives or timeout elapses.
        """
        self._ensure_joint_state_subscription()
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)
            msg = self._latest_joint_state
            if msg is not None:
                names = list(msg.name)
                if joint_name in names:
                    idx = names.index(joint_name)
                    effort = (
                        float(msg.effort[idx])
                        if msg.effort and idx < len(msg.effort)
                        else 0.0
                    )
                    position = (
                        float(msg.position[idx])
                        if msg.position and idx < len(msg.position)
                        else 0.0
                    )
                    return {
                        "ok": True,
                        "joint_name": joint_name,
                        "effort": effort,
                        "position": position,
                    }
        return {
            "ok": False,
            "joint_name": joint_name,
            "error": (
                f"Timeout ({timeout_sec:.1f}s) waiting for joint state "
                f"containing '{joint_name}' on {self._joint_states_topic}."
            ),
        }

    def _ensure_joint_state_subscription(self) -> None:
        """Create the /joint_states subscription once, lazily."""
        if self._joint_state_sub is not None:
            return
        from sensor_msgs.msg import JointState

        def _callback(msg: Any) -> None:
            self._latest_joint_state = msg

        self._joint_state_sub = self._node.create_subscription(
            JointState,
            self._joint_states_topic,
            _callback,
            10,
        )

    def _create_node(self) -> Node:
        """Create an internal helper node if one was not supplied."""

        if not rclpy.ok():
            rclpy.init(args=None)
        return Node("robot_backend_gripper_client")

    def _call_service(self, binding: ServiceBinding, **kwargs: Any) -> Any:
        """Call a configured service synchronously through rclpy."""

        if binding.srv_type is None or binding.request_builder is None:
            raise NotImplementedError(
                f"Service binding for {binding.name} is incomplete. "
                "Confirm the service type and request fields, then inject a "
                "complete binding."
            )

        client = self._node.create_client(binding.srv_type, binding.name)
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(
                f"Service {binding.name} is not available within 2.0 seconds."
            )

        request = binding.request_builder(**kwargs)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=10.0,
        )

        if not future.done():
            raise TimeoutError(
                f"Timed out waiting for service response from {binding.name}."
            )

        exception = future.exception()
        if exception is not None:
            raise RuntimeError(
                f"Service call to {binding.name} failed: {exception}"
            ) from exception

        response = future.result()
        if response is None:
            raise RuntimeError(
                f"Service call to {binding.name} completed without a response."
            )

        return response

    @staticmethod
    def _build_trigger_request() -> Trigger.Request:
        """Build an empty Trigger request for open/close commands."""

        return Trigger.Request()

    @staticmethod
    def _build_set_position_request(
        position_rad: float,
    ) -> SetGripperPosition.Request:
        """Build the RG2 set-position request using joint radians."""

        request = SetGripperPosition.Request()
        request.position = float(position_rad)
        return request
