"""ROS-backed RG2 gripper backend.

The workspace confirms these default RG2 controller services:
- `/rg2_controller/open` using `std_srvs.srv.Trigger`
- `/rg2_controller/close` using `std_srvs.srv.Trigger`
- `/rg2_controller/set_position` using
  `isaac_trajectory_bridge_interfaces.srv.SetGripperPosition`
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from typing import Callable

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from isaac_trajectory_bridge_interfaces.srv import SetGripperPosition
from robot_backend.interfaces.gripper_backend import GripperBackend


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
        open_binding: ServiceBinding | None = None,
        close_binding: ServiceBinding | None = None,
        set_position_binding: ServiceBinding | None = None,
    ) -> None:
        self._owns_node = node is None
        self._node = node or self._create_node()
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
        """Call the RG2 set-position service.

        The backend interface expresses gripper width in meters, but the
        currently available RG2 controller service accepts a `position` field
        interpreted by the controller as the RG2 master joint target in
        radians. Until a confirmed width-to-joint mapping is defined in this
        workspace, this backend forwards the numeric argument into that
        `position` request field.

        TODO: Replace this pass-through with a confirmed width-to-master-joint
        conversion once the project defines the desired RG2 width convention.
        """

        return self._call_service(self._set_position_binding, width_m=width_m)

    def get_state(self) -> dict[str, Any]:
        """Return lightweight backend state for now.

        TODO: If the repo later exposes a gripper state topic or service,
        wrap it here with a stable backend-facing representation.
        """

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
        }

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
    def _build_set_position_request(width_m: float) -> SetGripperPosition.Request:
        """Build the RG2 set-position request.

        The service request field is named `position` in the interface package.
        See `isaac_trajectory_bridge/README_RG2_CONTROLLER.md` for the current
        controller-side interpretation.
        """

        request = SetGripperPosition.Request()
        request.position = float(width_m)
        return request
