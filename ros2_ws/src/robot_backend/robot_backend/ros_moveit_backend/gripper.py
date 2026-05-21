"""ROS-backed RG2 gripper backend.

This module is intentionally conservative in its assumptions. The repo already
contains RG2 controller services, but this package does not guess custom
service types or request fields without inspection. Callers can inject the
resolved service classes and request builders once they are confirmed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import rclpy
from rclpy.node import Node

from robot_backend.interfaces.gripper_backend import GripperBackend


RequestBuilder = Callable[..., Any]


@dataclass(frozen=True)
class ServiceBinding:
    """Describes one RG2 service endpoint and how to build its request."""

    name: str
    srv_type: type | None = None
    request_builder: RequestBuilder | None = None


class ROSGripperBackend(GripperBackend):
    """Gripper backend that talks to RG2 controller services through rclpy.

    TODO: Inspect the existing RG2 controller package in this repo and replace
    injected bindings with concrete imports once the exact service types and
    request fields are confirmed.
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
        self._open_binding = open_binding or ServiceBinding(name="/rg2_controller/open")
        self._close_binding = close_binding or ServiceBinding(name="/rg2_controller/close")
        self._set_position_binding = set_position_binding or ServiceBinding(
            name="/rg2_controller/set_position"
        )

    @property
    def node(self) -> Node:
        """Return the shared ROS node used for service calls."""

        return self._node

    def open(self) -> Any:
        """Call the RG2 open service."""

        return self._call_service(self._open_binding)

    def close(self) -> Any:
        """Call the RG2 close service."""

        return self._call_service(self._close_binding)

    def set_width(self, width_m: float) -> Any:
        """Call the RG2 position service with a target width in meters."""

        return self._call_service(self._set_position_binding, width_m=width_m)

    def get_state(self) -> dict[str, Any]:
        """Return lightweight backend state for now.

        TODO: If the repo already exposes a gripper state topic or service,
        wrap it here once the interface is confirmed.
        """

        return {
            "service_names": {
                "open": self._open_binding.name,
                "close": self._close_binding.name,
                "set_position": self._set_position_binding.name,
            }
        }

    def _create_node(self) -> Node:
        """Create an internal helper node if one was not supplied."""

        if not rclpy.ok():
            rclpy.init(args=None)
        return Node("robot_backend_gripper_client")

    def _call_service(self, binding: ServiceBinding, **kwargs: Any) -> Any:
        """Call a configured service synchronously through rclpy.

        The implementation stays generic until the exact RG2 service types are
        confirmed from the repo.
        """

        if binding.srv_type is None or binding.request_builder is None:
            raise NotImplementedError(
                f"Service binding for {binding.name} is incomplete. "
                "Confirm the RG2 service type and request fields in the repo "
                "and inject them here."
            )

        client = self._node.create_client(binding.srv_type, binding.name)
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Service {binding.name} is not available.")

        request = binding.request_builder(**kwargs)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        if not future.done():
            raise TimeoutError(f"Timed out waiting for service {binding.name}.")
        return future.result()
