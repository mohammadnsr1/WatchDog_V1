"""ROS-backed MoveIt planning scene backend."""

from __future__ import annotations

from collections.abc import Sequence
from copy import deepcopy
from typing import Any

import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty

from robot_backend.interfaces.scene_backend import SceneBackend


DEFAULT_APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"
DEFAULT_GET_PLANNING_SCENE_SERVICE = "/get_planning_scene"
DEFAULT_CLEAR_OCTOMAP_SERVICE = "/clear_octomap"

_PRIMITIVE_TYPE_NAMES = {
    SolidPrimitive.BOX: "box",
    SolidPrimitive.SPHERE: "sphere",
    SolidPrimitive.CYLINDER: "cylinder",
    SolidPrimitive.CONE: "cone",
}


class ROSMoveItSceneBackend(SceneBackend):
    """Planning scene backend wrapper for MoveIt scene operations."""

    def __init__(
        self,
        node: Node | None = None,
        *,
        apply_planning_scene_service: str = DEFAULT_APPLY_PLANNING_SCENE_SERVICE,
        get_planning_scene_service: str = DEFAULT_GET_PLANNING_SCENE_SERVICE,
        clear_octomap_service: str = DEFAULT_CLEAR_OCTOMAP_SERVICE,
    ) -> None:
        self._owns_node = node is None
        self._node = node or self._create_node()
        self._apply_planning_scene_service = apply_planning_scene_service
        self._get_planning_scene_service = get_planning_scene_service
        self._clear_octomap_service = clear_octomap_service
        self._touch_links_by_object: dict[str, list[str]] = {}
        self._collision_object_cache: dict[str, CollisionObject] = {}
        self._apply_planning_scene_client = self._node.create_client(
            ApplyPlanningScene,
            self._apply_planning_scene_service,
        )
        self._get_planning_scene_client = self._node.create_client(
            GetPlanningScene,
            self._get_planning_scene_service,
        )
        self._clear_octomap_client = self._node.create_client(
            Empty,
            self._clear_octomap_service,
        )

    @property
    def node(self) -> Node:
        """Return the shared ROS node used for scene operations."""

        return self._node

    def add_collision_object(
        self,
        object_id: str,
        pose: Any,
        shape: Any,
        **kwargs: Any,
    ) -> dict[str, Any]:
        """Add or update a collision object in the planning scene."""

        normalized_object_id = self._normalize_object_id(object_id)
        pose_stamped = self._coerce_pose_stamped(pose)
        geometry = self._coerce_collision_geometry(shape)
        if geometry["kind"] == "mesh_placeholder":
            raise NotImplementedError(
                "Mesh collision objects are not implemented in this Python backend yet. "
                "TODO: load mesh resources into shape_msgs/Mesh messages and cache the "
                "result for add/attach planning-scene diffs. "
                f"Requested mesh_resource={geometry['summary']['mesh_resource']!r}."
            )

        collision_object = CollisionObject()
        collision_object.header.frame_id = pose_stamped.header.frame_id
        collision_object.header.stamp = pose_stamped.header.stamp
        collision_object.id = normalized_object_id
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives = [geometry["primitive"]]
        collision_object.primitive_poses = [pose_stamped.pose]

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [collision_object]

        response = self._call_service(
            self._apply_planning_scene_client,
            request,
            self._apply_planning_scene_service,
        )
        self._collision_object_cache[normalized_object_id] = deepcopy(collision_object)
        verified_present = self._verify_world_object_present(normalized_object_id)

        return {
            "ok": verified_present,
            "command": "add_collision_object",
            "object_id": normalized_object_id,
            "frame_id": pose_stamped.header.frame_id,
            "shape": geometry["summary"],
            "apply_response_success": bool(response.success),
            "verified_present": verified_present,
            "response": self._summarize_apply_planning_scene_response(response),
        }

    def remove_collision_object(self, object_id: str) -> dict[str, Any]:
        """Remove a collision object from the planning scene."""

        normalized_object_id = self._normalize_object_id(object_id)
        world_exists_before = self._world_object_exists(normalized_object_id)
        attached_exists_before = self._attached_object_exists(normalized_object_id)

        if not world_exists_before:
            self._collision_object_cache.pop(normalized_object_id, None)
            self._touch_links_by_object.pop(normalized_object_id, None)
            return {
                "ok": True,
                "command": "remove_collision_object",
                "object_id": normalized_object_id,
                "already_absent": True,
                "world_object_exists_before": False,
                "attached_object_exists_before": attached_exists_before,
                "verified_absent": True,
                "apply_response_success": None,
                "response": None,
            }

        collision_object = CollisionObject()
        collision_object.id = normalized_object_id
        collision_object.operation = CollisionObject.REMOVE

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [collision_object]

        response = self._call_service(
            self._apply_planning_scene_client,
            request,
            self._apply_planning_scene_service,
        )
        verified_absent = self._verify_world_object_absent(normalized_object_id)
        if verified_absent:
            self._collision_object_cache.pop(normalized_object_id, None)
        self._touch_links_by_object.pop(normalized_object_id, None)

        return {
            "ok": verified_absent,
            "command": "remove_collision_object",
            "object_id": normalized_object_id,
            "already_absent": False,
            "world_object_exists_before": world_exists_before,
            "attached_object_exists_before": attached_exists_before,
            "apply_response_success": bool(response.success),
            "verified_absent": verified_absent,
            "response": self._summarize_apply_planning_scene_response(response),
        }

    def allow_touch(
        self,
        object_id: str,
        link_names: Sequence[str],
    ) -> dict[str, Any]:
        """Store touch links for use by later attach operations."""

        normalized_object_id = self._normalize_object_id(object_id)
        normalized_link_names = self._normalize_link_names(link_names)
        self._touch_links_by_object[normalized_object_id] = normalized_link_names
        return {
            "ok": True,
            "command": "allow_touch",
            "object_id": normalized_object_id,
            "touch_links": list(normalized_link_names),
            "note": (
                "Touch links are cached locally for attach_object(). "
                "AllowedCollisionMatrix updates are not implemented yet."
            ),
        }

    def attach_object(
        self,
        object_id: str,
        link_name: str,
        touch_links: Sequence[str] | None = None,
    ) -> dict[str, Any]:
        """Attach an object to a robot link using full collision geometry."""

        normalized_object_id = self._normalize_object_id(object_id)
        normalized_link_name = self._normalize_required_string(link_name, "link_name")
        if touch_links is None:
            resolved_touch_links = list(
                self._touch_links_by_object.get(normalized_object_id, [])
            )
        else:
            resolved_touch_links = self._normalize_link_names(touch_links)

        attached_collision_object, used_cached_geometry = self._get_world_collision_object(
            normalized_object_id
        )
        attached_collision_object.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = normalized_link_name
        attached.object = attached_collision_object
        attached.touch_links = list(resolved_touch_links)

        remove_world_object = CollisionObject()
        remove_world_object.id = normalized_object_id
        remove_world_object.operation = CollisionObject.REMOVE

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [attached]
        request.scene.world.collision_objects = [remove_world_object]

        response = self._call_service(
            self._apply_planning_scene_client,
            request,
            self._apply_planning_scene_service,
        )
        verified_attached = self._verify_attached_object_present(
            normalized_object_id,
            normalized_link_name,
        )
        verified_removed_from_world = self._verify_world_object_absent(
            normalized_object_id
        )

        return {
            "ok": verified_attached,
            "command": "attach_object",
            "object_id": normalized_object_id,
            "link_name": normalized_link_name,
            "touch_links": list(resolved_touch_links),
            "used_cached_geometry": used_cached_geometry,
            "apply_response_success": bool(response.success),
            "verified_attached": verified_attached,
            "verified_removed_from_world": verified_removed_from_world,
            "response": self._summarize_apply_planning_scene_response(response),
        }

    def detach_object(self, object_id: str) -> dict[str, Any]:
        """Detach an attached object from the robot.

        This only removes the object from the robot attached state. Place skills
        should re-add or update the object in the world collision set after
        detach when they know the released pose.
        """

        normalized_object_id = self._normalize_object_id(object_id)
        attached = AttachedCollisionObject()
        attached.object.id = normalized_object_id
        attached.object.operation = CollisionObject.REMOVE

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.robot_state.attached_collision_objects = [attached]

        response = self._call_service(
            self._apply_planning_scene_client,
            request,
            self._apply_planning_scene_service,
        )
        verified_detached = not self._attached_object_exists(normalized_object_id)
        world_object_present_after_detach = self._world_object_exists(normalized_object_id)

        return {
            "ok": verified_detached,
            "command": "detach_object",
            "object_id": normalized_object_id,
            "apply_response_success": bool(response.success),
            "verified_detached": verified_detached,
            "world_object_present_after_detach": world_object_present_after_detach,
            "response": self._summarize_apply_planning_scene_response(response),
            "note": (
                "detach_object only removes the object from attached robot state. "
                "Place skills should re-add or update the world collision object afterward."
            ),
        }

    def clear_octomap(self) -> dict[str, Any]:
        """Clear perception-derived occupancy data when needed."""

        self._call_service(
            self._clear_octomap_client,
            Empty.Request(),
            self._clear_octomap_service,
        )
        return {
            "ok": True,
            "command": "clear_octomap",
            "service_name": self._clear_octomap_service,
            "response": {
                "service": self._clear_octomap_service,
                "service_type": "std_srvs/srv/Empty",
                "result": "completed",
            },
        }

    def get_objects(self) -> dict[str, Any]:
        """Return known world and attached objects from the planning scene."""

        world_objects, attached_objects = self._fetch_scene_objects()
        return {
            "ok": True,
            "command": "get_objects",
            "world_collision_objects": [
                self._summarize_collision_object(collision_object)
                for collision_object in world_objects
            ],
            "attached_objects": [
                self._summarize_attached_collision_object(attached_object)
                for attached_object in attached_objects
            ],
        }

    def _create_node(self) -> Node:
        """Create an internal helper node if one was not supplied."""

        if not rclpy.ok():
            rclpy.init(args=None)
        return Node("robot_backend_moveit_scene_client")

    def _coerce_pose_stamped(self, pose: Any) -> PoseStamped:
        """Normalize a pose input into a PoseStamped."""

        if isinstance(pose, PoseStamped):
            if not pose.header.frame_id:
                raise ValueError("PoseStamped.header.frame_id must be non-empty.")
            return pose

        if isinstance(pose, dict):
            frame_id = self._normalize_required_string(pose.get("frame_id"), "frame_id")
            position = pose.get("position")
            orientation = pose.get("orientation")
            if not isinstance(position, dict):
                raise ValueError("Pose dict must include a 'position' mapping.")
            if not isinstance(orientation, dict):
                raise ValueError("Pose dict must include an 'orientation' mapping.")

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.pose = Pose()
            pose_stamped.pose.position.x = float(position["x"])
            pose_stamped.pose.position.y = float(position["y"])
            pose_stamped.pose.position.z = float(position["z"])
            pose_stamped.pose.orientation.x = float(orientation["x"])
            pose_stamped.pose.orientation.y = float(orientation["y"])
            pose_stamped.pose.orientation.z = float(orientation["z"])
            pose_stamped.pose.orientation.w = float(orientation["w"])
            return pose_stamped

        raise TypeError(
            "pose must be a geometry_msgs.msg.PoseStamped or a pose dict with "
            "frame_id/position/orientation fields."
        )

    def _coerce_collision_geometry(self, shape: Any) -> dict[str, Any]:
        """Normalize supported collision geometry definitions."""

        if not isinstance(shape, dict):
            raise TypeError("shape must be a dictionary describing the geometry.")

        shape_type = self._normalize_required_string(shape.get("type"), "shape.type").lower()
        if shape_type == "box":
            dimensions = self._coerce_float_list(shape.get("dimensions"), expected_len=3)
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = dimensions
            return {
                "kind": "primitive",
                "primitive": primitive,
                "summary": {"type": "box", "dimensions": dimensions},
            }

        if shape_type == "sphere":
            radius = float(shape["radius"])
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [radius]
            return {
                "kind": "primitive",
                "primitive": primitive,
                "summary": {"type": "sphere", "radius": radius},
            }

        if shape_type == "cylinder":
            height = float(shape["height"])
            radius = float(shape["radius"])
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [height, radius]
            return {
                "kind": "primitive",
                "primitive": primitive,
                "summary": {
                    "type": "cylinder",
                    "height": height,
                    "radius": radius,
                },
            }

        if shape_type == "mesh":
            mesh_resource = self._normalize_required_string(
                shape.get("mesh_resource"),
                "shape.mesh_resource",
            )
            scale = self._coerce_float_list(shape.get("scale", [1.0, 1.0, 1.0]), expected_len=3)
            return {
                "kind": "mesh_placeholder",
                "summary": {
                    "type": "mesh",
                    "mesh_resource": mesh_resource,
                    "scale": scale,
                },
            }

        raise ValueError(
            f"Unsupported collision shape type {shape_type!r}. "
            "Supported types are: box, sphere, cylinder, mesh."
        )

    def _fetch_scene_objects(self) -> tuple[list[CollisionObject], list[AttachedCollisionObject]]:
        """Fetch world and attached collision objects from MoveIt's planning scene."""

        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        response = self._call_service(
            self._get_planning_scene_client,
            request,
            self._get_planning_scene_service,
        )
        return (
            list(response.scene.world.collision_objects),
            list(response.scene.robot_state.attached_collision_objects),
        )

    def _world_object_exists(self, object_id: str) -> bool:
        normalized_object_id = self._normalize_object_id(object_id)
        world_objects, _ = self._fetch_scene_objects()
        return any(collision_object.id == normalized_object_id for collision_object in world_objects)

    def _attached_object_exists(self, object_id: str, link_name: str | None = None) -> bool:
        normalized_object_id = self._normalize_object_id(object_id)
        normalized_link_name = (
            self._normalize_required_string(link_name, "link_name")
            if link_name is not None
            else None
        )
        _, attached_objects = self._fetch_scene_objects()
        for attached_object in attached_objects:
            if attached_object.object.id != normalized_object_id:
                continue
            if normalized_link_name is None or attached_object.link_name == normalized_link_name:
                return True
        return False

    def _verify_world_object_absent(self, object_id: str) -> bool:
        return not self._world_object_exists(object_id)

    def _verify_world_object_present(self, object_id: str) -> bool:
        return self._world_object_exists(object_id)

    def _verify_attached_object_present(self, object_id: str, link_name: str | None = None) -> bool:
        return self._attached_object_exists(object_id, link_name)

    def _get_world_collision_object(self, object_id: str) -> tuple[CollisionObject, bool]:
        """Return full world collision geometry for one object ID."""

        normalized_object_id = self._normalize_object_id(object_id)
        cached_object = self._collision_object_cache.get(normalized_object_id)
        if cached_object is not None:
            return deepcopy(cached_object), True

        world_objects, _ = self._fetch_scene_objects()
        for collision_object in world_objects:
            if collision_object.id == normalized_object_id:
                self._collision_object_cache[normalized_object_id] = deepcopy(collision_object)
                return deepcopy(collision_object), False

        raise RuntimeError(
            f"Cannot attach object {normalized_object_id!r} because it is not present "
            "in the world collision objects. Add it first."
        )

    def _call_service(
        self,
        client: Any,
        request: Any,
        service_name: str,
        timeout_sec: float = 5.0,
    ) -> Any:
        """Call a configured service synchronously through rclpy."""

        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"Service {service_name} is not available within {timeout_sec:.1f} seconds."
            )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=timeout_sec,
        )

        if not future.done():
            raise TimeoutError(
                f"Timed out waiting for service response from {service_name} "
                f"after {timeout_sec:.1f} seconds."
            )

        exception = future.exception()
        if exception is not None:
            raise RuntimeError(
                f"Service call to {service_name} failed: {exception}"
            ) from exception

        response = future.result()
        if response is None:
            raise RuntimeError(
                f"Service call to {service_name} completed without a response."
            )

        return response

    def _summarize_apply_planning_scene_response(
        self,
        response: ApplyPlanningScene.Response,
    ) -> dict[str, Any]:
        return {
            "service": self._apply_planning_scene_service,
            "service_type": "moveit_msgs/srv/ApplyPlanningScene",
            "success": bool(response.success),
        }

    def _summarize_collision_object(
        self,
        collision_object: CollisionObject,
    ) -> dict[str, Any]:
        geometry: list[dict[str, Any]] = []
        for primitive in collision_object.primitives:
            geometry.append(self._summarize_primitive(primitive))
        if collision_object.meshes:
            geometry.append(
                {
                    "type": "mesh",
                    "mesh_count": len(collision_object.meshes),
                    "todo": "Mesh summary is limited because this backend does not load mesh resources yet.",
                }
            )
        return {
            "id": collision_object.id,
            "frame_id": collision_object.header.frame_id,
            "geometry": geometry,
        }

    def _summarize_attached_collision_object(
        self,
        attached_object: AttachedCollisionObject,
    ) -> dict[str, Any]:
        summary = self._summarize_collision_object(attached_object.object)
        summary["link_name"] = attached_object.link_name
        summary["touch_links"] = list(attached_object.touch_links)
        return summary

    def _summarize_primitive(self, primitive: SolidPrimitive) -> dict[str, Any]:
        primitive_type = _PRIMITIVE_TYPE_NAMES.get(primitive.type, f"primitive_{primitive.type}")
        if primitive.type == SolidPrimitive.BOX:
            return {
                "type": primitive_type,
                "dimensions": [float(value) for value in primitive.dimensions],
            }
        if primitive.type == SolidPrimitive.SPHERE:
            radius = float(primitive.dimensions[0]) if primitive.dimensions else None
            return {"type": primitive_type, "radius": radius}
        if primitive.type == SolidPrimitive.CYLINDER:
            height = float(primitive.dimensions[0]) if len(primitive.dimensions) > 0 else None
            radius = float(primitive.dimensions[1]) if len(primitive.dimensions) > 1 else None
            return {
                "type": primitive_type,
                "height": height,
                "radius": radius,
            }
        return {
            "type": primitive_type,
            "dimensions": [float(value) for value in primitive.dimensions],
        }

    def _normalize_object_id(self, object_id: str) -> str:
        return self._normalize_required_string(object_id, "object_id")

    @staticmethod
    def _normalize_required_string(value: Any, field_name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{field_name} must be a non-empty string.")
        return value.strip()

    def _normalize_link_names(self, link_names: Sequence[str]) -> list[str]:
        normalized = [self._normalize_required_string(link_name, "link_name") for link_name in link_names]
        deduplicated: list[str] = []
        for link_name in normalized:
            if link_name not in deduplicated:
                deduplicated.append(link_name)
        return deduplicated

    @staticmethod
    def _coerce_float_list(values: Any, *, expected_len: int) -> list[float]:
        if not isinstance(values, (list, tuple)) or len(values) != expected_len:
            raise ValueError(
                f"Expected a list/tuple of length {expected_len}, got {values!r}."
            )
        return [float(value) for value in values]
