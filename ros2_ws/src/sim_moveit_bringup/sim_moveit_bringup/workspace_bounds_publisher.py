#!/usr/bin/env python3
"""Publish a static floor slab into the MoveIt planning scene.

Why this exists: with use_octomap:=false the occupancy map is the thing that
disappears, and the octomap was the ONLY representation of the work surface --
no table exists in the URDF, and model_registry.yaml registers parts and the
base2 fixture but no table. Without it the planner will happily route the
elbow or a descent straight through the bench. Every assembly part is already
a registry collision mesh, so a single box under them is all the octomap was
really contributing.

The slab's X/Y footprint is sized to the arm's reach envelope rather than to
the whole cell: outside it, "the planner might route below the table" is not a
reachable failure, so extra slab is pure bulk in the RViz view and extra
collision-check surface. It spans from `top_z` down by `thickness_m`.

`top_z` defaults BELOW the true bench top rather than flush with it. base2 and
every unpicked part rest on the bench, so a flush slab is tangent to all of
them and FCL reports the scene in collision before anything moves; the same
goes for base_link, which sits at world origin. A few millimetres of give is
cheaper than an ACM pair per registry object and the planner cannot
meaningfully exploit it. `allowed_links` remains as a safety net for cells
that raise `top_z` back to the mount plane.

The box is re-asserted on a timer rather than published once. A move_group
restarted by hand comes up with an empty world while this node keeps running,
and a one-shot ADD would leave the cell with no table at all -- silently, and
exactly when someone is mid-task. The timer only writes when the object or its
ACM entries are actually absent, so the steady-state cost is one
get_planning_scene call every couple of seconds.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class WorkspaceBoundsPublisher(Node):
    """Keep a floor-slab collision object present in the planning scene."""

    def __init__(self) -> None:
        super().__init__("workspace_bounds_publisher")

        self.declare_parameter("object_id", "workspace_floor")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("min_x", -0.9)
        self.declare_parameter("max_x", 0.9)
        self.declare_parameter("min_y", -0.9)
        self.declare_parameter("max_y", 0.9)
        self.declare_parameter("top_z", -0.05)
        self.declare_parameter("thickness_m", 0.1)
        self.declare_parameter("allowed_links", ["base_link", "shoulder_link"])
        self.declare_parameter("reassert_period_sec", 2.0)
        self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
        self.declare_parameter("get_planning_scene_service", "/get_planning_scene")

        self._object_id = self._string("object_id")
        self._frame_id = self._string("frame_id")
        self._allowed_links = list(
            self.get_parameter("allowed_links").get_parameter_value().string_array_value
        )

        min_x = self._number("min_x")
        max_x = self._number("max_x")
        min_y = self._number("min_y")
        max_y = self._number("max_y")
        top_z = self._number("top_z")
        thickness = self._number("thickness_m")

        if max_x <= min_x or max_y <= min_y:
            raise ValueError(
                f"Degenerate footprint: x=[{min_x}, {max_x}] y=[{min_y}, {max_y}]. "
                "max must exceed min on both axes."
            )
        if thickness <= 0.0:
            raise ValueError(f"thickness_m must be positive, got {thickness}.")

        self._size = (max_x - min_x, max_y - min_y, thickness)
        self._center = (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            top_z - 0.5 * thickness,
        )

        self._apply_client = self.create_client(
            ApplyPlanningScene, self._string("apply_planning_scene_service")
        )
        self._get_client = self.create_client(
            GetPlanningScene, self._string("get_planning_scene_service")
        )

        self.get_logger().info(
            f"Workspace floor '{self._object_id}' in frame '{self._frame_id}': "
            f"x=[{min_x}, {max_x}] y=[{min_y}, {max_y}] "
            f"top_z={top_z} thickness={thickness} "
            f"(size={self._size} center={self._center})"
        )
        self.get_logger().info(
            f"Collision with the floor is allowed for links: {self._allowed_links}"
        )

        self._pending = False
        period = max(self._number("reassert_period_sec"), 0.1)
        self._timer = self.create_timer(period, self._tick)

    # -- parameter helpers ------------------------------------------------

    def _string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _number(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    # -- scene plumbing ---------------------------------------------------

    def _build_collision_object(self) -> CollisionObject:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(self._size)

        pose = Pose()
        pose.position.x = self._center[0]
        pose.position.y = self._center[1]
        pose.position.z = self._center[2]
        pose.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.id = self._object_id
        collision_object.header.frame_id = self._frame_id
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _tick(self) -> None:
        """Re-add the slab whenever the scene comes up without it."""
        if self._pending:
            return
        if not self._apply_client.service_is_ready():
            return
        if not self._get_client.service_is_ready():
            return
        self._pending = True
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self._get_client.call_async(request)
        future.add_done_callback(self._on_scene)

    def _on_scene(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - service transport failure
            self.get_logger().warn(f"get_planning_scene failed: {exc}")
            self._pending = False
            return

        scene = response.scene
        present = any(obj.id == self._object_id for obj in scene.world.collision_objects)
        acm = scene.allowed_collision_matrix
        missing_acm = [
            link
            for link in self._allowed_links
            if not self._pair_allowed(acm, self._object_id, link)
        ]
        if present and not missing_acm:
            self._pending = False
            return

        diff = PlanningScene()
        diff.is_diff = True
        diff.world.collision_objects = [self._build_collision_object()]
        diff.allowed_collision_matrix = self._extend_acm(acm)

        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = diff
        apply_future = self._apply_client.call_async(apply_request)
        apply_future.add_done_callback(
            lambda fut, was_present=present: self._on_applied(fut, was_present)
        )

    def _on_applied(self, future, was_present: bool) -> None:
        self._pending = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - service transport failure
            self.get_logger().warn(f"apply_planning_scene failed: {exc}")
            return
        if not response.success:
            self.get_logger().warn(
                f"move_group rejected the '{self._object_id}' floor slab."
            )
            return
        reason = "ACM entries restored" if was_present else "added to the world"
        self.get_logger().info(f"Workspace floor '{self._object_id}' {reason}.")

    @staticmethod
    def _pair_allowed(acm, name_a: str, name_b: str) -> bool:
        names = list(acm.entry_names)
        if name_a not in names or name_b not in names:
            return False
        row = acm.entry_values[names.index(name_a)].enabled
        return bool(row[names.index(name_b)])

    def _extend_acm(self, acm):
        """Return a copy of `acm` with the floor allowed against each link.

        The whole matrix is rewritten rather than sent as a sparse diff:
        move_group replaces the ACM wholesale when a diff carries one, so a
        partial matrix would drop every entry set elsewhere (the registry's
        acm_allowed_object_pairs, for instance).
        """
        names = list(acm.entry_names)
        values = [list(entry.enabled) for entry in acm.entry_values]

        for name in [self._object_id, *self._allowed_links]:
            if name in names:
                continue
            for row in values:
                row.append(False)
            names.append(name)
            values.append([False] * len(names))

        floor_index = names.index(self._object_id)
        for link in self._allowed_links:
            link_index = names.index(link)
            values[floor_index][link_index] = True
            values[link_index][floor_index] = True

        extended = type(acm)()
        extended.entry_names = names
        extended.entry_values = [AllowedCollisionEntry(enabled=row) for row in values]
        extended.default_entry_names = list(acm.default_entry_names)
        extended.default_entry_values = list(acm.default_entry_values)
        return extended


def main() -> None:
    rclpy.init()
    node = WorkspaceBoundsPublisher()
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
