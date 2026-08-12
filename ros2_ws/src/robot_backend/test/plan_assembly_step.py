#!/usr/bin/env python3
"""
plan_assembly_step.py

Class-1 instruction planner for WatchDog FMB assemblies: full pick-and-place.

WHY THIS EXISTS
---------------
Authoring a class-1 executor/watchdog YAML pair currently means guessing
grasp_center_offset and gripper_width_m, running the full agent stack, and
finding out about geometry mistakes (fingers in the table, TCP off by 4cm,
a lift that never clears the object, a place that drives through the part)
only after an LLM has burned tokens driving the arm there. This script closes
that loop directly, no agent in the middle:

  1. Resolve the object's CURRENT world pose from the single TF-buffer
     owner (scene_tf_sync_node, via /get_object_pose) -- this script does
     NOT open a second TF buffer.
  2. Resolve the grasp center in world frame from a grasp_center_offset you
     supply as {position: OBJECT-frame point, orientation: DESIRED WORLD
     orientation of the grasp center/TCP}. Position is genuinely
     object-relative and gets rotated+translated through the object's live
     TF orientation; orientation is taken directly, not composed with the
     object's orientation -- see grasp_center_world_from_object_and_offset's
     docstring for why this split is deliberate. NOTE: this is a narrower
     convention than the executor.yaml steps: block has historically stored
     (which composed orientation against the object frame too); the two are
     numerically identical only when the object's live orientation was
     identity at authoring time, which happens to be true for every
     grasp_center_offset in the registry today. See the ASSUMPTIONS header
     for what still needs to change in motion_mcp_server.py to make this
     the actual system-wide convention rather than just this script's.
  3. Compute pre-grasp/pre-place and grasp/place TCP poses the same way
     motion_mcp_server._compute_approach_geometry does: tcp_target_from_grasp_center
     at the target center, and at the target center offset by
     approach_distance_m along ITS OWN local Z axis (via compose_poses, not
     a hand-rolled quaternion rotation, and not a hardcoded world axis --
     this generalizes to a side approach/placement, not just top-down).
     approach_distance_m's SIGN is yours to choose: it is applied exactly as
     given, not forced negative, since which local direction backs away
     from the object depends on the orientation you picked -- see
     pregrasp_center_from_grasp_center's docstring.
  4. DRY-RUN by default. Prints every computed pose. Nothing moves.
  5. With --execute: runs the plan step by step, using the same skills
     (grasp_skill, release_skill, execute_approach_and_preposition-equivalent
     motion, open_gripper) underneath, pausing for confirmation between
     steps unless --yes.

A plan is a small YAML file with one or more steps, so a full
approach -> grasp -> lift -> place -> release -> retreat sequence can be
validated in one call before any of it becomes a registry corruption or
fault-injection target. Steps chain: a "delta_from_last" step moves by a
WORLD-frame translation relative to the last grasp center THIS SCRIPT
computed -- not a live re-query of the object's TF -- so a lift step stays
well-defined even while the post-attach object-TF-staleness question is
still open (see WatchDog's own investigation of that question elsewhere;
short answer: attach_object does not touch the object's world TF publisher,
only MoveIt's own planning-scene bookkeeping -- see grasp/attach notes below).

OUT OF SCOPE ON PURPOSE: grasp effort / max_clamp_effort characterization.
Once a step closes the gripper, run rg2_effort_sweep.py separately for
that. This script only proves the approach/preposition/lift/place geometry
is right before you spend fault-injection effort on it.

OBJECT ATTACH/DETACH LIFECYCLE (read this before authoring a plan)
--------------------------------------------------------------------------
There is no pick_object/place_object skill in this codebase (a stale
__pycache__/pick_place_skills.cpython-312.pyc is the only trace of one --
the source module is gone). The real attach/detach lifecycle lives in
robot_backend.skills.gripper_skills:
  - grasp_skill(backend, object_id=..., registry=..., expected_width_m=...)
    does a torque-guarded close via the GripperGuardedMove action AND, on a
    confirmed contact + plausible width, calls backend.scene.attach_object(...)
    -- this is what makes MoveIt's planning scene carry the object rigidly
    with the gripper. Skipping this (e.g. calling the bare close_gripper_skill
    instead, which is just backend.gripper.close() with no attach at all) is
    a real correctness bug for any --execute lift/carry: the object stays a
    stationary WORLD collision object at wherever it was last synced, and
    does not move with the gripper in the planning scene at all.
  - release_skill(backend, object_id=...) does the reverse: open the
    gripper, backend.scene.detach_object(...), THEN
    backend.scene.sync_collision_objects([object_id]) to re-register the
    object into the world collision set at its new (placed) pose --
    detach_object's own docstring says this resync is the caller's job, not
    optional. This script's "grasp" and "release" actions call grasp_skill /
    release_skill directly rather than hand-rolling a second, divergent
    open+detach(+forgotten resync) sequence.
  - approach_and_preposition and move_grasp_center_to_pose stay motion-only
    on purpose: attach state is a gripper/scene concern, not a motion one.

ASSUMPTIONS -- verified against the real source, not the previous draft's
memory of it. All of the below were checked by reading the actual modules
line by line (see the code for exact file:line references):
  - /get_object_pose is served by scene_tf_sync_node, request field
    object_id (string), response fields ok, object_id, frame_id, tf_frame,
    pose (geometry_msgs/Pose), error -- from robot_backend_interfaces.srv
    import GetObjectPose. CONFIRMED correct.
  - robot_backend.utils.geometry exposes make_pose(position, orientation,
    frame_id=None) -> PoseDict, compose_poses(base_pose, offset_pose) ->
    base * offset, and rotate_vector(vector, quaternion); PoseDict is
    {"position": {x,y,z}, "orientation": {x,y,z,w}, "frame_id": ...}.
    CONFIRMED correct.
  - robot_backend.utils.rg2_geometry.tcp_target_from_grasp_center(
    grasp_center_pose, width_m) -> PoseDict. CONFIRMED correct.
  - robot_backend.skills.motion_skills exposes move_tcp_to_pose_skill,
    move_grasp_center_to_pose_skill (gripper_width_m is a required
    positional), move_to_named_safe_joint_pose_skill. CONFIRMED correct.
  - robot_backend.skills.gripper_skills exposes open_gripper_skill (bare
    backend.gripper.open(), no attach semantics -- fine for a standalone
    open) and, for an actual grasp/release, grasp_skill / release_skill
    (NOT close_gripper_skill, which the previous draft used and which does
    not attach anything -- see the lifecycle note above).
  - robot_backend.ros_moveit_backend.create_ros_moveit_backend(node=None)
    -- CONFIRMED, and this script now passes it the SAME node used for
    /get_object_pose (one shared node total for backend + pose client,
    matching the convention in rg2_effort_sweep.py's exec_node, rather than
    the previous draft's 4 independent, un-shared nodes). A SEPARATE small
    node/executor thread is still used for the live gripper-width
    subscription (see GripperWidthReader) -- this is unavoidable, not an
    oversight: the shared node is spun exclusively via
    rclpy.spin_until_future_complete/spin_once on demand (that is how
    ROSMoveItMotionBackend/ROSGripperBackend spin it internally for their
    own service/action calls), and a second, continuously-spinning
    background thread on that SAME node would race it. rg2_effort_sweep.py
    hits the identical constraint and solves it the identical way (its
    TFReader/EffortLogger each get their own background-spun node).
  - grasp_center_offset.orientation semantics: CHANGED from the previous
    version of this script (which composed offset.orientation against the
    object's live TF orientation, matching executor.yaml's historical
    convention) to reading it as the DESIRED WORLD orientation of the
    grasp center/TCP directly. Rationale and exact math in
    grasp_center_world_from_object_and_offset's docstring. THIS IS NOT YET
    MIRRORED in motion_mcp_server.py's compute_grasp_approach_poses /
    execute_approach_and_preposition -- those tools (what the executor LLM
    actually calls at runtime) still compose orientation against the
    object frame as of this writing. Until that's changed too, a
    grasp_center_offset validated here and pasted into an executor.yaml
    will be interpreted DIFFERENTLY by the live agent stack unless the
    object's orientation is identity at grasp time (true for every task in
    the registry today, not guaranteed for a reorient/flip step). Treat
    this script's output as provisional until motion_mcp_server.py is
    updated to match -- do not paste a non-identity-derived offset into a
    real executor.yaml before that lands.
  - Approach/pre-place offset direction: both this script's
    pregrasp_center_from_grasp_center and motion_mcp_server._compute_approach_geometry
    compose the offset in the target pose's own LOCAL frame via
    compose_poses (not a hardcoded world axis), which is what makes either
    one work for a side approach/placement, not just top-down. They now
    DIVERGE on sign, deliberately: motion_mcp_server.py still forces the
    offset negative (assumes local +Z always points into the object).
    This script's version takes approach_distance_m exactly as given,
    positive or negative your choice, since that assumption breaks the
    moment someone picks a grasp/place orientation where local +Z points
    away from the surface instead (e.g. identity) -- see
    pregrasp_center_from_grasp_center's docstring. Both still produce
    identical results for every grasp orientation in this repo's registry
    today (all top-down, qx=1,qy=0,qz=0,qw=0) PROVIDED approach_distance_m
    is given negative here to match motion_mcp_server.py's forced sign.
  - Live gripper width: rg2_controller.py continuously publishes the
    measured true opening width on /gripper_width_true (std_msgs/Float64),
    computed from the TF separation of left_contact_frame/right_contact_frame
    (DEFAULT_LEFT_CONTACT_FRAME / DEFAULT_RIGHT_CONTACT_FRAME in that file).
    Nothing in robot_backend currently subscribes to it. Motion steps that
    plan for an ALREADY-established grasp (a post-grasp move via
    move_grasp_center_to_pose frame=delta_from_last) read this topic for the
    width used in tcp_target_from_grasp_center,
    instead of trusting the originally-COMMANDED width -- the true opening
    can differ from the commanded target by the tolerance rg2_effort_sweep.py
    exists to characterize, and using the stale commanded value would offset
    the TCP target by exactly that error. Steps that have not grasped
    anything yet (approach_and_preposition, grasp itself) still use the
    plan's specified/target gripper_width_m, since there is no "current"
    grasp width to read before contact.

PLAN FILE FORMAT
-----------------
object_id: u_green                  # default object for steps that omit it
gripper_width_m: 0.042              # default width for steps that omit it
approach_distance_m: -0.10          # default approach standoff -- SIGNED, taken as
                                     # given along the target's own local Z (see
                                     # pregrasp_center_from_grasp_center). Negative is
                                     # correct for the top-down orientation below
                                     # (local +Z points into the object); flip the
                                     # sign if you pick an orientation where local +Z
                                     # points away from the surface instead.

steps:
  - step_id: approach_and_grasp
    action: approach_and_preposition
    grasp_center_offset:
      position: {x: 0.049, y: -0.040, z: 0.0}   # OBJECT frame -- a point on the object
      orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}   # DESIRED WORLD orientation of the
                                                        # grasp center/TCP -- NOT composed
                                                        # against the object's orientation.
                                                        # {1,0,0,0} = top-down regardless of
                                                        # how the object is currently sitting.
    stop_at_pregrasp: false

  - step_id: grasp
    action: grasp
    # gripper_width_m defaults from the plan level (0.042 above)

  - step_id: lift
    action: move_grasp_center_to_pose
    frame: delta_from_last          # object | target_object | world | delta_from_last
    delta: {x: 0.0, y: 0.0, z: 0.12}

  - step_id: place
    action: move_grasp_center_to_pose
    frame: world                    # WORLD frame, absolute -- NOT resolved
                                     # against any object TF. TODO: replace
                                     # with a real target before --execute;
                                     # this placeholder is directly above
                                     # the pick pose, for geometry smoke-
                                     # testing only.
    world_pose:
      position: {x: 0.049, y: -0.040, z: 0.20}
      orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}

  - step_id: release
    action: release
    # open + detach_object + sync_collision_objects via release_skill. Add
    # another move_grasp_center_to_pose step after this if you want a
    # retreat -- no retreat is bundled in.

  - step_id: clear_octomap_after_regrasp
    action: clear_octomap
    # Put this after any grasp of a part that was standing tall/unattached:
    # the octomap still holds the voxels the part itself produced, attaching
    # does not clear them (clearing needs free-space raycasts through space
    # the part now fills), and the next move then fails
    # START_STATE_IN_COLLISION against the part's own stale voxels.

Optional per-step motion tuning (any move step):
    motion:
      solve_ik_first: true            # exact-IK + joint-space plan instead of
                                       # a tolerance-sphere pose goal; use for
                                       # placement steps that must land exactly
      joint_tolerance: 0.0005         # applies on the solve_ik_first path
      position_tolerance_m: 0.0002    # applies on the default pose-goal path
      orientation_tolerance_rad: 0.002
      planner_id: RRTConnectkConfigDefault   # any config in ompl_planning.yaml
      planner_fallbacks: []           # [] disables the automatic retry with
                                       # alternate planners; omit for defaults
      allowed_planning_time: 20.0
      max_velocity_scaling_factor: 0.2
Keys go straight through to the motion backend; omit the block for defaults.

On a planning failure (PLANNING_FAILED/INVALID_MOTION_PLAN/FAILURE) the
backend retries with alternate planners automatically -- see
DEFAULT_PLANNER_FALLBACKS in ros_moveit_backend/motion.py. It does NOT retry
START_STATE_IN_COLLISION, GOAL_IN_COLLISION, or CONTROL_FAILED: those are
geometry/execution problems no planner change can fix.

RUN (full stack up, workspace sourced, from test/):
    python3 plan_assembly_step.py --plan u_green_pick.plan.yaml
    python3 plan_assembly_step.py --plan u_green_pick.plan.yaml --execute
    python3 plan_assembly_step.py --plan u_green_pick.plan.yaml --execute --yes
    python3 plan_assembly_step.py --plan u_green_pick.plan.yaml --report out.json

    # quick single-shot dry run without a plan file:
    python3 plan_assembly_step.py --object u_green \\
        --offset-pos 0.049,-0.040,0.0 --offset-quat 1,0,0,0 --width 0.042
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path
from typing import Any

import yaml

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Float64

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity

from robot_backend_interfaces.srv import GetObjectPose

from robot_backend.ros_moveit_backend.motion import MoveGroupPlanningError

from robot_backend.registry import load_registry
from robot_backend.utils.geometry import make_pose, compose_poses
from robot_backend.utils.rg2_geometry import tcp_target_from_grasp_center

from robot_backend.skills.motion_skills import (
    move_tcp_to_pose_skill,
    move_grasp_center_to_pose_skill,
    move_to_named_safe_joint_pose_skill,
)
from robot_backend.skills.gripper_skills import (
    open_gripper_skill,
    grasp_skill,
    release_skill,
)
from robot_backend.skills.maintenance_skills import (
    set_permitted_contacts as set_permitted_contacts_skill,
)

IDENTITY_QUAT = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
GRIPPER_WIDTH_TOPIC = "/gripper_width_true"
DEFAULT_REGISTRY_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "model_registry.yaml"
)


# ---------------------------------------------------------------------
# pose helpers
# ---------------------------------------------------------------------

def fmt_pose(pose: dict) -> str:
    p, o = pose["position"], pose["orientation"]
    return (
        f"pos=({p['x']:+.4f}, {p['y']:+.4f}, {p['z']:+.4f})  "
        f"quat=({o['x']:+.4f}, {o['y']:+.4f}, {o['z']:+.4f}, {o['w']:+.4f})"
    )


def offset_pose_from_spec(spec: dict, frame_id: str | None = None) -> dict:
    """spec: {"position": {x,y,z}, "orientation": {x,y,z,w}} -> PoseDict.

    Used only for specs that are ALREADY a final world-frame pose taken
    literally: world_pose. NOT used for grasp_center_offset any more --
    that goes through grasp_center_world_from_object_and_offset instead,
    since its position and orientation resolve against the object pose
    differently (see that function's docstring).

    Callers building a final WORLD-frame pose directly from a spec
    (world_pose) MUST pass frame_id="world" explicitly, or the resulting
    PoseDict has no frame_id at all and move_to_pose's _coerce_pose_dict
    raises "must include a non-empty frame_id" the moment it's actually
    used to plan motion.
    """
    return make_pose(
        position=spec["position"],
        orientation=spec.get("orientation", IDENTITY_QUAT),
        frame_id=frame_id,
    )


def grasp_center_world_from_object_and_offset(object_pose: dict, offset_spec: dict) -> dict:
    """Resolve a grasp_center_offset against the object's LIVE world pose --
    POSITION and ORIENTATION are handled asymmetrically, on purpose.

    POSITION is genuinely object-relative: it names a physical point on the
    object (e.g. "4.9cm along local X, -4cm along local Y from the object's
    origin"), so it has to be rotated+translated through whatever the
    object's CURRENT world orientation happens to be. This is what makes
    the same position offset reusable across a pick/repick even after the
    object has moved or been reoriented.

    ORIENTATION is instead read as the grasp center's DESIRED FINAL WORLD
    orientation, not an object-local rotation to compose with the object's
    live orientation. The mathematically-equivalent alternative --
    grasp_center.orientation = object.orientation (x) offset.orientation --
    would require hand-solving
        offset.orientation = inverse(object.orientation) (x) desired_world
    every time the object's resting orientation changes (e.g. after a
    reorient step earlier in the same plan), just to keep meaning "top
    down". Reading orientation as the desired world orientation directly
    removes that inversion: {1,0,0,0} always means "top-down, jaws flipped
    to match the object's local Y", and {0.7071,0,0,0.7071} always means a
    horizontal approach, regardless of what the object's TF orientation is
    when this runs. This is why orientation, unlike position, does not
    depend on object_pose at all below.

    Corollary you have to hold yourself: position and orientation are now
    chosen independently, so nothing here checks that the named object
    point is actually reachable at the orientation you asked for (e.g. that
    a top-down orientation doesn't drive the fingers through the table for
    an object lying on its side). That's exactly what dry-running this
    script before --execute is for.

    There is no grasp_center TF frame -- tcp is the only physical frame
    that exists. "Grasp center" is a logical point this script computes
    (tcp_target_from_grasp_center, via the width-dependent RG2 calibration
    in robot_backend.utils.rg2_geometry) and never a rotation: every
    calibration entry (tcp_offset_from_width) carries IDENTITY orientation,
    always, at every width. So the orientation returned here IS the TCP's
    own world orientation, exactly, not a separate "grasp-center-frame"
    convention layered on top of it -- for a top-down pose, that's the TCP's
    own +Z pointing down into the object (the object's own frame still has
    +Z pointing up off the table; the TCP is flipped 180 relative to it).
    This is also why pregrasp_center_from_grasp_center's local-Z offset
    below is, byte-for-byte, the same offset applied to the TCP directly.
    """
    position_only_offset = make_pose(
        position=offset_spec["position"],
        orientation=IDENTITY_QUAT,
    )
    composed_position = compose_poses(object_pose, position_only_offset)["position"]
    desired_world_orientation = offset_spec.get("orientation", IDENTITY_QUAT)
    return make_pose(
        position=composed_position,
        orientation=desired_world_orientation,
        frame_id=object_pose.get("frame_id", "world"),
    )


def pregrasp_center_from_grasp_center(grasp_center_pose: dict, approach_distance_m: float) -> dict:
    """Offset from grasp/place center along ITS OWN local Z by approach_distance_m,
    taken exactly as given -- sign included, no implicit flip.

    "Grasp center" here is standing in for the TCP: they always share
    orientation (see grasp_center_world_from_object_and_offset's tail note),
    so this offset is, byte-for-byte, the same as offsetting the TCP pose
    directly along its own local Z.

    T_world_pregrasp = T_world_target * T_target_pregrasp (a pure
    local-frame translation composed via compose_poses, not a hand-rolled
    quaternion rotation and not a world-axis addition) -- this part is what
    makes it correct for any grasp/place orientation, top-down or side,
    with no separate "approach vector" parameter needed.

    Which SIGN backs away from the object depends entirely on which way you
    pointed the grasp/place orientation's own local +Z when you chose it --
    this function has no opinion and does not assume the common convention
    (local +Z pointing into the object/surface, so a NEGATIVE
    approach_distance_m backs away). Get the orientation backwards (e.g.
    identity, which points local +Z away from a table instead of into it)
    and the same positive value that worked for a top-down grasp will drive
    the target straight through the surface instead of hovering above it.
    Dry-run and check the printed pre_grasp/pre_place Z before --execute.
    """
    local_offset = make_pose(
        position={"x": 0.0, "y": 0.0, "z": approach_distance_m},
        orientation=IDENTITY_QUAT,
    )
    return compose_poses(grasp_center_pose, local_offset)


def ros_pose_to_posedict(pose_msg, frame_id: str) -> dict:
    return make_pose(
        position={"x": pose_msg.position.x, "y": pose_msg.position.y, "z": pose_msg.position.z},
        orientation={
            "x": pose_msg.orientation.x,
            "y": pose_msg.orientation.y,
            "z": pose_msg.orientation.z,
            "w": pose_msg.orientation.w,
        },
        frame_id=frame_id,
    )


# ---------------------------------------------------------------------
# TF resolution (thin client of scene_tf_sync_node, no second TF buffer)
# ---------------------------------------------------------------------

class ObjectPoseClient:
    """Shares `node` with the motion/gripper/scene backend (see main()) --
    both are only ever driven by blocking, on-demand spins
    (rclpy.spin_until_future_complete), sequentially from the main thread,
    so one shared node is safe here (unlike GripperWidthReader below)."""

    def __init__(self, node: Node, service_name: str = "/get_object_pose", timeout_sec: float = 5.0):
        self._node = node
        self._client = node.create_client(GetObjectPose, service_name)
        self._timeout = timeout_sec
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"{service_name} not available after {timeout_sec}s -- "
                "is scene_tf_sync_node running in this launch?"
            )

    def get_pose(self, object_id: str) -> dict:
        req = GetObjectPose.Request()
        req.object_id = object_id
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        if future.result() is None:
            raise RuntimeError(f"get_object_pose('{object_id}') timed out")
        resp = future.result()
        if not resp.ok:
            raise RuntimeError(f"get_object_pose('{object_id}') failed: {resp.error}")
        return ros_pose_to_posedict(resp.pose, resp.frame_id)


# ---------------------------------------------------------------------
# Live gripper width (rg2_controller's /gripper_width_true) -- own node,
# background-spun, same pattern as rg2_effort_sweep.py's TFReader/
# EffortLogger and for the same reason: it needs to keep receiving messages
# continuously, which would race the shared node's on-demand
# spin_until_future_complete calls if it lived on that node instead.
# ---------------------------------------------------------------------

class GripperWidthReader:
    def __init__(self, topic: str = GRIPPER_WIDTH_TOPIC):
        self.node = Node("assembly_step_planner_width_reader")
        self._lock = threading.Lock()
        self._latest: float | None = None
        self._sub = self.node.create_subscription(Float64, topic, self._on_msg, 10)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def _on_msg(self, msg: Float64) -> None:
        with self._lock:
            self._latest = float(msg.data)

    def get_width(self, timeout_sec: float = 1.5) -> float | None:
        """Latest published width, or None if nothing has arrived within timeout_sec."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest is not None:
                    return self._latest
            time.sleep(0.02)
        with self._lock:
            return self._latest

    def shutdown(self) -> None:
        self.executor.shutdown()
        self._thread.join(timeout=2.0)
        self.node.destroy_node()


# ---------------------------------------------------------------------
# registry loading (grasp_skill needs spec.grasp.force_threshold_n)
# ---------------------------------------------------------------------

def load_object_registry(explicit_path: str | None = None) -> dict[str, Any]:
    if explicit_path:
        return load_registry(Path(explicit_path)).raw_dict
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("robot_backend")) / "config" / "model_registry.yaml"
    except Exception:
        path = DEFAULT_REGISTRY_PATH
    return load_registry(path).raw_dict


# ---------------------------------------------------------------------
# plan loading
# ---------------------------------------------------------------------

def load_plan(args) -> dict:
    if args.plan:
        with open(args.plan) as fh:
            plan = yaml.safe_load(fh)
        if "steps" not in plan or not plan["steps"]:
            raise ValueError(f"{args.plan}: no steps: block found")
        return plan

    # quick single-shot mode from CLI flags
    if not args.object:
        raise ValueError("either --plan or --object is required")
    ox, oy, oz = (float(v) for v in args.offset_pos.split(","))
    qx, qy, qz, qw = (float(v) for v in args.offset_quat.split(","))
    return {
        "object_id": args.object,
        "gripper_width_m": args.width,
        "approach_distance_m": args.approach_distance,
        "steps": [
            {
                "step_id": "adhoc_approach_and_grasp",
                "action": "approach_and_preposition",
                "grasp_center_offset": {
                    "position": {"x": ox, "y": oy, "z": oz},
                    "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                },
                "stop_at_pregrasp": args.stop_at_pregrasp,
            }
        ],
    }


def plan_object_ids(plan: dict) -> list[str]:
    """Every object_id this plan touches: the plan-level default plus any
    step-level override, across every action that carries one.

    Used to sync all of them into MoveIt's world collision set once, up
    front -- attach_object requires an object to already be a world
    CollisionObject before it can attach it (see grasp_skill's
    backend.scene.attach_object call), and this script has no other point
    where that sync would happen otherwise.
    """
    ids: set[str] = set()
    if plan.get("object_id"):
        ids.add(plan["object_id"])
    for step in plan.get("steps", []):
        if step.get("object_id"):
            ids.add(step["object_id"])
    return sorted(ids)


def resolve(step: dict, plan: dict, key: str, required: bool = False, default: Any = None):
    val = step.get(key, plan.get(key, default))
    if required and val is None:
        raise ValueError(f"step '{step.get('step_id', '?')}': '{key}' is required and has no default")
    return val


def resolve_width(
    step: dict,
    plan: dict,
    state: dict,
    width_reader: "GripperWidthReader | None",
    *,
    prefer_live: bool,
    execute: bool,
) -> tuple[float, str]:
    """Resolve the gripper width to use for a TCP-offset computation.

    prefer_live steps (post-grasp motion: a delta_from_last move) read the
    ACTUAL current opening from /gripper_width_true when available -- the
    true width can differ from
    whatever was originally commanded by exactly the tolerance
    rg2_effort_sweep.py exists to characterize, and using the stale
    commanded value would offset the TCP target by that error. Steps before
    any grasp has happened (approach_and_preposition, grasp) always use the
    plan's specified/target width instead, since there is no "current"
    grasp width yet to read.

    Falls back to the plan's specified/tracked width if the topic hasn't
    published anything within the timeout (e.g. sim not up yet, or a pure
    dry run with no controller running at all) -- this script must still be
    usable for offline geometry review, not only against a live cell.

    Returns (width_m, source) where source is "live" or "specified", for
    the caller to print.
    """
    specified = resolve(step, plan, "gripper_width_m", default=state.get("last_width"))

    if prefer_live and width_reader is not None:
        live = width_reader.get_width(timeout_sec=1.5 if execute else 0.5)
        if live is not None:
            return live, "live"

    if specified is None:
        raise ValueError(
            f"step '{step.get('step_id', '?')}': no gripper_width_m specified "
            "and no live width available -- pass gripper_width_m explicitly."
        )
    return float(specified), "specified"


# ---------------------------------------------------------------------
# per-step execution
# ---------------------------------------------------------------------

def run_step(
    step: dict,
    plan: dict,
    pose_client: ObjectPoseClient,
    registry: dict[str, Any],
    width_reader: "GripperWidthReader | None",
    backend,
    state: dict,
    execute: bool,
    settle_sec: float = 0.0,
) -> dict:
    step_id = step.get("step_id", "?")
    action = step["action"]
    print(f"\n--- step '{step_id}'  action={action} ---")

    result: dict = {"step_id": step_id, "action": action}

    # Optional per-step motion tuning, passed straight through to the motion
    # backend. Anything move_to_pose/move_to_joint_positions accepts as a
    # kwarg works here -- planner_id, solve_ik_first, position_tolerance_m,
    # orientation_tolerance_rad, joint_tolerance, allowed_planning_time,
    # max_velocity_scaling_factor. Absent means the backend defaults apply.
    motion_kwargs = step.get("motion", {}) or {}
    if motion_kwargs:
        print(f"    motion overrides: {motion_kwargs}")

    if action == "sync_scene":
        # Explicit, plan-authored list -- deliberately NOT defaulting to
        # plan_object_ids(plan) (the set of objects this plan actually
        # manipulates). A static object this plan never touches still needs
        # to be in MoveIt's world scene at its actual TF pose, or the planner
        # either can't see it (never registered) or plans against wherever
        # it was last synced (registered but stale) -- both look identical
        # to a hard/failing planning problem from the outside. See
        # model_registry.yaml's acm_allowed_object_pairs for the mutual-
        # contact side of sharing a crowded table with these.
        object_ids = step.get("object_ids") or plan_object_ids(plan)
        print(f"    (sync_collision_objects for {object_ids})")
        result.update(object_ids=object_ids)

        if execute:
            def _do_sync():
                return backend.scene.sync_collision_objects(object_ids)
            outcome = confirm_and_run(step_id, _do_sync, settle_sec=settle_sec)
            result["sync_result"] = outcome

    elif action == "set_permitted_contacts":
        # sync_collision_objects (above) and this are two SEPARATE MoveIt
        # mechanisms -- syncing pushes CollisionObject geometry/pose into
        # the planning scene; it never touches the Allowed Collision Matrix.
        # acm_allowed_object_pairs in the registry is just declared data
        # until this actively pushes it into MoveIt. Reuses the same skill
        # (and therefore the same object_id validation) the executor's
        # set_permitted_contacts MCP tool calls -- see executor_system.txt's
        # "Intended contact between objects" section.
        pairs = registry.get("acm_allowed_object_pairs") or []
        print(f"    (set_permitted_contacts for {pairs})")
        result.update(acm_allowed_object_pairs=pairs)

        if execute:
            def _do_set_permitted_contacts():
                return set_permitted_contacts_skill(backend, registry=registry)
            outcome = confirm_and_run(step_id, _do_set_permitted_contacts, settle_sec=settle_sec)
            result["set_permitted_contacts_result"] = outcome

    elif action == "approach_and_preposition":
        object_id = resolve(step, plan, "object_id", required=True)
        width = resolve(step, plan, "gripper_width_m", required=True)
        approach_distance_m = resolve(step, plan, "approach_distance_m", default=0.10)
        stop_at_pregrasp = step.get("stop_at_pregrasp", False)

        object_pose = pose_client.get_pose(object_id)
        grasp_center_world = grasp_center_world_from_object_and_offset(object_pose, step["grasp_center_offset"])
        pre_grasp_center_world = pregrasp_center_from_grasp_center(grasp_center_world, approach_distance_m)

        pre_grasp_tcp = tcp_target_from_grasp_center(pre_grasp_center_world, width)
        grasp_tcp = tcp_target_from_grasp_center(grasp_center_world, width)

        offset_pos = step["grasp_center_offset"]["position"]
        offset_ori = step["grasp_center_offset"].get("orientation", IDENTITY_QUAT)
        print(f"    object '{object_id}' world pose      : {fmt_pose(object_pose)}")
        print(f"    grasp point (OBJECT-frame position)  : ({offset_pos['x']:+.4f}, {offset_pos['y']:+.4f}, {offset_pos['z']:+.4f})")
        print(f"    grasp orientation (DESIRED WORLD)     : ({offset_ori['x']:+.4f}, {offset_ori['y']:+.4f}, {offset_ori['z']:+.4f}, {offset_ori['w']:+.4f})")
        print(f"    grasp_center (world)             : {fmt_pose(grasp_center_world)}")
        print(f"    pre_grasp_center (world, {approach_distance_m:+.3f}m local Z): {fmt_pose(pre_grasp_center_world)}")
        print(f"    pre_grasp TCP                    : {fmt_pose(pre_grasp_tcp)}")
        print(f"    grasp TCP                        : {fmt_pose(grasp_tcp)}")

        state["last_grasp_center_world"] = grasp_center_world
        state["last_object_id"] = object_id
        state["last_width"] = width

        result.update(
            object_id=object_id,
            gripper_width_m=width,
            approach_distance_m=approach_distance_m,
            grasp_center_offset=step["grasp_center_offset"],
            grasp_center_world=grasp_center_world,
            pre_grasp_tcp=pre_grasp_tcp,
            grasp_tcp=grasp_tcp,
            structured=True,  # eligible for the pasteable steps: fragment
        )

        if execute:
            confirm_and_run(
                step_id,
                lambda: move_tcp_to_pose_skill(backend, pre_grasp_tcp, **motion_kwargs),
                settle_sec=settle_sec,
            )
            if not stop_at_pregrasp:
                confirm_and_run(
                    step_id,
                    lambda: move_tcp_to_pose_skill(backend, grasp_tcp, **motion_kwargs),
                    settle_sec=settle_sec,
                )

    elif action == "grasp":
        object_id = resolve(step, plan, "object_id", required=True)
        width = resolve(step, plan, "gripper_width_m", required=True)

        print(f"    (grasp '{object_id}', expected_width_m={width:.4f} -- "
              f"torque-guarded close via grasp_skill, attaches on confirmed contact)")
        print("    (reminder: run rg2_effort_sweep.py separately to characterize "
              "max_clamp_effort for this object/clearance)")

        state["last_width"] = width
        result.update(object_id=object_id, gripper_width_m=width)

        if execute:
            def _do_grasp():
                return grasp_skill(backend, object_id=object_id, registry=registry, expected_width_m=width)
            outcome = confirm_and_run(step_id, _do_grasp, settle_sec=settle_sec)
            result["grasp_result"] = outcome
            if not outcome.get("ok", False):
                # grasp_skill returns early on a failed close/contact check
                # WITHOUT ever calling attach_object (see its "Success
                # criteria" block) -- the object is still a WORLD collision
                # object, untouched by touch_links, while the gripper sits
                # closed right where it is. Continuing the plan from here
                # doesn't fail cleanly: the NEXT motion step hits
                # START_STATE_IN_COLLISION against the un-attached object,
                # which reads as a confusing planning/collision bug two
                # steps removed from the actual failure. Stop here instead,
                # at the step that actually failed and with the actual
                # reason (see outcome['reason']).
                raise RuntimeError(
                    f"step '{step_id}': grasp failed, not attached -- "
                    f"{outcome.get('reason', outcome)}. Stopping rather than "
                    "continuing with an ungrasped, un-attached object."
                )

    elif action == "move_grasp_center_to_pose":
        frame = step.get("frame", "object")
        prefer_live_width = frame == "delta_from_last"
        width, width_source = resolve_width(
            step, plan, state, width_reader, prefer_live=prefer_live_width, execute=execute
        )
        print(f"    gripper_width_m used: {width:.4f} (source: {width_source})")

        if frame == "object":
            object_id = resolve(step, plan, "object_id", required=True)
            object_pose = pose_client.get_pose(object_id)
            target_center_world = grasp_center_world_from_object_and_offset(object_pose, step["grasp_center_offset"])
            print(f"    object '{object_id}' world pose : {fmt_pose(object_pose)}")
            result.update(object_id=object_id, grasp_center_offset=step["grasp_center_offset"], structured=True)

        elif frame == "world":
            target_center_world = offset_pose_from_spec(step["world_pose"], frame_id="world")
            result.update(structured=False)

        elif frame == "target_object":
            # Same grasp_center_offset rule as frame: object -- position is
            # object-relative and gets rotated through the object's
            # orientation, orientation is the desired WORLD orientation taken
            # directly. The only difference is where the object pose comes
            # from: given here as the object's DESIRED pose (e.g. off an
            # Isaac Sim gizmo) rather than TF-queried, since the object is
            # attached and hasn't physically gotten there yet.
            #
            # Because the object is attached, the orientation you give is not
            # actually free: the object->gripper transform is fixed at grasp
            # time, so an orientation inconsistent with it means the object
            # lands somewhere other than object_pose. Dry-run and check.
            object_id = resolve(step, plan, "object_id", required=True)
            object_target_pose = offset_pose_from_spec(step["object_pose"], frame_id="world")
            target_center_world = grasp_center_world_from_object_and_offset(
                object_target_pose, step["grasp_center_offset"]
            )
            print(f"    object target pose (world, NOT queried) : {fmt_pose(object_target_pose)}")
            result.update(
                object_id=object_id, grasp_center_offset=step["grasp_center_offset"], structured=True
            )

        elif frame == "delta_from_last":
            if "last_grasp_center_world" not in state:
                raise ValueError(
                    f"step '{step_id}': frame=delta_from_last but no prior step "
                    "computed a grasp center yet"
                )
            base = state["last_grasp_center_world"]
            d = step["delta"]
            delta_pose = make_pose(
                position={"x": d.get("x", 0.0), "y": d.get("y", 0.0), "z": d.get("z", 0.0)},
                orientation=IDENTITY_QUAT,
            )
            # world-frame delta: translate the position directly, keep
            # orientation AND frame_id -- dropping frame_id here (as the
            # previous version of this branch did) produces a PoseDict
            # move_to_pose's _coerce_pose_dict rejects the moment it's
            # actually used to plan motion ("must include a non-empty
            # frame_id"), since base_pose.get("frame_id") is never queried
            # for a bare make_pose call the way compose_poses does it.
            target_center_world = make_pose(
                position={
                    "x": base["position"]["x"] + delta_pose["position"]["x"],
                    "y": base["position"]["y"] + delta_pose["position"]["y"],
                    "z": base["position"]["z"] + delta_pose["position"]["z"],
                },
                orientation=base["orientation"],
                frame_id=base.get("frame_id", "world"),
            )
            result.update(structured=False)

        else:
            raise ValueError(f"step '{step_id}': unknown frame '{frame}'")

        print(f"    target grasp_center (world) : {fmt_pose(target_center_world)}")
        state["last_grasp_center_world"] = target_center_world
        result.update(grasp_center_world=target_center_world, gripper_width_m=width)

        if execute:
            confirm_and_run(
                step_id,
                lambda: move_grasp_center_to_pose_skill(
                    backend, target_center_world, width, **motion_kwargs
                ),
                settle_sec=settle_sec,
            )

    elif action == "release":
        object_id = resolve(step, plan, "object_id", required=True)

        print(f"    (release '{object_id}' -- open + detach_object + sync_collision_objects "
              "via release_skill)")

        result.update(object_id=object_id)

        if execute:
            def _do_release():
                return release_skill(backend, object_id=object_id)
            outcome = confirm_and_run(step_id, _do_release, settle_sec=settle_sec)
            result["release_result"] = outcome

    elif action == "clear_octomap":
        # Drops all perception-derived occupancy. Useful right after a grasp:
        # the part's own voxels, marked while it sat there unattached, do not
        # clear the instant it is attached (clearing needs free-space
        # raycasts through them), so the freshly attached collision body can
        # overlap its own stale voxels.
        print("    (clear_octomap -- drop perception occupancy)")

        if execute:
            outcome = confirm_and_run(step_id, backend.scene.clear_octomap, settle_sec=settle_sec)
            result["clear_octomap_result"] = outcome

    elif action == "open_gripper":
        print("    (open gripper -- no attach/detach implied; use 'grasp'/'release' "
              "for a real pick/place)")
        if execute:
            confirm_and_run(step_id, lambda: open_gripper_skill(backend), settle_sec=settle_sec)

    elif action == "move_named_safe_pose":
        name = step["name"]
        print(f"    (move to named safe pose '{name}')")
        if execute:
            confirm_and_run(step_id, lambda: move_to_named_safe_joint_pose_skill(backend, name), settle_sec=settle_sec)

    else:
        raise ValueError(f"step '{step_id}': unknown action '{action}'")

    return result


def confirm_and_run(step_id: str, fn, settle_sec: float = 0.0):
    outcome = fn()
    print(f"    -> executed, result: {outcome}")
    if settle_sec > 0:
        # Per-ACTION settle, not just per-step: a step like
        # approach_and_preposition issues TWO motion commands back to back
        # (pre-grasp hover, then descend) with nothing between them, and
        # grasp/release each chain 2-3 backend calls (open, guarded-move,
        # attach / open, detach, sync) the same way. Only settling once at
        # the end of the whole step (the old behavior) left every one of
        # those intermediate transitions racing MoveIt's scene/octomap
        # update -- exactly the "runs too fast" symptom.
        print(f"    (settling {settle_sec:.1f}s...)")
        time.sleep(settle_sec)
    return outcome


# ---------------------------------------------------------------------
# output: pasteable executor.yaml steps: fragment
# ---------------------------------------------------------------------

def render_steps_fragment(results: list[dict]) -> str:
    """Pick steps (grasp_center_offset): position matches today's real
    executor.yaml convention exactly (object-frame point). ORIENTATION DOES
    NOT, as of this script version -- it is emitted here as the desired
    WORLD orientation (see grasp_center_world_from_object_and_offset), not
    composed against the object frame the way existing executor.yaml files
    and motion_mcp_server.py's tools still expect. Numerically identical
    for every task in the registry today (all authored at object-identity
    orientation) but DO NOT paste an orientation derived at a non-identity
    object pose into a real executor.yaml until motion_mcp_server.py is
    updated to match this convention -- see the ASSUMPTIONS header.

    World-frame moves (move_grasp_center_to_pose with frame: world or
    delta_from_last) are never eligible here -- only object-relative
    grasp_center_offset steps are, matching what a real executor.yaml's
    steps: block actually stores today (see e.g.
    two_cube_stack.executor.yaml: only its pick step is structured, its
    place step is prose-only).
    """
    entries = []
    for r in results:
        if not r.get("structured"):
            continue

        if "grasp_center_offset" in r:
            offset = r["grasp_center_offset"]
            entries.append(
                {
                    "step_id": r["step_id"],
                    "object_id": r["object_id"],
                    "grasp_width_m": r["gripper_width_m"],
                    "grasp_center_offset": {
                        "position": {k: round(v, 6) for k, v in offset["position"].items()},
                        "orientation": {k: round(v, 6) for k, v in offset.get("orientation", IDENTITY_QUAT).items()},
                    },
                }
            )

    if not entries:
        return "(no structured steps in this plan to paste)"
    return yaml.safe_dump({"steps": entries}, sort_keys=False, default_flow_style=False)


# ---------------------------------------------------------------------
# main
# ---------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--plan", type=str, default=None, help="path to a plan YAML file")
    p.add_argument("--object", type=str, default=None, help="ad hoc mode: object_id")
    p.add_argument("--offset-pos", type=str, default="0,0,0", help="ad hoc mode: x,y,z, a point on the object in the OBJECT frame")
    p.add_argument("--offset-quat", type=str, default="0,0,0,1", help="ad hoc mode: qx,qy,qz,qw, the DESIRED WORLD orientation of the grasp center/TCP (not object-frame)")
    p.add_argument("--width", type=float, default=None, help="ad hoc mode: gripper_width_m")
    p.add_argument("--approach-distance", type=float, default=0.10, help="ad hoc mode: approach_distance_m")
    p.add_argument("--stop-at-pregrasp", action="store_true", help="ad hoc mode: stop after pre-grasp waypoint")
    p.add_argument("--execute", action="store_true", help="actually move the robot (default: dry run only)")
    p.add_argument("--yes", action="store_true", help="skip per-step confirmation when --execute is set")
    p.add_argument("--report", type=str, default=None, help="write computed poses for every step to this JSON path")
    p.add_argument("--service", type=str, default="/get_object_pose", help="override the get-object-pose service name")
    p.add_argument("--registry", type=str, default=None, help="override the model_registry.yaml path")
    p.add_argument(
        "--settle-sec", type=float, default=2.5,
        help="seconds to sleep after each executed ACTION -- not just each step, but "
             "every individual backend call within one (default 2.5, --execute only): "
             "approach_and_preposition's pre-grasp-then-descend, grasp's "
             "open/guarded-move/attach, release's open/detach/sync all settle "
             "between their own sub-calls now, not just once at the end of the "
             "step. Gives MoveIt's point-cloud-derived octomap time to catch up "
             "with the scene (e.g. an object that just moved or was "
             "attached/detached) before the next command plans against it. A "
             "planner that plans against a stale octomap will route around "
             "obstacles that have already moved, or fail entirely against ones "
             "that haven't been cleared yet.",
    )
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    plan = load_plan(args)
    registry = load_object_registry(args.registry)

    rclpy.init()
    node = Node("assembly_step_planner")
    backend = None
    width_reader = None
    try:
        # One shared node for both the backend and the object-pose client --
        # see ASSUMPTIONS header for why the gripper-width reader can't join
        # this one.
        from robot_backend.ros_moveit_backend import create_ros_moveit_backend
        pose_client = ObjectPoseClient(node, service_name=args.service)
        backend = create_ros_moveit_backend(node=node)
        width_reader = GripperWidthReader()

        # No implicit sync here -- the plan owns this explicitly via its own
        # sync_scene / set_permitted_contacts steps (see those actions).
        # An earlier version auto-synced plan_object_ids(plan) (the object_id
        # this plan manipulates) before the step loop; that's exactly the
        # NARROW set sync_scene was added to stop being limited to -- static
        # objects the plan never directly targets (other parts sharing the
        # table) need syncing too, and plan_object_ids has no way to know
        # about them. Put a sync_scene step first in your plan.

        state: dict = {}
        results = []
        for step in plan["steps"]:
            if args.execute and not args.yes:
                ans = input(f"\nrun step '{step.get('step_id', '?')}' ({step['action']})? [y/N] ").strip().lower()
                if ans != "y":
                    print("    skipped")
                    continue
            try:
                result = run_step(
                    step, plan, pose_client, registry, width_reader, backend, state,
                    execute=args.execute,
                    settle_sec=args.settle_sec if args.execute else 0.0,
                )
            except MoveGroupPlanningError as exc:
                # The arm has not moved for a start-state rejection, so the
                # colliding configuration is still live and can be queried.
                print(f"\n!! step '{step.get('step_id', '?')}' failed: {exc}")
                if exc.error_code_val == MoveItErrorCodes.START_STATE_IN_COLLISION:
                    report_current_state_collisions(node)
                raise
            results.append(result)

        print("\n\n===== paste into executor.yaml steps: =====\n")
        print(render_steps_fragment(results))

        if args.report:
            with open(args.report, "w") as fh:
                json.dump(_json_safe(results), fh, indent=2)
            print(f"\nwrote {len(results)} step results to {args.report}")

        return 0

    finally:
        if backend is not None:
            try:
                backend.gripper.open()
            except Exception:
                pass
        if width_reader is not None:
            width_reader.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


_BODY_TYPE_NAMES = {0: "ROBOT_LINK", 1: "WORLD_OBJECT", 2: "ROBOT_ATTACHED"}


def report_current_state_collisions(node, planning_group: str = "ur_manipulator") -> None:
    """Ask MoveIt what is colliding RIGHT NOW and print the pairs.

    START_STATE_IN_COLLISION says only that the start state is invalid, never
    which bodies touch. /check_state_validity with an empty robot_state
    evaluates the current state and returns named contacts, which is the
    difference between a guess and knowing. Best effort: never re-raises, so
    it cannot mask the original planning error.
    """

    try:
        client = node.create_client(GetStateValidity, "/check_state_validity")
        if not client.wait_for_service(timeout_sec=5.0):
            print("    [collision detail] /check_state_validity unavailable")
            return
        request = GetStateValidity.Request()
        request.group_name = planning_group
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        if response is None:
            print("    [collision detail] no response from /check_state_validity")
            return

        print(f"    [collision detail] valid={response.valid} "
              f"contacts={len(response.contacts)}")
        if not response.contacts:
            print("      (no contacts reported -- the colliding state may already "
                  "have been cleared, or the collision is with the octomap, which "
                  "some MoveIt versions report without naming)")
        for contact in response.contacts:
            t1 = _BODY_TYPE_NAMES.get(contact.body_type_1, contact.body_type_1)
            t2 = _BODY_TYPE_NAMES.get(contact.body_type_2, contact.body_type_2)
            print(f"      {contact.contact_body_1} [{t1}]  <->  "
                  f"{contact.contact_body_2} [{t2}]   depth={contact.depth:.5f} m")
    except Exception as exc:
        print(f"    [collision detail] query failed: {exc}")


def _json_safe(obj):
    """Drop non-JSON-serializable fields (e.g. anything holding a live handle)."""
    return json.loads(json.dumps(obj, default=lambda o: str(o)))


if __name__ == "__main__":
    sys.exit(main())