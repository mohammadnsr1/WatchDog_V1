"""Grasp-centre offset plumbing for the motion server -- pure geometry.

The object-side offset (grasp_center_offset, T_object_grasp_center, sourced
from the task instruction's steps: block -- not the model registry) is
composed with the object pose to yield the grasp centre, exactly as
motion_mcp_server._compute_approach_geometry does when any offset component is
supplied. These tests exercise that composition through the ROS-free geometry
helpers the server delegates to, so they run with no cell and no MoveGroup.

A second, import-guarded block exercises the server function itself where the
full ROS overlay is importable, confirming the offset/no-offset branch and its
defaults_used marker.

Run with: pytest test/test_motion_geometry.py -v
"""

from __future__ import annotations

import pytest

from robot_backend.utils.geometry import compose_poses, make_pose


TOP_DOWN = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}  # 180 deg about x: z -> -z
IDENTITY = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def _object_pose(pos, orient):
    return make_pose(position=pos, orientation=orient, frame_id="world")


def test_identity_offset_leaves_grasp_centre_at_object_position():
    obj = _object_pose({"x": 0.5, "y": 0.2, "z": 0.1}, TOP_DOWN)
    offset = make_pose(position={"x": 0.0, "y": 0.0, "z": 0.0}, orientation=IDENTITY)
    gc = compose_poses(obj, offset)
    assert gc["position"]["x"] == pytest.approx(0.5)
    assert gc["position"]["y"] == pytest.approx(0.2)
    assert gc["position"]["z"] == pytest.approx(0.1)
    # Orientation unchanged (composed with identity).
    for axis in ("x", "y", "z", "w"):
        assert gc["orientation"][axis] == pytest.approx(TOP_DOWN[axis])


def test_translation_only_offset_with_identity_orientation():
    # Object at identity orientation: the offset applies as a pure translation.
    obj = _object_pose({"x": 0.5, "y": 0.2, "z": 0.1}, IDENTITY)
    offset = make_pose(
        position={"x": 0.01, "y": -0.02, "z": 0.03}, orientation=IDENTITY
    )
    gc = compose_poses(obj, offset)
    assert gc["position"]["x"] == pytest.approx(0.51)
    assert gc["position"]["y"] == pytest.approx(0.18)
    assert gc["position"]["z"] == pytest.approx(0.13)


def test_offset_is_expressed_in_object_frame_not_world():
    # With a top-down object orientation (180 deg about x), an object-frame +z
    # offset points along world -z: the grasp centre sits BELOW the object,
    # which is exactly why the offset must be composed in the object frame
    # rather than added in world coordinates.
    obj = _object_pose({"x": 0.5, "y": 0.2, "z": 0.1}, TOP_DOWN)
    offset = make_pose(position={"x": 0.0, "y": 0.0, "z": 0.03}, orientation=IDENTITY)
    gc = compose_poses(obj, offset)
    assert gc["position"]["x"] == pytest.approx(0.5)
    assert gc["position"]["y"] == pytest.approx(0.2)
    assert gc["position"]["z"] == pytest.approx(0.07)


def test_frame_id_preserved_through_composition():
    obj = _object_pose({"x": 0.0, "y": 0.0, "z": 0.0}, IDENTITY)
    offset = make_pose(position={"x": 0.1, "y": 0.0, "z": 0.0}, orientation=IDENTITY)
    gc = compose_poses(obj, offset)
    assert gc["frame_id"] == "world"


# --------------------------------------------------------------------------
# Server-branch test: only runs where the full ROS overlay is importable
# (rclpy + the isaac interfaces the motion backend pulls in). Skipped
# otherwise -- the pure-geometry tests above cover the composition itself.
# --------------------------------------------------------------------------

def _import_motion_server():
    # motion_mcp_server calls ensure_mcp_server_environment() at import time,
    # which os.execve()s the process into the workspace venv when the flag
    # below is unset -- that would replace the running pytest process, not
    # raise. Setting the flag makes it a no-op so the import either succeeds
    # (full ROS overlay present) or raises a catchable ModuleNotFoundError.
    import os

    os.environ.setdefault("_ROBOT_BACKEND_MCP_VENV_READY", "1")
    try:
        from robot_backend.mcp_servers import motion_mcp_server
    except Exception as exc:  # ROS overlay not sourced in this environment
        pytest.skip(f"motion_mcp_server not importable here: {exc}")
    return motion_mcp_server


def test_compute_geometry_omitting_offset_uses_pose_directly():
    m = _import_motion_server()
    res = m._compute_approach_geometry(
        frame_id="world", x=0.5, y=0.2, z=0.1,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        gripper_width_m=0.04, approach_distance_m=-0.05,
    )
    grasp_center_pose, _, _, _, _, _, defaults_used = res
    assert grasp_center_pose["position"] == {"x": 0.5, "y": 0.2, "z": 0.1}
    assert "grasp_center_offset" not in defaults_used


def test_compute_geometry_with_offset_interprets_pose_as_object():
    m = _import_motion_server()
    res = m._compute_approach_geometry(
        frame_id="world", x=0.5, y=0.2, z=0.1,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        gripper_width_m=0.04, approach_distance_m=-0.05,
        grasp_center_offset_z=0.03,
        # Orientation is now required whenever an offset position is given:
        # it is the DESIRED WORLD orientation and has no safe default.
        grasp_center_offset_qx=1.0, grasp_center_offset_qy=0.0,
        grasp_center_offset_qz=0.0, grasp_center_offset_qw=0.0,
    )
    grasp_center_pose, _, _, _, _, _, defaults_used = res
    # Object-frame +z offset under a top-down object orientation -> world -z.
    assert grasp_center_pose["position"]["x"] == pytest.approx(0.5)
    assert grasp_center_pose["position"]["y"] == pytest.approx(0.2)
    assert grasp_center_pose["position"]["z"] == pytest.approx(0.07)
    assert "grasp_center_offset" in defaults_used


def test_approach_distance_sign_is_taken_as_given():
    """approach_distance_m is applied EXACTLY as given along local Z.

    Regression guard for the convention change: motion_mcp_server used to
    reject negatives and force the offset to -dist. Nothing covered the
    resulting pre-grasp direction, so the flip could have gone unnoticed.
    Object orientation here is top-down (qx=1), whose local +Z points at
    world -Z, so a NEGATIVE distance must stage the pre-grasp ABOVE.
    """
    m = _import_motion_server()

    def pre_grasp_z(dist):
        res = m._compute_approach_geometry(
            frame_id="world", x=0.5, y=0.2, z=0.1,
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
            gripper_width_m=0.04, approach_distance_m=dist,
        )
        grasp_center_pose, pre_grasp_tcp_pose, grasp_tcp_pose, _, eff, _, _ = res
        return grasp_center_pose, pre_grasp_tcp_pose, grasp_tcp_pose, eff

    gc, pre, grasp, eff = pre_grasp_z(-0.05)
    assert eff == pytest.approx(-0.05), "distance must not be re-signed"
    # top-down: local +Z is world -Z, so negative distance backs UP
    assert pre["position"]["z"] > grasp["position"]["z"]
    assert pre["position"]["z"] - grasp["position"]["z"] == pytest.approx(0.05)

    # and a positive distance must go the other way, not be rejected
    gc2, pre2, grasp2, eff2 = pre_grasp_z(+0.05)
    assert eff2 == pytest.approx(+0.05)
    assert pre2["position"]["z"] < grasp2["position"]["z"]


def test_grasp_center_orientation_is_world_not_composed():
    """grasp_center_offset.orientation is the DESIRED WORLD orientation.

    Regression guard for the other half of the convention change. With the
    object NOT at identity, the old composed form returned a different
    orientation; the shared helper must return the offset's orientation
    verbatim.
    """
    m = _import_motion_server()
    # object yawed 180 deg about Z
    res = m._compute_approach_geometry(
        frame_id="world", x=0.5, y=0.2, z=0.1,
        qx=0.0, qy=0.0, qz=1.0, qw=0.0,
        gripper_width_m=0.04, approach_distance_m=-0.05,
        grasp_center_offset_y=0.04,
        grasp_center_offset_qx=1.0, grasp_center_offset_qy=0.0,
        grasp_center_offset_qz=0.0, grasp_center_offset_qw=0.0,
    )
    grasp_center_pose = res[0]
    o = grasp_center_pose["orientation"]
    assert (o["x"], o["y"], o["z"], o["w"]) == pytest.approx((1.0, 0.0, 0.0, 0.0))
    # position still rotates through the object's orientation: +y -> -y
    assert grasp_center_pose["position"]["y"] == pytest.approx(0.2 - 0.04)


def test_offset_without_orientation_is_rejected():
    """An offset position with no orientation must fail loud, not default.

    Under this convention the offset orientation is the grasp centre's
    DESIRED WORLD orientation, so defaulting it to identity silently means
    "align to world axes" -- which for a top-down grasp inverts the approach
    axis and stages the pre-grasp BELOW the object. Identity used to be
    harmless (it meant "inherit the object's orientation"), so this guard is
    what stops a call written against the old convention from quietly
    aiming into the table.
    """
    m = _import_motion_server()
    with pytest.raises(ValueError, match="grasp_center_offset orientation"):
        m._compute_approach_geometry(
            frame_id="world", x=0.5, y=0.2, z=0.1,
            qx=0.0, qy=0.0, qz=0.0, qw=1.0,
            gripper_width_m=0.04, approach_distance_m=-0.05,
            grasp_center_offset_x=0.049, grasp_center_offset_y=-0.040,
        )


def test_no_offset_at_all_still_treats_pose_as_grasp_centre():
    """The guard must not fire when no offset is supplied at all."""
    m = _import_motion_server()
    res = m._compute_approach_geometry(
        frame_id="world", x=0.5, y=0.2, z=0.1,
        qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        gripper_width_m=0.04, approach_distance_m=-0.05,
    )
    assert res[0]["position"] == {"x": 0.5, "y": 0.2, "z": 0.1}
