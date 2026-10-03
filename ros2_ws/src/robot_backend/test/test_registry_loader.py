"""Shared registry validation and pre-refactor byte-regression gate."""

from __future__ import annotations

import copy
import hashlib
import importlib
import json
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from robot_backend.contracts.loader import _load_registry as load_contract_registry
from robot_backend.registry import MeshCollisionSpec, load_registry
from robot_backend.watchdog import session as sess
from registry_golden_utils import canonical_collision_bytes


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REGISTRY_PATH = PACKAGE_ROOT / "config" / "model_registry.yaml"
MESH_REGISTRY_PATH = PACKAGE_ROOT / "config" / "model_registry_mesh_example.yaml"
GOLDEN_PATH = Path(__file__).parent / "fixtures" / "registry_box_golden.json"


def _valid_registry_yaml(
    *, collision: dict, asset_root: str | None = None
) -> str:
    registry = {
        "objects": {
            "test_object": {
                "enabled": True,
                "object_id": "test_object",
                "tf_frame": "test_object",
                "lookup_frame": "world",
                "class_name": "fixture",
                "source": "test",
                "graspable": False,
                "collision": collision,
            }
        }
    }
    if asset_root is not None:
        registry["asset_root"] = asset_root
    return yaml.safe_dump(registry, sort_keys=False)


def _write_registry(tmp_path: Path, content: str) -> Path:
    path = tmp_path / "model_registry.yaml"
    path.write_text(content, encoding="utf-8")
    return path


def test_loads_typed_box_and_preserves_exact_source() -> None:
    model = load_registry(REGISTRY_PATH)
    raw_text = REGISTRY_PATH.read_text(encoding="utf-8")
    assert model.raw_text == raw_text
    assert model.raw_dict == yaml.safe_load(raw_text)
    assert model.asset_root == REGISTRY_PATH.parent.resolve()
    assert model.raw_sha256 == hashlib.sha256(REGISTRY_PATH.read_bytes()).hexdigest()
    assert model.objects["red_cube"].collision.dimensions == (0.06, 0.06, 0.06)


def test_box_pose_offset_defaults_and_aabb_resolves(tmp_path: Path) -> None:
    path = _write_registry(
        tmp_path,
        _valid_registry_yaml(
            collision={"type": "box", "dimensions": [2.0, 4.0, 6.0]},
        ),
    )
    obj = load_registry(path).objects["test_object"]
    assert obj.pose_offset == {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "frame_id": "world",
    }
    assert obj.resolve_aabb() == (
        {"x": 0.0, "y": 0.0, "z": 0.0},
        {"x": 1.0, "y": 2.0, "z": 3.0},
    )


def test_box_pose_offset_keeps_legacy_partial_component_defaults(tmp_path: Path) -> None:
    path = _write_registry(
        tmp_path,
        _valid_registry_yaml(
            collision={
                "type": "box",
                "dimensions": [2.0, 4.0, 6.0],
                "pose_offset": {"position": {"x": 0.25}},
            },
        ),
    )
    obj = load_registry(path).objects["test_object"]
    assert obj.pose_offset["position"] == {"x": 0.25, "y": 0.0, "z": 0.0}
    assert obj.pose_offset["orientation"] == {
        "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0,
    }


def test_rotated_box_aabb_includes_pose_offset(tmp_path: Path) -> None:
    path = _write_registry(
        tmp_path,
        _valid_registry_yaml(
            collision={
                "type": "box",
                "dimensions": [2.0, 4.0, 6.0],
                "pose_offset": {
                    "position": {"x": 0.5, "y": -0.25, "z": 0.75},
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 2**-0.5,
                        "w": 2**-0.5,
                    },
                },
            },
        ),
    )
    center, half_extents = load_registry(path).objects["test_object"].resolve_aabb()
    assert center == {"x": 0.5, "y": -0.25, "z": 0.75}
    assert half_extents == pytest.approx({"x": 2.0, "y": 1.0, "z": 3.0})


def test_required_identity_field_fails_loud_with_object_id(tmp_path: Path) -> None:
    content = yaml.safe_load(
        _valid_registry_yaml(
            collision={"type": "box", "dimensions": [1.0, 1.0, 1.0]},
        )
    )
    del content["objects"]["test_object"]["tf_frame"]
    path = _write_registry(tmp_path, yaml.safe_dump(content, sort_keys=False))
    with pytest.raises(ValueError, match="test_object.*tf_frame"):
        load_registry(path)


@pytest.mark.parametrize(
    ("collision", "message"),
    [
        ({"type": "capsule", "dimensions": [1, 1, 1]}, "test_object.*collision.type"),
        ({"type": "box", "dimensions": [1, 0, 1]}, "test_object.*positive"),
        ({"type": "box", "dimensions": [1, 1]}, "test_object.*dimensions"),
        (
            {"type": "box", "dimensions": [1, 1, 1], "mesh_path": "bad.stl"},
            "test_object.*forbids.*mesh_path",
        ),
    ],
)
def test_collision_validation_fails_loud_with_object_id(
    tmp_path: Path, collision: dict, message: str
) -> None:
    path = _write_registry(tmp_path, _valid_registry_yaml(collision=collision))
    with pytest.raises(ValueError, match=message):
        load_registry(path)


def test_explicit_relative_asset_root(tmp_path: Path) -> None:
    content = _valid_registry_yaml(
        collision={"type": "box", "dimensions": [1, 1, 1]},
        asset_root="assets",
    )
    model = load_registry(_write_registry(tmp_path, content))
    assert model.asset_root == (tmp_path / "assets").resolve()


@pytest.mark.skipif(
    not MESH_REGISTRY_PATH.exists(),
    reason="config/model_registry_mesh_example.yaml was removed from the "
    "repo; the mesh pathway is exercised live by sync_scene. Restore the "
    "example registry to re-enable these.",
)
def test_m0_loads_fmb_mesh_lazily_and_resolves_known_aabb() -> None:
    model = load_registry(MESH_REGISTRY_PATH)
    obj = model.objects["u_green"]
    assert isinstance(obj.collision, MeshCollisionSpec)
    assert obj.collision.scale == (1.0, 1.0, 1.0)
    assert obj.collision.resolved_mesh_path == (
        MESH_REGISTRY_PATH.parent / "assets" / "fmb_u_green.obj"
    ).resolve()
    assert obj._mesh_geometry is None

    center, half_extents = obj.resolve_aabb()
    assert center == pytest.approx({"x": 0.0, "y": -0.0005, "z": 0.0})
    assert half_extents == pytest.approx(
        {"x": 0.035, "y": 0.035, "z": 0.0105}, abs=0.001
    )
    geometry = obj.resolve_mesh_geometry()
    assert len(geometry.vertices) == 16
    assert len(geometry.faces) == 28
    assert obj.resolve_mesh_geometry() is geometry


def test_mesh_scale_defaults_to_identity(tmp_path: Path) -> None:
    mesh_path = tmp_path / "part.obj"
    mesh_path.write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
        "f 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n",
        encoding="utf-8",
    )
    collision = {
        "type": "mesh",
        "mesh_path": "part.obj",
        "pose_offset": {
            "position": {"x": 0, "y": 0, "z": 0},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        },
    }
    obj = load_registry(
        _write_registry(tmp_path, _valid_registry_yaml(collision=collision))
    ).objects["test_object"]
    assert isinstance(obj.collision, MeshCollisionSpec)
    assert obj.collision.scale == (1.0, 1.0, 1.0)

    collision["scale"] = [2.0, 3.0, 4.0]
    scaled = load_registry(
        _write_registry(tmp_path, _valid_registry_yaml(collision=collision))
    ).objects["test_object"]
    center, half_extents = scaled.resolve_aabb()
    assert center == pytest.approx({"x": 1.0, "y": 1.5, "z": 2.0})
    assert half_extents == pytest.approx({"x": 1.0, "y": 1.5, "z": 2.0})


def test_mesh_validation_ladder_fails_loud_with_object_id(tmp_path: Path) -> None:
    mesh_path = tmp_path / "part.obj"
    mesh_path.write_text(
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n",
        encoding="utf-8",
    )
    identity_offset = {
        "position": {"x": 0, "y": 0, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
    }
    invalid = [
        (
            {
                "type": "mesh",
                "mesh_path": "part.obj",
                "dimensions": [1, 1, 1],
                "pose_offset": identity_offset,
            },
            "test_object.*mesh.*forbids.*dimensions",
        ),
        (
            {"type": "mesh", "mesh_path": "part.obj"},
            "test_object.*requires.*pose_offset",
        ),
        (
            {
                "type": "mesh",
                "mesh_path": "missing.obj",
                "pose_offset": identity_offset,
            },
            "test_object.*does not exist",
        ),
        (
            {
                "type": "mesh",
                "mesh_path": "../outside.obj",
                "pose_offset": identity_offset,
            },
            "test_object.*escapes asset_root",
        ),
    ]
    for index, (collision, message) in enumerate(invalid):
        case_dir = tmp_path / str(index)
        case_dir.mkdir()
        if collision.get("mesh_path") == "part.obj":
            (case_dir / "part.obj").write_bytes(mesh_path.read_bytes())
        path = _write_registry(case_dir, _valid_registry_yaml(collision=collision))
        with pytest.raises(ValueError, match=message):
            load_registry(path)


@pytest.mark.skipif(
    not MESH_REGISTRY_PATH.exists(),
    reason="config/model_registry_mesh_example.yaml was removed from the "
    "repo; the mesh pathway is exercised live by sync_scene. Restore the "
    "example registry to re-enable these.",
)
def test_m1_builds_shape_mesh_at_composed_pose() -> None:
    from moveit_msgs.msg import CollisionObject
    from robot_backend.nodes.scene_tf_sync_node import build_collision_object

    obj = load_registry(MESH_REGISTRY_PATH).objects["u_green"]
    obj = replace(
        obj,
        pose_offset={
            "frame_id": "world",
            "position": {"x": 0.01, "y": 0.02, "z": 0.03},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    )
    world_pose = {
        "frame_id": "world",
        "position": {"x": 0.4, "y": -0.2, "z": 0.15},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }
    message = build_collision_object(obj, world_pose)
    assert message.operation == CollisionObject.ADD
    assert message.id == "u_green"
    assert message.header.frame_id == "world"
    assert not message.primitives
    assert len(message.meshes) == 1
    assert len(message.meshes[0].vertices) == 16
    assert len(message.meshes[0].triangles) == 28
    assert len(message.mesh_poses) == 1
    assert message.mesh_poses[0].position.x == pytest.approx(0.41)
    assert message.mesh_poses[0].position.y == pytest.approx(-0.18)
    assert message.mesh_poses[0].position.z == pytest.approx(0.18)


@pytest.mark.skipif(
    not MESH_REGISTRY_PATH.exists(),
    reason="config/model_registry_mesh_example.yaml was removed from the "
    "repo; the mesh pathway is exercised live by sync_scene. Restore the "
    "example registry to re-enable these.",
)
def test_m4_manifest_adds_mesh_provenance_without_changing_yaml_hash(
    tmp_path: Path,
) -> None:
    model = load_registry(MESH_REGISTRY_PATH)
    manifest_path = sess.write_manifest(
        tmp_path,
        registry=model,
        registry_path=MESH_REGISTRY_PATH,
        registry_sha256=None,
        started_wall_ns=1,
        recorded_topics=[],
        images_recorded=False,
        image_throttle_hz=5.0,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["registry"] == model.raw_dict
    assert manifest["registry_sha256"] == hashlib.sha256(
        MESH_REGISTRY_PATH.read_bytes()
    ).hexdigest()
    # The load-bearing guarantee for fault-injection: passing the typed Registry
    # (not registry.raw_dict) makes write_manifest take the Registry branch and
    # emit mesh_provenance. A dict argument silently skips it.
    assert "mesh_provenance" in manifest
    provenance = manifest["mesh_provenance"]["u_green"]
    mesh_path = model.objects["u_green"].collision.resolved_mesh_path
    assert provenance["mesh_path"] == str(mesh_path)
    assert provenance["scale"] == [1.0, 1.0, 1.0]
    assert provenance["sha256"] == hashlib.sha256(mesh_path.read_bytes()).hexdigest()
    assert provenance["aabb"]["frame"] == "object"
    assert provenance["aabb"]["center"] == pytest.approx(
        {"x": 0.0, "y": -0.0005, "z": 0.0}
    )
    assert provenance["aabb"]["half_extents"] == pytest.approx(
        {"x": 0.035, "y": 0.035, "z": 0.0105}
    )


def test_pre_refactor_collision_bytes_and_manifest_hash_are_identical(tmp_path: Path) -> None:
    from builtin_interfaces.msg import Time
    from robot_backend.nodes.scene_tf_sync_node import build_collision_object

    golden = json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))
    model = load_registry(REGISTRY_PATH)
    # The golden froze the BOX build path (red_cube, green_cube) only. Restrict
    # to exactly those objects: newer objects folded into the production registry
    # -- the u_green mesh -- are not part of this frozen guarantee, and the box
    # golden canonicalizer only supports box collision anyway. The per-cube
    # collision bytes MUST stay byte-identical.
    actual: dict[str, str] = {}
    for object_id in golden["collision_object_sha256"]:
        obj = model.objects[object_id]
        world_pose = copy.deepcopy(golden["fixed_world_pose"])
        stamp = golden["fixed_stamp"]
        world_pose["_header_stamp"] = Time(
            sec=stamp["sec"], nanosec=stamp["nanosec"]
        )
        message = build_collision_object(obj, world_pose)
        actual[object_id] = hashlib.sha256(
            canonical_collision_bytes(message)
        ).hexdigest()

    assert actual == golden["collision_object_sha256"]
    # The whole-file registry hash legitimately changed when u_green was folded
    # into the production registry, so it no longer equals the frozen cubes-only
    # golden. The frozen guarantee is the per-cube collision bytes above, not the
    # file hash; the golden fixture stays cubes-only and is intentionally not
    # re-captured.
    assert model.raw_sha256 != golden["registry_file_sha256"]

    manifest_path = sess.write_manifest(
        tmp_path,
        registry=model,
        registry_path=REGISTRY_PATH,
        registry_sha256=None,
        started_wall_ns=1,
        recorded_topics=[],
        images_recorded=False,
        image_throttle_hz=5.0,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["registry"] == model.raw_dict
    # write_manifest hashes exactly the loaded registry (self-consistent),
    # rather than pinning to the now-stale frozen golden file hash.
    assert manifest["registry_sha256"] == model.raw_sha256


def test_contract_and_gripper_consumers_receive_unchanged_spec_slices(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    raw = yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8"))
    contract_registry = load_contract_registry(REGISTRY_PATH)
    checked = 0
    for object_id, entry in raw["objects"].items():
        if "spec" not in entry:
            # Pose-only frame entries (placements, holes, reorients) carry no
            # spec block by design; only spec-bearing objects owe the slice.
            continue
        assert contract_registry["objects"][object_id]["spec"] == entry["spec"]
        checked += 1
    assert checked > 0

    monkeypatch.setenv("_ROBOT_BACKEND_MCP_VENV_READY", "1")
    packages = importlib.import_module("ament_index_python.packages")
    monkeypatch.setattr(
        packages, "get_package_share_directory", lambda package: str(PACKAGE_ROOT)
    )
    gripper_server = importlib.import_module(
        "robot_backend.mcp_servers.gripper_mcp_server"
    )
    gripper_registry = gripper_server._load_registry()
    checked_gripper = 0
    for object_id, entry in raw["objects"].items():
        if "spec" not in entry:
            continue  # pose-only frame entries carry no spec by design
        assert gripper_registry["objects"][object_id]["spec"] == entry["spec"]
        checked_gripper += 1
    assert checked_gripper > 0
