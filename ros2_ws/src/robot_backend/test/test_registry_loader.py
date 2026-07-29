"""Shared registry validation and pre-refactor byte-regression gate."""

from __future__ import annotations

import copy
import hashlib
import importlib
import json
from pathlib import Path

import pytest
import yaml

from robot_backend.contracts.loader import _load_registry as load_contract_registry
from robot_backend.registry import load_registry
from robot_backend.watchdog import session as sess
from registry_golden_utils import canonical_collision_bytes


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REGISTRY_PATH = PACKAGE_ROOT / "config" / "model_registry.yaml"
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


def test_pre_refactor_collision_bytes_and_manifest_hash_are_identical(tmp_path: Path) -> None:
    from builtin_interfaces.msg import Time
    from robot_backend.nodes.scene_tf_sync_node import build_collision_object

    golden = json.loads(GOLDEN_PATH.read_text(encoding="utf-8"))
    model = load_registry(REGISTRY_PATH)
    actual: dict[str, str] = {}
    for obj in model.objects.values():
        if not obj.enabled:
            continue
        world_pose = copy.deepcopy(golden["fixed_world_pose"])
        stamp = golden["fixed_stamp"]
        world_pose["_header_stamp"] = Time(
            sec=stamp["sec"], nanosec=stamp["nanosec"]
        )
        message = build_collision_object(obj, world_pose)
        actual[obj.object_id] = hashlib.sha256(
            canonical_collision_bytes(message)
        ).hexdigest()

    assert actual == golden["collision_object_sha256"]
    assert model.raw_sha256 == golden["registry_file_sha256"]

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
    assert manifest["registry_sha256"] == golden["registry_file_sha256"]


def test_contract_and_gripper_consumers_receive_unchanged_spec_slices(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    raw = yaml.safe_load(REGISTRY_PATH.read_text(encoding="utf-8"))
    contract_registry = load_contract_registry(REGISTRY_PATH)
    for object_id, entry in raw["objects"].items():
        assert contract_registry["objects"][object_id]["spec"] == entry["spec"]

    monkeypatch.setenv("_ROBOT_BACKEND_MCP_VENV_READY", "1")
    packages = importlib.import_module("ament_index_python.packages")
    monkeypatch.setattr(
        packages, "get_package_share_directory", lambda package: str(PACKAGE_ROOT)
    )
    gripper_server = importlib.import_module(
        "robot_backend.mcp_servers.gripper_mcp_server"
    )
    gripper_registry = gripper_server._load_registry()
    for object_id, entry in raw["objects"].items():
        assert gripper_registry["objects"][object_id]["spec"] == entry["spec"]
