#!/usr/bin/env python3
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ISAAC_URDF = ROOT / "generated_urdf/rg2_only/rg2_only_isaac.urdf"
SEMANTIC_URDFS = [
    ROOT / "generated_urdf/rg2_only/rg2_only.urdf",
    ROOT / "ros2_ws/src/robot_description/urdf/ur5e_ros.urdf",
]
URDFS = [ISAAC_URDF, *SEMANTIC_URDFS]

REVOLUTE_JOINTS = {
    "onrobot_rg2_body_to_finger_1_origin_joint",
    "onrobot_rg2_body_to_finger_2_origin_joint",
    "onrobot_rg2_finger_1_origin_to_moment_arm_joint",
    "onrobot_rg2_finger_1_origin_to_truss_arm_joint",
    "onrobot_rg2_finger_1_truss_arm_to_finger_tip_joint",
    "onrobot_rg2_finger_2_origin_to_moment_arm_joint",
    "onrobot_rg2_finger_2_origin_to_truss_arm_joint",
    "onrobot_rg2_finger_2_truss_arm_to_finger_tip_joint",
}
FIXED_JOINTS = {
    "onrobot_rg2_base_to_body_joint",
    "onrobot_rg2_body_to_grasp_frame_joint",
    "onrobot_rg2_finger_1_finger_tip_to_flex_finger_joint",
    "onrobot_rg2_finger_2_finger_tip_to_flex_finger_joint",
}


def summarize_joint(joint):
    parent = joint.find("parent")
    child = joint.find("child")
    axis = joint.find("axis")
    limit = joint.find("limit")
    mimic = joint.find("mimic")
    return {
        "name": joint.get("name"),
        "type": joint.get("type"),
        "parent": parent.get("link") if parent is not None else None,
        "child": child.get("link") if child is not None else None,
        "axis": axis.get("xyz") if axis is not None else None,
        "limit": (
            limit.get("lower") if limit is not None else None,
            limit.get("upper") if limit is not None else None,
            limit.get("effort") if limit is not None else None,
            limit.get("velocity") if limit is not None else None,
        ),
        "mimic": (
            mimic.get("joint") if mimic is not None else None,
            mimic.get("multiplier") if mimic is not None else None,
            mimic.get("offset") if mimic is not None else None,
        ),
    }


def inspect_urdf(path: Path):
    root = ET.parse(path).getroot()
    joints = [joint for joint in root.findall("joint") if joint.get("name", "").startswith("onrobot_rg2")]
    joint_map = {joint.get("name"): joint for joint in joints}

    print()
    print(f"{path}:")
    mimic_report = []
    for joint in joints:
        summary = summarize_joint(joint)
        print(f"  name: {summary['name']}")
        print(f"    type: {summary['type']}")
        print(f"    parent: {summary['parent']}")
        print(f"    child: {summary['child']}")
        print(f"    axis: {summary['axis']}")
        print(
            f"    limit: lower={summary['limit'][0]} upper={summary['limit'][1]} effort={summary['limit'][2]} velocity={summary['limit'][3]}"
        )
        print(
            f"    mimic: joint={summary['mimic'][0]} multiplier={summary['mimic'][1]} offset={summary['mimic'][2]}"
        )
        if summary["mimic"][0] is not None:
            mimic_report.append((summary["name"], summary["mimic"]))

    for name in REVOLUTE_JOINTS:
        joint = joint_map[name]
        assert joint.get("type") == "revolute", f"{name} should be revolute in {path}"
        axis = joint.find("axis")
        limit = joint.find("limit")
        assert axis is not None and axis.get("xyz"), f"{name} missing axis in {path}"
        assert axis.get("xyz") != "0 1 0", f"{name} still uses placeholder axis in {path}"
        assert limit is not None, f"{name} missing limit in {path}"

    for name in FIXED_JOINTS:
        joint = joint_map[name]
        assert joint.get("type") == "fixed", f"{name} should remain fixed in {path}"

    if path == ISAAC_URDF:
        for name in REVOLUTE_JOINTS:
            mimic = joint_map[name].find("mimic")
            assert mimic is None, f"{name} should not use mimic in {path}"
        print("  Isaac validation: direct-controllable RG2 linkage joints verified.")
    else:
        print("  Semantic mimic report:")
        if mimic_report:
            for name, mimic in mimic_report:
                print(f"    {name}: joint={mimic[0]} multiplier={mimic[1]} offset={mimic[2]}")
        else:
            print("    no mimic tags present")


if __name__ == "__main__":
    for urdf in URDFS:
        inspect_urdf(urdf)
    print()
    print("Validation passed. Run with: python3 scripts/inspect_rg2_urdf_joints.py")
