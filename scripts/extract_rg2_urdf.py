#!/usr/bin/env python3
import copy
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = REPO_ROOT / "ros2_ws" / "src" / "robot_description" / "urdf" / "ur5e_ros.urdf"
DEFAULT_OUTPUT = REPO_ROOT / "generated_urdf" / "rg2_only" / "rg2_only.urdf"
ROOT_LINK = "onrobot_rg2_base_link"
EXCLUDED_JOINTS = {"Gripper_joint", "wrist_3_link_to_TCP"}
EXCLUDED_LINKS = {"world", "base_link", "tcp"}


def indent(elem, level=0):
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def pretty_xml(root):
    indent(root)
    rough = ET.tostring(root, encoding="utf-8")
    reparsed = minidom.parseString(rough)
    return reparsed.toprettyxml(indent="  ")


def parse_urdf(path):
    return ET.parse(path).getroot()


def get_links_and_joints(robot):
    links = {}
    joints = {}
    for child in robot:
        if child.tag == "link":
            links[child.attrib["name"]] = child
        elif child.tag == "joint":
            joints[child.attrib["name"]] = child
    return links, joints


def joint_parent_child(joint):
    parent = joint.find("parent")
    child = joint.find("child")
    return (
        parent.attrib["link"] if parent is not None else None,
        child.attrib["link"] if child is not None else None,
    )


def descendants(root_link, joints):
    reachable_links = {root_link}
    reachable_joints = []
    changed = True
    while changed:
        changed = False
        for joint in joints.values():
            if joint.attrib["name"] in EXCLUDED_JOINTS:
                continue
            parent, child = joint_parent_child(joint)
            if parent in reachable_links and child not in reachable_links:
                reachable_links.add(child)
                reachable_joints.append(joint)
                changed = True
    return reachable_links, reachable_joints


def check_meshes(links):
    warnings = []
    for link in links.values():
        for mesh in link.findall(".//mesh"):
            filename = mesh.attrib.get("filename")
            if not filename:
                continue
            if not filename.startswith("package://"):
                warnings.append(f"Non-package mesh path left unchanged: {filename}")
                continue
            pkg_rel = filename[len("package://") :]
            if "/" not in pkg_rel:
                warnings.append(f"Malformed package mesh path: {filename}")
                continue
            pkg, rel = pkg_rel.split("/", 1)
            pkg_root = REPO_ROOT / "ros2_ws" / "src" / pkg
            mesh_path = pkg_root / rel
            if not mesh_path.exists():
                warnings.append(f"Missing mesh for {filename}: {mesh_path}")
    return warnings


def make_bracket_fix(links, joints, warnings):
    base_to_bracket = joints.get("onrobot_rg2_base_to_bracket_joint")
    bracket_to_body = joints.get("onrobot_rg2_bracket_to_body_joint")
    if base_to_bracket is None or bracket_to_body is None:
        return

    parent_a, child_a = joint_parent_child(base_to_bracket)
    parent_b, child_b = joint_parent_child(bracket_to_body)
    if child_a != "onrobot_rg2_bracket_link" or parent_b != "onrobot_rg2_bracket_link":
        return

    if "onrobot_rg2_bracket_link" in links:
        return

    warnings.append(
        "Dangling link detected: onrobot_rg2_bracket_link referenced by joints but missing as a link."
    )
    warnings.append(
        "Applied fix: removed bracket-link dependency and attached onrobot_rg2_body_link directly to "
        "onrobot_rg2_base_link using the existing bracket-to-body origin."
    )

    body_joint = copy.deepcopy(bracket_to_body)
    body_joint.attrib["name"] = "onrobot_rg2_base_to_body_joint"
    body_joint.find("parent").attrib["link"] = ROOT_LINK

    joints.pop("onrobot_rg2_base_to_bracket_joint", None)
    joints.pop("onrobot_rg2_bracket_to_body_joint", None)
    joints[body_joint.attrib["name"]] = body_joint


def validate_joint_links(links, joints):
    missing = []
    for name, joint in joints.items():
        parent, child = joint_parent_child(joint)
        if parent not in links:
            missing.append(f"{name}: missing parent link {parent}")
        if child not in links:
            missing.append(f"{name}: missing child link {child}")
    return missing


def extract(input_path, output_path):
    robot = parse_urdf(input_path)
    links, joints = get_links_and_joints(robot)

    if ROOT_LINK not in links:
        raise RuntimeError(f"Root link {ROOT_LINK} not found in {input_path}")

    rg2_links, rg2_joints = descendants(ROOT_LINK, joints)
    rg2_links -= EXCLUDED_LINKS

    selected_links = {name: copy.deepcopy(links[name]) for name in sorted(rg2_links) if name in links}
    selected_joints = {
        joint.attrib["name"]: copy.deepcopy(joint)
        for joint in rg2_joints
        if joint.attrib["name"] not in EXCLUDED_JOINTS
    }

    warnings = []
    if "Gripper_joint" in joints:
        warnings.append("Excluded joint: Gripper_joint")
    if "wrist_3_link_to_TCP" in joints or "tcp" in links:
        warnings.append("Excluded TCP frame and wrist_3_link_to_TCP joint")

    make_bracket_fix(selected_links, selected_joints, warnings)

    missing_links = validate_joint_links(selected_links, selected_joints)
    if missing_links:
        raise RuntimeError("Unresolved missing links after extraction:\n" + "\n".join(missing_links))

    mesh_warnings = check_meshes(selected_links)

    output_root = ET.Element("robot", {"name": "onrobot_rg2"})
    for link_name in sorted(selected_links):
        output_root.append(selected_links[link_name])
    for joint_name in sorted(selected_joints):
        output_root.append(selected_joints[joint_name])

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(pretty_xml(output_root), encoding="utf-8")

    return {
        "output_path": output_path,
        "root_link": ROOT_LINK,
        "links": sorted(selected_links),
        "joints": sorted(selected_joints),
        "warnings": warnings,
        "mesh_warnings": mesh_warnings,
    }


def main():
    input_path = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_INPUT
    output_path = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else DEFAULT_OUTPUT

    result = extract(input_path, output_path)

    print(f"Generated URDF: {result['output_path']}")
    print(f"Root link: {result['root_link']}")
    print("Links:")
    for name in result["links"]:
        print(f"  - {name}")
    print("Joints:")
    for name in result["joints"]:
        print(f"  - {name}")
    print("Warnings:")
    if result["warnings"]:
        for warning in result["warnings"]:
            print(f"  - {warning}")
    else:
        print("  - None")
    print("Mesh warnings:")
    if result["mesh_warnings"]:
        for warning in result["mesh_warnings"]:
            print(f"  - {warning}")
    else:
        print("  - None")


if __name__ == "__main__":
    main()
