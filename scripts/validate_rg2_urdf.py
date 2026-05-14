#!/usr/bin/env python3
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parent.parent


def fmt_origin(origin):
    if origin is None:
        return "xyz=0 0 0 rpy=0 0 0"
    xyz = origin.attrib.get("xyz", "0 0 0")
    rpy = origin.attrib.get("rpy", "0 0 0")
    return f"xyz={xyz} rpy={rpy}"


def mesh_status(filename):
    if not filename.startswith("package://"):
        return "not a package path"
    pkg_rel = filename[len("package://") :]
    if "/" not in pkg_rel:
        return "malformed package path"
    pkg, rel = pkg_rel.split("/", 1)
    if pkg != "robot_description":
        pkg_root = REPO_ROOT / "ros2_ws" / "src" / pkg
        return f"package {pkg} local_exists={pkg_root.joinpath(rel).exists()}"
    path = REPO_ROOT / "ros2_ws" / "src" / "robot_description" / rel
    return f"robot_description local_exists={path.exists()} path={path}"


def validate(path):
    root = ET.parse(path).getroot()
    links = {}
    joints = {}
    parent_links = set()
    child_links = set()

    for child in root:
        if child.tag == "link":
            links[child.attrib["name"]] = child
        elif child.tag == "joint":
            joints[child.attrib["name"]] = child

    print(f"robot name: {root.attrib.get('name', '<missing>')}")
    print("links:")
    for name in sorted(links):
        print(f"  - {name}")

    print("joints:")
    for name in sorted(joints):
        joint = joints[name]
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        parent_name = parent.attrib.get("link", "<missing>") if parent is not None else "<missing>"
        child_name = child.attrib.get("link", "<missing>") if child is not None else "<missing>"
        parent_links.add(parent_name)
        child_links.add(child_name)
        print(
            f"  - {name}: type={joint.attrib.get('type', '<missing>')} "
            f"parent={parent_name} child={child_name} {fmt_origin(origin)}"
        )

    root_candidates = sorted(set(links) - child_links)
    print(f"root link candidates: {root_candidates}")

    missing = []
    for name, joint in sorted(joints.items()):
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.attrib.get("link", "<missing>") if parent is not None else "<missing>"
        child_name = child.attrib.get("link", "<missing>") if child is not None else "<missing>"
        if parent_name not in links:
            missing.append(f"{name}: missing parent link {parent_name}")
        if child_name not in links:
            missing.append(f"{name}: missing child link {child_name}")

    print("joint link integrity:")
    if missing:
        for item in missing:
            print(f"  - {item}")
    else:
        print("  - all joint parent/child links exist")

    print("meshes:")
    found_any = False
    for link_name in sorted(links):
        for mesh in links[link_name].findall(".//mesh"):
            found_any = True
            filename = mesh.attrib.get("filename", "<missing>")
            print(f"  - {link_name}: {filename} [{mesh_status(filename)}]")
    if not found_any:
        print("  - none")


def main():
    if len(sys.argv) != 2:
        print("usage: python3 scripts/validate_rg2_urdf.py <urdf_path>")
        raise SystemExit(1)
    validate(Path(sys.argv[1]).resolve())


if __name__ == "__main__":
    main()
