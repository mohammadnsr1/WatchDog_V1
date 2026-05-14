#!/usr/bin/env python3
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.dom import minidom


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INPUT = REPO_ROOT / "generated_urdf" / "rg2_only" / "rg2_only.urdf"
DEFAULT_OUTPUT = REPO_ROOT / "generated_urdf" / "rg2_only" / "rg2_only_isaac.urdf"
PACKAGE_SEARCH_ROOTS = [
    REPO_ROOT / "ros2_ws" / "src",
    REPO_ROOT / "external",
]


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
    return minidom.parseString(rough).toprettyxml(indent="  ")


def find_package_root(package_name):
    for search_root in PACKAGE_SEARCH_ROOTS:
        candidate = search_root / package_name
        if candidate.exists():
            return candidate
    return None


def resolve_mesh_path(filename):
    if not filename.startswith("package://"):
        return filename, None, "UNCHANGED"

    pkg_rel = filename[len("package://") :]
    if "/" not in pkg_rel:
        return filename, None, "MALFORMED"

    package_name, rel_path = pkg_rel.split("/", 1)
    package_root = find_package_root(package_name)
    if package_root is None:
        return None, None, f"MISSING package root for {package_name}"

    target = package_root / rel_path
    if not target.exists():
        return f"file://{target}", target, "MISSING"

    return f"file://{target}", target, "OK"


def parse_links_and_joints(robot):
    links = []
    joints = []
    child_links = set()
    for child in robot:
        if child.tag == "link":
            links.append(child.attrib["name"])
        elif child.tag == "joint":
            joints.append(child.attrib["name"])
            child_elem = child.find("child")
            if child_elem is not None:
                child_links.add(child_elem.attrib.get("link", ""))
    root_candidates = sorted(set(links) - child_links)
    return sorted(links), sorted(joints), root_candidates


def convert(input_path, output_path):
    robot = ET.parse(input_path).getroot()
    links, joints, root_candidates = parse_links_and_joints(robot)

    mesh_results = []
    missing = []
    for mesh in robot.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        resolved, target_path, status = resolve_mesh_path(filename)
        if resolved is not None and filename.startswith("package://"):
            mesh.attrib["filename"] = resolved
        mesh_results.append(
            {
                "original": filename,
                "resolved": resolved if resolved is not None else "<unresolved>",
                "status": status,
            }
        )
        if status != "OK":
            missing.append(
                {
                    "original": filename,
                    "resolved": resolved if resolved is not None else "<unresolved>",
                    "status": status,
                }
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(pretty_xml(robot), encoding="utf-8")

    return {
        "robot_name": robot.attrib.get("name", "<missing>"),
        "links": links,
        "joints": joints,
        "root_link": root_candidates[0] if len(root_candidates) == 1 else root_candidates,
        "mesh_results": mesh_results,
        "missing": missing,
        "output_path": output_path,
    }


def main():
    input_path = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_INPUT
    output_path = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else DEFAULT_OUTPUT

    result = convert(input_path, output_path)

    print(f"robot name: {result['robot_name']}")
    print("links:")
    for link in result["links"]:
        print(f"  - {link}")
    print("joints:")
    for joint in result["joints"]:
        print(f"  - {joint}")
    print(f"root link: {result['root_link']}")
    print("meshes:")
    for item in result["mesh_results"]:
        print(f"  - {item['status']}: {item['original']} -> {item['resolved']}")
    print(f"output: {result['output_path']}")
    if result["missing"]:
        print("missing meshes:")
        for item in result["missing"]:
            print(f"  - {item['status']}: {item['original']} -> {item['resolved']}")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
