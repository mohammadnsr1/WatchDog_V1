from setuptools import find_packages
from setuptools import setup


package_name = "robot_backend"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/robot_backend"],
        ),
        ("share/robot_backend", ["package.xml"]),
        (
            "share/robot_backend/nodes",
            ["robot_backend/nodes/README.md"],
        ),
        (
            "share/robot_backend/config",
            ["config/model_registry.yaml"],
        ),
    ],
    install_requires=["setuptools", "mcp", "PyYAML"],
    zip_safe=True,
    maintainer="mnasr3@asu.edu",
    maintainer_email="todo@example.com",
    description=(
        "Backend abstraction layer and backend-agnostic robot skills for "
        "Isaac Sim + ROS 2 + MoveIt workflows."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gripper_mcp_server = robot_backend.mcp_servers.gripper_mcp_server:main",
            "motion_mcp_server = robot_backend.mcp_servers.motion_mcp_server:main",
            "scene_tf_sync_node = robot_backend.nodes.scene_tf_sync_node:main",
        ],
    },
)
