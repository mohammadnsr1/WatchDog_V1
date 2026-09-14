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
        (
            "share/robot_backend/config/assets",
            [
                "config/assets/fmb_u_green.obj",
                "config/assets/u_green_2x.obj",
                "config/assets/base2_2x.obj",
                "config/assets/hex_blue_2x.obj",
                "config/assets/hex_red_2x.obj",
                "config/assets/inverted_u_brown_2x.obj",
                # fmb1 set (fmb_assembly_2), native size -- no _2x suffix
                # because nothing was scaled.
                "config/assets/base1.obj",
                "config/assets/u_brown.obj",
                "config/assets/u_orange.obj",
                "config/assets/inverted_u_yellow.obj",
                "config/assets/line_green.obj",
            ],
        ),
    ],
    # mcp<2: 2.x renamed FastMCP -> MCPServer and the mcp_servers modules
    # import mcp.server.fastmcp. numpy<2 so the workspace venv does not shadow
    # the numpy 1.26 that ROS 2 Jazzy's dist-packages were built against.
    install_requires=["setuptools", "mcp<2", "PyYAML", "trimesh", "numpy<2"],
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
            "scene_backend_smoke_test = robot_backend.manual_tests.scene_backend_smoke_test:main",
            "scene_mcp_server = robot_backend.mcp_servers.scene_mcp_server:main",
            "watchdog_mcp_server = robot_backend.mcp_servers.watchdog_mcp_server:main",
        ],
    },
)
