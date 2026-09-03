from setuptools import find_packages, setup


package_name = "isaac_trajectory_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/trajectory_bridge.launch.py"]),
        ("share/" + package_name, ["README.md", "README_RG2_CONTROLLER.md"]),
        # rg2_calibration_table.csv is NOT optional and must not be dropped:
        # robot_backend's rg2_geometry resolves DEFAULT_RG2_WIDTH_TABLE_PATH to
        # this installed share path. When the file is absent the lookup falls
        # back to a 2-point linear model whose grasp-centre depth is ~12.5 mm
        # too shallow at a 42 mm opening, which drives the TCP that much too
        # low on every grasp -- silently, with no error anywhere.
        (
            "share/" + package_name + "/config",
            [
                "config/rg2_width_table.csv",
                "config/rg2_calibration_table.csv",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mohammadnsr1",
    maintainer_email="mohammadnsr1@example.com",
    description="ROS 2 trajectory bridge and RG2 gripper controller for Isaac Sim joint commands.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "follow_joint_trajectory_server = isaac_trajectory_bridge.follow_joint_trajectory_server:main",
            "rg2_controller = isaac_trajectory_bridge.rg2_controller:main",
            "calibrate_rg2_width = isaac_trajectory_bridge.calibrate_rg2_width:main",
        ],
    },
)
