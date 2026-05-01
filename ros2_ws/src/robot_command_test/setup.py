from setuptools import find_packages, setup


package_name = "robot_command_test"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mohammadnsr1",
    maintainer_email="mohammadnsr1@example.com",
    description="Minimal ROS 2 joint command publisher for testing UR5e control in Isaac Sim.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_command_test = robot_command_test.joint_command_test:main",
        ],
    },
)
