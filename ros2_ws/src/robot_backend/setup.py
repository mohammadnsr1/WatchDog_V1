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
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/nodes",
            ["robot_backend/nodes/README.md"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO Maintainer",
    maintainer_email="todo@example.com",
    description=(
        "Backend abstraction layer and backend-agnostic robot skills for "
        "Isaac Sim + ROS 2 + MoveIt workflows."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
)
