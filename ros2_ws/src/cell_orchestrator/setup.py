import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = "cell_orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mnasr3@asu.edu",
    maintainer_email="todo@example.com",
    description=(
        "The cell's timing interlock: serves /cell/request_go and "
        "/cell/signal_ready, relaying between the executor stack and the "
        "monitoring agent. A plain ROS node -- no LLM, no reasoning, no "
        "knowledge of contracts or monitors."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator_node = cell_orchestrator.node:main",
        ],
    },
)
