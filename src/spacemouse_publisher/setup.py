from setuptools import find_packages, setup
from glob import glob
import os

package_name = "spacemouse_publisher"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyspacemouse"],
    zip_safe=True,
    maintainer="Niklas Schlueter",
    maintainer_email="niklas.schlueter@tum.de",
    description=(
        "This package provides a node that publishes the data from a 3Dconnexion SpaceMouse."
    ),
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pyspacemouse_publisher = spacemouse_publisher.pyspacemouse_publisher:main",
            "twist_to_pose_node = spacemouse_publisher.twist_to_pose_node:main",
        ],
    },
)
