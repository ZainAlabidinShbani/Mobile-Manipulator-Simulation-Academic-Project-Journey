from setuptools import find_packages, setup
import os
from glob import glob

package_name = "perception_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name,                ["package.xml"]),
        ("share/" + package_name + "/launch",    glob("launch/*.py")),
        ("share/" + package_name + "/config",    glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zain Alabidin Shbani",
    maintainer_email="zain.alabidin.shbani@gmail.com",
    description="Phase 3 — YOLOv8 perception + depth localisation for mobile manipulator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_detector_node    = perception_pkg.yolo_detector_node:main",
            "depth_localizer_node  = perception_pkg.depth_localizer_node:main",
            "camera_node           = perception_pkg.camera_node:main",
        ],
    },
)
