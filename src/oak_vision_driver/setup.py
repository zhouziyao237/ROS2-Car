from setuptools import setup
import os
from glob import glob

package_name = "oak_vision_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "weights"), glob("weights/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="OAK-D Lite ROS2 driver with RGB, aligned depth, camera_info and optional OAK YOLO detection publishing compatible with yolov5_node topics.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "oak_vision_driver_node = oak_vision_driver.oak_vision_driver_node:main",
        ],
    },
)
