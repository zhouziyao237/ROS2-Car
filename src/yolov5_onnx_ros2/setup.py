from setuptools import setup
import os
from glob import glob

package_name = "yolov5_onnx_ros2"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*.onnx")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@example.com",
    description="Integrated YOLOv5 ONNX INT8 ROS2 detector node",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolov5_onnx_node = yolov5_onnx_ros2.yolov5_onnx_node:main",
        ],
    },
)