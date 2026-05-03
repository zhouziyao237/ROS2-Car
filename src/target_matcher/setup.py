from setuptools import setup
import os
from glob import glob

package_name = "target_matcher"

setup(
    name=package_name,
    version="0.0.3",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Target matcher for YOLO detections, OCR cache, overlay and depth fusion.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_matcher_node = target_matcher.target_matcher_node:main",
            "matched_overlay_node = target_matcher.matched_overlay_node:main",
            "matched_depth_locator_node = target_matcher.matched_depth_locator_node:main",
        ],
    },
)