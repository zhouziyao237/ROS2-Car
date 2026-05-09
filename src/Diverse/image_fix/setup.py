from setuptools import setup

package_name = "image_fix"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Image flip node for camera mirror correction.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgb_flip_node = image_fix.rgb_flip_node:main",
        ],
    },
)
