import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'my_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'lidar_src/*.py'), glob('lidar_src/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 driver for lidar using sdk_py',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_src.lidar:main',
        ],
    },
)