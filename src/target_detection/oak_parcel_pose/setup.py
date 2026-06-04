from glob import glob
import os
from setuptools import setup

package_name = 'oak_parcel_pose'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 RGB-depth Data Matrix detector that outputs camera-frame XYZ in millimeters.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_parcel_pose_node = oak_parcel_pose.oak_parcel_pose_node:main',        ],
    },
)
