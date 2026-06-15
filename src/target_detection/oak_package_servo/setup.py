from setuptools import setup
from glob import glob
import os

package_name = 'oak_package_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot2',
    maintainer_email='robot2@example.com',
    description='OAK DataMatrix package visual servo controller for robot2 pickup.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_package_servo_node = oak_package_servo.oak_package_servo_node:main',
            'fake_oak_package_target = oak_package_servo.fake_oak_package_target:main',
            'oak_servo_error_watch = oak_package_servo.oak_servo_error_watch:main',
        ],
    },
)
