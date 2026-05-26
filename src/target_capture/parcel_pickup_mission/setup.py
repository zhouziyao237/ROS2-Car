from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'parcel_pickup_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzy',
    maintainer_email='zzy@example.com',
    description='Parcel pickup mission state machine',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pickup_state_machine_node = parcel_pickup_mission.pickup_state_machine_node:main',
            'pickup_osd_node = parcel_pickup_mission.pickup_osd_node:main',
            'wide_qr_matcher_node = parcel_pickup_mission.wide_qr_matcher_node:main',
        ],
    },
)
