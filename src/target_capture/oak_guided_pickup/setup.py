from glob import glob
from setuptools import setup

package_name = 'oak_guided_pickup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot2',
    maintainer_email='robot2@example.com',
    description='OAK guided pickup lift alignment for robot2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_guided_pickup_node = oak_guided_pickup.oak_guided_pickup_node:main',
            'send_lift_align_request = oak_guided_pickup.send_lift_align_request:main',
            'calibrate_visual_align_point = oak_guided_pickup.calibrate_visual_align_point:main',
        ],
    },
)
