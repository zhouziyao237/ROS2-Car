from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'oak_package_search'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot logistics team',
    maintainer_email='user@example.com',
    description='Active OAK package search state machine.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_package_search_node = oak_package_search.oak_package_search_node:main',
            'send_package_search_request = oak_package_search.send_package_search_request:main',
            'mock_datamatrix_result = oak_package_search.mock_datamatrix_result:main',
            'watch_package_search_status = oak_package_search.watch_package_search_status:main',
            'check_oak_mount_yaw_offset = oak_package_search.check_oak_mount_yaw_offset:main',
        ],
    },
)
