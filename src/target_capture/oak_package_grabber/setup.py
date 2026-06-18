from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'oak_package_grabber'

setup(
    name=package_name,
    version='1.0.0',
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
    description='Fixed-distance two-stage package grabber with OAK z verification.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_package_grabber_node = oak_package_grabber.oak_package_grabber_node:main',
            'send_grabber_request = oak_package_grabber.send_grabber_request:main',
        ],
    },
)
