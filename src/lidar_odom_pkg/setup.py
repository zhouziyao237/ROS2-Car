from glob import glob
from setuptools import setup

package_name = 'lidar_odom_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='you@example.com',
    description='2D LiDAR scan-matching odometry with configurable sector, Kalman prediction and ICP.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_match_odom = lidar_odom_pkg.scan_match_odom:main',
            'scan_match_odom.py = lidar_odom_pkg.scan_match_odom:main',
        ],
    },
)
