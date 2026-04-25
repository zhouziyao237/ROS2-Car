from glob import glob
from setuptools import setup

package_name = 'm10p_lidar_odom_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='you@example.com',
    description='M10P-like Gazebo/RViz2 verification bringup for lidar_odom_pkg.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_to_path = m10p_lidar_odom_bringup.odom_to_path:main',
            'cmd_vel_demo = m10p_lidar_odom_bringup.cmd_vel_demo:main',
            'odom_error_monitor = m10p_lidar_odom_bringup.odom_error_monitor:main',
        ],
    },
)
