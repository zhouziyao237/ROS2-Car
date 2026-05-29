from setuptools import setup
from glob import glob
import os

package_name = 'nav_stack_core_v2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot2',
    maintainer_email='robot2@example.com',
    description='robot2 LiDAR + D435 fused online SLAM and Nav2 stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_mode_mux.py = nav_stack_core_v2.cmd_vel_mode_mux:main',
            'rf2o_odom_adapter = nav_stack_core_v2.rf2o_odom_adapter:main',
            'control_mode_manager = nav_stack_core_v2.control_mode_manager:main',
            'front_scan_filter = nav_stack_core_v2.front_scan_filter:main',
            'd435_depth_to_scan = nav_stack_core_v2.d435_depth_to_scan:main',
            'scan_fusion_node = nav_stack_core_v2.scan_fusion_node:main',
            'cmd_vel_mode_mux = nav_stack_core_v2.cmd_vel_mode_mux:main',
            'goal_pose_bridge = nav_stack_core_v2.goal_pose_bridge:main',
            'odom_to_tf_bridge = nav_stack_core_v2.odom_to_tf_bridge:main',
        ],
    },
)
