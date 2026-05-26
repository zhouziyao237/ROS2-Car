from setuptools import setup, find_packages
from glob import glob
from pathlib import Path

def files_in(pattern):
    return [str(x) for x in Path('.').glob(pattern) if x.is_file()]

import os

package_name = 'robot_logistics_semantic'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for folder in ['launch', 'config', 'rviz']:
    if os.path.isdir(folder):
        files = glob(os.path.join(folder, '*'))
        if files:
            data_files.append((os.path.join('share', package_name, folder), files))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_logistics',
    maintainer_email='robot_logistics@example.com',
    description='Semantic mapping, frontier exploration, relocalization and task management for logistics robot simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_restrict_region_node = robot_logistics_semantic.rviz_restrict_region_node:main',

            'sim_model_state_odom_node = robot_logistics_semantic.sim_model_state_odom_node:main',
            'vision_qr_depth_node = robot_logistics_semantic.vision_qr_depth_node:main',
            'frontier_explorer_node = robot_logistics_semantic.frontier_explorer_node:main',
            'hybrid_explorer_node = robot_logistics_semantic.frontier_explorer_node:main',

            'semantic_task_manager_node = robot_logistics_semantic.semantic_task_manager_node:main',
            'semantic_registrar_node = robot_logistics_semantic.semantic_registrar_node:main',
            'semantic_relocalizer_node = robot_logistics_semantic.semantic_relocalizer_node:main',
            'semantic_marker_publisher_node = robot_logistics_semantic.semantic_marker_publisher_node:main',
            'nav_to_qr_server = robot_logistics_semantic.nav_to_qr_server:main',
            'rviz_goal_relay_node = robot_logistics_semantic.rviz_goal_relay_node:main',
            'sim_semantic_sensor_node = robot_logistics_semantic.sim_semantic_sensor_node:main',

            'depth_align_node = robot_logistics_semantic.depth_align_node:main',
            'grab_execute_node = robot_logistics_semantic.grab_execute_node:main',
        ],
    },
)
