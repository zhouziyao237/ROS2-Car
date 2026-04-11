from setuptools import find_packages
from setuptools import setup

setup(
    name='mecanum_gazebo_plugin',
    version='0.0.0',
    packages=find_packages(
        include=('mecanum_gazebo_plugin', 'mecanum_gazebo_plugin.*')),
)
