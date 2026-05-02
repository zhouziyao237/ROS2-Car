from setuptools import setup
from glob import glob
import os

package_name = 'ocr_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Time-synchronized PP-OCRv5 ONNXRuntime OCR extraction node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ocr_node = ocr_node.ocr_node:main',
        ],
    },
)
