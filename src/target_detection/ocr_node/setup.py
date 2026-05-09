from setuptools import setup
from glob import glob

package_name = 'ocr_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/weights', glob('weights/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzy',
    maintainer_email='zzy@example.com',
    description='OCR node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ocr_node = ocr_node.ocr_node:main',
            'paddle_ocr_node = ocr_node.paddle_ocr_node:main',
        ],
    },
)
