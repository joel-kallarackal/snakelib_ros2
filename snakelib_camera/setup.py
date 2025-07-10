from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'snakelib_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.py')),
    ],
    install_requires=['setuptools', 'snakelib_hebi', 'snakelib_msgs'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='joelgeorgekal@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fisheye_node = snakelib_camera.fisheye_node:main',
            'pinhole_node = snakelib_camera.pinhole_node:main',
            'thermal_node = snakelib_camera.thermal_node:main',
            'image_processing_node = snakelib_camera.image_processing_node:main',
            'toggle_streaming = snakelib_camera.toggle_streaming:main'
        ],
    },
)
