from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'snakelib_hebi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'snakelib_hebi'), glob('snakelib_hebi/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='joelgeorgekal@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hebi_control_interface_node = snakelib_hebi.hebi_control_interface:main',
            'hebi_sensor_interface_node = snakelib_hebi.hebi_sensor_interface:main',
            'snake_visualizer_node = snakelib_hebi.snake_state_visualizer:main'
        ],
    },
)
