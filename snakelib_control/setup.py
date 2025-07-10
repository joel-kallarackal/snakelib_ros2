from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'snakelib_control'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    (os.path.join('share', package_name, 'pykdl_utils'), glob('pykdl_utils/*.py')),
    (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),   
]

# Walk gaitlib and adjust path
for root, _, files in os.walk('gaitlib'):
    install_path = os.path.join(
        'share', package_name, root  # this keeps original structure
    )
    source_files = [os.path.join(root, f) for f in files]
    data_files.append((install_path, source_files))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'snakelib_msgs'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='joelgeorgekal@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_manager_node = snakelib_control.command_manager_node:main',
            'gait_script_node = snakelib_control.gait_script_node:main',
            'joystick_teleop_node = snakelib_control.joystick_teleop_node:main'
        ],
    },
)
