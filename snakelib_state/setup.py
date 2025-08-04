from setuptools import find_packages, setup

package_name = 'snakelib_state'
import os
from glob import glob

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joel',
    maintainer_email='joelgeorgekal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_chassis = snakelib_state.virtual_chassis:main',
            'state_estimation = snakelib_state.state_estimation:main',
        ],
    },
)
