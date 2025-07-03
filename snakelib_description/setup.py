from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'snakelib_description'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # # Copy all files (recursively)
        # # REU_snake 
        # *[(os.path.join('share', package_name, 'REU_snake', os.path.relpath(f, 'REU_snake')), [f])
        #   for f in glob('REU_snake/**/*', recursive=True) if os.path.isfile(f)],

        # # SEA_snake
        # *[(os.path.join('share', package_name, 'SEA_snake', os.path.relpath(f, 'SEA_snake')), [f])
        #   for f in glob('SEA_snake/**/*', recursive=True) if os.path.isfile(f)],

        # # RSNAKE_snake
        # *[(os.path.join('share', package_name, 'RSNAKE_snake', os.path.relpath(f, 'RSNAKE_snake')), [f])
        #   for f in glob('RSNAKE_snake/**/*', recursive=True) if os.path.isfile(f)],
    ]

for root, dirs, files in os.walk('REU_snake'):
    files = [os.path.join(root, f) for f in files]
    if files:
        dest_dir = os.path.join('share', package_name, root)
        data_files.append((dest_dir, files))

for root, dirs, files in os.walk('SEA_snake'):
    files = [os.path.join(root, f) for f in files]
    if files:
        dest_dir = os.path.join('share', package_name, root)
        data_files.append((dest_dir, files))

for root, dirs, files in os.walk('RSNAKE_snake'):
    files = [os.path.join(root, f) for f in files]
    if files:
        dest_dir = os.path.join('share', package_name, root)
        data_files.append((dest_dir, files))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joel Kallarackal',
    maintainer_email='jkallara@andrew.cmu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
