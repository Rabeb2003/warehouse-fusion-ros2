from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_orchestrator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabeb',
    maintainer_email='rabeb@todo.todo',
    description='Orchestrator package for integrated trailer navigation and panda pick-and-place mission',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_orchestrator = warehouse_orchestrator.mission_orchestrator:main',
        ],
    },
)
