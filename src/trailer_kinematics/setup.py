from setuptools import setup, find_packages

package_name = 'trailer_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.'),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rabeb',
    maintainer_email='rabeb@todo.todo',
    description='Trailer kinematics nodes: beta EKF, multi-circle footprint, odometry publisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'beta_ekf_node = trailer_kinematics.beta_ekf_node:main',
            'multi_circle_footprint_node = trailer_kinematics.multi_circle_footprint_node:main',
            'odometry_publisher = trailer_kinematics.odometry_publisher:main',
            'hitch_angle_stabilizer = trailer_kinematics.hitch_angle_stabilizer:main',
        ],
    },
)
