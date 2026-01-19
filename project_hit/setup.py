from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'project_hit'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include reference DRL files in the share folder
        (os.path.join('share', package_name, 'reference'), glob('*.drl')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Project H.I.T - Tactile-based Robot Arm Motion Control',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'task_controller_node = project_hit.task_controller_node:main',
            'robot_interface_node = project_hit.robot_interface_node:main',
            'gripper_service_node = project_hit.gripper_service_node:main',
        ],
    },
)

