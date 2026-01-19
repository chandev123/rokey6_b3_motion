from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ddr5_tactile_handler'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='user@example.com',
    description='DDR5 Memory Assembly with Tactile Force Sensing for DRL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tactile_sensor = ddr5_tactile_handler.tactile_sensor:main',
            'motion_control = ddr5_tactile_handler.motion_control:main',
            'main_state_machine = ddr5_tactile_handler.main_state_machine:main',
        ],
    },
)
