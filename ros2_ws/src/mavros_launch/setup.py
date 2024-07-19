from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mavros_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eli',
    maintainer_email='eli@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick = mavros_launch.joystick:main',
            'arm_disarm = mavros_launch.arm_disarm:main',
            'mode_set = mavros_launch.mode_set:main',
            'setpoint_control = mavros_launch.setpoint_control:main',
            'preset_topic_aggregator = mavros_launch.preset_topic_aggregator:main',
        ],
    },
)
