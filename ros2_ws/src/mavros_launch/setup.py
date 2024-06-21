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
            'cmd_vel_republisher = mavros_launch.cmd_vel_republisher:main',
            'joystick_arm_disarm = mavros_launch.joystick_arm_disarm:main',
        ],
    },
)
