from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14540@localhost:14557',
                #'fcu_url': '/dev/serial/by-id/usb-Freefly_Systems_Long_Range_RF_0000BC210001-if00:921600',
                #'fcu_url': '/dev/ttyACM0:921600',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'log_level': 'info'
            }],
        ),
        Node(
            package='mavros_launch',
            executable='cmd_vel_republisher',
            name='cmd_vel_republisher',
            output='screen',
        ),
        Node(
            package='mavros_launch',
            executable='joystick_arm_disarm',
            name='joystick_arm_disarm',
            output='screen',
        ),
    ])