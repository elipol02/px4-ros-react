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
                # simulation:
                'fcu_url': 'udp://:14540@localhost:14557',
                # Alta X:
                #'fcu_url': '/dev/ttyACM0:921600',
                # Sentinel (on wifi):
                #'fcu_url': 'udp://:14550@192.168.8.1:14550',
                # Sentinel (on microhard):
                #'fcu_url': 'udp://:14550@192.168.168.100:14550',
                # zippy:
                #'fcu_url': '/dev/ttyUSB0:57600',
                'gcs_url': 'udp://@localhost:14551',
                'target_system_id': 1,
                'fcu_protocol': 'v2.0',
                'target_component_id': 1,
                'log_level': 'info',
                'heartbeat_mav_type': 'GCS'
            }],
        ),
        Node(
            package='mavros_launch',
            executable='preset_topic_aggregator',
            name='preset_topic_aggregator',
            output='screen',
        ),
        Node(
            package='mavros_launch',
            executable='joystick',
            name='joystick',
            output='screen',
        ),
        Node(
            package='mavros_launch',
            executable='arm_disarm',
            name='arm_disarm',
            output='screen',
        ),
        Node(
            package='mavros_launch',
            executable='mode_set',
            name='mode_set',
            output='screen',
        ),
        Node(
            package='mavros_launch',
            executable='setpoint_control',
            name='setpoint_control',
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',  # Adjust if necessary
                'deadzone': 0.05
            }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{
                'axis_linear.z': 2,         # Throttle (e.g., left stick up/down)
                'axis_angular.yaw': 3,      # Yaw (e.g., left stick left/right)
                'axis_linear.y': 0,    # Pitch (e.g., right stick up/down)
                'axis_linear.x': 1,     # Roll (e.g., right stick left/right)
                'scale_linear.z': -4.0,
                'scale_angular.yaw': 4.0,
                'scale_linear.y': -4.0,
                'scale_linear.x': 4.0,
                'enable_button': 7,
                'enable_arm_button': 1,    # Button to arm the vehicle
                'enable_disarm_button': 0, # Button to disarm the vehicle
            }]
        ),
    ])