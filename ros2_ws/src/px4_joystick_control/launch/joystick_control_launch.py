from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                'axis_angular.pitch': 1,    # Pitch (e.g., right stick up/down)
                'axis_angular.roll': 0,     # Roll (e.g., right stick left/right)
                'scale_linear.z': -2.0,
                'scale_angular.yaw': 2.0,
                'scale_angular.pitch': -0.5,
                'scale_angular.roll': 0.5,
                'enable_button': 7,
                'enable_arm_button': 1,    # Button to arm the vehicle
                'enable_disarm_button': 0, # Button to disarm the vehicle
            }]
        ),
    ])
