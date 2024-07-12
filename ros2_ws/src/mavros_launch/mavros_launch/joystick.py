import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoystickNode(Node):

    def __init__(self):
        super().__init__('joystick_node')

        # Joystick arm/disarm
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        self.arm_publisher = self.create_publisher(
            String,
            '/arm',
            10)
        
        self.get_logger().info('Joystick arm/disarm node started, listening to /joy')

        self.previous_buttons = []
        self.last_command_was_arm = False  # Track the last command state

        # cmd_vel republisher
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10)
        
        self.get_logger().info('Cmd_vel republisher node started, listening on /cmd_vel and publishing to /mavros/setpoint_velocity/cmd_vel_unstamped')

        # Start mission subscriber
        self.start_mission_subscription = self.create_subscription(
            String,
            '/startmission',
            self.start_mission_callback,
            10)
        
        self.get_logger().info('Listening to /startmission')

        self.mission_active = False
        self.cmd_vel_timer = self.create_timer(0.1, self.cmd_vel_timer_callback)
        self.last_cmd_vel_msg = None

    def joy_callback(self, msg):
        arm_button_index = 1  # Replace with the index of the button you want to use
        disarm_button_index = 0  # Replace with the index of the button you want to use

        if not self.previous_buttons:
            self.previous_buttons = msg.buttons
            return

        if msg.buttons[arm_button_index] == 1 and self.previous_buttons[arm_button_index] == 0:
            self.get_logger().info('Arm button pressed')
            self.last_command_was_arm = True
            self.publish_arm_message("arm")
        elif msg.buttons[disarm_button_index] == 1 and self.previous_buttons[disarm_button_index] == 0:
            self.get_logger().info('Disarm button pressed')
            self.last_command_was_arm = False
            self.publish_arm_message("disarm")

        self.previous_buttons = msg.buttons

    def publish_arm_message(self, command):
        msg = String()
        msg.data = command
        self.arm_publisher.publish(msg)
        self.get_logger().info(f'Published {command} command')

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_msg = msg
        if self.mission_active:
            self.cmd_vel_publisher.publish(msg)

    def start_mission_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Start mission command received')
            self.mission_active = False
        elif msg.data == "stop":
            self.get_logger().info('Stop mission command received')
            self.mission_active = True

    def cmd_vel_timer_callback(self):
        if self.mission_active:
            if self.last_cmd_vel_msg is None:
                # Publish zeros if no cmd_vel messages received
                zero_msg = Twist()
                self.cmd_vel_publisher.publish(zero_msg)
            else:
                self.cmd_vel_publisher.publish(self.last_cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = JoystickNode()
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger('joystick_node').error(f'Error in main: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
