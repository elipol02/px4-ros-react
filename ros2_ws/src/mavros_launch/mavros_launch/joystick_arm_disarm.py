import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String

class JoystickArmDisarm(Node):

    def __init__(self):
        super().__init__('joystick_arm_disarm')
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.log_publisher = self.create_publisher(
            String,
            '/joystick_arm_disarm/log',
            10)
        
        self.arm_service_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        
        while not self.arm_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm/disarm service...')
        
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        
        self.get_logger().info('Node started, listening to /joy')

        self.previous_buttons = []
        self.last_command_was_arm = False  # Track the last command state

    def joy_callback(self, msg):
        arm_button_index = 1  # Replace with the index of the button you want to use
        disarm_button_index = 0  # Replace with the index of the button you want to use

        if not self.previous_buttons:
            self.previous_buttons = msg.buttons
            return

        if msg.buttons[arm_button_index] == 1 and self.previous_buttons[arm_button_index] == 0:
            self.log_message('Arm button pressed')
            self.set_position_mode()
            self.last_command_was_arm = True
            self.call_arm_service(True)
        elif msg.buttons[disarm_button_index] == 1 and self.previous_buttons[disarm_button_index] == 0:
            self.log_message('Disarm button pressed')
            self.last_command_was_arm = False
            self.call_arm_service(False)

        self.previous_buttons = msg.buttons

    def call_arm_service(self, arm):
        req = CommandBool.Request()
        req.value = arm
        future = self.arm_service_client.call_async(req)
        future.add_done_callback(self.arm_service_response_callback)

    def arm_service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.log_message('Arm/Disarm command successful')
            else:
                self.log_message('Arm/Disarm command failed')
        except Exception as e:
            self.log_message(f'Arm/Disarm service call failed: {str(e)}')

    def set_position_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'POSCTL'
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.set_mode_response_callback)

    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.log_message('position mode set successfully')
            else:
                self.log_message('Failed to set position mode')
        except Exception as e:
            self.log_message(f'Set mode service call failed: {str(e)}')

    def log_message(self, message):
        log_msg = String()
        log_msg.data = message
        self.log_publisher.publish(log_msg)
        self.get_logger().info(message)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickArmDisarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
