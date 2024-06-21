import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelRepublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_republisher')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10)
        self.log_publisher = self.create_publisher(
            String,
            '/cmd_vel_republisher/log',
            10)
        self.log_message('Node started, listening on /cmd_vel and publishing to /mavros/setpoint_velocity/cmd_vel_unstamped')

    def cmd_vel_callback(self, msg):
        self.publisher.publish(msg)

    def log_message(self, message):
        log_msg = String()
        log_msg.data = message
        self.log_publisher.publish(log_msg)
        # Also print to console for immediate feedback
        self.get_logger().info(message)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelRepublisher()
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger('cmd_vel_republisher').error(f'Error in main: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
