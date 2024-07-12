import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import SetMode

class ModeSetNode(Node):
    def __init__(self):
        super().__init__('mode_set_node')
        self.subscription = self.create_subscription(
            String,
            '/modeset',
            self.mode_set_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('ModeSetNode has been started.')

    def mode_set_callback(self, msg):
        mode = msg.data
        self.get_logger().info(f'Received mode: {mode}')

        client = self.create_client(SetMode, '/mavros/set_mode')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/mavros/set_mode service not available, waiting...')

        request = SetMode.Request()
        request.custom_mode = mode

        future = client.call_async(request)
        future.add_done_callback(self.set_mode_response_callback)

    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Mode set successfully.')
            else:
                self.get_logger().warn('Failed to set mode.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ModeSetNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
