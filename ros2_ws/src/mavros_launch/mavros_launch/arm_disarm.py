import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool

class ArmDisarmNode(Node):
    def __init__(self):
        super().__init__('arm_disarm_node')
        self.subscription = self.create_subscription(
            String,
            '/arm',
            self.arm_disarm_callback,
            10
        )
        self.get_logger().info("ArmDisarmNode has been started.")
        
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm service...')
        
    def arm_disarm_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        if command == 'arm':
            self.arm_disarm(True)
        elif command == 'disarm':
            self.arm_disarm(False)
        else:
            self.get_logger().info(f"Invalid command: {command}")
    
    def arm_disarm(self, arm):
        req = CommandBool.Request()
        req.value = arm
        future = self.arm_service.call_async(req)
        future.add_done_callback(self.arm_disarm_response_callback)

    def arm_disarm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Success: Arming/Disarming succeeded.')
            else:
                self.get_logger().info('Error: Arming/Disarming failed.')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
