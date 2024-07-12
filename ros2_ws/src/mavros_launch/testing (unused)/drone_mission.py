import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached
from std_msgs.msg import String
import time

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission')
        self.wp_clear_client = self.create_client(WaypointClear, 'mavros/mission/clear')
        self.wp_push_client = self.create_client(WaypointPush, 'mavros/mission/push')

        self.arm_publisher = self.create_publisher(String, '/arm', 10)
        self.set_mode_publisher = self.create_publisher(String, '/mode_set', 10)
        
        self.waypoints_subscriber = self.create_subscription(
            WaypointList, 
            'mavros/mission/waypoints', 
            self.waypoints_callback, 
            10)
        
        self.start_mission_subscriber = self.create_subscription(
            String, 
            '/startMission', 
            self.start_mission_callback, 
            10)
        
        self.drone_status = {
            'armed': False,
            'mode': 'MANUAL',
            'waypoints_cleared': False,
        }

        self.get_logger().info('DroneMission node started')

    def waypoints_callback(self, msg):
        self.get_logger().info('Received new waypoints')
        self.clear_waypoints()

        # Store the waypoints to be uploaded once cleared
        self.new_waypoints = msg.waypoints

    def start_mission_callback(self, msg):
        if msg.data == 'start':
            self.get_logger().info('Received start mission command')
            if self.drone_status['mode'] == 'AUTO.MISSION' and self.drone_status['armed']:
                self.set_mode('AUTO.MISSION')
            else:
                if not self.drone_status['armed']:
                    self.arm_drone(True)
                self.set_mode('AUTO.TAKEOFF')
                time.sleep(10)  # wait for takeoff
                self.set_mode('AUTO.MISSION')

    def arm_drone(self, arm: bool):
        arm_msg = String()
        arm_msg.data = 'arm' if arm else 'disarm'
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info(f'Published {arm_msg.data} to /arm')
        self.drone_status['armed'] = arm

    def set_mode(self, mode: str):
        mode_msg = String()
        mode_msg.data = mode
        self.set_mode_publisher.publish(mode_msg)
        self.get_logger().info(f'Published {mode} to /mode_set')
        self.drone_status['mode'] = mode

    def clear_waypoints(self):
        self.get_logger().info('Clearing waypoints')
        
        request = WaypointClear.Request()
        future = self.wp_clear_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Waypoints cleared successfully')
            self.drone_status['waypoints_cleared'] = True
            self.upload_waypoints()
        else:
            self.get_logger().error('Failed to clear waypoints')

    def upload_waypoints(self):
        if self.drone_status['waypoints_cleared']:
            self.get_logger().info('Uploading new waypoints')
            
            request = WaypointPush.Request()
            request.start_index = 0
            request.waypoints = self.new_waypoints

            future = self.wp_push_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result().success:
                self.get_logger().info('Waypoints set successfully')
            else:
                self.get_logger().error('Failed to set waypoints')
        else:
            self.get_logger().info('Waiting for waypoints to be cleared before uploading new waypoints')

def main(args=None):
    rclpy.init(args=args)
    drone_mission = DroneMission()
    rclpy.spin(drone_mission)

    drone_mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
