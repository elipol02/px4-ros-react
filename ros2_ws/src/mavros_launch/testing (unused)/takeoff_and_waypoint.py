import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from mavros_msgs.msg import Waypoint
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
import time

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission')
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.wp_clear_client = self.create_client(WaypointClear, 'mavros/mission/clear')
        self.wp_push_client = self.create_client(WaypointPush, 'mavros/mission/push')
        self.wp_set_current_client = self.create_client(WaypointSetCurrent, 'mavros/mission/set_current')

        self.takeoff()

    def takeoff(self):
        # Clear existing waypoints
        self.clear_waypoints()

        # Arm the drone
        self.arm_drone(True)

        # Set to takeoff mode
        self.set_mode('AUTO.TAKEOFF')

        # Set a single takeoff waypoint
        takeoff_wp = Waypoint()
        takeoff_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        takeoff_wp.command = 22  # CMD_NAV_TAKEOFF
        takeoff_wp.is_current = True
        takeoff_wp.autocontinue = True
        takeoff_wp.param1 = 0.0  # Minimum pitch (if airframe capable)
        takeoff_wp.param4 = 0.0  # Yaw
        takeoff_wp.x_lat = 47.397742  # Latitude
        takeoff_wp.y_long = 8.545594  # Longitude
        takeoff_wp.z_alt = 10.0  # Altitude

        # Set the waypoint
        self.set_waypoints([takeoff_wp])

        # Wait a bit before setting to mission mode
        self.get_logger().info('Waiting for takeoff...')
        time.sleep(10)

        # Set to mission mode
        self.set_mode('AUTO.MISSION')

        # Create and upload waypoints for the mission
        waypoint = Waypoint()
        waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint.command = 16  # CMD_NAV_WAYPOINT
        waypoint.is_current = False
        waypoint.autocontinue = True
        waypoint.x_lat = 47.398
        waypoint.y_long = 8.545
        waypoint.z_alt = 20.0

        self.set_waypoints([takeoff_wp, waypoint])
        self.get_logger().info('Mission started.')

    def arm_drone(self, arm: bool):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        
        request = CommandBool.Request()
        request.value = arm

        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Drone {"armed" if arm else "disarmed"} successfully.')
        else:
            self.get_logger().error(f'Failed to {"arm" if arm else "disarm"} drone.')

    def set_mode(self, mode: str):
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')
        
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().mode_sent:
            self.get_logger().info(f'Mode set to {mode} successfully.')
        else:
            self.get_logger().error(f'Failed to set mode to {mode}.')

    def clear_waypoints(self):
        while not self.wp_clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint clear service not available, waiting...')
        
        request = WaypointClear.Request()

        future = self.wp_clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Waypoints cleared successfully.')
        else:
            self.get_logger().error('Failed to clear waypoints.')

    def set_waypoints(self, waypoints):
        while not self.wp_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waypoint push service not available, waiting...')
        
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints

        future = self.wp_push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Waypoints set successfully.')
        else:
            self.get_logger().error('Failed to set waypoints.')

def main(args=None):
    rclpy.init(args=args)
    drone_mission = DroneMission()
    rclpy.spin(drone_mission)

    drone_mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
