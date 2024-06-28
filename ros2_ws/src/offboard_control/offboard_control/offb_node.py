#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
import math

class TakeoffNode(Node):

    def __init__(self):
        super().__init__('takeoff_node')

        self.state = State()
        self.local_position = PoseStamped()
        self.offboard_set = False
        self.armed = False
        self.target_received = False

        self.home_lat = None
        self.home_lon = None

        self.subscription = self.create_subscription(Point, '/flytopoint', self.listener_callback, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.home_pos_sub = self.create_subscription(HomePosition, '/mavros/home_position/home', self.home_pos_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.offboard_setpoint_counter = 0
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.x = 0
        self.y = 0
        self.z = 10.0
        self.get_logger().info('Takeoff Node has been started.')
        self.get_logger().info('Wating for /Flytopoint.')

    def state_cb(self, msg):
        self.state = msg

    def home_pos_cb(self, msg):
        self.home_lat = msg.geo.latitude
        self.home_lon = msg.geo.longitude
        self.get_logger().info(f'Home position set: lat={self.home_lat}, lon={self.home_lon}')

    def timer_cb(self):
        if not self.target_received:
            return
        
        self.publish_setpoint()
        self.set_offboard_mode()
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 10:
                self.get_logger().info('Sent 10 setpoints.')
        elif self.state.mode != "OFFBOARD":
            self.set_offboard_mode()
        elif not self.state.armed and not self.armed:
            self.arm()

    def publish_setpoint(self):
        self.local_position.header.stamp = self.get_clock().now().to_msg()
        self.local_position.pose.position.x = self.x
        self.local_position.pose.position.y = self.y
        self.local_position.pose.position.z = self.z

        self.local_pos_pub.publish(self.local_position)
        
    def set_offboard_mode(self):
        if self.state.mode != "OFFBOARD":
            self.get_logger().info('Setting Offboard mode.')
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'

            future = self.set_mode_client.call_async(req)
            self.get_logger().info('Offboard mode request sent.')

    def arm(self):
        if not self.state.armed:
            self.get_logger().info('Arming the drone.')
            req = CommandBool.Request()
            req.value = True

            future = self.arming_client.call_async(req)
            self.get_logger().info('Arming request sent.')

    def listener_callback(self, msg):
        if self.home_lat is None or self.home_lon is None:
            self.get_logger().warn('Home position not set yet. Ignoring target coordinates.')
            return
        
        self.x, self.y = self.convert_to_local_coordinates(msg.x, msg.y)
        self.z = msg.z
        self.target_received = True
        self.get_logger().info(f'Received target coordinates: lat={msg.x}, lon={msg.y}, alt={msg.z}')
        self.get_logger().info(f'Converted target coordinates: x={self.x}, y={self.y}, z={self.z}')
        self.get_logger().info('Setting target coordinates and preparing for takeoff.')

    def convert_to_local_coordinates(self, lat, lon):
        # WGS84 ellipsoid constants
        a = 6378137.0
        f = 1 / 298.257223563
        b = (1 - f) * a
        
        # Convert degrees to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        home_lat_rad = math.radians(self.home_lat)
        home_lon_rad = math.radians(self.home_lon)

        # Differences in latitude and longitude
        d_lat = lat_rad - home_lat_rad
        d_lon = lon_rad - home_lon_rad

        # Transverse radius of curvature
        N = a / math.sqrt(1 - f * (2 - f) * math.sin(home_lat_rad)**2)

        # Local ENU coordinates
        x = N * d_lon * math.cos(home_lat_rad)
        y = N * d_lat

        return x, y

def main(args=None):
    rclpy.init(args=args)
    takeoff_node = TakeoffNode()
    rclpy.spin(takeoff_node)

    takeoff_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
