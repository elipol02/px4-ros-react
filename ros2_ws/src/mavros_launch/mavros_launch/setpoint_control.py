#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import HomePosition, Altitude
import math

class SetpointControl(Node):

    def __init__(self):
        super().__init__('setpoint_control')

        self.local_position = PoseStamped()
        self.path = None
        self.mission_started = False
        self.home_position_set = False
        self.previous_waypoint = None

        self.home_lat = None
        self.home_lon = None
        self.home_local_x = None
        self.home_local_y = None
        self.final_setpoint = None
        self.origin_lat = None
        self.origin_lon = None

        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.setpoint_sub = self.create_subscription(Path, '/setpoints', self.setpoint_callback, qos_profile_reliable)
        self.start_mission_sub = self.create_subscription(String, '/startmission', self.start_mission_callback, qos_profile_best_effort)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile_best_effort)
        self.setpoint_pub = self.create_publisher(Path, '/setpoints', qos_profile_reliable)
        self.home_pos_sub = self.create_subscription(HomePosition, '/mavros/home_position/home', self.home_pos_cb, qos_profile_best_effort)
        self.current_position_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.current_position_cb, qos_profile_best_effort)
        self.altitude_sub = self.create_subscription(Altitude, '/mavros/altitude', self.altitude_cb, qos_profile_best_effort)
        
        # Added publisher for /modeset with reliable and transient local QoS
        qos_profile_modeset = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.mode_set_pub = self.create_publisher(String, '/modeset', qos_profile_modeset)

        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('Setpoint Control Node has been started.')
        self.get_logger().info('Waiting for /setpoints.')

    def setpoint_callback(self, msg):
        self.path = msg
        self.final_setpoint = None  # Reset final setpoint when new path is received
        self.get_logger().info('Received new path.')

    def start_mission_callback(self, msg):
        if msg.data == "start":
            if self.home_position_set and self.local_position.pose.position.x != 0.0 and self.local_position.pose.position.y != 0.0:
                self.mission_started = True
                self.get_logger().info('Mission started.')
            else:
                self.get_logger().warn('Cannot start mission. Home position or local position is not set.')
        elif msg.data == "stop":
            self.mission_started = False
            self.get_logger().info('Mission stopped.')

    def home_pos_cb(self, msg):
        self.home_lat = msg.geo.latitude
        self.home_lon = msg.geo.longitude
        self.home_local_x = msg.position.x
        self.home_local_y = msg.position.y

        self.origin_lat, self.origin_lon = self.calculate_origin(self.home_lat, self.home_lon, self.home_local_x, self.home_local_y)

        self.home_position_set = True

    def current_position_cb(self, msg):
        # Convert the current global position to local coordinates
        lat = msg.latitude  # NavSatFix message type
        lon = msg.longitude  # NavSatFix message type
        local_x, local_y = self.convert_to_local_coordinates(lat, lon)

        self.local_position.pose.position.x = local_x
        self.local_position.pose.position.y = local_y

    def altitude_cb(self, msg):
        # Update the local altitude
        self.local_position.pose.position.z = msg.local

    def timer_cb(self):
        if not self.mission_started:
            return

        current_setpoint = None

        if self.path and len(self.path.poses) > 0:
            current_point = self.path.poses[0]
            lon = current_point.pose.position.x  # assuming x is longitude
            lat = current_point.pose.position.y  # assuming y is latitude
            local_point = self.convert_to_local_coordinates(lat, lon)

            if self.previous_waypoint is None or \
            (self.previous_waypoint.pose.position.x != current_point.pose.position.x or
                self.previous_waypoint.pose.position.y != current_point.pose.position.y or
                self.previous_waypoint.pose.position.z != current_point.pose.position.z):
                distance = self.calculate_distance(self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z,
                                                local_point[0], local_point[1], current_point.pose.position.z)
                self.get_logger().info(f'Flying to new waypoint: Current position: x={self.local_position.pose.position.x}, y={self.local_position.pose.position.y}, z={self.local_position.pose.position.z}')
                self.get_logger().info(f'Waypoint position: x={local_point[0]}, y={local_point[1]}, z={current_point.pose.position.z}')
                self.get_logger().info(f'Distance to waypoint: {distance}')
                self.previous_waypoint = current_point

            current_setpoint = PoseStamped()
            current_setpoint.header.stamp = self.get_clock().now().to_msg()
            current_setpoint.pose.position.x = local_point[0]
            current_setpoint.pose.position.y = local_point[1]
            current_setpoint.pose.position.z = current_point.pose.position.z
            self.local_pos_pub.publish(current_setpoint)

            if self.reached_setpoint(local_point[0], local_point[1], current_point.pose.position.z):
                self.path.poses.pop(0)
                self.setpoint_pub.publish(self.path)
                self.get_logger().info('Reached setpoint, waypoint popped and path updated.')
        else:
            # Publish "AUTO.LOITER" to /modeset
            mode_msg = String()
            mode_msg.data = "AUTO.LOITER"
            self.mode_set_pub.publish(mode_msg)

    def calculate_origin(self, home_lat, home_lon, home_local_x, home_local_y):
        R = 6378137.0  # Earth radius in meters

        # Convert degrees to radians
        home_lat_rad = math.radians(home_lat)
        home_lon_rad = math.radians(home_lon)

        # Calculate the origin latitude and longitude
        origin_lat = home_lat - (home_local_y / R) * (180 / math.pi)
        origin_lon = home_lon - (home_local_x / (R * math.cos(home_lat_rad))) * (180 / math.pi)

        return origin_lat, origin_lon

    def convert_to_local_coordinates(self, lat, lon):
        if self.origin_lat is None or self.origin_lon is None:
            return 0.0, 0.0

        # Convert degrees to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        # Earth radius in meters
        R = 6378137.0

        # Local coordinates in meters using Equirectangular projection
        x = R * (lon_rad - origin_lon_rad) * math.cos((origin_lat_rad + lat_rad) / 2)
        y = R * (lat_rad - origin_lat_rad)

        return x, y

    def reached_setpoint(self, x, y, z):
        tolerance = 0.5  # tolerance in meters
        distance = self.calculate_distance(self.local_position.pose.position.x, self.local_position.pose.position.y, self.local_position.pose.position.z,
                                           x, y, z)
        return distance < tolerance

    def calculate_distance(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def main(args=None):
    rclpy.init(args=args)
    setpoint_control = SetpointControl()
    rclpy.spin(setpoint_control)

    setpoint_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
