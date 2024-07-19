import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, ExtendedState, SysStatus, Altitude, StatusText
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
import json

class PresetTopicAggregator(Node):
    def __init__(self):
        super().__init__('preset_topic_aggregator')

        try:
            # Hardcoded parameters with actual MAVROS message types and QoS settings
            self.topics = [
                {
                    'name': '/mavros/state',
                    'msg_type': State,
                    'keys': [('connected',), ('armed',), ('mode',), ('system_status',)],
                    'new_keys': ['connected', 'armed', 'mode', 'system_status'],
                    'qos': {'reliability': ReliabilityPolicy.RELIABLE, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/extended_state',
                    'msg_type': ExtendedState,
                    'keys': [('landed_state',)],
                    'new_keys': ['landed_state'],
                    'qos': {'reliability': ReliabilityPolicy.RELIABLE, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/sys_status',
                    'msg_type': SysStatus,
                    'keys': [('battery_remaining',)],
                    'new_keys': ['battery'],
                    'qos': {'reliability': ReliabilityPolicy.RELIABLE, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/global_position/raw/satellites',
                    'msg_type': NavSatFix,
                    'keys': [('data')],
                    'new_keys': ['satellites'],
                    'qos': {'reliability': ReliabilityPolicy.RELIABLE, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/altitude',
                    'msg_type': Altitude,
                    'keys': [('relative',)],
                    'new_keys': ['altitude'],
                    'qos': {'reliability': ReliabilityPolicy.BEST_EFFORT, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/global_position/global',
                    'msg_type': NavSatFix,
                    'keys': [('latitude',), ('longitude',)],
                    'new_keys': ['latitude', 'longitude'],
                    'qos': {'reliability': ReliabilityPolicy.BEST_EFFORT, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/global_position/compass_hdg',
                    'msg_type': Float64,
                    'keys': [('data',)],
                    'new_keys': ['compass_hdg'],
                    'qos': {'reliability': ReliabilityPolicy.BEST_EFFORT, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/statustext/recv',
                    'msg_type': StatusText,
                    'keys': [('severity',), ('text',), ('header', 'stamp', 'sec'), ('header', 'stamp', 'nanosec')],
                    'new_keys': ['severity', 'message', 'seconds', 'nanoseconds'],
                    'qos': {'reliability': ReliabilityPolicy.BEST_EFFORT, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/local_position/velocity_local',
                    'msg_type': TwistStamped,
                    'keys': [('twist', 'linear', 'x'), ('twist', 'linear', 'y'), ('twist', 'linear', 'z')],
                    'new_keys': ['Xvel', 'Yvel', 'Zvel'],
                    'qos': {'reliability': ReliabilityPolicy.BEST_EFFORT, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/local_position/pose',
                    'msg_type': PoseStamped,
                    'keys': [
                        ('pose', 'position', 'x'), ('pose', 'position', 'y'),
                        ('pose', 'orientation', 'x'), ('pose', 'orientation', 'y'),
                        ('pose', 'orientation', 'z'), ('pose', 'orientation', 'w')
                    ],
                    'new_keys': ['Xpos', 'Ypos', 'quatX', 'quatY', 'quatZ', 'quatW'],
                    'qos': {'reliability': ReliabilityPolicy.BEST_EFFORT, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                },
                {
                    'name': '/mavros/home_position/home',
                    'msg_type': NavSatFix,
                    'keys': [('latitude',), ('longitude',), ('altitude',)],
                    'new_keys': ['homeLat', 'homeLon', 'homeAlt'],
                    'qos': {'reliability': ReliabilityPolicy.RELIABLE, 'durability': DurabilityPolicy.VOLATILE, 'history': HistoryPolicy.KEEP_LAST, 'depth': 10}
                }
            ]
            self.output_topic = '/aggregated_data'

            self.data = {}  # Initialize the data attribute

            self.get_logger().info('Initializing subscribers for the following topics:')
            for topic_info in self.topics:
                topic_name = topic_info['name']
                msg_type = topic_info['msg_type']
                keys_to_display = topic_info['keys']
                new_key_names = topic_info['new_keys']
                qos_settings = topic_info['qos']
                
                qos_profile = QoSProfile(
                    reliability=qos_settings['reliability'],
                    durability=qos_settings['durability'],
                    history=qos_settings['history'],
                    depth=qos_settings['depth']
                )

                self.get_logger().info(
                    f'Topic: {topic_name}, '
                    f'Keys: {keys_to_display}, '
                    f'New Keys: {new_key_names}, '
                    f'QoS: {qos_profile}'
                )

                self.create_subscription(
                    msg_type, 
                    topic_name, 
                    self.create_topic_callback(keys_to_display, new_key_names),
                    qos_profile
                )

            self.publisher = self.create_publisher(String, self.output_topic, 10)
            self.timer = self.create_timer(0.1, self.publish_aggregated_data)  # Set timer to 10 Hz
            self.get_logger().info(f'Publishing aggregated data to topic: {self.output_topic}')
        except Exception as e:
            self.get_logger().error(f'Error during initialization: {e}')

    def create_topic_callback(self, keys_to_display, new_key_names):
        def topic_callback(msg):
            extracted_message = self.extract_keys(msg, keys_to_display)
            
            renamed_message = {
                new_key_names[i]: extracted_message[key] 
                for i, key in enumerate(keys_to_display) 
                if key in extracted_message
            }

            # Store the renamed message
            for i, key in enumerate(new_key_names):
                if key in renamed_message:
                    self.data[key] = renamed_message[key]

        return topic_callback

    def extract_keys(self, msg, keys):
        extracted = {}
        for key in keys:
            value = msg
            for part in key:
                value = getattr(value, part, None)
                if value is None:
                    break
            if value is not None:
                extracted[key] = value
        return extracted

    def publish_aggregated_data(self):
        if not self.data:
            self.get_logger().debug('No data to publish.')
            return

        # Publish only the aggregated values, not including topic names
        aggregated_message = json.dumps(self.data)
        self.publisher.publish(String(data=aggregated_message))

def main(args=None):
    rclpy.init(args=args)
    node = PresetTopicAggregator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
