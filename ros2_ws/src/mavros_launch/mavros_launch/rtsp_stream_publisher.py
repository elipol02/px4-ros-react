import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RTSPStreamPublisher(Node):
    def __init__(self):
        super().__init__('rtsp_stream_publisher')
        
        # Set the QoS profile for best effort delivery
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(Image, '/rtsp_stream', qos_profile)
        self.bridge = CvBridge()
        
        # Open the RTSP stream
        self.cap = cv2.VideoCapture('rtsp://192.168.168.100:8900/live')

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open RTSP stream')
            return
        
        # Increase the buffer size if needed
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        
        # Set the resolution (if known)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Set the timer to call publish_frame at a specific interval
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from RTSP stream')
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert frame: {e}')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RTSPStreamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
