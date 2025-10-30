import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

# Configuration 

VIDEO_SOURCE = "/home/jazakallah/ros2_ws/src/robot_controller/robot_controller/abc.mov"

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # timer to publish frames at a set rate (e.g., 30 Hz)
        self.timer_period = 1.0 / 30.0  # 30 frames per second
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Initialize OpenCV video capture
        self.cap = cv2.VideoCapture(VIDEO_SOURCE)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video source: {VIDEO_SOURCE}')
            sys.exit(1)

        self.get_logger().info(f'Video source opened successfully: {VIDEO_SOURCE}')
        
        # Initialize CvBridge
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # The default encoding from cv2.VideoCapture.read() is BGR8.
            # The encoding is set to 'bgr8' to match OpenCV's default color order.
            ros_image_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(ros_image_msg)
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().warn('Failed to read frame from video source.')

    def destroy_node(self):
        self.cap.release() # Release the video capture resource
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()