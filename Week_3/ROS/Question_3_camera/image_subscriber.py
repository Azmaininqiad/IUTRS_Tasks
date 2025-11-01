import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
    
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        
        # Convert ROS Image message to OpenCV image
        # Using 'bgr8' matches the encoding set by the publisher
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # Display the image
        cv2.imshow("camera", current_frame)
        
        # Key line for OpenCV to update the window
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Crucial: Destroy the node and then clean up OpenCV windows
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows() # Clean up all OpenCV windows

if __name__ == '__main__':
    main()