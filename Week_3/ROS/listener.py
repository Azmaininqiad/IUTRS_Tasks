import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AltairSubscriber(Node):
    def __init__(self):
        super().__init__('altair_subscriber')
        self.subscription = self.create_subscription(String,'altair',self.listener_callback,100)
        
        self.get_logger().info('Altair Subscriber has been started')
        self.get_logger().info('Waiting for messages on /altair topic...')

    def listener_callback(self,msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    altair_subscriber = AltairSubscriber()
    
    try:
        # Spin the node so the callback function is called
        rclpy.spin(altair_subscriber)
    except KeyboardInterrupt:
        altair_subscriber.get_logger().info('Node interrupted by user')
    finally:
        # Explicitly destroy the node
        altair_subscriber.destroy_node()
        
        # Shutdown the ROS client library for Python
        rclpy.shutdown()


if __name__ == '__main__':
    main()
