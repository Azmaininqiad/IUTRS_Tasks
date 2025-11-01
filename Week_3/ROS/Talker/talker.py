import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AltairPublisher(Node):
  def __init__(self):
    super().__init__('altair_publisher')

    self.publisher = self.create_publisher(String,'altair',10)
    timer_period = 1.0

    self.timer = self.create_timer(timer_period,self.timer_callback)

    self.get_logger().info('Altair Publisher has been started')

  def timer_callback(self):
    msg = String() # inits an msg object of the type String
    msg.data = f'Hello Altair! '
    self.publisher.publish(msg)
    self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
  rclpy.init(args=args)
  node = AltairPublisher()
  rclpy.spin(node)
  rclpy.shutdown()
if __name__ == '__main__':
    main()