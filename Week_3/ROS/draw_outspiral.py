#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub=self.create_publisher(Twist,'turtle1/cmd_vel',10)
        self.timer=self.create_timer(0.1,self.timer_callback)
        self.current_time=0.0
    def timer_callback(self):

        self.current_time += 0.05 # did that for increasing radius with time
        linear_x=2.0 +(self.current_time * 0.1)
        twist=Twist()
        twist.linear.x=2.0 +(self.current_time * 1)
        twist.angular.z=1.8
        self.cmd_vel_pub.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node=DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=='__main__':
    main()