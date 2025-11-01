#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class DrawCircleNode(Node):
    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub=self.create_publisher(Twist,'turtle1/cmd_vel',10) # creating a publisher object that can access cmd_vel topic of turtlesim
        self.timer=self.create_timer(0.1,self.timer_callback) #timer calls timer_callback function every 0.1 seconds
        
    def timer_callback(self):
        twist=Twist()
        twist.linear.x=2.0 # linear velocity along x-axis
        twist.angular.z=1.8 # angular velocity around z-axis (which we were taught as y axis from previous knowledge)
        self.cmd_vel_pub.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node=DrawCircleNode()
    rclpy.spin(node) # creating a loop
    rclpy.shutdown()
if __name__=='__main__':
    main()