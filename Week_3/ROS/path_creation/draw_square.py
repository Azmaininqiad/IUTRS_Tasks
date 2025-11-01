#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class DrawSquareNode(Node):
    def __init__(self):
        super().__init__('draw_square')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.get_logger().info("Starting to draw continuous squares...")

        # State machine variables
        self.state = "FORWARD"
        self.side = 0
        self.state_start = self.get_clock().now()

        # Timer (called every 0.1 sec)
        self.timer = self.create_timer(0.001, self.update_motion)

    def update_motion(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds /1e9
        twist = Twist()

        if self.state == "FORWARD":
            twist.linear.x = 2.0
            
            # The movement duration is 1.0 second
            if elapsed > 1.0:
                # To prevent the turtle from drawing a diagonal line at the start of the turn,
                twist.linear.x = 0.0 
                self.publisher_.publish(twist)
                
                self.state = "TURN"
                self.state_start = now
                self.get_logger().info(f"Turning corner {self.side + 1}")

        elif self.state == "TURN":
            # Command slower angular velocity: pi/4 rad/s (45 deg/s)
            twist.angular.z = math.pi/4 
            
            # Turn for 2.0 seconds (2.0 * pi/4 = pi/2 radians or 90 degrees)
            if elapsed >= 2.0:    
                # Used larger time threshold (>=) to ensure precise stopping
                # previously tested and got inaccurate squares
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                
                self.state = "FORWARD"
                self.side = (self.side + 1) % 4
                self.state_start = now
                
                if self.side == 0:
                    self.get_logger().info("Completed a square loop!")
        
        # Publish the current command
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()