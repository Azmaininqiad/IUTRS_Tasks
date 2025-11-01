#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import threading
import time

class TurtleMotionController(Node):
    def __init__(self):
        super().__init__('turtle_motion_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.mode = None
        self.get_logger().info("Press A (Circle), B (Square), or C (Outward Spiral) to start.")

        # Start a background thread to read user input
        thread = threading.Thread(target=self.get_user_input)
        thread.daemon = True
        thread.start()

        # Common timer for all motion updates
        self.timer = self.create_timer(0.01, self.motion_callback)
        self.state = "FORWARD"
        self.side = 0
        self.state_start = self.get_clock().now()
        self.start_time = time.time()

    def get_user_input(self):
        """Run in a separate thread to capture key input."""
        while True:
            user_input = input("Enter command (A=Circle, B=Square, C=Spiral): ").strip().upper()
            if user_input in ["A", "B", "C"]:
                self.mode = user_input
                self.get_logger().info(f"Mode switched to {self.mode}")
                self.state = "FORWARD"
                self.side = 0
                self.state_start = self.get_clock().now()
                self.start_time = time.time()
            else:
                self.get_logger().warn("Invalid input! Use A, B, or C.")

    def motion_callback(self):
        if self.mode is None:
            return

        twist = Twist()

        # A) Circle 
        if self.mode == "A":
            twist.linear.x = 2.0
            twist.angular.z = 1.8

        # B) Square 
        elif self.mode == "B":
            now = self.get_clock().now()
            elapsed = (now - self.state_start).nanoseconds / 1e9

            if self.state == "FORWARD":
                twist.linear.x = 2.0
                if elapsed > 1.0:
                    twist.linear.x = 0.0
                    self.publisher_.publish(twist)
                    self.state = "TURN"
                    self.state_start = now

            elif self.state == "TURN":
                twist.angular.z = math.pi / 4
                if elapsed >= 2.0:
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    self.side = (self.side + 1) % 4
                    self.state = "FORWARD"
                    self.state_start = now
                    if self.side == 0:
                        self.get_logger().info("Completed a square loop!")

        # C) Outward Spiral 
        elif self.mode == "C":
            elapsed = time.time() - self.start_time
            twist.linear.x = 2.0 + (elapsed * 0.2)  
            twist.angular.z = 1.8

        # Publish the twist message
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
