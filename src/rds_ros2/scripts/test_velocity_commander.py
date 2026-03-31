#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class TestVelocityCommander(Node):
    def __init__(self):
        super().__init__('test_velocity_commander')
        
        # Declare parameters
        self.declare_parameter('cmd_topic', '/remote_cmd_vel')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.3)
        self.declare_parameter('command_pattern', 'forward_oscillate')
        
        # Get parameters
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.rate = self.get_parameter('rate').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.command_pattern = self.get_parameter('command_pattern').value
        
        # Create publisher
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # State variables
        self.start_time = time.time()
        self.phase = 0.0
        
        self.get_logger().info(f"Test velocity commander started, pattern: {self.command_pattern}")
        self.get_logger().info(f"Publishing to: {self.cmd_topic}")
    
    def timer_callback(self):
        # Create command message
        cmd = Twist()
        
        # Calculate elapsed time since start
        elapsed = time.time() - self.start_time
        
        # Generate velocity commands based on the pattern
        if self.command_pattern == 'forward':
            # Simple forward motion
            cmd.linear.x = self.max_linear
            cmd.angular.z = 0.0
            
        elif self.command_pattern == 'circle':
            # Circular motion
            cmd.linear.x = self.max_linear
            cmd.angular.z = self.max_angular
            
        elif self.command_pattern == 'forward_oscillate':
            # Forward with oscillating rotational component
            cmd.linear.x = self.max_linear
            cmd.angular.z = self.max_angular * math.sin(2.0 * math.pi * 0.1 * elapsed)
            
        elif self.command_pattern == 'varying':
            # Varying both linear and angular velocities
            cycle = 10.0  # seconds per full cycle
            phase = (elapsed % cycle) / cycle
            
            if phase < 0.25:  # First quarter: Accelerate forward
                cmd.linear.x = self.max_linear * (phase * 4)
                cmd.angular.z = 0.0
            elif phase < 0.5:  # Second quarter: Turn right while moving forward
                cmd.linear.x = self.max_linear
                cmd.angular.z = -self.max_angular * ((phase - 0.25) * 4)
            elif phase < 0.75:  # Third quarter: Turn left while moving forward
                cmd.linear.x = self.max_linear
                cmd.angular.z = self.max_angular * ((phase - 0.5) * 4)
            else:  # Fourth quarter: Decelerate
                cmd.linear.x = self.max_linear * (1.0 - (phase - 0.75) * 4)
                cmd.angular.z = 0.0
                
        else:
            # Default: do nothing
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(f"Unknown command pattern: {self.command_pattern}")
        
        # Publish the command
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TestVelocityCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()