#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time

class FakeLaserPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_publisher')
        
        # Declare parameters
        self.declare_parameter('left_scan_topic', '/scan_left')
        self.declare_parameter('right_scan_topic', '/scan_right')
        self.declare_parameter('rate', 10.0)  # Hz
        self.declare_parameter('num_points', 360)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('simulate_obstacles', True)
        
        # Get parameters
        self.left_scan_topic = self.get_parameter('left_scan_topic').value
        self.right_scan_topic = self.get_parameter('right_scan_topic').value
        self.rate = self.get_parameter('rate').value
        self.num_points = self.get_parameter('num_points').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.simulate_obstacles = self.get_parameter('simulate_obstacles').value
        
        # Create publishers
        self.left_publisher = self.create_publisher(LaserScan, self.left_scan_topic, 10)
        self.right_publisher = self.create_publisher(LaserScan, self.right_scan_topic, 10)
        
        # Setup timer for publishing
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # Obstacle simulation variables
        self.obstacle_angle = 0.0
        self.obstacle_distance = 3.0
        
        self.get_logger().info("Fake laser scan publisher started")
        self.get_logger().info(f"Publishing to {self.left_scan_topic} and {self.right_scan_topic}")
    
    def timer_callback(self):
        # Create left and right scan messages
        left_scan = self.create_scan_msg("base_link", -math.pi/2, math.pi/2)
        right_scan = self.create_scan_msg("base_link", math.pi/2, 3*math.pi/2)
        
        # Publish messages
        self.left_publisher.publish(left_scan)
        self.right_publisher.publish(right_scan)
        
        # Update obstacle position for next iteration
        if self.simulate_obstacles:
            self.obstacle_angle += 0.05
            if self.obstacle_angle > 2 * math.pi:
                self.obstacle_angle = 0.0
    
    def create_scan_msg(self, frame_id, angle_min, angle_max):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = frame_id
        
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = (angle_max - angle_min) / self.num_points
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Generate random ranges with some obstacles
        ranges = np.ones(self.num_points) * self.range_max
        
        # Add some simulated obstacles
        if self.simulate_obstacles:
            # Static obstacles in a circle around the robot
            for i in range(self.num_points):
                angle = angle_min + i * scan.angle_increment
                
                # Add a moving obstacle
                obstacle_x = self.obstacle_distance * math.cos(self.obstacle_angle)
                obstacle_y = self.obstacle_distance * math.sin(self.obstacle_angle)
                
                # Distance from scan point to obstacle
                point_x = self.range_max * math.cos(angle)
                point_y = self.range_max * math.sin(angle)
                
                dist_to_obstacle = math.sqrt((point_x - obstacle_x)**2 + (point_y - obstacle_y)**2)
                
                if dist_to_obstacle < 0.5:  # Obstacle size
                    # Calculate distance from robot to obstacle along this ray
                    ray_dist = math.sqrt(obstacle_x**2 + obstacle_y**2) * math.cos(angle - math.atan2(obstacle_y, obstacle_x))
                    if self.range_min < ray_dist < self.range_max:
                        ranges[i] = ray_dist
                
                # Add some static obstacles
                if 0.8 < angle < 0.9 or 2.3 < angle < 2.4:
                    ranges[i] = 2.0
        
        scan.ranges = ranges.tolist()
        scan.intensities = [1.0] * self.num_points  # Fixed intensity
        
        return scan

def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()