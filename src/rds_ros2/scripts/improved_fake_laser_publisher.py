#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
# from rclcpp_lifecycle.lifecycle_node import LifecycleNode

class ImprovedFakeLaserPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_publisher')
        
        # Declare parameters for scan configuration
        self.declare_parameter('left_scan_topic', '/scan_left')
        self.declare_parameter('right_scan_topic', '/scan_right')
        self.declare_parameter('rate', 10.0)  # Hz
        self.declare_parameter('num_points', 360)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        
        # Obstacle parameters
        self.declare_parameter('obstacle_scenario', 'hallway')  # Options: 'hallway', 'dynamic', 'front'
        self.declare_parameter('hallway_width', 3.0)  # Width of hallway in meters
        self.declare_parameter('dynamic_obstacle_speed', 0.5)  # Speed of dynamic obstacle in m/s
        
        # Get parameters
        self.left_scan_topic = self.get_parameter('left_scan_topic').value
        self.right_scan_topic = self.get_parameter('right_scan_topic').value
        self.rate = self.get_parameter('rate').value
        self.num_points = self.get_parameter('num_points').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.obstacle_scenario = self.get_parameter('obstacle_scenario').value
        self.hallway_width = self.get_parameter('hallway_width').value
        self.dynamic_obstacle_speed = self.get_parameter('dynamic_obstacle_speed').value
        
        # Create publishers
        self.left_publisher = self.create_publisher(LaserScan, self.left_scan_topic, 10)
        self.right_publisher = self.create_publisher(LaserScan, self.right_scan_topic, 10)
        
        # Setup timer for publishing
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # Dynamic obstacle position
        self.obstacle_x = 5.0
        self.obstacle_y = 0.0
        self.obstacle_direction = -1.0  # Direction: -1 = left, 1 = right
        self.obstacle_radius = 0.5  # Size of dynamic obstacle
        
        self.get_logger().info(f"Fake laser publisher started with '{self.obstacle_scenario}' scenario")
        self.get_logger().info(f"Publishing to {self.left_scan_topic} and {self.right_scan_topic}")
    
    def timer_callback(self):
        # Update dynamic obstacle position if using that scenario
        if self.obstacle_scenario == 'dynamic':
            self.update_dynamic_obstacle()
        
        # Create left and right scan messages
        left_scan = self.create_scan_msg("base_link", -math.pi/2, math.pi/2)
        right_scan = self.create_scan_msg("base_link", math.pi/2, 3*math.pi/2)
        
        # Publish messages
        self.left_publisher.publish(left_scan)
        self.right_publisher.publish(right_scan)
    
    def update_dynamic_obstacle(self):
        # Move obstacle across the robot's path
        self.obstacle_y += self.obstacle_direction * self.dynamic_obstacle_speed * (1.0/self.rate)
        
        # Reverse direction at boundaries
        if abs(self.obstacle_y) > 5.0:
            self.obstacle_direction *= -1
    
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
        
        # Generate ranges based on the scenario
        ranges = np.ones(self.num_points) * self.range_max
        
        if self.obstacle_scenario == 'hallway':
            # Create parallel walls on either side
            for i in range(self.num_points):
                angle = angle_min + i * scan.angle_increment
                
                # Calculate distance to left wall (y = hallway_width/2)
                if abs(angle - math.pi/2) < math.pi/2:  # Forward-facing angles
                    left_dist = (self.hallway_width/2) / math.cos(angle)
                    if 0 < left_dist < self.range_max:
                        ranges[i] = min(ranges[i], left_dist)
                
                # Calculate distance to right wall (y = -hallway_width/2)
                if abs(angle + math.pi/2) < math.pi/2:  # Forward-facing angles
                    right_dist = (self.hallway_width/2) / math.cos(angle)
                    if 0 < right_dist < self.range_max:
                        ranges[i] = min(ranges[i], right_dist)
        
        elif self.obstacle_scenario == 'dynamic':
            # Add a moving obstacle
            for i in range(self.num_points):
                angle = angle_min + i * scan.angle_increment
                
                # Calculate ray from robot
                ray_x = math.cos(angle)
                ray_y = math.sin(angle)
                
                # Calculate closest point on ray to obstacle center
                t = ray_x * self.obstacle_x + ray_y * self.obstacle_y
                
                if t > 0:  # Only consider points in front of the ray origin
                    # Closest point on ray to obstacle center
                    closest_x = t * ray_x
                    closest_y = t * ray_y
                    
                    # Distance from closest point to obstacle center
                    dist_to_center = math.sqrt((closest_x - self.obstacle_x)**2 + 
                                             (closest_y - self.obstacle_y)**2)
                    
                    # If ray passes through the obstacle
                    if dist_to_center < self.obstacle_radius:
                        # Calculate intersection points
                        d = math.sqrt(self.obstacle_radius**2 - dist_to_center**2)
                        intersection_dist = t - d
                        
                        if intersection_dist > 0 and intersection_dist < self.range_max:
                            ranges[i] = min(ranges[i], intersection_dist)
        
        elif self.obstacle_scenario == 'front':
            # Add a static obstacle directly in front
            for i in range(self.num_points):
                angle = angle_min + i * scan.angle_increment
                
                # Obstacle at 3 meters directly ahead
                if abs(angle) < math.pi/6:  # Within 30 degrees of straight ahead
                    ranges[i] = min(ranges[i], 1.0)
        
        scan.ranges = ranges.tolist()
        scan.intensities = [1.0] * self.num_points  # Fixed intensity
        
        return scan

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedFakeLaserPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()