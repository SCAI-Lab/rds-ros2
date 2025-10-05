#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import threading
import time

class PipelineMonitor(Node):
    def __init__(self):
        super().__init__('pipeline_monitor')
        
        # Message counters
        self.scan_left_count = 0
        self.scan_right_count = 0
        self.points_left_count = 0
        self.points_right_count = 0
        self.points_all_count = 0
        self.points_filtered_count = 0
        self.command_count = 0
        
        # Timestamp tracking
        self.last_timestamps = {}
        
        # Subscriptions
        self.create_subscription(LaserScan, '/scan_left', 
                                 lambda msg: self.count_callback('scan_left', msg), 10)
        self.create_subscription(LaserScan, '/scan_right', 
                                 lambda msg: self.count_callback('scan_right', msg), 10)
        self.create_subscription(PointCloud2, '/scan/left/points', 
                                 lambda msg: self.count_callback('points_left', msg), 10)
        self.create_subscription(PointCloud2, '/scan/right/points', 
                                 lambda msg: self.count_callback('points_right', msg), 10)
        self.create_subscription(PointCloud2, '/rds/input/all/points', 
                                 lambda msg: self.count_callback('points_all', msg), 10)
        self.create_subscription(PointCloud2, '/rds/input/filtered/points', 
                                 lambda msg: self.count_callback('points_filtered', msg), 10)
        self.create_subscription(Twist, '/rds_modulated_cmd_vel', 
                                 lambda msg: self.count_callback('command', msg), 10)
        
        # Create timer for status reporting
        self.timer = self.create_timer(1.0, self.report_status)
        self.get_logger().info("Pipeline monitor started")
        
        # Thread to test RDS with commands
        self.test_thread = threading.Thread(target=self.send_test_commands)
        self.test_thread.daemon = True
        self.test_thread.start()
        
        # Create publisher for test commands
        self.command_publisher = self.create_publisher(Twist, '/remote_cmd_vel', 10)
    
    def count_callback(self, topic_name, msg):
        # Update counter
        if topic_name == 'scan_left':
            self.scan_left_count += 1
        elif topic_name == 'scan_right':
            self.scan_right_count += 1
        elif topic_name == 'points_left':
            self.points_left_count += 1
        elif topic_name == 'points_right':
            self.points_right_count += 1
        elif topic_name == 'points_all':
            self.points_all_count += 1
        elif topic_name == 'points_filtered':
            self.points_filtered_count += 1
        elif topic_name == 'command':
            self.command_count += 1
        
        # Update timestamp
        now = time.time()
        self.last_timestamps[topic_name] = now
    
    def report_status(self):
        self.get_logger().info("\n--- Pipeline Status ---")
        self.get_logger().info(f"LaserScan Left: {self.scan_left_count}")
        self.get_logger().info(f"LaserScan Right: {self.scan_right_count}")
        self.get_logger().info(f"PointCloud Left: {self.points_left_count}")
        self.get_logger().info(f"PointCloud Right: {self.points_right_count}")
        self.get_logger().info(f"PointCloud All: {self.points_all_count}")
        self.get_logger().info(f"PointCloud Filtered: {self.points_filtered_count}")
        self.get_logger().info(f"Commands: {self.command_count}")
        
        # Check for potential issues
        now = time.time()
        for topic, timestamp in self.last_timestamps.items():
            if now - timestamp > 5.0:  # More than 5 seconds since last message
                self.get_logger().warn(f"No recent messages on {topic}!")
    
    def send_test_commands(self):
        """Send test velocity commands to trigger RDS processing"""
        time.sleep(5.0) 
        
        self.get_logger().info("Starting to send test commands...")
        
        while True:
            # Create a simple command
            cmd = Twist()
            cmd.linear.x = 0.5  # Forward at 0.5 m/s
            cmd.angular.z = 0.2  # Turn slightly
            
            self.command_publisher.publish(cmd)
            time.sleep(1.0)  # Send command every second

def main(args=None):
    rclpy.init(args=args)
    node = PipelineMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()