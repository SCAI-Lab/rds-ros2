#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math

class ScanToPointcloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/scan/points')
        
        scan_topic = self.get_parameter('scan_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10)
        
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            cloud_topic,
            10)
        
        self.get_logger().info(f"Scan to Pointcloud converter started: {scan_topic} -> {cloud_topic}")
    
    def scan_callback(self, msg):
        try:
            points = []
            
            for i, r in enumerate(msg.ranges):
                if r < msg.range_min or r > msg.range_max or not math.isfinite(r):
                    continue

                if r < 0.1 or r > 10.0: 
                    continue
                
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                
                points.append([x, y, z])
            
            if points:
                cloud_msg = pc2.create_cloud_xyz32(msg.header, points)
                self.cloud_pub.publish(cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in scan callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()