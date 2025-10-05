#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg
import numpy as np
from collections import defaultdict
import threading
from functools import partial


class PointCloudConcatenate(Node):
    def __init__(self):
        super().__init__('pointcloud_concatenate')
        
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('clouds', 2)
        self.declare_parameter('hz', 20)
        
        self.target_frame = self.get_parameter('target_frame').value
        num_clouds = self.get_parameter('clouds').value
        self.publish_rate = self.get_parameter('hz').value
        
        # Store the latest clouds from each input
        self.latest_clouds = {}
        self.cloud_lock = threading.Lock()
        
        # Publisher for concatenated cloud
        self.cloud_pub = self.create_publisher(PointCloud2, 'cloud_out', 10)
        
        for i in range(1, num_clouds + 1):
            topic = f'cloud_in{i}'
            self.create_subscription(
                PointCloud2,
                topic,
                partial(self.cloud_callback, cloud_idx=i),
                10)

        
        # Timer to publish concatenated clouds at regular intervals
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_concatenated_cloud)
        
        self.get_logger().info(f"PointCloud concatenate started, waiting for {num_clouds} clouds")
    
    def cloud_callback(self, msg, cloud_idx):
        """Store the latest cloud from each input."""
        with self.cloud_lock:
            self.latest_clouds[cloud_idx] = msg
        # self.get_logger().debug(f"Received cloud from input {cloud_idx}, frame: {msg.header.frame_id}")
        # self.get_logger().info(f"Received cloud_in{cloud_idx} with {len(msg.data)} bytes")

    
    def publish_concatenated_cloud(self):
        """Concatenate all available clouds and publish."""
        with self.cloud_lock:
            if len(self.latest_clouds) == 0:
                return
            
            clouds_to_concat = list(self.latest_clouds.values())
        
        if len(clouds_to_concat) == 0:
            return
        
        try:
            # Convert all point clouds to numpy arrays
            all_points = []
            latest_timestamp = clouds_to_concat[0].header.stamp
            
            for cloud in clouds_to_concat:
                # Convert PointCloud2 to numpy array
                points = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
                if points:
                    # Convert structured array to regular numpy array
                    points_array = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
                    all_points.append(points_array)
                
                # Use the latest timestamp
                if cloud.header.stamp.sec > latest_timestamp.sec or \
                   (cloud.header.stamp.sec == latest_timestamp.sec and 
                    cloud.header.stamp.nanosec > latest_timestamp.nanosec):
                    latest_timestamp = cloud.header.stamp
            
            if not all_points:
                return
            
            # Concatenate all points
            concatenated_points = np.vstack(all_points)
            
            # Create new PointCloud2 message
            concatenated_cloud = PointCloud2()
            concatenated_cloud.header.stamp = latest_timestamp
            concatenated_cloud.header.frame_id = self.target_frame
            
            # Define point fields (x, y, z)
            concatenated_cloud.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            # Convert numpy array back to PointCloud2
            concatenated_cloud = pc2.create_cloud(concatenated_cloud.header, 
                                                 concatenated_cloud.fields, 
                                                 concatenated_points)
            
            # Publish the concatenated cloud
            self.cloud_pub.publish(concatenated_cloud)
            
            self.get_logger().debug(f"Published concatenated cloud with {len(concatenated_points)} points")
            
        except Exception as e:
            self.get_logger().error(f"Error concatenating point clouds: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConcatenate()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()