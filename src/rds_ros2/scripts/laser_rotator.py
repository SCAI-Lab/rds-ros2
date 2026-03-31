#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LidarRotationNode:
    """
    A ROS node that rotates a LaserScan message by 90 degrees and republishes it.
    """
    
    def __init__(self):
        """
        Constructor to initialize the node, subscribers, publishers, and parameters.
        """
        # Initializing node
        rospy.init_node('lidar_rotation_node')

        # Get parameters
        self.input_topic = rospy.get_param("~input_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/front_lidar/scan")
        self.new_frame_id = rospy.get_param("~new_frame_id", "rds_frame")

        # Subscriber
        self.subscriber = rospy.Subscriber(self.input_topic, LaserScan, self.callback)

        # Publisher
        self.publisher = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)

    def callback(self, msg: LaserScan):
        """
        Callback function to process LaserScan messages.
        
        Args:
        - msg (LaserScan): Incoming LaserScan message.
        """

        print(msg.header.seq)
        
        # Rotate the scan by 90 degrees.
        # One way to do this is by changing the angle_min and angle_max by pi/2 (which is approximately 1.5708 radians).
        # But, this is a simple rotation of the metadata. To actually rotate the scan data, we'd need to reorder the ranges.
        rotated_ranges =  msg.ranges[:-len(msg.ranges)//4] + msg.ranges[-len(msg.ranges)//4:]
        rotated_ranges = rotated_ranges[::-1]
        msg.ranges = rotated_ranges

        # Update the header frame ID
        msg.header.frame_id = self.new_frame_id

        # Publish the modified scan
        self.publisher.publish(msg)

    def run(self):
        """
        Keeps the node running.
        """
        rospy.spin()

if __name__ == "__main__":
    # Create an instance of the node and run it
    node = LidarRotationNode()
    node.run()
