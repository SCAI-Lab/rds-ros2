#!/usr/bin/env python3

import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from laser_geometry import LaserProjection


class LaserTransformNode:
    '''
    This node transforms a laser scan from the "scan" frame to the "base_link" frame.
    '''
    
    def __init__(self):
        '''
        Constructor
        '''
        
        # Node initialization
        rospy.init_node('laser_transform_node', anonymous=True)

        # Vars
        self.rear_cloud = None

        # Load parameters
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.cloud_topic = rospy.get_param('~cloud_topic', '/points')
        self.angle_min = rospy.get_param('~angle_min', None)
        self.angle_max = rospy.get_param('~angle_max', None)

        # Create a publisher for the transformed laser scan (now PointCloud2)
        self.pub_pc = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=10)

        # Subscribe to the laser scan topic
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

    def scan_callback(self, scan_msg):
        '''
        Callback function for the laser scan topic, 
        '''

        # Convert the LaserScan message to a PointCloud2 message
        projector = LaserProjection()

        if self.angle_min is not None and self.angle_max is not None :
            
            min_idx = int( (self.angle_min - scan_msg.angle_min) / scan_msg.angle_increment)
            max_idx = int( (self.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
            scan_msg.ranges = scan_msg.ranges[min_idx:max_idx]
            scan_msg.intensities = scan_msg.intensities[min_idx:max_idx]
            
            #scan_msg.angle_max = scan_msg.angle_min + scan_msg.angle_increment * (max_idx - min_idx)

            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
        
        scan_msg.header.frame_id = "base_link"
        cloud = projector.projectLaser(scan_msg)
        
        # Publish the data
        self.pub_pc.publish(cloud)


if __name__ == '__main__' :
    '''
    Main function
    '''
    
    # Create a LaserTransformNode object
    laser_transform_node = LaserTransformNode()
    
    # Spin
    rospy.spin()