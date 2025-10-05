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
        self.rear_scan_topic = rospy.get_param('~rear_scan_topic', '/rear/scan')
        self.cloud_topic = rospy.get_param('~cloud_topic', '/rds/input/points')

        # Create a publisher for the transformed laser scan (now PointCloud2)
        self.pub_pc = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=10)

        # Subscribe to the laser scan topic
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        rospy.Subscriber(self.rear_scan_topic, LaserScan, self.rear_scan_callback)


    def scan_callback(self, scan_msg):
        '''
        Callback function for the laser scan topic, 
        '''

        # Convert the LaserScan message to a PointCloud2 message
        projector = LaserProjection()
        cloud = projector.projectLaser(scan_msg)

        if self.rear_cloud is not None :
            # TODO: check timestampt is not too old
            cloud.width += self.rear_cloud.width
            cloud.data += self.rear_cloud.data
        
        # Publish the data
        self.pub_pc.publish(cloud)


    def rear_scan_callback(self, scan_msg):
        '''
        Callback function for the laser scan topic, 
        '''

        # Convert the LaserScan message to a PointCloud2 message
        projector = LaserProjection()
        self.rear_cloud = projector.projectLaser(scan_msg)


if __name__ == '__main__' :
    '''
    Main function
    '''
    
    # Create a LaserTransformNode object
    laser_transform_node = LaserTransformNode()
    
    # Spin
    rospy.spin()