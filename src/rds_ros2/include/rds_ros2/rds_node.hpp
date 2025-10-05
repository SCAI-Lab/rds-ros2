#ifndef RDS_NODE_HPP
#define RDS_NODE_HPP

#include <rds/geometry.hpp>
#include <rds/rds_5.hpp>

#include <rds_msgs/srv/velocity_command_correction_rds.hpp>
#include <rds_msgs/msg/to_gui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#include <vector>
#include <string>
#include <memory>
#include <chrono>

class RDSNode : public rclcpp::Node
{
public:
    RDSNode();

private:
    // Service callback
    void commandCorrectionService(
        const std::shared_ptr<rds_msgs::srv::VelocityCommandCorrectionRDS::Request> request,
        std::shared_ptr<rds_msgs::srv::VelocityCommandCorrectionRDS::Response> response);

    // Topic subscriptions
    void callbackLidarPoints(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg);

    // Service server and publisher
    rclcpp::Service<rds_msgs::srv::VelocityCommandCorrectionRDS>::SharedPtr command_correction_server_;
    rclcpp::Publisher<rds_msgs::msg::ToGui>::SharedPtr publisher_for_gui_;
    
    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_lidar_points_;

    // TF handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data storage
    std::vector<Geometry2D::Vec2> obstacle_points_;
    float command_correct_previous_linear_, command_correct_previous_angular_;
    unsigned int call_counter_;
};

#endif