#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>


class VoxelFilterNode : public rclcpp::Node
{
public:
  VoxelFilterNode()
  : Node("voxel_filter_node")
  {
    // Declare parameters
    this->declare_parameter("filter_field_name", "z");
    this->declare_parameter("filter_limit_min", -1.0);
    this->declare_parameter("filter_limit_max", 1.0);
    this->declare_parameter("filter_limit_negative", false);
    this->declare_parameter("leaf_size", 0.2);
    
    // Get parameters
    filter_field_name_ = this->get_parameter("filter_field_name").as_string();
    filter_limit_min_ = this->get_parameter("filter_limit_min").as_double();
    filter_limit_max_ = this->get_parameter("filter_limit_max").as_double();
    filter_limit_negative_ = this->get_parameter("filter_limit_negative").as_bool();
    leaf_size_ = this->get_parameter("leaf_size").as_double();
    
    // Create publisher
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
    
    // Create subscription
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", qos,
      std::bind(&VoxelFilterNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "VoxelGrid Filter Node initialized");
    RCLCPP_INFO(this->get_logger(), "Filter field: %s, min: %f, max: %f, negative: %s, leaf size: %f",
      filter_field_name_.c_str(), filter_limit_min_, filter_limit_max_,
      filter_limit_negative_ ? "true" : "false", leaf_size_);
  }
  
private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
  {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pass(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*input_cloud, *cloud);
    
    // Apply passthrough filter if specified
    if (!filter_field_name_.empty()) {
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName(filter_field_name_);
      pass.setFilterLimits(filter_limit_min_, filter_limit_max_);
      pass.setNegative(filter_limit_negative_);
      pass.filter(*cloud_filtered_pass);
    } else {
      *cloud_filtered_pass = *cloud;
    }
    
    // Apply voxel grid filter if leaf size > 0
    if (leaf_size_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud_filtered_pass);
      vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
      vg.filter(*cloud_filtered_voxel);
    } else {
      *cloud_filtered_voxel = *cloud_filtered_pass;
    }
    
    // Convert filtered cloud back to ROS message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud_filtered_voxel, output_cloud);
    output_cloud.header = input_cloud->header;
    
    // Publish filtered cloud
    cloud_pub_->publish(output_cloud);
  }
  
  // Publishers/subscribers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  // Parameters
  std::string filter_field_name_;
  double filter_limit_min_;
  double filter_limit_max_;
  bool filter_limit_negative_;
  double leaf_size_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}