#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cmath>

class SimpleAvoidanceNode : public rclcpp::Node
{
public:
    SimpleAvoidanceNode() : Node("simple_avoidance")
    {
        // Parameters
        this->declare_parameter("safety_distance", 0.8);
        this->declare_parameter("max_linear_vel", 0.8);
        this->declare_parameter("max_angular_vel", 1.0);
        
        safety_distance_ = this->get_parameter("safety_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_multi", 10, 
            std::bind(&SimpleAvoidanceNode::scanCallback, this, std::placeholders::_1));
            
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/whill/joy_vel", 10,
            std::bind(&SimpleAvoidanceNode::cmdCallback, this, std::placeholders::_1));
        
        // Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Simple avoidance started");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        latest_scan_ = scan;
    }
    
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        if (!latest_scan_) {
            // No scan data, pass through
            cmd_pub_->publish(*cmd);
            return;
        }
        
        geometry_msgs::msg::Twist safe_cmd = *cmd;
        
        // Check obstacles in different sectors
        float front_min = getMinRangeInSector(-30, 30);     // Front 60 degrees
        float left_min = getMinRangeInSector(30, 90);       // Left side
        float right_min = getMinRangeInSector(-90, -30);    // Right side
        float back_min = getMinRangeInSector(150, 210);     // Back 60 degrees
        
        // Forward movement safety
        if (safe_cmd.linear.x > 0 && front_min < safety_distance_) {
            float reduction = std::max(0.0, (front_min - 0.3) / safety_distance_);
            safe_cmd.linear.x *= reduction;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Front obstacle at %.2fm, reducing speed", front_min);
        }
        
        // Backward movement safety
        if (safe_cmd.linear.x < 0 && back_min < safety_distance_) {
            float reduction = std::max(0.0, (back_min - 0.3) / safety_distance_);
            safe_cmd.linear.x *= reduction;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Back obstacle at %.2fm, reducing speed", back_min);
        }
        
        // Angular movement safety
        if (safe_cmd.angular.z > 0 && left_min < safety_distance_ * 0.7) {
            safe_cmd.angular.z *= 0.5; // Reduce left turn
        }
        if (safe_cmd.angular.z < 0 && right_min < safety_distance_ * 0.7) {
            safe_cmd.angular.z *= 0.5; // Reduce right turn
        }
        
        // Clamp to maximum velocities
        safe_cmd.linear.x = std::clamp(safe_cmd.linear.x, -max_linear_vel_, max_linear_vel_);
        safe_cmd.angular.z = std::clamp(safe_cmd.angular.z, -max_angular_vel_, max_angular_vel_);
        
        cmd_pub_->publish(safe_cmd);
    }
    
    float getMinRangeInSector(int start_deg, int end_deg)
    {
        if (!latest_scan_ || latest_scan_->ranges.empty()) return 999.0;
        
        float angle_min = latest_scan_->angle_min;
        float angle_increment = latest_scan_->angle_increment;
        
        float start_rad = start_deg * M_PI / 180.0;
        float end_rad = end_deg * M_PI / 180.0;
        
        float min_range = 999.0;
        
        for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
            float angle = angle_min + i * angle_increment;
            
            // Normalize angle to [-pi, pi]
            while (angle > M_PI) angle -= 2*M_PI;
            while (angle < -M_PI) angle += 2*M_PI;
            
            if (angle >= start_rad && angle <= end_rad) {
                float range = latest_scan_->ranges[i];
                if (std::isfinite(range) && range > latest_scan_->range_min && 
                    range < latest_scan_->range_max) {
                    min_range = std::min(min_range, range);
                }
            }
        }
        
        return min_range;
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    double safety_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}