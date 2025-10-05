#include "rds_ros2/wrapper_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <thread>
#include <memory>

using namespace std::chrono_literals;

class Listener 
{
public:
    float nominal_command_linear = 0.0;
    float nominal_command_angular = 0.0;
    std::chrono::high_resolution_clock::time_point last_cmd_vel_stamp;
    bool active = true;

    void remote_joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        nominal_command_linear = msg->linear.x;
        nominal_command_angular = msg->angular.z;
        last_cmd_vel_stamp = std::chrono::high_resolution_clock::now();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rds_ros_nominal_command_node");
    
    WrapperBase calling_wrapper("config/whill_config_icra", node.get());
    
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/rds_modulated_cmd_vel", 10);
    
    auto listener = std::make_shared<Listener>();
    auto sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "remote_cmd_vel", 10,
        std::bind(&Listener::remote_joy_callback, listener, std::placeholders::_1));
    
    auto corrected_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
    
    float corrected_command_linear = 0.0;
    float corrected_command_angular = 0.0;
    
    auto command_cycle_time = 10ms;
    
    auto max_cmd_vel_timeout = 500ms;
    
    std::chrono::high_resolution_clock::time_point t1, t2;
    while (rclcpp::ok())
    {
        t1 = std::chrono::high_resolution_clock::now();
        
        rclcpp::spin_some(node);
        
        auto time_since_last_cmd = t1 - listener->last_cmd_vel_stamp;
        if (time_since_last_cmd < max_cmd_vel_timeout)
        {
            listener->active = true;
            
            // Call RDS to get corrected commands
            if (calling_wrapper.callRDS(listener->nominal_command_linear, listener->nominal_command_angular,
                &corrected_command_linear, &corrected_command_angular) == 0)
            {
                // RCLCPP_INFO(node->get_logger(), "Nominal command [%f,%f], corrected command [%f,%f]",
                //     listener->nominal_command_linear, listener->nominal_command_angular,
                //     corrected_command_linear, corrected_command_angular);
                
                corrected_vel_msg->linear.x = corrected_command_linear;
                corrected_vel_msg->angular.z = corrected_command_angular;
                pub->publish(*corrected_vel_msg);
            }
        }
        else if (listener->active)
        {
            // If we were active but timed out, send a stop command
            listener->active = false;
            corrected_vel_msg->linear.x = 0.0;
            corrected_vel_msg->linear.y = 0.0;
            corrected_vel_msg->angular.z = 0.0;
            pub->publish(*corrected_vel_msg);
        }
        else 
        {
            listener->active = false;
        }
        
        // Sleep for the remaining cycle time
        t2 = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        if (elapsed < command_cycle_time)
        {
            std::this_thread::sleep_for(command_cycle_time - elapsed);
        }
    }

    rclcpp::shutdown();
    return 0;
}
