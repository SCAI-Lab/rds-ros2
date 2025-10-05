#ifndef WRAPPER_BASE_HPP
#define WRAPPER_BASE_HPP

#include "rds_msgs/srv/velocity_command_correction_rds.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <fstream>

class WrapperBase
{
public:
    WrapperBase(const std::string& config_filepath = "config/default",
        rclcpp::Node* existing_node = nullptr, bool local_path_rds_ros_pkg = true);
    virtual ~WrapperBase();

    virtual int callRDS(float v_n, float w_n, float* v_c, float* w_c);
    
protected:
    static std::string getValueForKeyFromConfigFile(std::ifstream& config_file,
        const std::string& key);
    static float readFloat(const std::string& value, float default_value);
    static bool readBool(const std::string& value, bool default_value);

    rclcpp::Node* node_;
    bool owns_node_;
    rclcpp::Client<rds_msgs::srv::VelocityCommandCorrectionRDS>::SharedPtr rds_client_;
    int call_counter_;
    
public:
    std::shared_ptr<rds_msgs::srv::VelocityCommandCorrectionRDS::Request> service_request_;
};

#endif // WRAPPER_BASE_HPP