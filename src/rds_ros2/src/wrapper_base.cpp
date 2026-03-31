#include "rds_ros2/wrapper_base.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <chrono>

using namespace std::chrono_literals;

WrapperBase::WrapperBase(const std::string& config_filepath,
    rclcpp::Node* existing_node, bool local_path_rds_ros_pkg)
    : owns_node_(false), call_counter_(0)
{
    if (existing_node == nullptr)
    {
        // Create a new node if none was provided
        node_ = new rclcpp::Node("rds_wrapper_node");
        owns_node_ = true;
    }
    else
    {
        node_ = existing_node;
        owns_node_ = false;
    }

    // Create the service client
    rds_client_ = node_->create_client<rds_msgs::srv::VelocityCommandCorrectionRDS>(
        "rds_velocity_command_correction");
    
    // Create the service request
    service_request_ = std::make_shared<rds_msgs::srv::VelocityCommandCorrectionRDS::Request>();

    // Get the package path if needed
    std::string package_path = "";
    if (local_path_rds_ros_pkg)
    {
        try {
            package_path = ament_index_cpp::get_package_share_directory("rds_ros2") + "/";
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "Could not find package 'rds_ros2': %s", e.what());
        }
    }

    // Return if no config file is specified
    if (config_filepath.empty())
        return;

    // Try to open and read the config file
    std::ifstream file(package_path + config_filepath);
    if (file)
    {
        service_request_->capsule_center_front_y = readFloat(
            getValueForKeyFromConfigFile(file, "capsule_center_front_y"), 0.5f);
        service_request_->capsule_center_rear_y = readFloat(
            getValueForKeyFromConfigFile(file, "capsule_center_rear_y"), -0.5f);
        service_request_->capsule_radius = readFloat(
            getValueForKeyFromConfigFile(file, "capsule_radius"), 0.5f);
        service_request_->reference_point_y = readFloat(
            getValueForKeyFromConfigFile(file, "reference_point_y"), 0.5f);
        service_request_->rds_tau = readFloat(
            getValueForKeyFromConfigFile(file, "rds_tau"), 1.f);
        service_request_->rds_delta = readFloat(
            getValueForKeyFromConfigFile(file, "rds_delta"), 0.05f);
        service_request_->vel_lim_linear_min = readFloat(
            getValueForKeyFromConfigFile(file, "vel_lim_linear_min"), -1.f);
        service_request_->vel_lim_linear_max = readFloat(
            getValueForKeyFromConfigFile(file, "vel_lim_linear_max"), 1.f);
        service_request_->vel_lim_angular_abs_max = readFloat(
            getValueForKeyFromConfigFile(file, "vel_lim_angular_abs_max"), 1.f);
        service_request_->vel_linear_at_angular_abs_max = readFloat(
            getValueForKeyFromConfigFile(file, "vel_linear_at_angular_abs_max"), 0.f);    
        service_request_->acc_limit_linear_abs_max = readFloat(
            getValueForKeyFromConfigFile(file, "acc_limit_linear_abs_max"), 1.f);    
        service_request_->acc_limit_angular_abs_max = readFloat(
            getValueForKeyFromConfigFile(file, "acc_limit_angular_abs_max"), 1.f);    
        service_request_->dt = readFloat(
            getValueForKeyFromConfigFile(file, "dt"), 0.1f);
        service_request_->lrf_point_obstacles = readBool(
            getValueForKeyFromConfigFile(file, "lrf_point_obstacles"), true);    
        service_request_->orca_implementation = readBool(
            getValueForKeyFromConfigFile(file, "orca_implementation"), false);
        file.close();
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not open config file: %s", 
                    (package_path + config_filepath).c_str());
    }
}

WrapperBase::~WrapperBase()
{
    if (owns_node_)
    {
        delete node_;
    }
}

int WrapperBase::callRDS(float v_n, float w_n, float* v_c, float* w_c)
{
    if (!rds_client_->wait_for_service(1s))
    {
        RCLCPP_WARN(node_->get_logger(), "RDS service not available");
        return 1;
    }

    try
    {
        // Create a copy of the request
        auto request = std::make_shared<rds_msgs::srv::VelocityCommandCorrectionRDS::Request>(*service_request_);
        request->nominal_command.linear = v_n;
        request->nominal_command.angular = w_n;
        
        // Call the service synchronously
        auto future = rds_client_->async_send_request(request);
        
        // Wait for the result
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->call_counter > call_counter_)
            {
                call_counter_ = response->call_counter;
                *v_c = response->corrected_command.linear;
                *w_c = response->corrected_command.angular;
                return 0;
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call RDS service");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Exception in RDS service call: %s", e.what());
        RCLCPP_ERROR(node_->get_logger(), "Exception type: %s", typeid(e).name());
        
        // Return emergency stop
        *v_c = 0.0f;
        *w_c = 0.0f;
        return 0;  // Return success but with zero velocity
    }
    catch (...)
    {
        RCLCPP_ERROR(node_->get_logger(), "Unknown exception in RDS service call");
        
        // Return emergency stop
        *v_c = 0.0f;
        *w_c = 0.0f;
        return 0;  // Return success but with zero velocity
    }
    
    return 1;
}

std::string WrapperBase::getValueForKeyFromConfigFile(std::ifstream& config_file,
    const std::string& key)
{
    std::string result = "";
    std::string line;
    while (std::getline(config_file, line))
    {
        std::istringstream is_line(line);
        std::string key_present;
        if (std::getline(is_line, key_present, '='))
        {
            std::string value;
            if (std::getline(is_line, value))
            {
                if (key_present == key)
                    result = value;
            }
        }
    }
    config_file.clear();
    config_file.seekg(0);
    return result;
}

float WrapperBase::readFloat(const std::string& value, float default_value)
{
    try
    {
        return std::stof(value);
    }
    catch (...)
    {
        return default_value;
    }
}

bool WrapperBase::readBool(const std::string& value, bool default_value)
{
    if ((value == "1") || (value == "true") || (value == "True"))
        return true;
    else if ((value == "0") || (value == "false") || (value == "False"))
        return false;
    else
        return default_value;
}