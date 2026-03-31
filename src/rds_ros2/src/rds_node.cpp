#include "rds_ros2/rds_node.hpp"

#include <rds/capsule.hpp>
#include <rds/config_rds_5.hpp>
#include <rds/distance_minimizer.hpp>

#include <rds_msgs/msg/half_plane2_d.hpp>
#include <rds_msgs/msg/point2_d.hpp>
#include <rds_msgs/msg/circle.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#define _USE_MATH_DEFINES
#include <cmath>

using Geometry2D::Vec2;
using Geometry2D::Capsule;
using AdditionalPrimitives2D::Circle;

RDSNode::RDSNode()
    : Node("rds_node"),
      command_correct_previous_linear_(0.f),
      command_correct_previous_angular_(0.f),
      call_counter_(0),
      last_pedestrian_update_(Clock::now())
{
    // Declare parameters
    this->declare_parameter("enable_pedestrian_tracking", true);
    this->declare_parameter("pedestrian_track_topic", std::string("rds/input/pedestrian_tracks"));
    this->declare_parameter("default_pedestrian_radius", 0.3f);
    this->declare_parameter("pedestrian_timeout", 1.0f);

    // Read parameters
    enable_pedestrian_tracking_ = this->get_parameter("enable_pedestrian_tracking").as_bool();
    pedestrian_track_topic_ = this->get_parameter("pedestrian_track_topic").as_string();
    default_pedestrian_radius_ = static_cast<float>(this->get_parameter("default_pedestrian_radius").as_double());
    pedestrian_timeout_ = static_cast<float>(this->get_parameter("pedestrian_timeout").as_double());

    // Initialize TF2 components
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create subscriptions
    subscriber_lidar_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "rds/input/filtered/points",
        rclcpp::QoS(1),
        std::bind(&RDSNode::callbackLidarPoints, this, std::placeholders::_1));

    if (enable_pedestrian_tracking_)
    {
        subscriber_pedestrian_tracks_ = this->create_subscription<rds_msgs::msg::PedestrianTracks>(
            pedestrian_track_topic_,
            rclcpp::QoS(1),
            std::bind(&RDSNode::callbackPedestrianTracks, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Pedestrian tracking enabled on topic: %s", pedestrian_track_topic_.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Pedestrian tracking disabled");
    }

    // Create publisher
    publisher_for_gui_ = this->create_publisher<rds_msgs::msg::ToGui>(
        "rds_to_gui",
        rclcpp::QoS(1));

    // Create service
    command_correction_server_ = this->create_service<rds_msgs::srv::VelocityCommandCorrectionRDS>(
        "rds_velocity_command_correction",
        std::bind(&RDSNode::commandCorrectionService, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "RDS Node initialized");
}

void RDSNode::commandCorrectionService(
    const std::shared_ptr<rds_msgs::srv::VelocityCommandCorrectionRDS::Request> request,
    std::shared_ptr<rds_msgs::srv::VelocityCommandCorrectionRDS::Response> response)
{
    // prepare pedestrian tracks/ scan points retrieved from recent messages
    std::vector<MovingCircle> lrf_moving_objects;
    std::vector<MovingCircle> all_moving_objects;
    if (request->lrf_point_obstacles)
    {
        MovingCircle moving_object;
        moving_object.velocity = Vec2(0.0, 0.0);
        moving_object.circle.radius = 0.0;
        for (size_t i = 0; i < obstacle_points_.size(); i++)
        {
            moving_object.circle.center = obstacle_points_[i];
            lrf_moving_objects.push_back(moving_object);
            all_moving_objects.push_back(moving_object);
        }
    }

    if (enable_pedestrian_tracking_)
    {
        float age = std::chrono::duration<float>(Clock::now() - last_pedestrian_update_).count();
        if (age <= pedestrian_timeout_)
        {
            for (const auto& ped : pedestrian_objects_)
                all_moving_objects.push_back(ped);
        }
        else if (!pedestrian_objects_.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Pedestrian tracks stale (%.2fs > %.2fs timeout), ignoring", age, pedestrian_timeout_);
        }
    }

    // parse service parameters
    float a_v_min = command_correct_previous_linear_ - request->dt * request->acc_limit_linear_abs_max;
    float a_v_max = command_correct_previous_linear_ + request->dt * request->acc_limit_linear_abs_max;
    float a_w_min = command_correct_previous_angular_ - request->dt * request->acc_limit_angular_abs_max;
    float a_w_max = command_correct_previous_angular_ + request->dt * request->acc_limit_angular_abs_max;
    VWBox vw_box_limits(a_v_min, a_v_max, a_w_min, a_w_max);

    VWDiamond vw_diamond_limits(request->vel_lim_linear_min, request->vel_lim_linear_max,
        request->vel_lim_angular_abs_max, request->vel_linear_at_angular_abs_max);

    const RDS5CapsuleConfiguration rds_5_config = ConfigRDS5::ConfigWrap(request->dt).rds_5_config;

    float tau = request->rds_tau;// rds_5_config.tau;
    float delta = request->rds_delta;// rds_5_config.delta;
    float y_p_ref = request->reference_point_y;// rds_5_config.y_p_ref;

    Geometry2D::RDS5 rds_5(tau, delta, y_p_ref, vw_box_limits, vw_diamond_limits);

    rds_5.use_conservative_shift = false;
    rds_5.keep_origin_feasible = false;
    rds_5.no_VO_shift_at_contact = false;
    rds_5.shift_reduction_range = 0.35f;
    rds_5.ORCA_implementation = request->orca_implementation;
    rds_5.ORCA_use_p_ref = true;
    rds_5.ORCA_solver = true;

    float capsule_radius = request->capsule_radius;//rds_5_config.robot_shape.radius();
    float capsule_center_front_y = request->capsule_center_front_y;// rds_5_config.robot_shape.center_a().y;
    float capsule_center_rear_y = request->capsule_center_rear_y;//rds_5_config.robot_shape.center_b().y;

    Capsule robot_shape(capsule_radius, Vec2(0.0, capsule_center_front_y),
        Vec2(0.0, capsule_center_rear_y)); //0.45, 0.05, -0.5

    Vec2 v_nominal_p_ref(-y_p_ref * request->nominal_command.angular,
        request->nominal_command.linear);

    Vec2 v_previous_command(-command_correct_previous_angular_ * y_p_ref,
        command_correct_previous_linear_);

    Vec2 v_corrected_p_ref(0.f, 0.f);

    if (v_nominal_p_ref.norm() > std::abs(vw_diamond_limits.v_max))
        v_nominal_p_ref = v_nominal_p_ref.normalized() * std::abs(vw_diamond_limits.v_max);

    // compute collision avoidance command
    try
    {
        rds_5.computeCorrectedVelocity(robot_shape, v_nominal_p_ref, v_previous_command,
            std::vector<MovingCircle>(), all_moving_objects, &v_corrected_p_ref);
    }
    catch (Geometry2D::Vec2::NormalizationException&)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "NormalizationException: obstacle too close to robot, applying emergency brake");
        v_corrected_p_ref = Vec2(0.f, 0.f);
    }
    catch (Geometry2D::DistanceMinimizer::InfeasibilityException& e)
    {
        float breaking_step_linear = request->dt * rds_5_config.breaking_deceleration_linear;
        float breaking_step_angular = request->dt * rds_5_config.breaking_deceleration_angular;
        float new_v_linear, new_v_angular;
        if (command_correct_previous_linear_ > 0.f)
            new_v_linear = std::max(0.f, command_correct_previous_linear_ - breaking_step_linear);
        else
            new_v_linear = std::min(0.f, command_correct_previous_linear_ + breaking_step_linear);
        if (command_correct_previous_angular_ > 0.f)
            new_v_angular = std::max(0.f, command_correct_previous_angular_ - breaking_step_angular);
        else
            new_v_angular = std::min(0.f, command_correct_previous_angular_ + breaking_step_angular);
        v_corrected_p_ref.y = new_v_linear;
        v_corrected_p_ref.x = -new_v_angular * y_p_ref;//rds_5_config.y_p_ref;
    }

    // communicate the result and the underlying representations 
    response->corrected_command.linear = v_corrected_p_ref.y;
    response->corrected_command.angular = -1.0 / y_p_ref * v_corrected_p_ref.x;
    
    command_correct_previous_linear_ = response->corrected_command.linear;
    command_correct_previous_angular_ = response->corrected_command.angular;

    call_counter_++;
    response->call_counter = call_counter_;

    auto msg_to_gui = std::make_unique<rds_msgs::msg::ToGui>();
    msg_to_gui->nominal_command.linear = request->nominal_command.linear;
    msg_to_gui->nominal_command.angular = request->nominal_command.angular;
    msg_to_gui->corrected_command.linear = response->corrected_command.linear;
    msg_to_gui->corrected_command.angular = response->corrected_command.angular;
    msg_to_gui->reference_point.x = 0.f;
    msg_to_gui->reference_point.y = y_p_ref;
    msg_to_gui->reference_point_velocity_solution.x = v_corrected_p_ref.x;
    msg_to_gui->reference_point_velocity_solution.y = v_corrected_p_ref.y;
    msg_to_gui->reference_point_nominal_velocity.x = v_nominal_p_ref.x;
    msg_to_gui->reference_point_nominal_velocity.y = v_nominal_p_ref.y;
    
    for (auto& h : rds_5.constraints)
    {
        rds_msgs::msg::HalfPlane2D h_msg;
        h_msg.normal.x = h.getNormal().x;
        h_msg.normal.y = h.getNormal().y;
        h_msg.offset = h.getOffset();
        msg_to_gui->reference_point_velocity_constraints.push_back(h_msg);
    }

    for (auto& mo : all_moving_objects)
    {
        rds_msgs::msg::Circle c_msg;
        c_msg.center.x = mo.circle.center.x;
        c_msg.center.y = mo.circle.center.y;
        c_msg.radius = mo.circle.radius + delta;
        msg_to_gui->moving_objects.push_back(c_msg);
    }
    
    for (auto& mo : all_moving_objects)
    {
        if ((mo.velocity.x != 0.f) || (mo.velocity.y != 0.f))
        {
            rds_msgs::msg::Point2D head_msg, tail_msg;
            head_msg.x = mo.circle.center.x + mo.velocity.x * tau;
            head_msg.y = mo.circle.center.y + mo.velocity.y * tau;
            tail_msg.x = mo.circle.center.x;
            tail_msg.y = mo.circle.center.y;
            msg_to_gui->moving_objects_predictions.push_back(head_msg);
            msg_to_gui->moving_objects_predictions.push_back(tail_msg);
        }
    }

    msg_to_gui->robot_shape.radius = robot_shape.radius();
    msg_to_gui->robot_shape.center_a.x = robot_shape.center_a().x;
    msg_to_gui->robot_shape.center_a.y = robot_shape.center_a().y;
    msg_to_gui->robot_shape.center_b.x = robot_shape.center_b().x;
    msg_to_gui->robot_shape.center_b.y = robot_shape.center_b().y;

    if (rds_5.n_bounding_circles > 2)
    {
        for (const auto& bc : rds_5.bounding_circles.circles())
        {
            rds_msgs::msg::Circle bc_msg;
            bc_msg.center.x = bc.center.x;
            bc_msg.center.y = bc.center.y;
            bc_msg.radius = bc.radius;
            msg_to_gui->bounding_circles.push_back(bc_msg);
        }
    }

    publisher_for_gui_->publish(*msg_to_gui);
}

void RDSNode::callbackPedestrianTracks(const rds_msgs::msg::PedestrianTracks::SharedPtr msg)
{
    pedestrian_objects_.clear();
    for (const auto& track : msg->tracks)
    {
        MovingCircle ped;
        ped.circle.center = Vec2(track.x, track.y);
        ped.circle.radius = (track.radius > 0.f) ? track.radius : default_pedestrian_radius_;
        ped.velocity = Vec2(track.vx, track.vy);
        pedestrian_objects_.push_back(ped);
    }
    last_pedestrian_update_ = Clock::now();
}

void RDSNode::callbackLidarPoints(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg)
{
    // Convert PointCloud2 to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*points_msg, *cloud);  // Use fromROSMsg instead of toPCL

    // Transform the cloud to "rds_frame"
    sensor_msgs::msg::PointCloud2 cloud_out;
    try 
    {
        // Transform using tf2
        tf_buffer_->transform(*points_msg, cloud_out, "rds_frame", 
                             tf2::durationFromSec(1.0));
        
        // Convert transformed cloud to PCL
        pcl::fromROSMsg(cloud_out, *cloud);  // Use fromROSMsg instead of toPCL
    } 
    catch (tf2::TransformException &ex) 
    {
        RCLCPP_WARN(this->get_logger(), 
                   "%s exception, when looking up tf from %s to rds_frame", 
                   ex.what(), points_msg->header.frame_id.c_str());
        return;
    }

    // Clear the current points and store the new transformed points
    obstacle_points_.clear();
    for (const auto& pt : *cloud) {
        obstacle_points_.push_back(Vec2(pt.x, pt.y));
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RDSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


