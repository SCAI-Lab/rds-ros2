#include "rds_gui_ros_node.hpp"

using namespace Geometry2D;
using namespace AdditionalPrimitives2D;

RDSGUIROSNode::RDSGUIROSNode()
    : Node("rds_gui_node"),
      gui_command_space_("RDS Command Space", 3.0f, 800),
      gui_work_space_("RDS Work Space", 8.0f, 800)
{
    green_.r  = 0;   green_.g  = 200; green_.b  = 0;
    blue_.r   = 0;   blue_.g   = 100; blue_.b   = 255;
    red_.r    = 255; red_.g    = 0;   red_.b    = 0;
    orange_.r = 255; orange_.g = 165; orange_.b = 0;
    cyan_.r   = 0;   cyan_.g   = 220; cyan_.b   = 220;

    gui_command_space_.halfplanes = &command_space_halfplanes_;
    gui_command_space_.arrows     = &command_space_arrows_;
    gui_command_space_.capsules   = &command_space_capsules_;

    gui_work_space_.circles  = &work_space_circles_;
    gui_work_space_.arrows   = &work_space_arrows_;
    gui_work_space_.capsules = &work_space_capsules_;

    subscriber_ = this->create_subscription<rds_msgs::msg::ToGui>(
        "rds_to_gui", rclcpp::QoS(1),
        std::bind(&RDSGUIROSNode::toGuiCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RDS GUI node started");
}

void RDSGUIROSNode::toGuiCallback(const rds_msgs::msg::ToGui::SharedPtr msg)
{
    // --- Command space (velocity space) ---

    command_space_halfplanes_.clear();
    for (const auto& h : msg->reference_point_velocity_constraints)
        command_space_halfplanes_.emplace_back(Vec2(h.normal.x, h.normal.y), h.offset);

    command_space_arrows_.clear();
    command_space_arrows_colors_.clear();

    // Nominal velocity: blue arrow from origin
    command_space_arrows_.emplace_back(
        Vec2(msg->reference_point_nominal_velocity.x, msg->reference_point_nominal_velocity.y),
        Vec2(0.f, 0.f));
    command_space_arrows_colors_.push_back(blue_);

    // Corrected velocity: green arrow from origin
    command_space_arrows_.emplace_back(
        Vec2(msg->reference_point_velocity_solution.x, msg->reference_point_velocity_solution.y),
        Vec2(0.f, 0.f));
    command_space_arrows_colors_.push_back(green_);

    gui_command_space_.arrows_colors = command_space_arrows_colors_;

    // Robot capsule for scale reference
    command_space_capsules_.clear();
    command_space_capsules_.emplace_back(
        msg->robot_shape.radius,
        Vec2(msg->robot_shape.center_a.x, msg->robot_shape.center_a.y),
        Vec2(msg->robot_shape.center_b.x, msg->robot_shape.center_b.y));

    // --- Work space (position space) ---

    // Build a set of dynamic object positions (those with velocity predictions)
    // Predictions are [head, tail] pairs — tail.x/y == circle center
    std::unordered_set<size_t> dynamic_indices;
    for (size_t i = 0; i + 1 < msg->moving_objects_predictions.size(); i += 2)
    {
        const auto& tail = msg->moving_objects_predictions[i + 1];
        for (size_t j = 0; j < msg->moving_objects.size(); j++)
        {
            const auto& c = msg->moving_objects[j];
            if (std::abs(c.center.x - tail.x) < 1e-3f &&
                std::abs(c.center.y - tail.y) < 1e-3f)
            {
                dynamic_indices.insert(j);
                break;
            }
        }
    }

    work_space_circles_.clear();
    work_space_circles_colors_.clear();
    for (size_t j = 0; j < msg->moving_objects.size(); j++)
    {
        const auto& c = msg->moving_objects[j];
        work_space_circles_.emplace_back(Vec2(c.center.x, c.center.y), c.radius);
        // Dynamic obstacles (with velocity): cyan. Static LiDAR points: orange.
        work_space_circles_colors_.push_back(
            dynamic_indices.count(j) ? cyan_ : orange_);
    }
    gui_work_space_.circles_colors = work_space_circles_colors_;

    // Velocity prediction arrows — published as [head, tail] pairs
    work_space_arrows_.clear();
    work_space_arrows_colors_.clear();
    for (size_t i = 0; i + 1 < msg->moving_objects_predictions.size(); i += 2)
    {
        const auto& head = msg->moving_objects_predictions[i];
        const auto& tail = msg->moving_objects_predictions[i + 1];
        work_space_arrows_.emplace_back(Vec2(head.x, head.y), Vec2(tail.x, tail.y));
        work_space_arrows_colors_.push_back(red_);
    }
    gui_work_space_.arrows_colors = work_space_arrows_colors_;

    // Robot capsule in work space
    work_space_capsules_.clear();
    work_space_capsules_.emplace_back(
        msg->robot_shape.radius,
        Vec2(msg->robot_shape.center_a.x, msg->robot_shape.center_a.y),
        Vec2(msg->robot_shape.center_b.x, msg->robot_shape.center_b.y));
}

bool RDSGUIROSNode::update()
{
    int r1 = gui_command_space_.update();
    int r2 = gui_work_space_.update();
    return (r1 != 0) || (r2 != 0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RDSGUIROSNode>();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->update())
            break;
    }

    rclcpp::shutdown();
    return 0;
}
