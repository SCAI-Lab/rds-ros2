#ifndef RDS_GUI_ROS_NODE_HPP
#define RDS_GUI_ROS_NODE_HPP

#include <rds/gui.hpp>
#include <rds/geometry.hpp>

#include <rds_msgs/msg/to_gui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <unordered_set>

typedef Window::sdlColor GuiColor;

class RDSGUIROSNode : public rclcpp::Node
{
public:
    RDSGUIROSNode();

    // Call each frame from the main loop. Returns true if a window was closed.
    bool update();

private:
    void toGuiCallback(const rds_msgs::msg::ToGui::SharedPtr msg);

    GUI gui_command_space_;
    GUI gui_work_space_;

    rclcpp::Subscription<rds_msgs::msg::ToGui>::SharedPtr subscriber_;

    // Command space (velocity space)
    std::vector<Geometry2D::HalfPlane2> command_space_halfplanes_;
    std::vector<AdditionalPrimitives2D::Arrow> command_space_arrows_;
    std::vector<GuiColor> command_space_arrows_colors_;
    std::vector<Geometry2D::Capsule> command_space_capsules_;

    // Work space (position space)
    std::vector<AdditionalPrimitives2D::Circle> work_space_circles_;
    std::vector<GuiColor> work_space_circles_colors_;
    std::vector<AdditionalPrimitives2D::Arrow> work_space_arrows_;
    std::vector<GuiColor> work_space_arrows_colors_;
    std::vector<Geometry2D::Capsule> work_space_capsules_;

    GuiColor green_, blue_, red_, orange_, cyan_;
};

#endif
