#include "BabySharky.hpp"

AquabotNode::AquabotNode() : Node("all_star")
{
    RCLCPP_INFO(this->get_logger(), "Hello world from baby-sharky node in cpp!");

    // Topics publisher
    example_publisher = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/pos", 1);

    // Topics subscribers

    // loops
    m_placeholder_timer = this->create_wall_timer(100ms, std::bind(&AquabotNode::placeholder, this));
    m_timer = this->create_wall_timer(1s, std::bind(&AquabotNode::timer_callback, this));
}

void    AquabotNode::timer_callback() {

    RCLCPP_INFO(this->get_logger(), "timer Callback function");
}

void    AquabotNode::placeholder() {

    RCLCPP_INFO(this->get_logger(), "placeholder function");
}
