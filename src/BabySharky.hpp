#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class AquabotNode : public rclcpp::Node
{
  public:
    AquabotNode();

  private:
    void timer_callback();

    void placeholder();

    // Declare publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr example_publisher;

    // Declare subscribers

    // Declare timers
    rclcpp::TimerBase::SharedPtr  m_timer;
    rclcpp::TimerBase::SharedPtr  m_placeholder_timer;

};
