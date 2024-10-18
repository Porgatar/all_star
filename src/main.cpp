#include "BabySharky.hpp"

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotNode>());
  rclcpp::shutdown();

  return 0;
}
