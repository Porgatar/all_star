#include "BabySharky.hpp"

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor  executor;
  std::shared_ptr<AquabotNode>              node;
  
  node = std::make_shared<AquabotNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return (0);
}
