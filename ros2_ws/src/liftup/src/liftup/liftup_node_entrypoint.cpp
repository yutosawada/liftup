#include "rclcpp/rclcpp.hpp"
#include "liftup/liftup_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiftupNode>());
  rclcpp::shutdown();
  return 0;
}