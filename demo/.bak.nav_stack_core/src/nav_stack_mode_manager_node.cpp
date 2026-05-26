#include "nav_stack_core/nav_stack_mode_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav_stack_core::NavStackModeManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
