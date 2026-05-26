#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace nav_stack_core
{

class NavStackModeManager : public rclcpp::Node
{
public:
  explicit NavStackModeManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void publish_active_mode();
  void on_use_depth(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void on_enable_slam(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  std::mutex mutex_;
  std::string odom_mode_;
  bool slam_enabled_;
  std::string status_topic_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr use_depth_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_slam_srv_;
};

}  // namespace nav_stack_core
