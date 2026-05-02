#include "nav_stack_core/nav_stack_mode_manager.hpp"

#include <utility>

namespace nav_stack_core
{

NavStackModeManager::NavStackModeManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("nav_stack_mode_manager", options)
{
  declare_parameters();

  odom_mode_ = this->get_parameter("odom_mode").as_string();
  slam_enabled_ = this->get_parameter("use_slam").as_bool();
  status_topic_ = this->get_parameter("status_topic").as_string();

  status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);

  use_depth_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "set_use_depth_odom",
    std::bind(&NavStackModeManager::on_use_depth, this, std::placeholders::_1, std::placeholders::_2));

  enable_slam_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "set_use_slam",
    std::bind(&NavStackModeManager::on_enable_slam, this, std::placeholders::_1, std::placeholders::_2));

  publish_active_mode();
}

void NavStackModeManager::declare_parameters()
{
  this->declare_parameter<std::string>("odom_mode", "fused");
  this->declare_parameter<bool>("use_slam", true);
  this->declare_parameter<std::string>("status_topic", "/nav_stack/status");
}

void NavStackModeManager::publish_active_mode()
{
  std_msgs::msg::String msg;
  msg.data =
    std::string("odom_mode=") + odom_mode_ +
    ", use_slam=" + (slam_enabled_ ? "true" : "false");

  status_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "nav_stack status: odom_mode=%s use_slam=%s",
    odom_mode_.c_str(), slam_enabled_ ? "true" : "false");
}

void NavStackModeManager::on_use_depth(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_mode_ = request->data ? "depth" : "lidar";
  response->success = true;
  response->message = "odom_mode switched to " + odom_mode_;
  publish_active_mode();
}

void NavStackModeManager::on_enable_slam(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  slam_enabled_ = request->data;
  response->success = true;
  response->message = std::string("use_slam switched to ") + (slam_enabled_ ? "true" : "false");
  publish_active_mode();
}

}  // namespace nav_stack_core
