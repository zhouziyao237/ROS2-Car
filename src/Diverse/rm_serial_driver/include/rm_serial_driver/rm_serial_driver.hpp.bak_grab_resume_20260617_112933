// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/platform_current_status.hpp"
#include "auto_aim_interfaces/msg/platform_target.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);
  ~RMSerialDriver() override;

private:
  void getParams();
  void receiveData();
  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);
  void cacheNavigationCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendNavigationCmdTimer();
  void sendNavigationPacket(float vx, float vy, float wz);
  void sendPlatformTarget(const auto_aim_interfaces::msg::PlatformTarget::SharedPtr msg);
  void reopenPort();
  void setParam(const rclcpp::Parameter & param);
  void resetTracker();
  void processPacket(const std::vector<uint8_t> & data, uint16_t cmd_id);

  void publishWheelTwist(float vx, float vy, float wz);
  void publishChassisControlMode(uint8_t control_mode);
  void publishPlatformCurrentStatus(
    float target_lift_z_mm, float target_slide_x_mm, float target_screw_x_mm,
    uint8_t control_mode);

  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  using ResultFuturePtr =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  visualization_msgs::msg::Marker aiming_point_;

  double timestamp_offset_ = 0.0;
  bool publish_tf_ = false;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::PlatformTarget>::SharedPtr
    platform_target_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chassis_control_mode_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::PlatformCurrentStatus>::SharedPtr
    platform_current_status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_state_pub_;

  std::string platform_target_topic_;
  std::string platform_current_status_topic_;
  std::string wheel_twist_topic_;
  double wheel_vx_scale_ = 1.0;
  double wheel_vy_scale_ = 1.0;
  double wheel_wz_scale_ = 1.0;
  double wheel_cov_vx_ = 0.30;
  double wheel_cov_vy_ = 0.30;
  double wheel_cov_wz_ = 0.50;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wheel_twist_pub_;

  double cmd_vx_scale_ = 1.0;
  double cmd_vy_scale_ = 1.0;
  double cmd_wz_scale_ = 1.0;
  int chassis_manual_mode_value_ = 1;
  std::string last_chassis_control_mode_ = "unknown";


  // Nav command cache. Always send latest command periodically.
  // This prevents chassis firmware from keeping the previous non-zero velocity
  // when a single zero command is missed or filtered downstream.
  std::mutex nav_cmd_mutex_;
  rclcpp::TimerBase::SharedPtr nav_cmd_timer_;
  rclcpp::Time last_nav_cmd_time_;
  bool has_nav_cmd_ = false;
  float latest_nav_vx_ = 0.0F;
  float latest_nav_vy_ = 0.0F;
  float latest_nav_wz_ = 0.0F;
  double nav_cmd_timeout_ = 0.10;
  double nav_cmd_send_rate_ = 50.0;

  std::vector<uint8_t> rx_buffer_;

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
