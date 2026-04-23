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
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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
  void sendNavigationCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  void reopenPort();
  void setParam(const rclcpp::Parameter & param);
  void resetTracker();
  void processPacket(const std::vector<uint8_t> & data, uint16_t cmd_id);

  // 新增：发布轮上真实反馈速度
  void publishWheelTwist(float vx, float vy, float wz);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_color
  using ResultFuturePtr =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Aiming point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  // TF broadcaster
  double timestamp_offset_ = 0.0;
  bool publish_tf_ = false;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Navigation subscriber / publisher
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr nav_state_pub_;

  // 新增：轮上速度辅助里程计输出
  std::string wheel_twist_topic_;
  double wheel_vx_scale_ = 1.0;
  double wheel_vy_scale_ = 1.0;
  double wheel_wz_scale_ = 1.0;
  double wheel_cov_vx_ = 0.30;
  double wheel_cov_vy_ = 0.30;
  double wheel_cov_wz_ = 0.50;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wheel_twist_pub_;

  // Navigation command scaling (for chassis unit/deadband mismatch tuning)
  double cmd_vx_scale_ = 1.0;
  double cmd_vy_scale_ = 1.0;
  double cmd_wz_scale_ = 1.0;

  // Keep stream bytes across receive() calls to handle half-packets/sticky packets.
  std::vector<uint8_t> rx_buffer_;

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
