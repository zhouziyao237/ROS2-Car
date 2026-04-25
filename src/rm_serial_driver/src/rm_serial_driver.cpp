// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

/* 
警钟长鸣：

2026年1月13日 00:18 在经过3天的检查后发现自瞄抖动问题是相机线插usb2.0口导致
此前该车已经能正常自瞄，此前电控视觉都声称没有改过代码
在没有改过代码的情况下视觉立场一定要坚定
*/

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

// C++ system
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"
#include "rm_serial_driver/solve_trajectory.hpp"

namespace rm_serial_driver
{
namespace
{
constexpr size_t kMaxFrameLength = 256;

template<typename T>
bool decodePacketPrefix(const std::vector<uint8_t> & frame, T & packet)
{
  if (frame.size() < sizeof(T)) {
    return false;
  }
  std::memcpy(&packet, frame.data(), sizeof(T));
  return true;
}
}  // namespace

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster / wheel twist publisher
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  publish_tf_ = this->declare_parameter("publish_tf", false);
  wheel_twist_topic_ = this->declare_parameter("wheel_twist_topic", std::string("/wheel/twist"));
  wheel_vx_scale_ = this->declare_parameter("wheel_vx_scale", 1.0);
  wheel_vy_scale_ = this->declare_parameter("wheel_vy_scale", 1.0);
  wheel_wz_scale_ = this->declare_parameter("wheel_wz_scale", 1.0);
  wheel_cov_vx_ = this->declare_parameter("wheel_cov_vx", 0.30);
  wheel_cov_vy_ = this->declare_parameter("wheel_cov_vy", 0.30);
  wheel_cov_wz_ = this->declare_parameter("wheel_cov_wz", 0.50);
  cmd_vx_scale_ = this->declare_parameter("cmd_vx_scale", 1.0);
  cmd_vy_scale_ = this->declare_parameter("cmd_vy_scale", 1.0);
  cmd_wz_scale_ = this->declare_parameter("cmd_wz_scale", 1.0);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  nav_state_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/serial/gimbal_joint_state", 10);
  wheel_twist_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(wheel_twist_topic_, 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  // 创建导航指令订阅器
  nav_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&RMSerialDriver::sendNavigationCmd, this, std::placeholders::_1));

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  constexpr size_t BUFFER_SIZE = 1024;
  std::vector<uint8_t> read_buffer(BUFFER_SIZE);
  rx_buffer_.clear();

  while (rclcpp::ok()) {
    try {
      // 读取数据到缓冲区
      const size_t bytes_read = serial_driver_->port()->receive(read_buffer);

      if (bytes_read == 0) {
        continue;
      }

      // 持续缓存，解决半包/粘包问题
      rx_buffer_.insert(rx_buffer_.end(), read_buffer.begin(), read_buffer.begin() + bytes_read);

      while (rx_buffer_.size() >= sizeof(Header)) {
        auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), 0xA5);
        if (it == rx_buffer_.end()) {
          rx_buffer_.clear();
          break;
        }
        if (it != rx_buffer_.begin()) {
          rx_buffer_.erase(rx_buffer_.begin(), it);
        }

        if (rx_buffer_.size() < sizeof(Header)) {
          break;
        }

        Header header{};
        std::memcpy(&header, rx_buffer_.data(), sizeof(Header));
        if (!crc8::Verify_CRC8_Check_Sum(
              reinterpret_cast<const uint8_t *>(&header), sizeof(Header) - 2)) {
          // 帧头CRC错误，滑窗一字节继续找
          rx_buffer_.erase(rx_buffer_.begin());
          continue;
        }

        const size_t total_length = sizeof(Header) + header.data_length + 2;
        if (total_length < sizeof(Header) + 2 || total_length > kMaxFrameLength) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Drop abnormal serial frame: cmd=0x%04X len=%zu", header.cmd_id, total_length);
          rx_buffer_.erase(rx_buffer_.begin());
          continue;
        }

        if (rx_buffer_.size() < total_length) {
          break;  // 数据不完整，等待下一次接收
        }

        std::vector<uint8_t> frame(rx_buffer_.begin(), rx_buffer_.begin() + total_length);
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_length);

        if (!crc16::Verify_CRC16_Check_Sum(frame.data(), frame.size())) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Frame CRC16 error: cmd=0x%04X len=%zu", header.cmd_id, frame.size());
          continue;
        }

        processPacket(frame, header.cmd_id);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::processPacket(const std::vector<uint8_t> & data, uint16_t cmd_id)
{
  switch (cmd_id) {
    case 0x502: {
      IMUPacket imu_packet{};
      if (!decodePacketPrefix(data, imu_packet)) {
        RCLCPP_WARN(
          get_logger(), "IMU frame too short: %zu < %zu", data.size(), sizeof(IMUPacket));
        return;
      }

      if (!initial_set_param_ || (imu_packet.roboid > 100 ? 0 : 1) != previous_receive_color_) {
        setParam(rclcpp::Parameter("detect_color", (imu_packet.roboid > 100 ? 0 : 1)));
        previous_receive_color_ = imu_packet.roboid > 100 ? 0 : 1;
      }

      st.current_pitch = imu_packet.pitch;
      st.current_yaw = imu_packet.yaw;

      // st.time = imu_packet.timeseries; //时间戳
      // RCLCPP_INFO(get_logger(), "时间戳%d", st.time);

      // 发布TF变换（默认关闭，避免干扰底盘定位主链）
      if (publish_tf_ && tf_broadcaster_) {
        auto t = std::make_unique<geometry_msgs::msg::TransformStamped>();
        t->header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        t->header.frame_id = "odom";
        t->child_frame_id = "gimbal_link";

        tf2::Quaternion q;
        q.setRPY(imu_packet.roll, imu_packet.pitch, imu_packet.yaw);
        t->transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(*t);
      }

      //导航部分////////////////////////////////////////
      // 发布导航状态////////////////////////////////////////////
      auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
      twist_msg->angular.x = imu_packet.pitch;  // pitch
      twist_msg->angular.y = imu_packet.roll;   // roll
      twist_msg->angular.z = imu_packet.yaw;    // yaw
      nav_state_pub_->publish(*twist_msg);

      // RCLCPP_INFO(get_logger(), "[接收导航状态] pitch:%.2f, yaw:%.2f",
      //             imu_packet.pitch, imu_packet.yaw);
      break;
    }
    case 0x503: {
      chassisCurrentStatusPacket chassis_status{};
      if (!decodePacketPrefix(data, chassis_status)) {
        RCLCPP_WARN(
          get_logger(), "Chassis status frame too short: %zu < %zu",
          data.size(), sizeof(chassisCurrentStatusPacket));
        break;
      }

      // 0x0503 current chassis status from MCU:
      // speed_x / speed_y are body-frame linear speeds, speed_turn is yaw rate.
      publishWheelTwist(
        chassis_status.speed_x, chassis_status.speed_y, chassis_status.speed_turn);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Chassis status enabled=%u raw(vx=%.3f vy=%.3f wz=%.3f) "
        "scaled(vx=%.3f vy=%.3f wz=%.3f)",
        static_cast<unsigned int>(chassis_status.chassis_enabled),
        chassis_status.speed_x, chassis_status.speed_y, chassis_status.speed_turn,
        chassis_status.speed_x * wheel_vx_scale_,
        chassis_status.speed_y * wheel_vy_scale_,
        chassis_status.speed_turn * wheel_wz_scale_);
      break;
    }
    case 0x504: {
      bottonPacket botton_packet{};
      if (!decodePacketPrefix(data, botton_packet)) {
        RCLCPP_WARN(
          get_logger(), "Button frame too short: %zu < %zu", data.size(), sizeof(bottonPacket));
      }
      break;
    }
    case 0x505: {
      bassPacket bass_packet{};
      if (!decodePacketPrefix(data, bass_packet)) {
        RCLCPP_WARN(
          get_logger(), "Bass frame too short: %zu < %zu", data.size(), sizeof(bassPacket));
        return;
      }

      if (bass_packet.reset) {
        resetTracker();
      }

      break;
    }
  }
}

void RMSerialDriver::publishWheelTwist(float vx, float vy, float wz)
{
  const double sx = static_cast<double>(vx) * wheel_vx_scale_;
  const double sy = static_cast<double>(vy) * wheel_vy_scale_;
  const double swz = static_cast<double>(wz) * wheel_wz_scale_;

  geometry_msgs::msg::TwistWithCovarianceStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.twist.twist.linear.x = sx;
  msg.twist.twist.linear.y = sy;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = swz;

  msg.twist.covariance = {
    wheel_cov_vx_, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, wheel_cov_vy_, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, wheel_cov_wz_
  };

  wheel_twist_pub_->publish(msg);
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  const static std::map<std::string, uint8_t> robo_id_unit8_map{
    {"", 0x000},  {"outpost", 0x010}, {"1", 0x001},     {"2", 0x002},   {"3", 0x003},
    {"4", 0x004}, {"5", 0x005},       {"guard", 0x007}, {"base", 0x011}};

  try {
    Header header;
    SendPacket packet;
    header.data_length = sizeof(packet) - sizeof(header) - 2;
    crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&header), sizeof(header) - 2);
    header.cmd_id = 0x402;
    packet.header = header;
    float aim_x = 0, aim_y = 0, aim_z = 0;
    float pitch = 0, yaw = 0;

    st.xw = msg->position.x;
    st.yw = msg->position.y;
    st.zw = msg->position.z;
    st.vxw = msg->velocity.x;
    st.vyw = msg->velocity.y;
    st.vzw = msg->velocity.z;
    st.tar_yaw = msg->yaw;
    st.v_yaw = msg->v_yaw;
    st.r1 = msg->radius_1;
    st.r2 = msg->radius_2;
    st.dz = msg->dz;
    st.armor_id = ARMOR_ID(id_unit8_map.at(msg->id));
    st.armor_num = ARMOR_NUM(msg->armors_num);

    // 设置固定值
    // packet.pitch = 0.0;
    // packet.yaw = -1.57;

    packet.shoot =
      autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z) && (msg->tracking ? 1 : 0);

    // packet.shoot = 1;

    packet.timeseries = st.time;




    packet.robo_id = robo_id_unit8_map.at(msg->id);
    if (packet.shoot) {
      packet.pitch = pitch;
      //2025年11月11日23点00分 fpc要求，由于电控未知原因，yaw加180度
      packet.yaw = yaw;
    }
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    RCLCPP_INFO(get_logger(), "[Send] pitch %f!", packet.pitch);
    RCLCPP_INFO(get_logger(), "[Send] yaw %f!", packet.yaw);
    RCLCPP_INFO(get_logger(), "[Send] shoot %d!", packet.shoot);

    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);

    if (abs(aim_x) > 0.01) {
      aiming_point_.header.stamp = this->now();
      aiming_point_.pose.position.x = aim_x;
      aiming_point_.pose.position.y = aim_y;
      aiming_point_.pose.position.z = aim_z;
      marker_pub_->publish(aiming_point_);
    }

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::sendNavigationCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    NavigationPacket packet;
    packet.header.data_length = sizeof(packet) - sizeof(Header) - 2;
    packet.header.cmd_id = 0x405;
    crc8::Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&packet.header), sizeof(Header) - 2);

    const float vx = static_cast<float>(msg->linear.x * cmd_vx_scale_);
    const float vy = static_cast<float>(msg->linear.y * cmd_vy_scale_);
    const float wz = static_cast<float>(msg->angular.z * cmd_wz_scale_);

    packet.vx = vx;
    packet.vy = vy;
    packet.wz = wz;

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    std::vector<uint8_t> data(sizeof(packet));
    std::copy(
      reinterpret_cast<const uint8_t *>(&packet),
      reinterpret_cast<const uint8_t *>(&packet) + sizeof(packet), data.begin());

    serial_driver_->port()->send(data);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[发送导航指令] raw(vx:%.2f, vy:%.2f, wz:%.2f) scaled(vx:%.2f, vy:%.2f, wz:%.2f)",
      msg->linear.x, msg->linear.y, msg->angular.z,
      packet.vx, packet.vy, packet.wz);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "发送导航指令失败: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    // RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
