#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>   // 如果需要也可加
#include <cmath>

// 如果实车有自定义电机消息，可在此处包含，本示例暂用占位
// #include "your_motor_msgs/msg/motor_speeds.hpp"

class UnifiedOdometry : public rclcpp::Node
{
public:
  UnifiedOdometry() : Node("unified_odometry")
  {
    // 声明参数（与 YAML 中 odometry.ros__parameters 对应）
    this->declare_parameter("use_sim", true);
    this->declare_parameter("wheel_radius", 0.0375);
    this->declare_parameter("wheel_base", 0.1550);
    this->declare_parameter("track_width", 0.4166);
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("publish_rate", 50.0);

    use_sim_ = this->get_parameter("use_sim").as_bool();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_   = this->get_parameter("wheel_base").as_double();
    track_width_  = this->get_parameter("track_width").as_double();
    odom_frame_   = this->get_parameter("odom_frame").as_string();
    base_frame_   = this->get_parameter("base_frame").as_string();
    double rate   = this->get_parameter("publish_rate").as_double();

    lx_ = track_width_ / 2.0;
    ly_ = wheel_base_ / 2.0;

    if (use_sim_) {
      joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&UnifiedOdometry::simJointCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "Odometry in SIMULATION mode");
    } else {
      // 实车模式：订阅电机反馈（此处用占位，实际应订阅真实话题）
      // motor_sub_ = this->create_subscription<your_motor_msgs::msg::MotorSpeeds>(
      //   "motor_speeds", 10,
      //   std::bind(&UnifiedOdometry::realMotorCallback, this, std::placeholders::_1));
      RCLCPP_WARN(this->get_logger(), "REAL mode not fully implemented yet.");
    }

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&UnifiedOdometry::publishOdometry, this));

    last_time_ = this->now();
  }

private:
  void simJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "FL_Joint") fl_vel_ = msg->velocity[i];
      else if (msg->name[i] == "FR_Joint") fr_vel_ = msg->velocity[i];
      else if (msg->name[i] == "BL_Joint") bl_vel_ = msg->velocity[i];
      else if (msg->name[i] == "BR_Joint") br_vel_ = msg->velocity[i];
    }
  }

  // void realMotorCallback(...) { ... }  // 实车时实现

  void publishOdometry()
  {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) return;

    double vx = (wheel_radius_ / 4.0) * (fl_vel_ + fr_vel_ + bl_vel_ + br_vel_);
    double vy = (wheel_radius_ / 4.0) * (-fl_vel_ + fr_vel_ + bl_vel_ - br_vel_);
    double vt = (wheel_radius_ / (4.0 * (lx_ + ly_))) * (-fl_vel_ + fr_vel_ - bl_vel_ + br_vel_);

    double delta_x = (vx * cos(yaw_) - vy * sin(yaw_)) * dt;
    double delta_y = (vx * sin(yaw_) + vy * cos(yaw_)) * dt;
    double delta_yaw = vt * dt;

    x_ += delta_x;
    y_ += delta_y;
    yaw_ += delta_yaw;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = transform.transform.rotation;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vt;
    odom_pub_->publish(odom);

    last_time_ = current_time;
  }

  bool use_sim_;
  double wheel_radius_, wheel_base_, track_width_, lx_, ly_;
  std::string odom_frame_, base_frame_;

  double fl_vel_ = 0.0, fr_vel_ = 0.0, bl_vel_ = 0.0, br_vel_ = 0.0;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  rclcpp::Time last_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  // rclcpp::Subscription<...>::SharedPtr motor_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnifiedOdometry>());
  rclcpp::shutdown();
  return 0;
}