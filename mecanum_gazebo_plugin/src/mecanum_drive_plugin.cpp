#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mecanum_gazebo_plugin/msg/mecanum_cmd.hpp"

namespace gazebo
{
class MecanumDrivePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    // 初始化 ROS 节点
    this->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // 从 SDF 参数读取关节名称（如果没有则使用默认值）
    std::string fl_joint_name = "FL_Joint";
    std::string fr_joint_name = "FR_Joint";
    std::string bl_joint_name = "BL_Joint";
    std::string br_joint_name = "BR_Joint";
    if (_sdf->HasElement("fl_joint")) fl_joint_name = _sdf->Get<std::string>("fl_joint");
    if (_sdf->HasElement("fr_joint")) fr_joint_name = _sdf->Get<std::string>("fr_joint");
    if (_sdf->HasElement("bl_joint")) bl_joint_name = _sdf->Get<std::string>("bl_joint");
    if (_sdf->HasElement("br_joint")) br_joint_name = _sdf->Get<std::string>("br_joint");

    // 读取轮子参数（可从 SDF 读取，这里硬编码为常用值，也可以从 ROS 参数读取）
    wheel_radius_ = 0.0375;
    wheel_base_   = 0.1550;
    track_width_  = 0.4166;
    if (_sdf->HasElement("wheel_radius")) wheel_radius_ = _sdf->Get<double>("wheel_radius");
    if (_sdf->HasElement("wheel_base"))   wheel_base_   = _sdf->Get<double>("wheel_base");
    if (_sdf->HasElement("track_width"))  track_width_  = _sdf->Get<double>("track_width");

    // 获取关节指针
    fl_joint_ = _model->GetJoint(fl_joint_name);
    fr_joint_ = _model->GetJoint(fr_joint_name);
    bl_joint_ = _model->GetJoint(bl_joint_name);
    br_joint_ = _model->GetJoint(br_joint_name);

    if (!fl_joint_ || !fr_joint_ || !bl_joint_ || !br_joint_)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to find one or more wheel joints!");
      return;
    }

    // 订阅自定义话题
    sub_ = ros_node_->create_subscription<mecanum_gazebo_plugin::msg::MecanumCmd>(
        "mecanum_cmd", rclcpp::QoS(1),
        std::bind(&MecanumDrivePlugin::OnCmd, this, std::placeholders::_1));

    // 连接 Gazebo 更新事件
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MecanumDrivePlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(), "MecanumDrivePlugin loaded successfully");
  }

private:
  void OnCmd(const mecanum_gazebo_plugin::msg::MecanumCmd::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    target_vx_ = msg->vx;
    target_vy_ = msg->vy;
    target_vt_ = msg->vt;
  }

  void OnUpdate()
  {
    double vx, vy, vt;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vx = target_vx_;
      vy = target_vy_;
      vt = target_vt_;
    }

    const double lx = track_width_ / 2.0;
    const double ly = wheel_base_ / 2.0;
    const double inv_r = 1.0 / wheel_radius_;

    double w_fl = inv_r * (vx - vy - (lx + ly) * vt);
    double w_fr = inv_r * (vx + vy + (lx + ly) * vt);
    double w_bl = inv_r * (vx + vy - (lx + ly) * vt);
    double w_br = inv_r * (vx - vy + (lx + ly) * vt);

    fl_joint_->SetVelocity(0, w_fl);
    fr_joint_->SetVelocity(0, w_fr);
    bl_joint_->SetVelocity(0, w_bl);
    br_joint_->SetVelocity(0, w_br);
  }

  // Gazebo 对象
  physics::JointPtr fl_joint_, fr_joint_, bl_joint_, br_joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<mecanum_gazebo_plugin::msg::MecanumCmd>::SharedPtr sub_;
  event::ConnectionPtr update_connection_;

  // 参数
  double wheel_radius_, wheel_base_, track_width_;

  // 指令缓存
  std::mutex cmd_mutex_;
  double target_vx_ = 0.0, target_vy_ = 0.0, target_vt_ = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(MecanumDrivePlugin)
}