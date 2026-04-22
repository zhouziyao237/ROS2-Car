#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "mecanum_gazebo_plugin/msg/mecanum_cmd.hpp"

namespace gazebo
{
class MecanumDrivePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // 从 SDF 获取配置文件路径（必须）
    std::string config_path;
    if (_sdf->HasElement("config_file")) {
      config_path = _sdf->Get<std::string>("config_file");
    } else {
      RCLCPP_ERROR(ros_node_->get_logger(), "Missing 'config_file' in plugin SDF!");
      return;
    }

    // 解析 YAML
    YAML::Node config;
    try {
      config = YAML::LoadFile(config_path);
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to load YAML: %s", e.what());
      return;
    }

    // 提取 mecanum_drive_plugin.ros__parameters 节点
    YAML::Node plugin_params;
    if (config["mecanum_drive_plugin"] && config["mecanum_drive_plugin"]["ros__parameters"]) {
      plugin_params = config["mecanum_drive_plugin"]["ros__parameters"];
    } else {
      RCLCPP_ERROR(ros_node_->get_logger(), "YAML missing 'mecanum_drive_plugin.ros__parameters' section");
      return;
    }

    // 读取参数（支持默认值）
    wheel_radius_ = plugin_params["wheel_radius"].as<double>(0.0375);
    wheel_base_   = plugin_params["wheel_base"].as<double>(0.1550);
    track_width_  = plugin_params["track_width"].as<double>(0.4166);
    fl_dir_ = plugin_params["fl_dir"].as<double>(1.0);
    fr_dir_ = plugin_params["fr_dir"].as<double>(1.0);
    bl_dir_ = plugin_params["bl_dir"].as<double>(1.0);
    br_dir_ = plugin_params["br_dir"].as<double>(1.0);

    std::string fl_joint = plugin_params["fl_joint"].as<std::string>("FL_Joint");
    std::string fr_joint = plugin_params["fr_joint"].as<std::string>("FR_Joint");
    std::string bl_joint = plugin_params["bl_joint"].as<std::string>("BL_Joint");
    std::string br_joint = plugin_params["br_joint"].as<std::string>("BR_Joint");

    // SDF 可覆盖关节名（优先级最高）
    if (_sdf->HasElement("fl_joint")) fl_joint = _sdf->Get<std::string>("fl_joint");
    if (_sdf->HasElement("fr_joint")) fr_joint = _sdf->Get<std::string>("fr_joint");
    if (_sdf->HasElement("bl_joint")) bl_joint = _sdf->Get<std::string>("bl_joint");
    if (_sdf->HasElement("br_joint")) br_joint = _sdf->Get<std::string>("br_joint");

    // 获取关节指针
    fl_joint_ = _model->GetJoint(fl_joint);
    fr_joint_ = _model->GetJoint(fr_joint);
    bl_joint_ = _model->GetJoint(bl_joint);
    br_joint_ = _model->GetJoint(br_joint);
    if (!fl_joint_ || !fr_joint_ || !bl_joint_ || !br_joint_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Failed to find wheel joints!");
      return;
    }

    // 订阅自定义话题
    sub_ = ros_node_->create_subscription<mecanum_gazebo_plugin::msg::MecanumCmd>(
        "mecanum_cmd", rclcpp::QoS(1),
        std::bind(&MecanumDrivePlugin::OnCmd, this, std::placeholders::_1));

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MecanumDrivePlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
                "MecanumDrivePlugin loaded. r=%.4f, base=%.4f, track=%.4f, dirs=[%.1f,%.1f,%.1f,%.1f]",
                wheel_radius_, wheel_base_, track_width_, fl_dir_, fr_dir_, bl_dir_, br_dir_);
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

    double w_fl = fl_dir_ * inv_r * (vx - vy - (lx + ly) * vt);
    double w_fr = fr_dir_ * inv_r * (vx + vy + (lx + ly) * vt);
    double w_bl = bl_dir_ * inv_r * (vx + vy - (lx + ly) * vt);
    double w_br = br_dir_ * inv_r * (vx - vy + (lx + ly) * vt);

    fl_joint_->SetVelocity(0, w_fl);
    fr_joint_->SetVelocity(0, w_fr);
    bl_joint_->SetVelocity(0, w_bl);
    br_joint_->SetVelocity(0, w_br);
  }

  physics::JointPtr fl_joint_, fr_joint_, bl_joint_, br_joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<mecanum_gazebo_plugin::msg::MecanumCmd>::SharedPtr sub_;
  event::ConnectionPtr update_connection_;

  double wheel_radius_, wheel_base_, track_width_;
  double fl_dir_, fr_dir_, bl_dir_, br_dir_;
  std::mutex cmd_mutex_;
  double target_vx_ = 0.0, target_vy_ = 0.0, target_vt_ = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(MecanumDrivePlugin)
}