// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice

#ifndef MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__BUILDER_HPP_
#define MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mecanum_gazebo_plugin/msg/detail/mecanum_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mecanum_gazebo_plugin
{

namespace msg
{

namespace builder
{

class Init_MecanumCmd_vt
{
public:
  explicit Init_MecanumCmd_vt(::mecanum_gazebo_plugin::msg::MecanumCmd & msg)
  : msg_(msg)
  {}
  ::mecanum_gazebo_plugin::msg::MecanumCmd vt(::mecanum_gazebo_plugin::msg::MecanumCmd::_vt_type arg)
  {
    msg_.vt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mecanum_gazebo_plugin::msg::MecanumCmd msg_;
};

class Init_MecanumCmd_vy
{
public:
  explicit Init_MecanumCmd_vy(::mecanum_gazebo_plugin::msg::MecanumCmd & msg)
  : msg_(msg)
  {}
  Init_MecanumCmd_vt vy(::mecanum_gazebo_plugin::msg::MecanumCmd::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_MecanumCmd_vt(msg_);
  }

private:
  ::mecanum_gazebo_plugin::msg::MecanumCmd msg_;
};

class Init_MecanumCmd_vx
{
public:
  Init_MecanumCmd_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MecanumCmd_vy vx(::mecanum_gazebo_plugin::msg::MecanumCmd::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_MecanumCmd_vy(msg_);
  }

private:
  ::mecanum_gazebo_plugin::msg::MecanumCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mecanum_gazebo_plugin::msg::MecanumCmd>()
{
  return mecanum_gazebo_plugin::msg::builder::Init_MecanumCmd_vx();
}

}  // namespace mecanum_gazebo_plugin

#endif  // MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__BUILDER_HPP_
