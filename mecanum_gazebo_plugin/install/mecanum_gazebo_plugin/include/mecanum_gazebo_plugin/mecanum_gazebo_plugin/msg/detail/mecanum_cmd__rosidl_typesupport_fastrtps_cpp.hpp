// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice

#ifndef MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mecanum_gazebo_plugin/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mecanum_gazebo_plugin/msg/detail/mecanum_cmd__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace mecanum_gazebo_plugin
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mecanum_gazebo_plugin
cdr_serialize(
  const mecanum_gazebo_plugin::msg::MecanumCmd & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mecanum_gazebo_plugin
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mecanum_gazebo_plugin::msg::MecanumCmd & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mecanum_gazebo_plugin
get_serialized_size(
  const mecanum_gazebo_plugin::msg::MecanumCmd & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mecanum_gazebo_plugin
max_serialized_size_MecanumCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mecanum_gazebo_plugin

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mecanum_gazebo_plugin
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mecanum_gazebo_plugin, msg, MecanumCmd)();

#ifdef __cplusplus
}
#endif

#endif  // MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
