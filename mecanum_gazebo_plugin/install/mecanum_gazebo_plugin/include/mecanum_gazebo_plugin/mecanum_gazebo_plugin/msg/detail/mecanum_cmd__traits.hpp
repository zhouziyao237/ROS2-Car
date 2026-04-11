// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice

#ifndef MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__TRAITS_HPP_
#define MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mecanum_gazebo_plugin/msg/detail/mecanum_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mecanum_gazebo_plugin
{

namespace msg
{

inline void to_flow_style_yaml(
  const MecanumCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: vt
  {
    out << "vt: ";
    rosidl_generator_traits::value_to_yaml(msg.vt, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MecanumCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: vt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vt: ";
    rosidl_generator_traits::value_to_yaml(msg.vt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MecanumCmd & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mecanum_gazebo_plugin

namespace rosidl_generator_traits
{

[[deprecated("use mecanum_gazebo_plugin::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mecanum_gazebo_plugin::msg::MecanumCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  mecanum_gazebo_plugin::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mecanum_gazebo_plugin::msg::to_yaml() instead")]]
inline std::string to_yaml(const mecanum_gazebo_plugin::msg::MecanumCmd & msg)
{
  return mecanum_gazebo_plugin::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mecanum_gazebo_plugin::msg::MecanumCmd>()
{
  return "mecanum_gazebo_plugin::msg::MecanumCmd";
}

template<>
inline const char * name<mecanum_gazebo_plugin::msg::MecanumCmd>()
{
  return "mecanum_gazebo_plugin/msg/MecanumCmd";
}

template<>
struct has_fixed_size<mecanum_gazebo_plugin::msg::MecanumCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mecanum_gazebo_plugin::msg::MecanumCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mecanum_gazebo_plugin::msg::MecanumCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__TRAITS_HPP_
