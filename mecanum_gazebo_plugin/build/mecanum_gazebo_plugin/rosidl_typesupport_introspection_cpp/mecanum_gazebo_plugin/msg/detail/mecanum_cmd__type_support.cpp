// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mecanum_gazebo_plugin/msg/detail/mecanum_cmd__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mecanum_gazebo_plugin
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MecanumCmd_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mecanum_gazebo_plugin::msg::MecanumCmd(_init);
}

void MecanumCmd_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mecanum_gazebo_plugin::msg::MecanumCmd *>(message_memory);
  typed_message->~MecanumCmd();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MecanumCmd_message_member_array[3] = {
  {
    "vx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mecanum_gazebo_plugin::msg::MecanumCmd, vx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mecanum_gazebo_plugin::msg::MecanumCmd, vy),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vt",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mecanum_gazebo_plugin::msg::MecanumCmd, vt),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MecanumCmd_message_members = {
  "mecanum_gazebo_plugin::msg",  // message namespace
  "MecanumCmd",  // message name
  3,  // number of fields
  sizeof(mecanum_gazebo_plugin::msg::MecanumCmd),
  MecanumCmd_message_member_array,  // message members
  MecanumCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  MecanumCmd_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MecanumCmd_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MecanumCmd_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mecanum_gazebo_plugin


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mecanum_gazebo_plugin::msg::MecanumCmd>()
{
  return &::mecanum_gazebo_plugin::msg::rosidl_typesupport_introspection_cpp::MecanumCmd_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mecanum_gazebo_plugin, msg, MecanumCmd)() {
  return &::mecanum_gazebo_plugin::msg::rosidl_typesupport_introspection_cpp::MecanumCmd_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
