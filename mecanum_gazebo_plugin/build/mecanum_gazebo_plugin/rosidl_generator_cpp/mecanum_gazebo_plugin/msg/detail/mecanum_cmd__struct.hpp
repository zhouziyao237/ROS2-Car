// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice

#ifndef MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__STRUCT_HPP_
#define MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mecanum_gazebo_plugin__msg__MecanumCmd __attribute__((deprecated))
#else
# define DEPRECATED__mecanum_gazebo_plugin__msg__MecanumCmd __declspec(deprecated)
#endif

namespace mecanum_gazebo_plugin
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MecanumCmd_
{
  using Type = MecanumCmd_<ContainerAllocator>;

  explicit MecanumCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->vt = 0.0;
    }
  }

  explicit MecanumCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vx = 0.0;
      this->vy = 0.0;
      this->vt = 0.0;
    }
  }

  // field types and members
  using _vx_type =
    double;
  _vx_type vx;
  using _vy_type =
    double;
  _vy_type vy;
  using _vt_type =
    double;
  _vt_type vt;

  // setters for named parameter idiom
  Type & set__vx(
    const double & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const double & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__vt(
    const double & _arg)
  {
    this->vt = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mecanum_gazebo_plugin__msg__MecanumCmd
    std::shared_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mecanum_gazebo_plugin__msg__MecanumCmd
    std::shared_ptr<mecanum_gazebo_plugin::msg::MecanumCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MecanumCmd_ & other) const
  {
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->vt != other.vt) {
      return false;
    }
    return true;
  }
  bool operator!=(const MecanumCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MecanumCmd_

// alias to use template instance with default allocator
using MecanumCmd =
  mecanum_gazebo_plugin::msg::MecanumCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mecanum_gazebo_plugin

#endif  // MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__STRUCT_HPP_
