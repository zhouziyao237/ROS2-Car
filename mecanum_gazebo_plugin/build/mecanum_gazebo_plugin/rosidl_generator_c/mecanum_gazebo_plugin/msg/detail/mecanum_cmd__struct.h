// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice

#ifndef MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__STRUCT_H_
#define MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MecanumCmd in the package mecanum_gazebo_plugin.
typedef struct mecanum_gazebo_plugin__msg__MecanumCmd
{
  /// m/s
  double vx;
  /// m/s
  double vy;
  /// rad/s
  double vt;
} mecanum_gazebo_plugin__msg__MecanumCmd;

// Struct for a sequence of mecanum_gazebo_plugin__msg__MecanumCmd.
typedef struct mecanum_gazebo_plugin__msg__MecanumCmd__Sequence
{
  mecanum_gazebo_plugin__msg__MecanumCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mecanum_gazebo_plugin__msg__MecanumCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__STRUCT_H_
