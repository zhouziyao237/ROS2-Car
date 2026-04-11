// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice

#ifndef MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__FUNCTIONS_H_
#define MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mecanum_gazebo_plugin/msg/rosidl_generator_c__visibility_control.h"

#include "mecanum_gazebo_plugin/msg/detail/mecanum_cmd__struct.h"

/// Initialize msg/MecanumCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mecanum_gazebo_plugin__msg__MecanumCmd
 * )) before or use
 * mecanum_gazebo_plugin__msg__MecanumCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
bool
mecanum_gazebo_plugin__msg__MecanumCmd__init(mecanum_gazebo_plugin__msg__MecanumCmd * msg);

/// Finalize msg/MecanumCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
void
mecanum_gazebo_plugin__msg__MecanumCmd__fini(mecanum_gazebo_plugin__msg__MecanumCmd * msg);

/// Create msg/MecanumCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mecanum_gazebo_plugin__msg__MecanumCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
mecanum_gazebo_plugin__msg__MecanumCmd *
mecanum_gazebo_plugin__msg__MecanumCmd__create();

/// Destroy msg/MecanumCmd message.
/**
 * It calls
 * mecanum_gazebo_plugin__msg__MecanumCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
void
mecanum_gazebo_plugin__msg__MecanumCmd__destroy(mecanum_gazebo_plugin__msg__MecanumCmd * msg);

/// Check for msg/MecanumCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
bool
mecanum_gazebo_plugin__msg__MecanumCmd__are_equal(const mecanum_gazebo_plugin__msg__MecanumCmd * lhs, const mecanum_gazebo_plugin__msg__MecanumCmd * rhs);

/// Copy a msg/MecanumCmd message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
bool
mecanum_gazebo_plugin__msg__MecanumCmd__copy(
  const mecanum_gazebo_plugin__msg__MecanumCmd * input,
  mecanum_gazebo_plugin__msg__MecanumCmd * output);

/// Initialize array of msg/MecanumCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * mecanum_gazebo_plugin__msg__MecanumCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
bool
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__init(mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * array, size_t size);

/// Finalize array of msg/MecanumCmd messages.
/**
 * It calls
 * mecanum_gazebo_plugin__msg__MecanumCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
void
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__fini(mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * array);

/// Create array of msg/MecanumCmd messages.
/**
 * It allocates the memory for the array and calls
 * mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence *
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__create(size_t size);

/// Destroy array of msg/MecanumCmd messages.
/**
 * It calls
 * mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
void
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__destroy(mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * array);

/// Check for msg/MecanumCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
bool
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__are_equal(const mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * lhs, const mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * rhs);

/// Copy an array of msg/MecanumCmd messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanum_gazebo_plugin
bool
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__copy(
  const mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * input,
  mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MECANUM_GAZEBO_PLUGIN__MSG__DETAIL__MECANUM_CMD__FUNCTIONS_H_
