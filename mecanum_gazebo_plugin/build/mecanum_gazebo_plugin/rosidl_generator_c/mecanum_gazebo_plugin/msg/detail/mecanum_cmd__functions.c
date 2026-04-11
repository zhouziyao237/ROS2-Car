// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mecanum_gazebo_plugin:msg/MecanumCmd.idl
// generated code does not contain a copyright notice
#include "mecanum_gazebo_plugin/msg/detail/mecanum_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mecanum_gazebo_plugin__msg__MecanumCmd__init(mecanum_gazebo_plugin__msg__MecanumCmd * msg)
{
  if (!msg) {
    return false;
  }
  // vx
  // vy
  // vt
  return true;
}

void
mecanum_gazebo_plugin__msg__MecanumCmd__fini(mecanum_gazebo_plugin__msg__MecanumCmd * msg)
{
  if (!msg) {
    return;
  }
  // vx
  // vy
  // vt
}

bool
mecanum_gazebo_plugin__msg__MecanumCmd__are_equal(const mecanum_gazebo_plugin__msg__MecanumCmd * lhs, const mecanum_gazebo_plugin__msg__MecanumCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vx
  if (lhs->vx != rhs->vx) {
    return false;
  }
  // vy
  if (lhs->vy != rhs->vy) {
    return false;
  }
  // vt
  if (lhs->vt != rhs->vt) {
    return false;
  }
  return true;
}

bool
mecanum_gazebo_plugin__msg__MecanumCmd__copy(
  const mecanum_gazebo_plugin__msg__MecanumCmd * input,
  mecanum_gazebo_plugin__msg__MecanumCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // vx
  output->vx = input->vx;
  // vy
  output->vy = input->vy;
  // vt
  output->vt = input->vt;
  return true;
}

mecanum_gazebo_plugin__msg__MecanumCmd *
mecanum_gazebo_plugin__msg__MecanumCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanum_gazebo_plugin__msg__MecanumCmd * msg = (mecanum_gazebo_plugin__msg__MecanumCmd *)allocator.allocate(sizeof(mecanum_gazebo_plugin__msg__MecanumCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mecanum_gazebo_plugin__msg__MecanumCmd));
  bool success = mecanum_gazebo_plugin__msg__MecanumCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mecanum_gazebo_plugin__msg__MecanumCmd__destroy(mecanum_gazebo_plugin__msg__MecanumCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mecanum_gazebo_plugin__msg__MecanumCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__init(mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanum_gazebo_plugin__msg__MecanumCmd * data = NULL;

  if (size) {
    data = (mecanum_gazebo_plugin__msg__MecanumCmd *)allocator.zero_allocate(size, sizeof(mecanum_gazebo_plugin__msg__MecanumCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mecanum_gazebo_plugin__msg__MecanumCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mecanum_gazebo_plugin__msg__MecanumCmd__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__fini(mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mecanum_gazebo_plugin__msg__MecanumCmd__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mecanum_gazebo_plugin__msg__MecanumCmd__Sequence *
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * array = (mecanum_gazebo_plugin__msg__MecanumCmd__Sequence *)allocator.allocate(sizeof(mecanum_gazebo_plugin__msg__MecanumCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__destroy(mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__are_equal(const mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * lhs, const mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mecanum_gazebo_plugin__msg__MecanumCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mecanum_gazebo_plugin__msg__MecanumCmd__Sequence__copy(
  const mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * input,
  mecanum_gazebo_plugin__msg__MecanumCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mecanum_gazebo_plugin__msg__MecanumCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mecanum_gazebo_plugin__msg__MecanumCmd * data =
      (mecanum_gazebo_plugin__msg__MecanumCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mecanum_gazebo_plugin__msg__MecanumCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mecanum_gazebo_plugin__msg__MecanumCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mecanum_gazebo_plugin__msg__MecanumCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
