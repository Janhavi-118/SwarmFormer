// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `robot_id`
// Member `neighbors`
// Member `task_status`
// Member `error_flags`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
swarm_msgs__msg__RobotState__init(swarm_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    swarm_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    swarm_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // position
  // velocity
  // neighbors
  if (!rosidl_runtime_c__String__Sequence__init(&msg->neighbors, 0)) {
    swarm_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // local_obstacle_count
  // battery_level
  // task_status
  if (!rosidl_runtime_c__String__init(&msg->task_status)) {
    swarm_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // error_flags
  if (!rosidl_runtime_c__String__Sequence__init(&msg->error_flags, 0)) {
    swarm_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // heartbeat_count
  return true;
}

void
swarm_msgs__msg__RobotState__fini(swarm_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // position
  // velocity
  // neighbors
  rosidl_runtime_c__String__Sequence__fini(&msg->neighbors);
  // local_obstacle_count
  // battery_level
  // task_status
  rosidl_runtime_c__String__fini(&msg->task_status);
  // error_flags
  rosidl_runtime_c__String__Sequence__fini(&msg->error_flags);
  // heartbeat_count
}

bool
swarm_msgs__msg__RobotState__are_equal(const swarm_msgs__msg__RobotState * lhs, const swarm_msgs__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_id), &(rhs->robot_id)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // position
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->position[i] != rhs->position[i]) {
      return false;
    }
  }
  // velocity
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->velocity[i] != rhs->velocity[i]) {
      return false;
    }
  }
  // neighbors
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->neighbors), &(rhs->neighbors)))
  {
    return false;
  }
  // local_obstacle_count
  if (lhs->local_obstacle_count != rhs->local_obstacle_count) {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
    return false;
  }
  // task_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->task_status), &(rhs->task_status)))
  {
    return false;
  }
  // error_flags
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->error_flags), &(rhs->error_flags)))
  {
    return false;
  }
  // heartbeat_count
  if (lhs->heartbeat_count != rhs->heartbeat_count) {
    return false;
  }
  return true;
}

bool
swarm_msgs__msg__RobotState__copy(
  const swarm_msgs__msg__RobotState * input,
  swarm_msgs__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_id), &(output->robot_id)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // position
  for (size_t i = 0; i < 3; ++i) {
    output->position[i] = input->position[i];
  }
  // velocity
  for (size_t i = 0; i < 3; ++i) {
    output->velocity[i] = input->velocity[i];
  }
  // neighbors
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->neighbors), &(output->neighbors)))
  {
    return false;
  }
  // local_obstacle_count
  output->local_obstacle_count = input->local_obstacle_count;
  // battery_level
  output->battery_level = input->battery_level;
  // task_status
  if (!rosidl_runtime_c__String__copy(
      &(input->task_status), &(output->task_status)))
  {
    return false;
  }
  // error_flags
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->error_flags), &(output->error_flags)))
  {
    return false;
  }
  // heartbeat_count
  output->heartbeat_count = input->heartbeat_count;
  return true;
}

swarm_msgs__msg__RobotState *
swarm_msgs__msg__RobotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__RobotState * msg = (swarm_msgs__msg__RobotState *)allocator.allocate(sizeof(swarm_msgs__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_msgs__msg__RobotState));
  bool success = swarm_msgs__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_msgs__msg__RobotState__destroy(swarm_msgs__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_msgs__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_msgs__msg__RobotState__Sequence__init(swarm_msgs__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__RobotState * data = NULL;

  if (size) {
    data = (swarm_msgs__msg__RobotState *)allocator.zero_allocate(size, sizeof(swarm_msgs__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_msgs__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_msgs__msg__RobotState__fini(&data[i - 1]);
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
swarm_msgs__msg__RobotState__Sequence__fini(swarm_msgs__msg__RobotState__Sequence * array)
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
      swarm_msgs__msg__RobotState__fini(&array->data[i]);
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

swarm_msgs__msg__RobotState__Sequence *
swarm_msgs__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__RobotState__Sequence * array = (swarm_msgs__msg__RobotState__Sequence *)allocator.allocate(sizeof(swarm_msgs__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_msgs__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_msgs__msg__RobotState__Sequence__destroy(swarm_msgs__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_msgs__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_msgs__msg__RobotState__Sequence__are_equal(const swarm_msgs__msg__RobotState__Sequence * lhs, const swarm_msgs__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_msgs__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__RobotState__Sequence__copy(
  const swarm_msgs__msg__RobotState__Sequence * input,
  swarm_msgs__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_msgs__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_msgs__msg__RobotState * data =
      (swarm_msgs__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_msgs__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_msgs__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_msgs__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
