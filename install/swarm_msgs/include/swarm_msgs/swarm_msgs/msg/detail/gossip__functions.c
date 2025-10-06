// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/gossip__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `sender`
// Member `failed_robots`
// Member `message_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `robot_state`
// Member `known_neighbors`
#include "swarm_msgs/msg/detail/robot_state__functions.h"
// Member `gmm_params`
#include "swarm_msgs/msg/detail/gmm_params__functions.h"

bool
swarm_msgs__msg__Gossip__init(swarm_msgs__msg__Gossip * msg)
{
  if (!msg) {
    return false;
  }
  // sender
  if (!rosidl_runtime_c__String__init(&msg->sender)) {
    swarm_msgs__msg__Gossip__fini(msg);
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    swarm_msgs__msg__Gossip__fini(msg);
    return false;
  }
  // robot_state
  if (!swarm_msgs__msg__RobotState__init(&msg->robot_state)) {
    swarm_msgs__msg__Gossip__fini(msg);
    return false;
  }
  // known_neighbors
  if (!swarm_msgs__msg__RobotState__Sequence__init(&msg->known_neighbors, 0)) {
    swarm_msgs__msg__Gossip__fini(msg);
    return false;
  }
  // failed_robots
  if (!rosidl_runtime_c__String__Sequence__init(&msg->failed_robots, 0)) {
    swarm_msgs__msg__Gossip__fini(msg);
    return false;
  }
  // gmm_params
  if (!swarm_msgs__msg__GMMParams__init(&msg->gmm_params)) {
    swarm_msgs__msg__Gossip__fini(msg);
    return false;
  }
  // message_type
  if (!rosidl_runtime_c__String__init(&msg->message_type)) {
    swarm_msgs__msg__Gossip__fini(msg);
    return false;
  }
  return true;
}

void
swarm_msgs__msg__Gossip__fini(swarm_msgs__msg__Gossip * msg)
{
  if (!msg) {
    return;
  }
  // sender
  rosidl_runtime_c__String__fini(&msg->sender);
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // robot_state
  swarm_msgs__msg__RobotState__fini(&msg->robot_state);
  // known_neighbors
  swarm_msgs__msg__RobotState__Sequence__fini(&msg->known_neighbors);
  // failed_robots
  rosidl_runtime_c__String__Sequence__fini(&msg->failed_robots);
  // gmm_params
  swarm_msgs__msg__GMMParams__fini(&msg->gmm_params);
  // message_type
  rosidl_runtime_c__String__fini(&msg->message_type);
}

bool
swarm_msgs__msg__Gossip__are_equal(const swarm_msgs__msg__Gossip * lhs, const swarm_msgs__msg__Gossip * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sender
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sender), &(rhs->sender)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // robot_state
  if (!swarm_msgs__msg__RobotState__are_equal(
      &(lhs->robot_state), &(rhs->robot_state)))
  {
    return false;
  }
  // known_neighbors
  if (!swarm_msgs__msg__RobotState__Sequence__are_equal(
      &(lhs->known_neighbors), &(rhs->known_neighbors)))
  {
    return false;
  }
  // failed_robots
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->failed_robots), &(rhs->failed_robots)))
  {
    return false;
  }
  // gmm_params
  if (!swarm_msgs__msg__GMMParams__are_equal(
      &(lhs->gmm_params), &(rhs->gmm_params)))
  {
    return false;
  }
  // message_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message_type), &(rhs->message_type)))
  {
    return false;
  }
  return true;
}

bool
swarm_msgs__msg__Gossip__copy(
  const swarm_msgs__msg__Gossip * input,
  swarm_msgs__msg__Gossip * output)
{
  if (!input || !output) {
    return false;
  }
  // sender
  if (!rosidl_runtime_c__String__copy(
      &(input->sender), &(output->sender)))
  {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // robot_state
  if (!swarm_msgs__msg__RobotState__copy(
      &(input->robot_state), &(output->robot_state)))
  {
    return false;
  }
  // known_neighbors
  if (!swarm_msgs__msg__RobotState__Sequence__copy(
      &(input->known_neighbors), &(output->known_neighbors)))
  {
    return false;
  }
  // failed_robots
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->failed_robots), &(output->failed_robots)))
  {
    return false;
  }
  // gmm_params
  if (!swarm_msgs__msg__GMMParams__copy(
      &(input->gmm_params), &(output->gmm_params)))
  {
    return false;
  }
  // message_type
  if (!rosidl_runtime_c__String__copy(
      &(input->message_type), &(output->message_type)))
  {
    return false;
  }
  return true;
}

swarm_msgs__msg__Gossip *
swarm_msgs__msg__Gossip__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__Gossip * msg = (swarm_msgs__msg__Gossip *)allocator.allocate(sizeof(swarm_msgs__msg__Gossip), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_msgs__msg__Gossip));
  bool success = swarm_msgs__msg__Gossip__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_msgs__msg__Gossip__destroy(swarm_msgs__msg__Gossip * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_msgs__msg__Gossip__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_msgs__msg__Gossip__Sequence__init(swarm_msgs__msg__Gossip__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__Gossip * data = NULL;

  if (size) {
    data = (swarm_msgs__msg__Gossip *)allocator.zero_allocate(size, sizeof(swarm_msgs__msg__Gossip), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_msgs__msg__Gossip__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_msgs__msg__Gossip__fini(&data[i - 1]);
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
swarm_msgs__msg__Gossip__Sequence__fini(swarm_msgs__msg__Gossip__Sequence * array)
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
      swarm_msgs__msg__Gossip__fini(&array->data[i]);
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

swarm_msgs__msg__Gossip__Sequence *
swarm_msgs__msg__Gossip__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__Gossip__Sequence * array = (swarm_msgs__msg__Gossip__Sequence *)allocator.allocate(sizeof(swarm_msgs__msg__Gossip__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_msgs__msg__Gossip__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_msgs__msg__Gossip__Sequence__destroy(swarm_msgs__msg__Gossip__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_msgs__msg__Gossip__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_msgs__msg__Gossip__Sequence__are_equal(const swarm_msgs__msg__Gossip__Sequence * lhs, const swarm_msgs__msg__Gossip__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_msgs__msg__Gossip__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__Gossip__Sequence__copy(
  const swarm_msgs__msg__Gossip__Sequence * input,
  swarm_msgs__msg__Gossip__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_msgs__msg__Gossip);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_msgs__msg__Gossip * data =
      (swarm_msgs__msg__Gossip *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_msgs__msg__Gossip__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_msgs__msg__Gossip__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_msgs__msg__Gossip__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
