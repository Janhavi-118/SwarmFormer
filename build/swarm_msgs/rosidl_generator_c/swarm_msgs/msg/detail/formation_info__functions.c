// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/formation_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `formation_type`
#include "rosidl_runtime_c/string_functions.h"

bool
swarm_msgs__msg__FormationInfo__init(swarm_msgs__msg__FormationInfo * msg)
{
  if (!msg) {
    return false;
  }
  // formation_type
  if (!rosidl_runtime_c__String__init(&msg->formation_type)) {
    swarm_msgs__msg__FormationInfo__fini(msg);
    return false;
  }
  // formation_spacing
  // formation_goal
  return true;
}

void
swarm_msgs__msg__FormationInfo__fini(swarm_msgs__msg__FormationInfo * msg)
{
  if (!msg) {
    return;
  }
  // formation_type
  rosidl_runtime_c__String__fini(&msg->formation_type);
  // formation_spacing
  // formation_goal
}

bool
swarm_msgs__msg__FormationInfo__are_equal(const swarm_msgs__msg__FormationInfo * lhs, const swarm_msgs__msg__FormationInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // formation_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->formation_type), &(rhs->formation_type)))
  {
    return false;
  }
  // formation_spacing
  if (lhs->formation_spacing != rhs->formation_spacing) {
    return false;
  }
  // formation_goal
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->formation_goal[i] != rhs->formation_goal[i]) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__FormationInfo__copy(
  const swarm_msgs__msg__FormationInfo * input,
  swarm_msgs__msg__FormationInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // formation_type
  if (!rosidl_runtime_c__String__copy(
      &(input->formation_type), &(output->formation_type)))
  {
    return false;
  }
  // formation_spacing
  output->formation_spacing = input->formation_spacing;
  // formation_goal
  for (size_t i = 0; i < 2; ++i) {
    output->formation_goal[i] = input->formation_goal[i];
  }
  return true;
}

swarm_msgs__msg__FormationInfo *
swarm_msgs__msg__FormationInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__FormationInfo * msg = (swarm_msgs__msg__FormationInfo *)allocator.allocate(sizeof(swarm_msgs__msg__FormationInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_msgs__msg__FormationInfo));
  bool success = swarm_msgs__msg__FormationInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_msgs__msg__FormationInfo__destroy(swarm_msgs__msg__FormationInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_msgs__msg__FormationInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_msgs__msg__FormationInfo__Sequence__init(swarm_msgs__msg__FormationInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__FormationInfo * data = NULL;

  if (size) {
    data = (swarm_msgs__msg__FormationInfo *)allocator.zero_allocate(size, sizeof(swarm_msgs__msg__FormationInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_msgs__msg__FormationInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_msgs__msg__FormationInfo__fini(&data[i - 1]);
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
swarm_msgs__msg__FormationInfo__Sequence__fini(swarm_msgs__msg__FormationInfo__Sequence * array)
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
      swarm_msgs__msg__FormationInfo__fini(&array->data[i]);
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

swarm_msgs__msg__FormationInfo__Sequence *
swarm_msgs__msg__FormationInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__FormationInfo__Sequence * array = (swarm_msgs__msg__FormationInfo__Sequence *)allocator.allocate(sizeof(swarm_msgs__msg__FormationInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_msgs__msg__FormationInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_msgs__msg__FormationInfo__Sequence__destroy(swarm_msgs__msg__FormationInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_msgs__msg__FormationInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_msgs__msg__FormationInfo__Sequence__are_equal(const swarm_msgs__msg__FormationInfo__Sequence * lhs, const swarm_msgs__msg__FormationInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_msgs__msg__FormationInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__FormationInfo__Sequence__copy(
  const swarm_msgs__msg__FormationInfo__Sequence * input,
  swarm_msgs__msg__FormationInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_msgs__msg__FormationInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_msgs__msg__FormationInfo * data =
      (swarm_msgs__msg__FormationInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_msgs__msg__FormationInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_msgs__msg__FormationInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_msgs__msg__FormationInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
