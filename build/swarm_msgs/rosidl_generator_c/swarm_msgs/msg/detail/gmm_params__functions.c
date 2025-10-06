// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarm_msgs:msg/GMMParams.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/gmm_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `means`
// Member `covariances`
// Member `weights`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
swarm_msgs__msg__GMMParams__init(swarm_msgs__msg__GMMParams * msg)
{
  if (!msg) {
    return false;
  }
  // means
  if (!rosidl_runtime_c__double__Sequence__init(&msg->means, 0)) {
    swarm_msgs__msg__GMMParams__fini(msg);
    return false;
  }
  // covariances
  if (!rosidl_runtime_c__double__Sequence__init(&msg->covariances, 0)) {
    swarm_msgs__msg__GMMParams__fini(msg);
    return false;
  }
  // weights
  if (!rosidl_runtime_c__double__Sequence__init(&msg->weights, 0)) {
    swarm_msgs__msg__GMMParams__fini(msg);
    return false;
  }
  return true;
}

void
swarm_msgs__msg__GMMParams__fini(swarm_msgs__msg__GMMParams * msg)
{
  if (!msg) {
    return;
  }
  // means
  rosidl_runtime_c__double__Sequence__fini(&msg->means);
  // covariances
  rosidl_runtime_c__double__Sequence__fini(&msg->covariances);
  // weights
  rosidl_runtime_c__double__Sequence__fini(&msg->weights);
}

bool
swarm_msgs__msg__GMMParams__are_equal(const swarm_msgs__msg__GMMParams * lhs, const swarm_msgs__msg__GMMParams * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // means
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->means), &(rhs->means)))
  {
    return false;
  }
  // covariances
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->covariances), &(rhs->covariances)))
  {
    return false;
  }
  // weights
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->weights), &(rhs->weights)))
  {
    return false;
  }
  return true;
}

bool
swarm_msgs__msg__GMMParams__copy(
  const swarm_msgs__msg__GMMParams * input,
  swarm_msgs__msg__GMMParams * output)
{
  if (!input || !output) {
    return false;
  }
  // means
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->means), &(output->means)))
  {
    return false;
  }
  // covariances
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->covariances), &(output->covariances)))
  {
    return false;
  }
  // weights
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->weights), &(output->weights)))
  {
    return false;
  }
  return true;
}

swarm_msgs__msg__GMMParams *
swarm_msgs__msg__GMMParams__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__GMMParams * msg = (swarm_msgs__msg__GMMParams *)allocator.allocate(sizeof(swarm_msgs__msg__GMMParams), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarm_msgs__msg__GMMParams));
  bool success = swarm_msgs__msg__GMMParams__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarm_msgs__msg__GMMParams__destroy(swarm_msgs__msg__GMMParams * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarm_msgs__msg__GMMParams__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarm_msgs__msg__GMMParams__Sequence__init(swarm_msgs__msg__GMMParams__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__GMMParams * data = NULL;

  if (size) {
    data = (swarm_msgs__msg__GMMParams *)allocator.zero_allocate(size, sizeof(swarm_msgs__msg__GMMParams), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarm_msgs__msg__GMMParams__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarm_msgs__msg__GMMParams__fini(&data[i - 1]);
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
swarm_msgs__msg__GMMParams__Sequence__fini(swarm_msgs__msg__GMMParams__Sequence * array)
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
      swarm_msgs__msg__GMMParams__fini(&array->data[i]);
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

swarm_msgs__msg__GMMParams__Sequence *
swarm_msgs__msg__GMMParams__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarm_msgs__msg__GMMParams__Sequence * array = (swarm_msgs__msg__GMMParams__Sequence *)allocator.allocate(sizeof(swarm_msgs__msg__GMMParams__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarm_msgs__msg__GMMParams__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarm_msgs__msg__GMMParams__Sequence__destroy(swarm_msgs__msg__GMMParams__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarm_msgs__msg__GMMParams__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarm_msgs__msg__GMMParams__Sequence__are_equal(const swarm_msgs__msg__GMMParams__Sequence * lhs, const swarm_msgs__msg__GMMParams__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarm_msgs__msg__GMMParams__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarm_msgs__msg__GMMParams__Sequence__copy(
  const swarm_msgs__msg__GMMParams__Sequence * input,
  swarm_msgs__msg__GMMParams__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarm_msgs__msg__GMMParams);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarm_msgs__msg__GMMParams * data =
      (swarm_msgs__msg__GMMParams *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarm_msgs__msg__GMMParams__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarm_msgs__msg__GMMParams__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarm_msgs__msg__GMMParams__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
