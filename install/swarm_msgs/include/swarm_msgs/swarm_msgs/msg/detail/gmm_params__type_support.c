// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from swarm_msgs:msg/GMMParams.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "swarm_msgs/msg/detail/gmm_params__rosidl_typesupport_introspection_c.h"
#include "swarm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "swarm_msgs/msg/detail/gmm_params__functions.h"
#include "swarm_msgs/msg/detail/gmm_params__struct.h"


// Include directives for member types
// Member `means`
// Member `covariances`
// Member `weights`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  swarm_msgs__msg__GMMParams__init(message_memory);
}

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_fini_function(void * message_memory)
{
  swarm_msgs__msg__GMMParams__fini(message_memory);
}

size_t swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__size_function__GMMParams__means(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__means(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__means(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__fetch_function__GMMParams__means(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__means(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__assign_function__GMMParams__means(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__means(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__resize_function__GMMParams__means(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__size_function__GMMParams__covariances(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__covariances(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__covariances(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__fetch_function__GMMParams__covariances(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__covariances(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__assign_function__GMMParams__covariances(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__covariances(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__resize_function__GMMParams__covariances(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__size_function__GMMParams__weights(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__weights(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__weights(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__fetch_function__GMMParams__weights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__weights(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__assign_function__GMMParams__weights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__weights(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__resize_function__GMMParams__weights(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_member_array[3] = {
  {
    "means",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__GMMParams, means),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__size_function__GMMParams__means,  // size() function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__means,  // get_const(index) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__means,  // get(index) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__fetch_function__GMMParams__means,  // fetch(index, &value) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__assign_function__GMMParams__means,  // assign(index, value) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__resize_function__GMMParams__means  // resize(index) function pointer
  },
  {
    "covariances",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__GMMParams, covariances),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__size_function__GMMParams__covariances,  // size() function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__covariances,  // get_const(index) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__covariances,  // get(index) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__fetch_function__GMMParams__covariances,  // fetch(index, &value) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__assign_function__GMMParams__covariances,  // assign(index, value) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__resize_function__GMMParams__covariances  // resize(index) function pointer
  },
  {
    "weights",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__GMMParams, weights),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__size_function__GMMParams__weights,  // size() function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_const_function__GMMParams__weights,  // get_const(index) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__get_function__GMMParams__weights,  // get(index) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__fetch_function__GMMParams__weights,  // fetch(index, &value) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__assign_function__GMMParams__weights,  // assign(index, value) function pointer
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__resize_function__GMMParams__weights  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_members = {
  "swarm_msgs__msg",  // message namespace
  "GMMParams",  // message name
  3,  // number of fields
  sizeof(swarm_msgs__msg__GMMParams),
  swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_member_array,  // message members
  swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_init_function,  // function to initialize message memory (memory has to be allocated)
  swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_type_support_handle = {
  0,
  &swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_swarm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, GMMParams)() {
  if (!swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_type_support_handle.typesupport_identifier) {
    swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &swarm_msgs__msg__GMMParams__rosidl_typesupport_introspection_c__GMMParams_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
