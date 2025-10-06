// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from swarm_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "swarm_msgs/msg/detail/robot_state__rosidl_typesupport_introspection_c.h"
#include "swarm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "swarm_msgs/msg/detail/robot_state__functions.h"
#include "swarm_msgs/msg/detail/robot_state__struct.h"


// Include directives for member types
// Member `robot_id`
// Member `neighbors`
// Member `task_status`
// Member `error_flags`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  swarm_msgs__msg__RobotState__init(message_memory);
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_fini_function(void * message_memory)
{
  swarm_msgs__msg__RobotState__fini(message_memory);
}

size_t swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__neighbors(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__neighbors(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__neighbors(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__neighbors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__neighbors(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__neighbors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__neighbors(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__resize_function__RobotState__neighbors(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__error_flags(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__error_flags(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__error_flags(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__error_flags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__error_flags(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__error_flags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__error_flags(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__resize_function__RobotState__error_flags(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_member_array[10] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, position),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__position,  // size() function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__position,  // get_const(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__position,  // get(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__position,  // fetch(index, &value) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, velocity),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__velocity,  // size() function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__velocity,  // get_const(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__velocity,  // get(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__velocity,  // fetch(index, &value) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "neighbors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, neighbors),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__neighbors,  // size() function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__neighbors,  // get_const(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__neighbors,  // get(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__neighbors,  // fetch(index, &value) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__neighbors,  // assign(index, value) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__resize_function__RobotState__neighbors  // resize(index) function pointer
  },
  {
    "local_obstacle_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, local_obstacle_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_level",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, battery_level),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, task_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, error_flags),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__size_function__RobotState__error_flags,  // size() function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_const_function__RobotState__error_flags,  // get_const(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__get_function__RobotState__error_flags,  // get(index) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__fetch_function__RobotState__error_flags,  // fetch(index, &value) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__assign_function__RobotState__error_flags,  // assign(index, value) function pointer
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__resize_function__RobotState__error_flags  // resize(index) function pointer
  },
  {
    "heartbeat_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__RobotState, heartbeat_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_members = {
  "swarm_msgs__msg",  // message namespace
  "RobotState",  // message name
  10,  // number of fields
  sizeof(swarm_msgs__msg__RobotState),
  swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_member_array,  // message members
  swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_init_function,  // function to initialize message memory (memory has to be allocated)
  swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle = {
  0,
  &swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_swarm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, RobotState)() {
  swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle.typesupport_identifier) {
    swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &swarm_msgs__msg__RobotState__rosidl_typesupport_introspection_c__RobotState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
