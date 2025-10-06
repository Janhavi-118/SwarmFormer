// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "swarm_msgs/msg/detail/gossip__rosidl_typesupport_introspection_c.h"
#include "swarm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "swarm_msgs/msg/detail/gossip__functions.h"
#include "swarm_msgs/msg/detail/gossip__struct.h"


// Include directives for member types
// Member `sender`
// Member `failed_robots`
// Member `message_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `robot_state`
// Member `known_neighbors`
#include "swarm_msgs/msg/robot_state.h"
// Member `robot_state`
// Member `known_neighbors`
#include "swarm_msgs/msg/detail/robot_state__rosidl_typesupport_introspection_c.h"
// Member `gmm_params`
#include "swarm_msgs/msg/gmm_params.h"
// Member `gmm_params`
#include "swarm_msgs/msg/detail/gmm_params__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  swarm_msgs__msg__Gossip__init(message_memory);
}

void swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_fini_function(void * message_memory)
{
  swarm_msgs__msg__Gossip__fini(message_memory);
}

size_t swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__size_function__Gossip__known_neighbors(
  const void * untyped_member)
{
  const swarm_msgs__msg__RobotState__Sequence * member =
    (const swarm_msgs__msg__RobotState__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_const_function__Gossip__known_neighbors(
  const void * untyped_member, size_t index)
{
  const swarm_msgs__msg__RobotState__Sequence * member =
    (const swarm_msgs__msg__RobotState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_function__Gossip__known_neighbors(
  void * untyped_member, size_t index)
{
  swarm_msgs__msg__RobotState__Sequence * member =
    (swarm_msgs__msg__RobotState__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__fetch_function__Gossip__known_neighbors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const swarm_msgs__msg__RobotState * item =
    ((const swarm_msgs__msg__RobotState *)
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_const_function__Gossip__known_neighbors(untyped_member, index));
  swarm_msgs__msg__RobotState * value =
    (swarm_msgs__msg__RobotState *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__assign_function__Gossip__known_neighbors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  swarm_msgs__msg__RobotState * item =
    ((swarm_msgs__msg__RobotState *)
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_function__Gossip__known_neighbors(untyped_member, index));
  const swarm_msgs__msg__RobotState * value =
    (const swarm_msgs__msg__RobotState *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__resize_function__Gossip__known_neighbors(
  void * untyped_member, size_t size)
{
  swarm_msgs__msg__RobotState__Sequence * member =
    (swarm_msgs__msg__RobotState__Sequence *)(untyped_member);
  swarm_msgs__msg__RobotState__Sequence__fini(member);
  return swarm_msgs__msg__RobotState__Sequence__init(member, size);
}

size_t swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__size_function__Gossip__failed_robots(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_const_function__Gossip__failed_robots(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_function__Gossip__failed_robots(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__fetch_function__Gossip__failed_robots(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_const_function__Gossip__failed_robots(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__assign_function__Gossip__failed_robots(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_function__Gossip__failed_robots(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__resize_function__Gossip__failed_robots(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_member_array[7] = {
  {
    "sender",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__Gossip, sender),  // bytes offset in struct
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
    offsetof(swarm_msgs__msg__Gossip, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__Gossip, robot_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "known_neighbors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__Gossip, known_neighbors),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__size_function__Gossip__known_neighbors,  // size() function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_const_function__Gossip__known_neighbors,  // get_const(index) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_function__Gossip__known_neighbors,  // get(index) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__fetch_function__Gossip__known_neighbors,  // fetch(index, &value) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__assign_function__Gossip__known_neighbors,  // assign(index, value) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__resize_function__Gossip__known_neighbors  // resize(index) function pointer
  },
  {
    "failed_robots",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__Gossip, failed_robots),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__size_function__Gossip__failed_robots,  // size() function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_const_function__Gossip__failed_robots,  // get_const(index) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__get_function__Gossip__failed_robots,  // get(index) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__fetch_function__Gossip__failed_robots,  // fetch(index, &value) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__assign_function__Gossip__failed_robots,  // assign(index, value) function pointer
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__resize_function__Gossip__failed_robots  // resize(index) function pointer
  },
  {
    "gmm_params",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__Gossip, gmm_params),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__Gossip, message_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_members = {
  "swarm_msgs__msg",  // message namespace
  "Gossip",  // message name
  7,  // number of fields
  sizeof(swarm_msgs__msg__Gossip),
  swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_member_array,  // message members
  swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_init_function,  // function to initialize message memory (memory has to be allocated)
  swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_type_support_handle = {
  0,
  &swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_swarm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, Gossip)() {
  swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, RobotState)();
  swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, RobotState)();
  swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, GMMParams)();
  if (!swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_type_support_handle.typesupport_identifier) {
    swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &swarm_msgs__msg__Gossip__rosidl_typesupport_introspection_c__Gossip_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
