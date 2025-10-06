// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "swarm_msgs/msg/detail/formation_info__rosidl_typesupport_introspection_c.h"
#include "swarm_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "swarm_msgs/msg/detail/formation_info__functions.h"
#include "swarm_msgs/msg/detail/formation_info__struct.h"


// Include directives for member types
// Member `formation_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  swarm_msgs__msg__FormationInfo__init(message_memory);
}

void swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_fini_function(void * message_memory)
{
  swarm_msgs__msg__FormationInfo__fini(message_memory);
}

size_t swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__size_function__FormationInfo__formation_goal(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__get_const_function__FormationInfo__formation_goal(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__get_function__FormationInfo__formation_goal(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__fetch_function__FormationInfo__formation_goal(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__get_const_function__FormationInfo__formation_goal(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__assign_function__FormationInfo__formation_goal(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__get_function__FormationInfo__formation_goal(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_member_array[3] = {
  {
    "formation_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__FormationInfo, formation_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "formation_spacing",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__FormationInfo, formation_spacing),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "formation_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs__msg__FormationInfo, formation_goal),  // bytes offset in struct
    NULL,  // default value
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__size_function__FormationInfo__formation_goal,  // size() function pointer
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__get_const_function__FormationInfo__formation_goal,  // get_const(index) function pointer
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__get_function__FormationInfo__formation_goal,  // get(index) function pointer
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__fetch_function__FormationInfo__formation_goal,  // fetch(index, &value) function pointer
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__assign_function__FormationInfo__formation_goal,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_members = {
  "swarm_msgs__msg",  // message namespace
  "FormationInfo",  // message name
  3,  // number of fields
  sizeof(swarm_msgs__msg__FormationInfo),
  swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_member_array,  // message members
  swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_type_support_handle = {
  0,
  &swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_swarm_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarm_msgs, msg, FormationInfo)() {
  if (!swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_type_support_handle.typesupport_identifier) {
    swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &swarm_msgs__msg__FormationInfo__rosidl_typesupport_introspection_c__FormationInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
