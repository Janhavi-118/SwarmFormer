// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GOSSIP__STRUCT_H_
#define SWARM_MSGS__MSG__DETAIL__GOSSIP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sender'
// Member 'failed_robots'
// Member 'message_type'
#include "rosidl_runtime_c/string.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'robot_state'
// Member 'known_neighbors'
#include "swarm_msgs/msg/detail/robot_state__struct.h"
// Member 'gmm_params'
#include "swarm_msgs/msg/detail/gmm_params__struct.h"

/// Struct defined in msg/Gossip in the package swarm_msgs.
typedef struct swarm_msgs__msg__Gossip
{
  rosidl_runtime_c__String sender;
  builtin_interfaces__msg__Time timestamp;
  swarm_msgs__msg__RobotState robot_state;
  swarm_msgs__msg__RobotState__Sequence known_neighbors;
  rosidl_runtime_c__String__Sequence failed_robots;
  swarm_msgs__msg__GMMParams gmm_params;
  rosidl_runtime_c__String message_type;
} swarm_msgs__msg__Gossip;

// Struct for a sequence of swarm_msgs__msg__Gossip.
typedef struct swarm_msgs__msg__Gossip__Sequence
{
  swarm_msgs__msg__Gossip * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_msgs__msg__Gossip__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__GOSSIP__STRUCT_H_
