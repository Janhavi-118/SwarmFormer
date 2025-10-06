// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__STRUCT_H_
#define SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'formation_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/FormationInfo in the package swarm_msgs.
typedef struct swarm_msgs__msg__FormationInfo
{
  /// e.g. "line", "circle", "triangle", "diamond"
  rosidl_runtime_c__String formation_type;
  /// spacing between robots (meters)
  double formation_spacing;
  /// world-frame anchor point [x, y]
  double formation_goal[2];
} swarm_msgs__msg__FormationInfo;

// Struct for a sequence of swarm_msgs__msg__FormationInfo.
typedef struct swarm_msgs__msg__FormationInfo__Sequence
{
  swarm_msgs__msg__FormationInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_msgs__msg__FormationInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__STRUCT_H_
