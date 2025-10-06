// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define SWARM_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_id'
// Member 'neighbors'
// Member 'task_status'
// Member 'error_flags'
#include "rosidl_runtime_c/string.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/RobotState in the package swarm_msgs.
typedef struct swarm_msgs__msg__RobotState
{
  rosidl_runtime_c__String robot_id;
  builtin_interfaces__msg__Time timestamp;
  /// x, y, theta
  double position[3];
  /// vx, vy, angular_z
  double velocity[3];
  rosidl_runtime_c__String__Sequence neighbors;
  int32_t local_obstacle_count;
  double battery_level;
  rosidl_runtime_c__String task_status;
  rosidl_runtime_c__String__Sequence error_flags;
  int32_t heartbeat_count;
} swarm_msgs__msg__RobotState;

// Struct for a sequence of swarm_msgs__msg__RobotState.
typedef struct swarm_msgs__msg__RobotState__Sequence
{
  swarm_msgs__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_msgs__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
