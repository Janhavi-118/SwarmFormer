// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarm_msgs:msg/GMMParams.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__STRUCT_H_
#define SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'means'
// Member 'covariances'
// Member 'weights'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/GMMParams in the package swarm_msgs.
/**
  * Gaussian Mixture Model Parameters
 */
typedef struct swarm_msgs__msg__GMMParams
{
  /// Flattened list of means, e.g. [x1, y1, x2, y2, ...]
  rosidl_runtime_c__double__Sequence means;
  /// Flattened covariances, e.g. for K components: [c11, c12, c21, c22, ... repeated K times]
  rosidl_runtime_c__double__Sequence covariances;
  /// Weights per component
  rosidl_runtime_c__double__Sequence weights;
} swarm_msgs__msg__GMMParams;

// Struct for a sequence of swarm_msgs__msg__GMMParams.
typedef struct swarm_msgs__msg__GMMParams__Sequence
{
  swarm_msgs__msg__GMMParams * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarm_msgs__msg__GMMParams__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__STRUCT_H_
