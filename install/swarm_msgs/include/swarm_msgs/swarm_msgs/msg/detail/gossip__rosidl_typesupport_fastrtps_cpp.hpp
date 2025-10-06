// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GOSSIP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SWARM_MSGS__MSG__DETAIL__GOSSIP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "swarm_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "swarm_msgs/msg/detail/gossip__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace swarm_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
cdr_serialize(
  const swarm_msgs::msg::Gossip & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  swarm_msgs::msg::Gossip & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
get_serialized_size(
  const swarm_msgs::msg::Gossip & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
max_serialized_size_Gossip(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace swarm_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, swarm_msgs, msg, Gossip)();

#ifdef __cplusplus
}
#endif

#endif  // SWARM_MSGS__MSG__DETAIL__GOSSIP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
