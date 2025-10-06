// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice
#include "swarm_msgs/msg/detail/formation_info__rosidl_typesupport_fastrtps_cpp.hpp"
#include "swarm_msgs/msg/detail/formation_info__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace swarm_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
cdr_serialize(
  const swarm_msgs::msg::FormationInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: formation_type
  cdr << ros_message.formation_type;
  // Member: formation_spacing
  cdr << ros_message.formation_spacing;
  // Member: formation_goal
  {
    cdr << ros_message.formation_goal;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  swarm_msgs::msg::FormationInfo & ros_message)
{
  // Member: formation_type
  cdr >> ros_message.formation_type;

  // Member: formation_spacing
  cdr >> ros_message.formation_spacing;

  // Member: formation_goal
  {
    cdr >> ros_message.formation_goal;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
get_serialized_size(
  const swarm_msgs::msg::FormationInfo & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: formation_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.formation_type.size() + 1);
  // Member: formation_spacing
  {
    size_t item_size = sizeof(ros_message.formation_spacing);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: formation_goal
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.formation_goal[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_swarm_msgs
max_serialized_size_FormationInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: formation_type
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: formation_spacing
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: formation_goal
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = swarm_msgs::msg::FormationInfo;
    is_plain =
      (
      offsetof(DataType, formation_goal) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _FormationInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const swarm_msgs::msg::FormationInfo *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FormationInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<swarm_msgs::msg::FormationInfo *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FormationInfo__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const swarm_msgs::msg::FormationInfo *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FormationInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FormationInfo(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FormationInfo__callbacks = {
  "swarm_msgs::msg",
  "FormationInfo",
  _FormationInfo__cdr_serialize,
  _FormationInfo__cdr_deserialize,
  _FormationInfo__get_serialized_size,
  _FormationInfo__max_serialized_size
};

static rosidl_message_type_support_t _FormationInfo__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FormationInfo__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace swarm_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_swarm_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<swarm_msgs::msg::FormationInfo>()
{
  return &swarm_msgs::msg::typesupport_fastrtps_cpp::_FormationInfo__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, swarm_msgs, msg, FormationInfo)() {
  return &swarm_msgs::msg::typesupport_fastrtps_cpp::_FormationInfo__handle;
}

#ifdef __cplusplus
}
#endif
