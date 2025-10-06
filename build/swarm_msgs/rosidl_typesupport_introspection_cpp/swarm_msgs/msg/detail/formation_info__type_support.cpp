// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "swarm_msgs/msg/detail/formation_info__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace swarm_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FormationInfo_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) swarm_msgs::msg::FormationInfo(_init);
}

void FormationInfo_fini_function(void * message_memory)
{
  auto typed_message = static_cast<swarm_msgs::msg::FormationInfo *>(message_memory);
  typed_message->~FormationInfo();
}

size_t size_function__FormationInfo__formation_goal(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__FormationInfo__formation_goal(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__FormationInfo__formation_goal(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__FormationInfo__formation_goal(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__FormationInfo__formation_goal(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__FormationInfo__formation_goal(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__FormationInfo__formation_goal(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FormationInfo_message_member_array[3] = {
  {
    "formation_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::FormationInfo, formation_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "formation_spacing",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::FormationInfo, formation_spacing),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "formation_goal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::FormationInfo, formation_goal),  // bytes offset in struct
    nullptr,  // default value
    size_function__FormationInfo__formation_goal,  // size() function pointer
    get_const_function__FormationInfo__formation_goal,  // get_const(index) function pointer
    get_function__FormationInfo__formation_goal,  // get(index) function pointer
    fetch_function__FormationInfo__formation_goal,  // fetch(index, &value) function pointer
    assign_function__FormationInfo__formation_goal,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FormationInfo_message_members = {
  "swarm_msgs::msg",  // message namespace
  "FormationInfo",  // message name
  3,  // number of fields
  sizeof(swarm_msgs::msg::FormationInfo),
  FormationInfo_message_member_array,  // message members
  FormationInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  FormationInfo_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FormationInfo_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FormationInfo_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace swarm_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<swarm_msgs::msg::FormationInfo>()
{
  return &::swarm_msgs::msg::rosidl_typesupport_introspection_cpp::FormationInfo_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, swarm_msgs, msg, FormationInfo)() {
  return &::swarm_msgs::msg::rosidl_typesupport_introspection_cpp::FormationInfo_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
