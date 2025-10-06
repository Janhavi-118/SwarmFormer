// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "swarm_msgs/msg/detail/gossip__struct.hpp"
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

void Gossip_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) swarm_msgs::msg::Gossip(_init);
}

void Gossip_fini_function(void * message_memory)
{
  auto typed_message = static_cast<swarm_msgs::msg::Gossip *>(message_memory);
  typed_message->~Gossip();
}

size_t size_function__Gossip__known_neighbors(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<swarm_msgs::msg::RobotState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Gossip__known_neighbors(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<swarm_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void * get_function__Gossip__known_neighbors(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<swarm_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void fetch_function__Gossip__known_neighbors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const swarm_msgs::msg::RobotState *>(
    get_const_function__Gossip__known_neighbors(untyped_member, index));
  auto & value = *reinterpret_cast<swarm_msgs::msg::RobotState *>(untyped_value);
  value = item;
}

void assign_function__Gossip__known_neighbors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<swarm_msgs::msg::RobotState *>(
    get_function__Gossip__known_neighbors(untyped_member, index));
  const auto & value = *reinterpret_cast<const swarm_msgs::msg::RobotState *>(untyped_value);
  item = value;
}

void resize_function__Gossip__known_neighbors(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<swarm_msgs::msg::RobotState> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Gossip__failed_robots(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Gossip__failed_robots(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__Gossip__failed_robots(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__Gossip__failed_robots(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__Gossip__failed_robots(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__Gossip__failed_robots(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__Gossip__failed_robots(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__Gossip__failed_robots(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Gossip_message_member_array[7] = {
  {
    "sender",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::Gossip, sender),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::Gossip, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<swarm_msgs::msg::RobotState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::Gossip, robot_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "known_neighbors",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<swarm_msgs::msg::RobotState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::Gossip, known_neighbors),  // bytes offset in struct
    nullptr,  // default value
    size_function__Gossip__known_neighbors,  // size() function pointer
    get_const_function__Gossip__known_neighbors,  // get_const(index) function pointer
    get_function__Gossip__known_neighbors,  // get(index) function pointer
    fetch_function__Gossip__known_neighbors,  // fetch(index, &value) function pointer
    assign_function__Gossip__known_neighbors,  // assign(index, value) function pointer
    resize_function__Gossip__known_neighbors  // resize(index) function pointer
  },
  {
    "failed_robots",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::Gossip, failed_robots),  // bytes offset in struct
    nullptr,  // default value
    size_function__Gossip__failed_robots,  // size() function pointer
    get_const_function__Gossip__failed_robots,  // get_const(index) function pointer
    get_function__Gossip__failed_robots,  // get(index) function pointer
    fetch_function__Gossip__failed_robots,  // fetch(index, &value) function pointer
    assign_function__Gossip__failed_robots,  // assign(index, value) function pointer
    resize_function__Gossip__failed_robots  // resize(index) function pointer
  },
  {
    "gmm_params",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<swarm_msgs::msg::GMMParams>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::Gossip, gmm_params),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::Gossip, message_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Gossip_message_members = {
  "swarm_msgs::msg",  // message namespace
  "Gossip",  // message name
  7,  // number of fields
  sizeof(swarm_msgs::msg::Gossip),
  Gossip_message_member_array,  // message members
  Gossip_init_function,  // function to initialize message memory (memory has to be allocated)
  Gossip_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Gossip_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Gossip_message_members,
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
get_message_type_support_handle<swarm_msgs::msg::Gossip>()
{
  return &::swarm_msgs::msg::rosidl_typesupport_introspection_cpp::Gossip_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, swarm_msgs, msg, Gossip)() {
  return &::swarm_msgs::msg::rosidl_typesupport_introspection_cpp::Gossip_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
