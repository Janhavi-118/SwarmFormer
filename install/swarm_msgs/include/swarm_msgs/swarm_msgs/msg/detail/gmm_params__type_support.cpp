// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from swarm_msgs:msg/GMMParams.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "swarm_msgs/msg/detail/gmm_params__struct.hpp"
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

void GMMParams_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) swarm_msgs::msg::GMMParams(_init);
}

void GMMParams_fini_function(void * message_memory)
{
  auto typed_message = static_cast<swarm_msgs::msg::GMMParams *>(message_memory);
  typed_message->~GMMParams();
}

size_t size_function__GMMParams__means(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GMMParams__means(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GMMParams__means(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__GMMParams__means(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GMMParams__means(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GMMParams__means(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GMMParams__means(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__GMMParams__means(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GMMParams__covariances(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GMMParams__covariances(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GMMParams__covariances(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__GMMParams__covariances(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GMMParams__covariances(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GMMParams__covariances(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GMMParams__covariances(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__GMMParams__covariances(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GMMParams__weights(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GMMParams__weights(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__GMMParams__weights(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__GMMParams__weights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GMMParams__weights(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GMMParams__weights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GMMParams__weights(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__GMMParams__weights(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GMMParams_message_member_array[3] = {
  {
    "means",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::GMMParams, means),  // bytes offset in struct
    nullptr,  // default value
    size_function__GMMParams__means,  // size() function pointer
    get_const_function__GMMParams__means,  // get_const(index) function pointer
    get_function__GMMParams__means,  // get(index) function pointer
    fetch_function__GMMParams__means,  // fetch(index, &value) function pointer
    assign_function__GMMParams__means,  // assign(index, value) function pointer
    resize_function__GMMParams__means  // resize(index) function pointer
  },
  {
    "covariances",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::GMMParams, covariances),  // bytes offset in struct
    nullptr,  // default value
    size_function__GMMParams__covariances,  // size() function pointer
    get_const_function__GMMParams__covariances,  // get_const(index) function pointer
    get_function__GMMParams__covariances,  // get(index) function pointer
    fetch_function__GMMParams__covariances,  // fetch(index, &value) function pointer
    assign_function__GMMParams__covariances,  // assign(index, value) function pointer
    resize_function__GMMParams__covariances  // resize(index) function pointer
  },
  {
    "weights",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarm_msgs::msg::GMMParams, weights),  // bytes offset in struct
    nullptr,  // default value
    size_function__GMMParams__weights,  // size() function pointer
    get_const_function__GMMParams__weights,  // get_const(index) function pointer
    get_function__GMMParams__weights,  // get(index) function pointer
    fetch_function__GMMParams__weights,  // fetch(index, &value) function pointer
    assign_function__GMMParams__weights,  // assign(index, value) function pointer
    resize_function__GMMParams__weights  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GMMParams_message_members = {
  "swarm_msgs::msg",  // message namespace
  "GMMParams",  // message name
  3,  // number of fields
  sizeof(swarm_msgs::msg::GMMParams),
  GMMParams_message_member_array,  // message members
  GMMParams_init_function,  // function to initialize message memory (memory has to be allocated)
  GMMParams_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GMMParams_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GMMParams_message_members,
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
get_message_type_support_handle<swarm_msgs::msg::GMMParams>()
{
  return &::swarm_msgs::msg::rosidl_typesupport_introspection_cpp::GMMParams_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, swarm_msgs, msg, GMMParams)() {
  return &::swarm_msgs::msg::rosidl_typesupport_introspection_cpp::GMMParams_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
