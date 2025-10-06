// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__BUILDER_HPP_
#define SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_msgs/msg/detail/formation_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_msgs
{

namespace msg
{

namespace builder
{

class Init_FormationInfo_formation_goal
{
public:
  explicit Init_FormationInfo_formation_goal(::swarm_msgs::msg::FormationInfo & msg)
  : msg_(msg)
  {}
  ::swarm_msgs::msg::FormationInfo formation_goal(::swarm_msgs::msg::FormationInfo::_formation_goal_type arg)
  {
    msg_.formation_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_msgs::msg::FormationInfo msg_;
};

class Init_FormationInfo_formation_spacing
{
public:
  explicit Init_FormationInfo_formation_spacing(::swarm_msgs::msg::FormationInfo & msg)
  : msg_(msg)
  {}
  Init_FormationInfo_formation_goal formation_spacing(::swarm_msgs::msg::FormationInfo::_formation_spacing_type arg)
  {
    msg_.formation_spacing = std::move(arg);
    return Init_FormationInfo_formation_goal(msg_);
  }

private:
  ::swarm_msgs::msg::FormationInfo msg_;
};

class Init_FormationInfo_formation_type
{
public:
  Init_FormationInfo_formation_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FormationInfo_formation_spacing formation_type(::swarm_msgs::msg::FormationInfo::_formation_type_type arg)
  {
    msg_.formation_type = std::move(arg);
    return Init_FormationInfo_formation_spacing(msg_);
  }

private:
  ::swarm_msgs::msg::FormationInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_msgs::msg::FormationInfo>()
{
  return swarm_msgs::msg::builder::Init_FormationInfo_formation_type();
}

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__BUILDER_HPP_
