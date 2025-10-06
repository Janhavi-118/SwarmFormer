// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GOSSIP__BUILDER_HPP_
#define SWARM_MSGS__MSG__DETAIL__GOSSIP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_msgs/msg/detail/gossip__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_msgs
{

namespace msg
{

namespace builder
{

class Init_Gossip_message_type
{
public:
  explicit Init_Gossip_message_type(::swarm_msgs::msg::Gossip & msg)
  : msg_(msg)
  {}
  ::swarm_msgs::msg::Gossip message_type(::swarm_msgs::msg::Gossip::_message_type_type arg)
  {
    msg_.message_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_msgs::msg::Gossip msg_;
};

class Init_Gossip_gmm_params
{
public:
  explicit Init_Gossip_gmm_params(::swarm_msgs::msg::Gossip & msg)
  : msg_(msg)
  {}
  Init_Gossip_message_type gmm_params(::swarm_msgs::msg::Gossip::_gmm_params_type arg)
  {
    msg_.gmm_params = std::move(arg);
    return Init_Gossip_message_type(msg_);
  }

private:
  ::swarm_msgs::msg::Gossip msg_;
};

class Init_Gossip_failed_robots
{
public:
  explicit Init_Gossip_failed_robots(::swarm_msgs::msg::Gossip & msg)
  : msg_(msg)
  {}
  Init_Gossip_gmm_params failed_robots(::swarm_msgs::msg::Gossip::_failed_robots_type arg)
  {
    msg_.failed_robots = std::move(arg);
    return Init_Gossip_gmm_params(msg_);
  }

private:
  ::swarm_msgs::msg::Gossip msg_;
};

class Init_Gossip_known_neighbors
{
public:
  explicit Init_Gossip_known_neighbors(::swarm_msgs::msg::Gossip & msg)
  : msg_(msg)
  {}
  Init_Gossip_failed_robots known_neighbors(::swarm_msgs::msg::Gossip::_known_neighbors_type arg)
  {
    msg_.known_neighbors = std::move(arg);
    return Init_Gossip_failed_robots(msg_);
  }

private:
  ::swarm_msgs::msg::Gossip msg_;
};

class Init_Gossip_robot_state
{
public:
  explicit Init_Gossip_robot_state(::swarm_msgs::msg::Gossip & msg)
  : msg_(msg)
  {}
  Init_Gossip_known_neighbors robot_state(::swarm_msgs::msg::Gossip::_robot_state_type arg)
  {
    msg_.robot_state = std::move(arg);
    return Init_Gossip_known_neighbors(msg_);
  }

private:
  ::swarm_msgs::msg::Gossip msg_;
};

class Init_Gossip_timestamp
{
public:
  explicit Init_Gossip_timestamp(::swarm_msgs::msg::Gossip & msg)
  : msg_(msg)
  {}
  Init_Gossip_robot_state timestamp(::swarm_msgs::msg::Gossip::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Gossip_robot_state(msg_);
  }

private:
  ::swarm_msgs::msg::Gossip msg_;
};

class Init_Gossip_sender
{
public:
  Init_Gossip_sender()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gossip_timestamp sender(::swarm_msgs::msg::Gossip::_sender_type arg)
  {
    msg_.sender = std::move(arg);
    return Init_Gossip_timestamp(msg_);
  }

private:
  ::swarm_msgs::msg::Gossip msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_msgs::msg::Gossip>()
{
  return swarm_msgs::msg::builder::Init_Gossip_sender();
}

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__GOSSIP__BUILDER_HPP_
