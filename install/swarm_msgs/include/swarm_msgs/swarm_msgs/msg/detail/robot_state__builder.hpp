// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define SWARM_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_heartbeat_count
{
public:
  explicit Init_RobotState_heartbeat_count(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::swarm_msgs::msg::RobotState heartbeat_count(::swarm_msgs::msg::RobotState::_heartbeat_count_type arg)
  {
    msg_.heartbeat_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_error_flags
{
public:
  explicit Init_RobotState_error_flags(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_heartbeat_count error_flags(::swarm_msgs::msg::RobotState::_error_flags_type arg)
  {
    msg_.error_flags = std::move(arg);
    return Init_RobotState_heartbeat_count(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_task_status
{
public:
  explicit Init_RobotState_task_status(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_error_flags task_status(::swarm_msgs::msg::RobotState::_task_status_type arg)
  {
    msg_.task_status = std::move(arg);
    return Init_RobotState_error_flags(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_battery_level
{
public:
  explicit Init_RobotState_battery_level(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_task_status battery_level(::swarm_msgs::msg::RobotState::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_RobotState_task_status(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_local_obstacle_count
{
public:
  explicit Init_RobotState_local_obstacle_count(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_battery_level local_obstacle_count(::swarm_msgs::msg::RobotState::_local_obstacle_count_type arg)
  {
    msg_.local_obstacle_count = std::move(arg);
    return Init_RobotState_battery_level(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_neighbors
{
public:
  explicit Init_RobotState_neighbors(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_local_obstacle_count neighbors(::swarm_msgs::msg::RobotState::_neighbors_type arg)
  {
    msg_.neighbors = std::move(arg);
    return Init_RobotState_local_obstacle_count(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_velocity
{
public:
  explicit Init_RobotState_velocity(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_neighbors velocity(::swarm_msgs::msg::RobotState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_RobotState_neighbors(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_position
{
public:
  explicit Init_RobotState_position(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_velocity position(::swarm_msgs::msg::RobotState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_RobotState_velocity(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_timestamp
{
public:
  explicit Init_RobotState_timestamp(::swarm_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_position timestamp(::swarm_msgs::msg::RobotState::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RobotState_position(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

class Init_RobotState_robot_id
{
public:
  Init_RobotState_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_timestamp robot_id(::swarm_msgs::msg::RobotState::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotState_timestamp(msg_);
  }

private:
  ::swarm_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_msgs::msg::RobotState>()
{
  return swarm_msgs::msg::builder::Init_RobotState_robot_id();
}

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
