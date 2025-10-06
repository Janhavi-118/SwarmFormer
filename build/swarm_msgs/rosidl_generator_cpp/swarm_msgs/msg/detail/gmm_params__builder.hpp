// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarm_msgs:msg/GMMParams.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__BUILDER_HPP_
#define SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarm_msgs/msg/detail/gmm_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarm_msgs
{

namespace msg
{

namespace builder
{

class Init_GMMParams_weights
{
public:
  explicit Init_GMMParams_weights(::swarm_msgs::msg::GMMParams & msg)
  : msg_(msg)
  {}
  ::swarm_msgs::msg::GMMParams weights(::swarm_msgs::msg::GMMParams::_weights_type arg)
  {
    msg_.weights = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarm_msgs::msg::GMMParams msg_;
};

class Init_GMMParams_covariances
{
public:
  explicit Init_GMMParams_covariances(::swarm_msgs::msg::GMMParams & msg)
  : msg_(msg)
  {}
  Init_GMMParams_weights covariances(::swarm_msgs::msg::GMMParams::_covariances_type arg)
  {
    msg_.covariances = std::move(arg);
    return Init_GMMParams_weights(msg_);
  }

private:
  ::swarm_msgs::msg::GMMParams msg_;
};

class Init_GMMParams_means
{
public:
  Init_GMMParams_means()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GMMParams_covariances means(::swarm_msgs::msg::GMMParams::_means_type arg)
  {
    msg_.means = std::move(arg);
    return Init_GMMParams_covariances(msg_);
  }

private:
  ::swarm_msgs::msg::GMMParams msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarm_msgs::msg::GMMParams>()
{
  return swarm_msgs::msg::builder::Init_GMMParams_means();
}

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__BUILDER_HPP_
