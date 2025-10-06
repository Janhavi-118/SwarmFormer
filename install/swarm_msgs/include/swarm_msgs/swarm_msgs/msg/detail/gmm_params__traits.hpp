// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarm_msgs:msg/GMMParams.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__TRAITS_HPP_
#define SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarm_msgs/msg/detail/gmm_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace swarm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GMMParams & msg,
  std::ostream & out)
{
  out << "{";
  // member: means
  {
    if (msg.means.size() == 0) {
      out << "means: []";
    } else {
      out << "means: [";
      size_t pending_items = msg.means.size();
      for (auto item : msg.means) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: covariances
  {
    if (msg.covariances.size() == 0) {
      out << "covariances: []";
    } else {
      out << "covariances: [";
      size_t pending_items = msg.covariances.size();
      for (auto item : msg.covariances) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: weights
  {
    if (msg.weights.size() == 0) {
      out << "weights: []";
    } else {
      out << "weights: [";
      size_t pending_items = msg.weights.size();
      for (auto item : msg.weights) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GMMParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: means
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.means.size() == 0) {
      out << "means: []\n";
    } else {
      out << "means:\n";
      for (auto item : msg.means) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: covariances
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.covariances.size() == 0) {
      out << "covariances: []\n";
    } else {
      out << "covariances:\n";
      for (auto item : msg.covariances) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: weights
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.weights.size() == 0) {
      out << "weights: []\n";
    } else {
      out << "weights:\n";
      for (auto item : msg.weights) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GMMParams & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace swarm_msgs

namespace rosidl_generator_traits
{

[[deprecated("use swarm_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swarm_msgs::msg::GMMParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const swarm_msgs::msg::GMMParams & msg)
{
  return swarm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<swarm_msgs::msg::GMMParams>()
{
  return "swarm_msgs::msg::GMMParams";
}

template<>
inline const char * name<swarm_msgs::msg::GMMParams>()
{
  return "swarm_msgs/msg/GMMParams";
}

template<>
struct has_fixed_size<swarm_msgs::msg::GMMParams>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<swarm_msgs::msg::GMMParams>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<swarm_msgs::msg::GMMParams>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__TRAITS_HPP_
