// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GOSSIP__TRAITS_HPP_
#define SWARM_MSGS__MSG__DETAIL__GOSSIP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarm_msgs/msg/detail/gossip__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'robot_state'
// Member 'known_neighbors'
#include "swarm_msgs/msg/detail/robot_state__traits.hpp"
// Member 'gmm_params'
#include "swarm_msgs/msg/detail/gmm_params__traits.hpp"

namespace swarm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Gossip & msg,
  std::ostream & out)
{
  out << "{";
  // member: sender
  {
    out << "sender: ";
    rosidl_generator_traits::value_to_yaml(msg.sender, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: robot_state
  {
    out << "robot_state: ";
    to_flow_style_yaml(msg.robot_state, out);
    out << ", ";
  }

  // member: known_neighbors
  {
    if (msg.known_neighbors.size() == 0) {
      out << "known_neighbors: []";
    } else {
      out << "known_neighbors: [";
      size_t pending_items = msg.known_neighbors.size();
      for (auto item : msg.known_neighbors) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: failed_robots
  {
    if (msg.failed_robots.size() == 0) {
      out << "failed_robots: []";
    } else {
      out << "failed_robots: [";
      size_t pending_items = msg.failed_robots.size();
      for (auto item : msg.failed_robots) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gmm_params
  {
    out << "gmm_params: ";
    to_flow_style_yaml(msg.gmm_params, out);
    out << ", ";
  }

  // member: message_type
  {
    out << "message_type: ";
    rosidl_generator_traits::value_to_yaml(msg.message_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Gossip & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sender
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sender: ";
    rosidl_generator_traits::value_to_yaml(msg.sender, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: robot_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_state:\n";
    to_block_style_yaml(msg.robot_state, out, indentation + 2);
  }

  // member: known_neighbors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.known_neighbors.size() == 0) {
      out << "known_neighbors: []\n";
    } else {
      out << "known_neighbors:\n";
      for (auto item : msg.known_neighbors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: failed_robots
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.failed_robots.size() == 0) {
      out << "failed_robots: []\n";
    } else {
      out << "failed_robots:\n";
      for (auto item : msg.failed_robots) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gmm_params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gmm_params:\n";
    to_block_style_yaml(msg.gmm_params, out, indentation + 2);
  }

  // member: message_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message_type: ";
    rosidl_generator_traits::value_to_yaml(msg.message_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Gossip & msg, bool use_flow_style = false)
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
  const swarm_msgs::msg::Gossip & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const swarm_msgs::msg::Gossip & msg)
{
  return swarm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<swarm_msgs::msg::Gossip>()
{
  return "swarm_msgs::msg::Gossip";
}

template<>
inline const char * name<swarm_msgs::msg::Gossip>()
{
  return "swarm_msgs/msg/Gossip";
}

template<>
struct has_fixed_size<swarm_msgs::msg::Gossip>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<swarm_msgs::msg::Gossip>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<swarm_msgs::msg::Gossip>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWARM_MSGS__MSG__DETAIL__GOSSIP__TRAITS_HPP_
