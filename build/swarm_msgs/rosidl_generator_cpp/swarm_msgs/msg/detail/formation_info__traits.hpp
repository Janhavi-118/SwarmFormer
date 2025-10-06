// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__TRAITS_HPP_
#define SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarm_msgs/msg/detail/formation_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace swarm_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FormationInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: formation_type
  {
    out << "formation_type: ";
    rosidl_generator_traits::value_to_yaml(msg.formation_type, out);
    out << ", ";
  }

  // member: formation_spacing
  {
    out << "formation_spacing: ";
    rosidl_generator_traits::value_to_yaml(msg.formation_spacing, out);
    out << ", ";
  }

  // member: formation_goal
  {
    if (msg.formation_goal.size() == 0) {
      out << "formation_goal: []";
    } else {
      out << "formation_goal: [";
      size_t pending_items = msg.formation_goal.size();
      for (auto item : msg.formation_goal) {
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
  const FormationInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: formation_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "formation_type: ";
    rosidl_generator_traits::value_to_yaml(msg.formation_type, out);
    out << "\n";
  }

  // member: formation_spacing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "formation_spacing: ";
    rosidl_generator_traits::value_to_yaml(msg.formation_spacing, out);
    out << "\n";
  }

  // member: formation_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.formation_goal.size() == 0) {
      out << "formation_goal: []\n";
    } else {
      out << "formation_goal:\n";
      for (auto item : msg.formation_goal) {
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

inline std::string to_yaml(const FormationInfo & msg, bool use_flow_style = false)
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
  const swarm_msgs::msg::FormationInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarm_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarm_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const swarm_msgs::msg::FormationInfo & msg)
{
  return swarm_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<swarm_msgs::msg::FormationInfo>()
{
  return "swarm_msgs::msg::FormationInfo";
}

template<>
inline const char * name<swarm_msgs::msg::FormationInfo>()
{
  return "swarm_msgs/msg/FormationInfo";
}

template<>
struct has_fixed_size<swarm_msgs::msg::FormationInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<swarm_msgs::msg::FormationInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<swarm_msgs::msg::FormationInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__TRAITS_HPP_
