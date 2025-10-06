// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarm_msgs:msg/FormationInfo.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__STRUCT_HPP_
#define SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarm_msgs__msg__FormationInfo __attribute__((deprecated))
#else
# define DEPRECATED__swarm_msgs__msg__FormationInfo __declspec(deprecated)
#endif

namespace swarm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FormationInfo_
{
  using Type = FormationInfo_<ContainerAllocator>;

  explicit FormationInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->formation_type = "";
      this->formation_spacing = 0.0;
      std::fill<typename std::array<double, 2>::iterator, double>(this->formation_goal.begin(), this->formation_goal.end(), 0.0);
    }
  }

  explicit FormationInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : formation_type(_alloc),
    formation_goal(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->formation_type = "";
      this->formation_spacing = 0.0;
      std::fill<typename std::array<double, 2>::iterator, double>(this->formation_goal.begin(), this->formation_goal.end(), 0.0);
    }
  }

  // field types and members
  using _formation_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _formation_type_type formation_type;
  using _formation_spacing_type =
    double;
  _formation_spacing_type formation_spacing;
  using _formation_goal_type =
    std::array<double, 2>;
  _formation_goal_type formation_goal;

  // setters for named parameter idiom
  Type & set__formation_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->formation_type = _arg;
    return *this;
  }
  Type & set__formation_spacing(
    const double & _arg)
  {
    this->formation_spacing = _arg;
    return *this;
  }
  Type & set__formation_goal(
    const std::array<double, 2> & _arg)
  {
    this->formation_goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarm_msgs::msg::FormationInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarm_msgs::msg::FormationInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::FormationInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::FormationInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarm_msgs__msg__FormationInfo
    std::shared_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarm_msgs__msg__FormationInfo
    std::shared_ptr<swarm_msgs::msg::FormationInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FormationInfo_ & other) const
  {
    if (this->formation_type != other.formation_type) {
      return false;
    }
    if (this->formation_spacing != other.formation_spacing) {
      return false;
    }
    if (this->formation_goal != other.formation_goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const FormationInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FormationInfo_

// alias to use template instance with default allocator
using FormationInfo =
  swarm_msgs::msg::FormationInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__FORMATION_INFO__STRUCT_HPP_
