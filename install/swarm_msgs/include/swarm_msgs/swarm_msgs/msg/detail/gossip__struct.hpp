// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarm_msgs:msg/Gossip.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GOSSIP__STRUCT_HPP_
#define SWARM_MSGS__MSG__DETAIL__GOSSIP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'robot_state'
// Member 'known_neighbors'
#include "swarm_msgs/msg/detail/robot_state__struct.hpp"
// Member 'gmm_params'
#include "swarm_msgs/msg/detail/gmm_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swarm_msgs__msg__Gossip __attribute__((deprecated))
#else
# define DEPRECATED__swarm_msgs__msg__Gossip __declspec(deprecated)
#endif

namespace swarm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gossip_
{
  using Type = Gossip_<ContainerAllocator>;

  explicit Gossip_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init),
    robot_state(_init),
    gmm_params(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sender = "";
      this->message_type = "";
    }
  }

  explicit Gossip_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sender(_alloc),
    timestamp(_alloc, _init),
    robot_state(_alloc, _init),
    gmm_params(_alloc, _init),
    message_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sender = "";
      this->message_type = "";
    }
  }

  // field types and members
  using _sender_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sender_type sender;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _robot_state_type =
    swarm_msgs::msg::RobotState_<ContainerAllocator>;
  _robot_state_type robot_state;
  using _known_neighbors_type =
    std::vector<swarm_msgs::msg::RobotState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<swarm_msgs::msg::RobotState_<ContainerAllocator>>>;
  _known_neighbors_type known_neighbors;
  using _failed_robots_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _failed_robots_type failed_robots;
  using _gmm_params_type =
    swarm_msgs::msg::GMMParams_<ContainerAllocator>;
  _gmm_params_type gmm_params;
  using _message_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type_type message_type;

  // setters for named parameter idiom
  Type & set__sender(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sender = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__robot_state(
    const swarm_msgs::msg::RobotState_<ContainerAllocator> & _arg)
  {
    this->robot_state = _arg;
    return *this;
  }
  Type & set__known_neighbors(
    const std::vector<swarm_msgs::msg::RobotState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<swarm_msgs::msg::RobotState_<ContainerAllocator>>> & _arg)
  {
    this->known_neighbors = _arg;
    return *this;
  }
  Type & set__failed_robots(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->failed_robots = _arg;
    return *this;
  }
  Type & set__gmm_params(
    const swarm_msgs::msg::GMMParams_<ContainerAllocator> & _arg)
  {
    this->gmm_params = _arg;
    return *this;
  }
  Type & set__message_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarm_msgs::msg::Gossip_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarm_msgs::msg::Gossip_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::Gossip_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::Gossip_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarm_msgs__msg__Gossip
    std::shared_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarm_msgs__msg__Gossip
    std::shared_ptr<swarm_msgs::msg::Gossip_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gossip_ & other) const
  {
    if (this->sender != other.sender) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->robot_state != other.robot_state) {
      return false;
    }
    if (this->known_neighbors != other.known_neighbors) {
      return false;
    }
    if (this->failed_robots != other.failed_robots) {
      return false;
    }
    if (this->gmm_params != other.gmm_params) {
      return false;
    }
    if (this->message_type != other.message_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gossip_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gossip_

// alias to use template instance with default allocator
using Gossip =
  swarm_msgs::msg::Gossip_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__GOSSIP__STRUCT_HPP_
