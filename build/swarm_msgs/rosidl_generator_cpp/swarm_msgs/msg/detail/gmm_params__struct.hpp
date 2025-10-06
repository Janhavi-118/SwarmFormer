// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarm_msgs:msg/GMMParams.idl
// generated code does not contain a copyright notice

#ifndef SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__STRUCT_HPP_
#define SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarm_msgs__msg__GMMParams __attribute__((deprecated))
#else
# define DEPRECATED__swarm_msgs__msg__GMMParams __declspec(deprecated)
#endif

namespace swarm_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GMMParams_
{
  using Type = GMMParams_<ContainerAllocator>;

  explicit GMMParams_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GMMParams_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _means_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _means_type means;
  using _covariances_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _covariances_type covariances;
  using _weights_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _weights_type weights;

  // setters for named parameter idiom
  Type & set__means(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->means = _arg;
    return *this;
  }
  Type & set__covariances(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->covariances = _arg;
    return *this;
  }
  Type & set__weights(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->weights = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarm_msgs::msg::GMMParams_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarm_msgs::msg::GMMParams_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::GMMParams_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarm_msgs::msg::GMMParams_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarm_msgs__msg__GMMParams
    std::shared_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarm_msgs__msg__GMMParams
    std::shared_ptr<swarm_msgs::msg::GMMParams_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GMMParams_ & other) const
  {
    if (this->means != other.means) {
      return false;
    }
    if (this->covariances != other.covariances) {
      return false;
    }
    if (this->weights != other.weights) {
      return false;
    }
    return true;
  }
  bool operator!=(const GMMParams_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GMMParams_

// alias to use template instance with default allocator
using GMMParams =
  swarm_msgs::msg::GMMParams_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace swarm_msgs

#endif  // SWARM_MSGS__MSG__DETAIL__GMM_PARAMS__STRUCT_HPP_
