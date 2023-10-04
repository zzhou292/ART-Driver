// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosbridge_test_msgs:srv/TestResponseOnly.idl
// generated code does not contain a copyright notice

#ifndef ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_RESPONSE_ONLY__STRUCT_HPP_
#define ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_RESPONSE_ONLY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Request __declspec(deprecated)
#endif

namespace rosbridge_test_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TestResponseOnly_Request_
{
  using Type = TestResponseOnly_Request_<ContainerAllocator>;

  explicit TestResponseOnly_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit TestResponseOnly_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Request
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Request
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestResponseOnly_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestResponseOnly_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestResponseOnly_Request_

// alias to use template instance with default allocator
using TestResponseOnly_Request =
  rosbridge_test_msgs::srv::TestResponseOnly_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbridge_test_msgs


#ifndef _WIN32
# define DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Response __declspec(deprecated)
#endif

namespace rosbridge_test_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TestResponseOnly_Response_
{
  using Type = TestResponseOnly_Response_<ContainerAllocator>;

  explicit TestResponseOnly_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  explicit TestResponseOnly_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0l;
    }
  }

  // field types and members
  using _data_type =
    int32_t;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const int32_t & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Response
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Response
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestResponseOnly_Response_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestResponseOnly_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestResponseOnly_Response_

// alias to use template instance with default allocator
using TestResponseOnly_Response =
  rosbridge_test_msgs::srv::TestResponseOnly_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbridge_test_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Event __attribute__((deprecated))
#else
# define DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Event __declspec(deprecated)
#endif

namespace rosbridge_test_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TestResponseOnly_Event_
{
  using Type = TestResponseOnly_Event_<ContainerAllocator>;

  explicit TestResponseOnly_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit TestResponseOnly_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rosbridge_test_msgs::srv::TestResponseOnly_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rosbridge_test_msgs::srv::TestResponseOnly_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Event
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosbridge_test_msgs__srv__TestResponseOnly_Event
    std::shared_ptr<rosbridge_test_msgs::srv::TestResponseOnly_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestResponseOnly_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestResponseOnly_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestResponseOnly_Event_

// alias to use template instance with default allocator
using TestResponseOnly_Event =
  rosbridge_test_msgs::srv::TestResponseOnly_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosbridge_test_msgs

namespace rosbridge_test_msgs
{

namespace srv
{

struct TestResponseOnly
{
  using Request = rosbridge_test_msgs::srv::TestResponseOnly_Request;
  using Response = rosbridge_test_msgs::srv::TestResponseOnly_Response;
  using Event = rosbridge_test_msgs::srv::TestResponseOnly_Event;
};

}  // namespace srv

}  // namespace rosbridge_test_msgs

#endif  // ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_RESPONSE_ONLY__STRUCT_HPP_
