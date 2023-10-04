// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbridge_test_msgs:srv/TestNestedService.idl
// generated code does not contain a copyright notice

#ifndef ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_NESTED_SERVICE__BUILDER_HPP_
#define ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_NESTED_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbridge_test_msgs/srv/detail/test_nested_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_TestNestedService_Request_pose
{
public:
  Init_TestNestedService_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbridge_test_msgs::srv::TestNestedService_Request pose(::rosbridge_test_msgs::srv::TestNestedService_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestNestedService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::TestNestedService_Request>()
{
  return rosbridge_test_msgs::srv::builder::Init_TestNestedService_Request_pose();
}

}  // namespace rosbridge_test_msgs


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_TestNestedService_Response_data
{
public:
  Init_TestNestedService_Response_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbridge_test_msgs::srv::TestNestedService_Response data(::rosbridge_test_msgs::srv::TestNestedService_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestNestedService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::TestNestedService_Response>()
{
  return rosbridge_test_msgs::srv::builder::Init_TestNestedService_Response_data();
}

}  // namespace rosbridge_test_msgs


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_TestNestedService_Event_response
{
public:
  explicit Init_TestNestedService_Event_response(::rosbridge_test_msgs::srv::TestNestedService_Event & msg)
  : msg_(msg)
  {}
  ::rosbridge_test_msgs::srv::TestNestedService_Event response(::rosbridge_test_msgs::srv::TestNestedService_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestNestedService_Event msg_;
};

class Init_TestNestedService_Event_request
{
public:
  explicit Init_TestNestedService_Event_request(::rosbridge_test_msgs::srv::TestNestedService_Event & msg)
  : msg_(msg)
  {}
  Init_TestNestedService_Event_response request(::rosbridge_test_msgs::srv::TestNestedService_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_TestNestedService_Event_response(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestNestedService_Event msg_;
};

class Init_TestNestedService_Event_info
{
public:
  Init_TestNestedService_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TestNestedService_Event_request info(::rosbridge_test_msgs::srv::TestNestedService_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_TestNestedService_Event_request(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestNestedService_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::TestNestedService_Event>()
{
  return rosbridge_test_msgs::srv::builder::Init_TestNestedService_Event_info();
}

}  // namespace rosbridge_test_msgs

#endif  // ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_NESTED_SERVICE__BUILDER_HPP_
