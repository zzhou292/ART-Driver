// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbridge_test_msgs:srv/TestRequestOnly.idl
// generated code does not contain a copyright notice

#ifndef ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_REQUEST_ONLY__BUILDER_HPP_
#define ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_REQUEST_ONLY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbridge_test_msgs/srv/detail/test_request_only__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_TestRequestOnly_Request_data
{
public:
  Init_TestRequestOnly_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbridge_test_msgs::srv::TestRequestOnly_Request data(::rosbridge_test_msgs::srv::TestRequestOnly_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestRequestOnly_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::TestRequestOnly_Request>()
{
  return rosbridge_test_msgs::srv::builder::Init_TestRequestOnly_Request_data();
}

}  // namespace rosbridge_test_msgs


namespace rosbridge_test_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::TestRequestOnly_Response>()
{
  return ::rosbridge_test_msgs::srv::TestRequestOnly_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosbridge_test_msgs


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_TestRequestOnly_Event_response
{
public:
  explicit Init_TestRequestOnly_Event_response(::rosbridge_test_msgs::srv::TestRequestOnly_Event & msg)
  : msg_(msg)
  {}
  ::rosbridge_test_msgs::srv::TestRequestOnly_Event response(::rosbridge_test_msgs::srv::TestRequestOnly_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestRequestOnly_Event msg_;
};

class Init_TestRequestOnly_Event_request
{
public:
  explicit Init_TestRequestOnly_Event_request(::rosbridge_test_msgs::srv::TestRequestOnly_Event & msg)
  : msg_(msg)
  {}
  Init_TestRequestOnly_Event_response request(::rosbridge_test_msgs::srv::TestRequestOnly_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_TestRequestOnly_Event_response(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestRequestOnly_Event msg_;
};

class Init_TestRequestOnly_Event_info
{
public:
  Init_TestRequestOnly_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TestRequestOnly_Event_request info(::rosbridge_test_msgs::srv::TestRequestOnly_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_TestRequestOnly_Event_request(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::TestRequestOnly_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::TestRequestOnly_Event>()
{
  return rosbridge_test_msgs::srv::builder::Init_TestRequestOnly_Event_info();
}

}  // namespace rosbridge_test_msgs

#endif  // ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_REQUEST_ONLY__BUILDER_HPP_
