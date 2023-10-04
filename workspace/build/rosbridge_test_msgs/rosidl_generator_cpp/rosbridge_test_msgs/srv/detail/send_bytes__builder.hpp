// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosbridge_test_msgs:srv/SendBytes.idl
// generated code does not contain a copyright notice

#ifndef ROSBRIDGE_TEST_MSGS__SRV__DETAIL__SEND_BYTES__BUILDER_HPP_
#define ROSBRIDGE_TEST_MSGS__SRV__DETAIL__SEND_BYTES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosbridge_test_msgs/srv/detail/send_bytes__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_SendBytes_Request_count
{
public:
  Init_SendBytes_Request_count()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbridge_test_msgs::srv::SendBytes_Request count(::rosbridge_test_msgs::srv::SendBytes_Request::_count_type arg)
  {
    msg_.count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::SendBytes_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::SendBytes_Request>()
{
  return rosbridge_test_msgs::srv::builder::Init_SendBytes_Request_count();
}

}  // namespace rosbridge_test_msgs


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_SendBytes_Response_data
{
public:
  Init_SendBytes_Response_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosbridge_test_msgs::srv::SendBytes_Response data(::rosbridge_test_msgs::srv::SendBytes_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::SendBytes_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::SendBytes_Response>()
{
  return rosbridge_test_msgs::srv::builder::Init_SendBytes_Response_data();
}

}  // namespace rosbridge_test_msgs


namespace rosbridge_test_msgs
{

namespace srv
{

namespace builder
{

class Init_SendBytes_Event_response
{
public:
  explicit Init_SendBytes_Event_response(::rosbridge_test_msgs::srv::SendBytes_Event & msg)
  : msg_(msg)
  {}
  ::rosbridge_test_msgs::srv::SendBytes_Event response(::rosbridge_test_msgs::srv::SendBytes_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::SendBytes_Event msg_;
};

class Init_SendBytes_Event_request
{
public:
  explicit Init_SendBytes_Event_request(::rosbridge_test_msgs::srv::SendBytes_Event & msg)
  : msg_(msg)
  {}
  Init_SendBytes_Event_response request(::rosbridge_test_msgs::srv::SendBytes_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SendBytes_Event_response(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::SendBytes_Event msg_;
};

class Init_SendBytes_Event_info
{
public:
  Init_SendBytes_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendBytes_Event_request info(::rosbridge_test_msgs::srv::SendBytes_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SendBytes_Event_request(msg_);
  }

private:
  ::rosbridge_test_msgs::srv::SendBytes_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosbridge_test_msgs::srv::SendBytes_Event>()
{
  return rosbridge_test_msgs::srv::builder::Init_SendBytes_Event_info();
}

}  // namespace rosbridge_test_msgs

#endif  // ROSBRIDGE_TEST_MSGS__SRV__DETAIL__SEND_BYTES__BUILDER_HPP_
