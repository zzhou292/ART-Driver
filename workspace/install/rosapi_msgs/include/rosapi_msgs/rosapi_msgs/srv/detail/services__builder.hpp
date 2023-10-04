// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosapi_msgs:srv/Services.idl
// generated code does not contain a copyright notice

#ifndef ROSAPI_MSGS__SRV__DETAIL__SERVICES__BUILDER_HPP_
#define ROSAPI_MSGS__SRV__DETAIL__SERVICES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosapi_msgs/srv/detail/services__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosapi_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::Services_Request>()
{
  return ::rosapi_msgs::srv::Services_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_Services_Response_services
{
public:
  Init_Services_Response_services()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosapi_msgs::srv::Services_Response services(::rosapi_msgs::srv::Services_Response::_services_type arg)
  {
    msg_.services = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::Services_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::Services_Response>()
{
  return rosapi_msgs::srv::builder::Init_Services_Response_services();
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_Services_Event_response
{
public:
  explicit Init_Services_Event_response(::rosapi_msgs::srv::Services_Event & msg)
  : msg_(msg)
  {}
  ::rosapi_msgs::srv::Services_Event response(::rosapi_msgs::srv::Services_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::Services_Event msg_;
};

class Init_Services_Event_request
{
public:
  explicit Init_Services_Event_request(::rosapi_msgs::srv::Services_Event & msg)
  : msg_(msg)
  {}
  Init_Services_Event_response request(::rosapi_msgs::srv::Services_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Services_Event_response(msg_);
  }

private:
  ::rosapi_msgs::srv::Services_Event msg_;
};

class Init_Services_Event_info
{
public:
  Init_Services_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Services_Event_request info(::rosapi_msgs::srv::Services_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Services_Event_request(msg_);
  }

private:
  ::rosapi_msgs::srv::Services_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::Services_Event>()
{
  return rosapi_msgs::srv::builder::Init_Services_Event_info();
}

}  // namespace rosapi_msgs

#endif  // ROSAPI_MSGS__SRV__DETAIL__SERVICES__BUILDER_HPP_
