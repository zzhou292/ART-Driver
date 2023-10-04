// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosapi_msgs:srv/ServiceProviders.idl
// generated code does not contain a copyright notice

#ifndef ROSAPI_MSGS__SRV__DETAIL__SERVICE_PROVIDERS__BUILDER_HPP_
#define ROSAPI_MSGS__SRV__DETAIL__SERVICE_PROVIDERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosapi_msgs/srv/detail/service_providers__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_ServiceProviders_Request_service
{
public:
  Init_ServiceProviders_Request_service()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosapi_msgs::srv::ServiceProviders_Request service(::rosapi_msgs::srv::ServiceProviders_Request::_service_type arg)
  {
    msg_.service = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::ServiceProviders_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::ServiceProviders_Request>()
{
  return rosapi_msgs::srv::builder::Init_ServiceProviders_Request_service();
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_ServiceProviders_Response_providers
{
public:
  Init_ServiceProviders_Response_providers()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosapi_msgs::srv::ServiceProviders_Response providers(::rosapi_msgs::srv::ServiceProviders_Response::_providers_type arg)
  {
    msg_.providers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::ServiceProviders_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::ServiceProviders_Response>()
{
  return rosapi_msgs::srv::builder::Init_ServiceProviders_Response_providers();
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_ServiceProviders_Event_response
{
public:
  explicit Init_ServiceProviders_Event_response(::rosapi_msgs::srv::ServiceProviders_Event & msg)
  : msg_(msg)
  {}
  ::rosapi_msgs::srv::ServiceProviders_Event response(::rosapi_msgs::srv::ServiceProviders_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::ServiceProviders_Event msg_;
};

class Init_ServiceProviders_Event_request
{
public:
  explicit Init_ServiceProviders_Event_request(::rosapi_msgs::srv::ServiceProviders_Event & msg)
  : msg_(msg)
  {}
  Init_ServiceProviders_Event_response request(::rosapi_msgs::srv::ServiceProviders_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ServiceProviders_Event_response(msg_);
  }

private:
  ::rosapi_msgs::srv::ServiceProviders_Event msg_;
};

class Init_ServiceProviders_Event_info
{
public:
  Init_ServiceProviders_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServiceProviders_Event_request info(::rosapi_msgs::srv::ServiceProviders_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ServiceProviders_Event_request(msg_);
  }

private:
  ::rosapi_msgs::srv::ServiceProviders_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::ServiceProviders_Event>()
{
  return rosapi_msgs::srv::builder::Init_ServiceProviders_Event_info();
}

}  // namespace rosapi_msgs

#endif  // ROSAPI_MSGS__SRV__DETAIL__SERVICE_PROVIDERS__BUILDER_HPP_
