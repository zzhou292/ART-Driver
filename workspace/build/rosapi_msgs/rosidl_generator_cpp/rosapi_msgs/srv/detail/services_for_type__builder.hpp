// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosapi_msgs:srv/ServicesForType.idl
// generated code does not contain a copyright notice

#ifndef ROSAPI_MSGS__SRV__DETAIL__SERVICES_FOR_TYPE__BUILDER_HPP_
#define ROSAPI_MSGS__SRV__DETAIL__SERVICES_FOR_TYPE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosapi_msgs/srv/detail/services_for_type__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_ServicesForType_Request_type
{
public:
  Init_ServicesForType_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosapi_msgs::srv::ServicesForType_Request type(::rosapi_msgs::srv::ServicesForType_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::ServicesForType_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::ServicesForType_Request>()
{
  return rosapi_msgs::srv::builder::Init_ServicesForType_Request_type();
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_ServicesForType_Response_services
{
public:
  Init_ServicesForType_Response_services()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosapi_msgs::srv::ServicesForType_Response services(::rosapi_msgs::srv::ServicesForType_Response::_services_type arg)
  {
    msg_.services = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::ServicesForType_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::ServicesForType_Response>()
{
  return rosapi_msgs::srv::builder::Init_ServicesForType_Response_services();
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_ServicesForType_Event_response
{
public:
  explicit Init_ServicesForType_Event_response(::rosapi_msgs::srv::ServicesForType_Event & msg)
  : msg_(msg)
  {}
  ::rosapi_msgs::srv::ServicesForType_Event response(::rosapi_msgs::srv::ServicesForType_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::ServicesForType_Event msg_;
};

class Init_ServicesForType_Event_request
{
public:
  explicit Init_ServicesForType_Event_request(::rosapi_msgs::srv::ServicesForType_Event & msg)
  : msg_(msg)
  {}
  Init_ServicesForType_Event_response request(::rosapi_msgs::srv::ServicesForType_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ServicesForType_Event_response(msg_);
  }

private:
  ::rosapi_msgs::srv::ServicesForType_Event msg_;
};

class Init_ServicesForType_Event_info
{
public:
  Init_ServicesForType_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServicesForType_Event_request info(::rosapi_msgs::srv::ServicesForType_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ServicesForType_Event_request(msg_);
  }

private:
  ::rosapi_msgs::srv::ServicesForType_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::ServicesForType_Event>()
{
  return rosapi_msgs::srv::builder::Init_ServicesForType_Event_info();
}

}  // namespace rosapi_msgs

#endif  // ROSAPI_MSGS__SRV__DETAIL__SERVICES_FOR_TYPE__BUILDER_HPP_
