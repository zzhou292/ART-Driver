// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosapi_msgs:srv/GetParamNames.idl
// generated code does not contain a copyright notice

#ifndef ROSAPI_MSGS__SRV__DETAIL__GET_PARAM_NAMES__BUILDER_HPP_
#define ROSAPI_MSGS__SRV__DETAIL__GET_PARAM_NAMES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosapi_msgs/srv/detail/get_param_names__struct.hpp"
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
auto build<::rosapi_msgs::srv::GetParamNames_Request>()
{
  return ::rosapi_msgs::srv::GetParamNames_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_GetParamNames_Response_names
{
public:
  Init_GetParamNames_Response_names()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosapi_msgs::srv::GetParamNames_Response names(::rosapi_msgs::srv::GetParamNames_Response::_names_type arg)
  {
    msg_.names = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::GetParamNames_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::GetParamNames_Response>()
{
  return rosapi_msgs::srv::builder::Init_GetParamNames_Response_names();
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_GetParamNames_Event_response
{
public:
  explicit Init_GetParamNames_Event_response(::rosapi_msgs::srv::GetParamNames_Event & msg)
  : msg_(msg)
  {}
  ::rosapi_msgs::srv::GetParamNames_Event response(::rosapi_msgs::srv::GetParamNames_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::GetParamNames_Event msg_;
};

class Init_GetParamNames_Event_request
{
public:
  explicit Init_GetParamNames_Event_request(::rosapi_msgs::srv::GetParamNames_Event & msg)
  : msg_(msg)
  {}
  Init_GetParamNames_Event_response request(::rosapi_msgs::srv::GetParamNames_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetParamNames_Event_response(msg_);
  }

private:
  ::rosapi_msgs::srv::GetParamNames_Event msg_;
};

class Init_GetParamNames_Event_info
{
public:
  Init_GetParamNames_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetParamNames_Event_request info(::rosapi_msgs::srv::GetParamNames_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetParamNames_Event_request(msg_);
  }

private:
  ::rosapi_msgs::srv::GetParamNames_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::GetParamNames_Event>()
{
  return rosapi_msgs::srv::builder::Init_GetParamNames_Event_info();
}

}  // namespace rosapi_msgs

#endif  // ROSAPI_MSGS__SRV__DETAIL__GET_PARAM_NAMES__BUILDER_HPP_
