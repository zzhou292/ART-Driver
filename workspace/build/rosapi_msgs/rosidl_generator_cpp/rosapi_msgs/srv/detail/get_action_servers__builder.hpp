// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosapi_msgs:srv/GetActionServers.idl
// generated code does not contain a copyright notice

#ifndef ROSAPI_MSGS__SRV__DETAIL__GET_ACTION_SERVERS__BUILDER_HPP_
#define ROSAPI_MSGS__SRV__DETAIL__GET_ACTION_SERVERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosapi_msgs/srv/detail/get_action_servers__struct.hpp"
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
auto build<::rosapi_msgs::srv::GetActionServers_Request>()
{
  return ::rosapi_msgs::srv::GetActionServers_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_GetActionServers_Response_action_servers
{
public:
  Init_GetActionServers_Response_action_servers()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosapi_msgs::srv::GetActionServers_Response action_servers(::rosapi_msgs::srv::GetActionServers_Response::_action_servers_type arg)
  {
    msg_.action_servers = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::GetActionServers_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::GetActionServers_Response>()
{
  return rosapi_msgs::srv::builder::Init_GetActionServers_Response_action_servers();
}

}  // namespace rosapi_msgs


namespace rosapi_msgs
{

namespace srv
{

namespace builder
{

class Init_GetActionServers_Event_response
{
public:
  explicit Init_GetActionServers_Event_response(::rosapi_msgs::srv::GetActionServers_Event & msg)
  : msg_(msg)
  {}
  ::rosapi_msgs::srv::GetActionServers_Event response(::rosapi_msgs::srv::GetActionServers_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosapi_msgs::srv::GetActionServers_Event msg_;
};

class Init_GetActionServers_Event_request
{
public:
  explicit Init_GetActionServers_Event_request(::rosapi_msgs::srv::GetActionServers_Event & msg)
  : msg_(msg)
  {}
  Init_GetActionServers_Event_response request(::rosapi_msgs::srv::GetActionServers_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetActionServers_Event_response(msg_);
  }

private:
  ::rosapi_msgs::srv::GetActionServers_Event msg_;
};

class Init_GetActionServers_Event_info
{
public:
  Init_GetActionServers_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetActionServers_Event_request info(::rosapi_msgs::srv::GetActionServers_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetActionServers_Event_request(msg_);
  }

private:
  ::rosapi_msgs::srv::GetActionServers_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosapi_msgs::srv::GetActionServers_Event>()
{
  return rosapi_msgs::srv::builder::Init_GetActionServers_Event_info();
}

}  // namespace rosapi_msgs

#endif  // ROSAPI_MSGS__SRV__DETAIL__GET_ACTION_SERVERS__BUILDER_HPP_
