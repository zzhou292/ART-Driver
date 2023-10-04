// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosapi_msgs:srv/ServiceType.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosapi_msgs/srv/detail/service_type__rosidl_typesupport_introspection_c.h"
#include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosapi_msgs/srv/detail/service_type__functions.h"
#include "rosapi_msgs/srv/detail/service_type__struct.h"


// Include directives for member types
// Member `service`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosapi_msgs__srv__ServiceType_Request__init(message_memory);
}

void rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_fini_function(void * message_memory)
{
  rosapi_msgs__srv__ServiceType_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_member_array[1] = {
  {
    "service",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosapi_msgs__srv__ServiceType_Request, service),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_members = {
  "rosapi_msgs__srv",  // message namespace
  "ServiceType_Request",  // message name
  1,  // number of fields
  sizeof(rosapi_msgs__srv__ServiceType_Request),
  rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_member_array,  // message members
  rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_type_support_handle = {
  0,
  &rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_members,
  get_message_typesupport_handle_function,
  &rosapi_msgs__srv__ServiceType_Request__get_type_hash,
  &rosapi_msgs__srv__ServiceType_Request__get_type_description,
  &rosapi_msgs__srv__ServiceType_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Request)() {
  if (!rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rosapi_msgs/srv/detail/service_type__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rosapi_msgs/srv/detail/service_type__functions.h"
// already included above
// #include "rosapi_msgs/srv/detail/service_type__struct.h"


// Include directives for member types
// Member `type`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosapi_msgs__srv__ServiceType_Response__init(message_memory);
}

void rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_fini_function(void * message_memory)
{
  rosapi_msgs__srv__ServiceType_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_member_array[1] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosapi_msgs__srv__ServiceType_Response, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_members = {
  "rosapi_msgs__srv",  // message namespace
  "ServiceType_Response",  // message name
  1,  // number of fields
  sizeof(rosapi_msgs__srv__ServiceType_Response),
  rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_member_array,  // message members
  rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_type_support_handle = {
  0,
  &rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_members,
  get_message_typesupport_handle_function,
  &rosapi_msgs__srv__ServiceType_Response__get_type_hash,
  &rosapi_msgs__srv__ServiceType_Response__get_type_description,
  &rosapi_msgs__srv__ServiceType_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Response)() {
  if (!rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rosapi_msgs/srv/detail/service_type__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rosapi_msgs/srv/detail/service_type__functions.h"
// already included above
// #include "rosapi_msgs/srv/detail/service_type__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "rosapi_msgs/srv/service_type.h"
// Member `request`
// Member `response`
// already included above
// #include "rosapi_msgs/srv/detail/service_type__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosapi_msgs__srv__ServiceType_Event__init(message_memory);
}

void rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_fini_function(void * message_memory)
{
  rosapi_msgs__srv__ServiceType_Event__fini(message_memory);
}

size_t rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__size_function__ServiceType_Event__request(
  const void * untyped_member)
{
  const rosapi_msgs__srv__ServiceType_Request__Sequence * member =
    (const rosapi_msgs__srv__ServiceType_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_const_function__ServiceType_Event__request(
  const void * untyped_member, size_t index)
{
  const rosapi_msgs__srv__ServiceType_Request__Sequence * member =
    (const rosapi_msgs__srv__ServiceType_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_function__ServiceType_Event__request(
  void * untyped_member, size_t index)
{
  rosapi_msgs__srv__ServiceType_Request__Sequence * member =
    (rosapi_msgs__srv__ServiceType_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__fetch_function__ServiceType_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosapi_msgs__srv__ServiceType_Request * item =
    ((const rosapi_msgs__srv__ServiceType_Request *)
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_const_function__ServiceType_Event__request(untyped_member, index));
  rosapi_msgs__srv__ServiceType_Request * value =
    (rosapi_msgs__srv__ServiceType_Request *)(untyped_value);
  *value = *item;
}

void rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__assign_function__ServiceType_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosapi_msgs__srv__ServiceType_Request * item =
    ((rosapi_msgs__srv__ServiceType_Request *)
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_function__ServiceType_Event__request(untyped_member, index));
  const rosapi_msgs__srv__ServiceType_Request * value =
    (const rosapi_msgs__srv__ServiceType_Request *)(untyped_value);
  *item = *value;
}

bool rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__resize_function__ServiceType_Event__request(
  void * untyped_member, size_t size)
{
  rosapi_msgs__srv__ServiceType_Request__Sequence * member =
    (rosapi_msgs__srv__ServiceType_Request__Sequence *)(untyped_member);
  rosapi_msgs__srv__ServiceType_Request__Sequence__fini(member);
  return rosapi_msgs__srv__ServiceType_Request__Sequence__init(member, size);
}

size_t rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__size_function__ServiceType_Event__response(
  const void * untyped_member)
{
  const rosapi_msgs__srv__ServiceType_Response__Sequence * member =
    (const rosapi_msgs__srv__ServiceType_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_const_function__ServiceType_Event__response(
  const void * untyped_member, size_t index)
{
  const rosapi_msgs__srv__ServiceType_Response__Sequence * member =
    (const rosapi_msgs__srv__ServiceType_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_function__ServiceType_Event__response(
  void * untyped_member, size_t index)
{
  rosapi_msgs__srv__ServiceType_Response__Sequence * member =
    (rosapi_msgs__srv__ServiceType_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__fetch_function__ServiceType_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosapi_msgs__srv__ServiceType_Response * item =
    ((const rosapi_msgs__srv__ServiceType_Response *)
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_const_function__ServiceType_Event__response(untyped_member, index));
  rosapi_msgs__srv__ServiceType_Response * value =
    (rosapi_msgs__srv__ServiceType_Response *)(untyped_value);
  *value = *item;
}

void rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__assign_function__ServiceType_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosapi_msgs__srv__ServiceType_Response * item =
    ((rosapi_msgs__srv__ServiceType_Response *)
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_function__ServiceType_Event__response(untyped_member, index));
  const rosapi_msgs__srv__ServiceType_Response * value =
    (const rosapi_msgs__srv__ServiceType_Response *)(untyped_value);
  *item = *value;
}

bool rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__resize_function__ServiceType_Event__response(
  void * untyped_member, size_t size)
{
  rosapi_msgs__srv__ServiceType_Response__Sequence * member =
    (rosapi_msgs__srv__ServiceType_Response__Sequence *)(untyped_member);
  rosapi_msgs__srv__ServiceType_Response__Sequence__fini(member);
  return rosapi_msgs__srv__ServiceType_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosapi_msgs__srv__ServiceType_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rosapi_msgs__srv__ServiceType_Event, request),  // bytes offset in struct
    NULL,  // default value
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__size_function__ServiceType_Event__request,  // size() function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_const_function__ServiceType_Event__request,  // get_const(index) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_function__ServiceType_Event__request,  // get(index) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__fetch_function__ServiceType_Event__request,  // fetch(index, &value) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__assign_function__ServiceType_Event__request,  // assign(index, value) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__resize_function__ServiceType_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rosapi_msgs__srv__ServiceType_Event, response),  // bytes offset in struct
    NULL,  // default value
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__size_function__ServiceType_Event__response,  // size() function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_const_function__ServiceType_Event__response,  // get_const(index) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__get_function__ServiceType_Event__response,  // get(index) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__fetch_function__ServiceType_Event__response,  // fetch(index, &value) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__assign_function__ServiceType_Event__response,  // assign(index, value) function pointer
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__resize_function__ServiceType_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_members = {
  "rosapi_msgs__srv",  // message namespace
  "ServiceType_Event",  // message name
  3,  // number of fields
  sizeof(rosapi_msgs__srv__ServiceType_Event),
  rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_member_array,  // message members
  rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_type_support_handle = {
  0,
  &rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_members,
  get_message_typesupport_handle_function,
  &rosapi_msgs__srv__ServiceType_Event__get_type_hash,
  &rosapi_msgs__srv__ServiceType_Event__get_type_description,
  &rosapi_msgs__srv__ServiceType_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Event)() {
  rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Request)();
  rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Response)();
  if (!rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosapi_msgs/srv/detail/service_type__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_service_members = {
  "rosapi_msgs__srv",  // service namespace
  "ServiceType",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_Request_message_type_support_handle,
  NULL,  // response message
  // rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_Response_message_type_support_handle
  NULL  // event_message
  // rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_Response_message_type_support_handle
};


static rosidl_service_type_support_t rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_service_type_support_handle = {
  0,
  &rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_service_members,
  get_service_typesupport_handle_function,
  &rosapi_msgs__srv__ServiceType_Request__rosidl_typesupport_introspection_c__ServiceType_Request_message_type_support_handle,
  &rosapi_msgs__srv__ServiceType_Response__rosidl_typesupport_introspection_c__ServiceType_Response_message_type_support_handle,
  &rosapi_msgs__srv__ServiceType_Event__rosidl_typesupport_introspection_c__ServiceType_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    rosapi_msgs,
    srv,
    ServiceType
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    rosapi_msgs,
    srv,
    ServiceType
  ),
  &rosapi_msgs__srv__ServiceType__get_type_hash,
  &rosapi_msgs__srv__ServiceType__get_type_description,
  &rosapi_msgs__srv__ServiceType__get_type_description_sources,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Response)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Event)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType)() {
  if (!rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_service_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, ServiceType_Event)()->data;
  }

  return &rosapi_msgs__srv__detail__service_type__rosidl_typesupport_introspection_c__ServiceType_service_type_support_handle;
}
