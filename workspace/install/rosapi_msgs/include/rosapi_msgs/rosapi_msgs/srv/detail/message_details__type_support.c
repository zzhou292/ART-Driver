// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosapi_msgs:srv/MessageDetails.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosapi_msgs/srv/detail/message_details__rosidl_typesupport_introspection_c.h"
#include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosapi_msgs/srv/detail/message_details__functions.h"
#include "rosapi_msgs/srv/detail/message_details__struct.h"


// Include directives for member types
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosapi_msgs__srv__MessageDetails_Request__init(message_memory);
}

void rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_fini_function(void * message_memory)
{
  rosapi_msgs__srv__MessageDetails_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_member_array[1] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosapi_msgs__srv__MessageDetails_Request, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_members = {
  "rosapi_msgs__srv",  // message namespace
  "MessageDetails_Request",  // message name
  1,  // number of fields
  sizeof(rosapi_msgs__srv__MessageDetails_Request),
  rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_member_array,  // message members
  rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_type_support_handle = {
  0,
  &rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_members,
  get_message_typesupport_handle_function,
  &rosapi_msgs__srv__MessageDetails_Request__get_type_hash,
  &rosapi_msgs__srv__MessageDetails_Request__get_type_description,
  &rosapi_msgs__srv__MessageDetails_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Request)() {
  if (!rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rosapi_msgs/srv/detail/message_details__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rosapi_msgs/srv/detail/message_details__functions.h"
// already included above
// #include "rosapi_msgs/srv/detail/message_details__struct.h"


// Include directives for member types
// Member `typedefs`
#include "rosapi_msgs/msg/type_def.h"
// Member `typedefs`
#include "rosapi_msgs/msg/detail/type_def__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosapi_msgs__srv__MessageDetails_Response__init(message_memory);
}

void rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_fini_function(void * message_memory)
{
  rosapi_msgs__srv__MessageDetails_Response__fini(message_memory);
}

size_t rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__size_function__MessageDetails_Response__typedefs(
  const void * untyped_member)
{
  const rosapi_msgs__msg__TypeDef__Sequence * member =
    (const rosapi_msgs__msg__TypeDef__Sequence *)(untyped_member);
  return member->size;
}

const void * rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Response__typedefs(
  const void * untyped_member, size_t index)
{
  const rosapi_msgs__msg__TypeDef__Sequence * member =
    (const rosapi_msgs__msg__TypeDef__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__get_function__MessageDetails_Response__typedefs(
  void * untyped_member, size_t index)
{
  rosapi_msgs__msg__TypeDef__Sequence * member =
    (rosapi_msgs__msg__TypeDef__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__fetch_function__MessageDetails_Response__typedefs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosapi_msgs__msg__TypeDef * item =
    ((const rosapi_msgs__msg__TypeDef *)
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Response__typedefs(untyped_member, index));
  rosapi_msgs__msg__TypeDef * value =
    (rosapi_msgs__msg__TypeDef *)(untyped_value);
  *value = *item;
}

void rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__assign_function__MessageDetails_Response__typedefs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosapi_msgs__msg__TypeDef * item =
    ((rosapi_msgs__msg__TypeDef *)
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__get_function__MessageDetails_Response__typedefs(untyped_member, index));
  const rosapi_msgs__msg__TypeDef * value =
    (const rosapi_msgs__msg__TypeDef *)(untyped_value);
  *item = *value;
}

bool rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__resize_function__MessageDetails_Response__typedefs(
  void * untyped_member, size_t size)
{
  rosapi_msgs__msg__TypeDef__Sequence * member =
    (rosapi_msgs__msg__TypeDef__Sequence *)(untyped_member);
  rosapi_msgs__msg__TypeDef__Sequence__fini(member);
  return rosapi_msgs__msg__TypeDef__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_member_array[1] = {
  {
    "typedefs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosapi_msgs__srv__MessageDetails_Response, typedefs),  // bytes offset in struct
    NULL,  // default value
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__size_function__MessageDetails_Response__typedefs,  // size() function pointer
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Response__typedefs,  // get_const(index) function pointer
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__get_function__MessageDetails_Response__typedefs,  // get(index) function pointer
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__fetch_function__MessageDetails_Response__typedefs,  // fetch(index, &value) function pointer
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__assign_function__MessageDetails_Response__typedefs,  // assign(index, value) function pointer
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__resize_function__MessageDetails_Response__typedefs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_members = {
  "rosapi_msgs__srv",  // message namespace
  "MessageDetails_Response",  // message name
  1,  // number of fields
  sizeof(rosapi_msgs__srv__MessageDetails_Response),
  rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_member_array,  // message members
  rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_type_support_handle = {
  0,
  &rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_members,
  get_message_typesupport_handle_function,
  &rosapi_msgs__srv__MessageDetails_Response__get_type_hash,
  &rosapi_msgs__srv__MessageDetails_Response__get_type_description,
  &rosapi_msgs__srv__MessageDetails_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Response)() {
  rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, msg, TypeDef)();
  if (!rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rosapi_msgs/srv/detail/message_details__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rosapi_msgs/srv/detail/message_details__functions.h"
// already included above
// #include "rosapi_msgs/srv/detail/message_details__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "rosapi_msgs/srv/message_details.h"
// Member `request`
// Member `response`
// already included above
// #include "rosapi_msgs/srv/detail/message_details__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosapi_msgs__srv__MessageDetails_Event__init(message_memory);
}

void rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_fini_function(void * message_memory)
{
  rosapi_msgs__srv__MessageDetails_Event__fini(message_memory);
}

size_t rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__size_function__MessageDetails_Event__request(
  const void * untyped_member)
{
  const rosapi_msgs__srv__MessageDetails_Request__Sequence * member =
    (const rosapi_msgs__srv__MessageDetails_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Event__request(
  const void * untyped_member, size_t index)
{
  const rosapi_msgs__srv__MessageDetails_Request__Sequence * member =
    (const rosapi_msgs__srv__MessageDetails_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_function__MessageDetails_Event__request(
  void * untyped_member, size_t index)
{
  rosapi_msgs__srv__MessageDetails_Request__Sequence * member =
    (rosapi_msgs__srv__MessageDetails_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__fetch_function__MessageDetails_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosapi_msgs__srv__MessageDetails_Request * item =
    ((const rosapi_msgs__srv__MessageDetails_Request *)
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Event__request(untyped_member, index));
  rosapi_msgs__srv__MessageDetails_Request * value =
    (rosapi_msgs__srv__MessageDetails_Request *)(untyped_value);
  *value = *item;
}

void rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__assign_function__MessageDetails_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosapi_msgs__srv__MessageDetails_Request * item =
    ((rosapi_msgs__srv__MessageDetails_Request *)
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_function__MessageDetails_Event__request(untyped_member, index));
  const rosapi_msgs__srv__MessageDetails_Request * value =
    (const rosapi_msgs__srv__MessageDetails_Request *)(untyped_value);
  *item = *value;
}

bool rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__resize_function__MessageDetails_Event__request(
  void * untyped_member, size_t size)
{
  rosapi_msgs__srv__MessageDetails_Request__Sequence * member =
    (rosapi_msgs__srv__MessageDetails_Request__Sequence *)(untyped_member);
  rosapi_msgs__srv__MessageDetails_Request__Sequence__fini(member);
  return rosapi_msgs__srv__MessageDetails_Request__Sequence__init(member, size);
}

size_t rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__size_function__MessageDetails_Event__response(
  const void * untyped_member)
{
  const rosapi_msgs__srv__MessageDetails_Response__Sequence * member =
    (const rosapi_msgs__srv__MessageDetails_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Event__response(
  const void * untyped_member, size_t index)
{
  const rosapi_msgs__srv__MessageDetails_Response__Sequence * member =
    (const rosapi_msgs__srv__MessageDetails_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_function__MessageDetails_Event__response(
  void * untyped_member, size_t index)
{
  rosapi_msgs__srv__MessageDetails_Response__Sequence * member =
    (rosapi_msgs__srv__MessageDetails_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__fetch_function__MessageDetails_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosapi_msgs__srv__MessageDetails_Response * item =
    ((const rosapi_msgs__srv__MessageDetails_Response *)
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Event__response(untyped_member, index));
  rosapi_msgs__srv__MessageDetails_Response * value =
    (rosapi_msgs__srv__MessageDetails_Response *)(untyped_value);
  *value = *item;
}

void rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__assign_function__MessageDetails_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosapi_msgs__srv__MessageDetails_Response * item =
    ((rosapi_msgs__srv__MessageDetails_Response *)
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_function__MessageDetails_Event__response(untyped_member, index));
  const rosapi_msgs__srv__MessageDetails_Response * value =
    (const rosapi_msgs__srv__MessageDetails_Response *)(untyped_value);
  *item = *value;
}

bool rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__resize_function__MessageDetails_Event__response(
  void * untyped_member, size_t size)
{
  rosapi_msgs__srv__MessageDetails_Response__Sequence * member =
    (rosapi_msgs__srv__MessageDetails_Response__Sequence *)(untyped_member);
  rosapi_msgs__srv__MessageDetails_Response__Sequence__fini(member);
  return rosapi_msgs__srv__MessageDetails_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosapi_msgs__srv__MessageDetails_Event, info),  // bytes offset in struct
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
    offsetof(rosapi_msgs__srv__MessageDetails_Event, request),  // bytes offset in struct
    NULL,  // default value
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__size_function__MessageDetails_Event__request,  // size() function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Event__request,  // get_const(index) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_function__MessageDetails_Event__request,  // get(index) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__fetch_function__MessageDetails_Event__request,  // fetch(index, &value) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__assign_function__MessageDetails_Event__request,  // assign(index, value) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__resize_function__MessageDetails_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rosapi_msgs__srv__MessageDetails_Event, response),  // bytes offset in struct
    NULL,  // default value
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__size_function__MessageDetails_Event__response,  // size() function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_const_function__MessageDetails_Event__response,  // get_const(index) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__get_function__MessageDetails_Event__response,  // get(index) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__fetch_function__MessageDetails_Event__response,  // fetch(index, &value) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__assign_function__MessageDetails_Event__response,  // assign(index, value) function pointer
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__resize_function__MessageDetails_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_members = {
  "rosapi_msgs__srv",  // message namespace
  "MessageDetails_Event",  // message name
  3,  // number of fields
  sizeof(rosapi_msgs__srv__MessageDetails_Event),
  rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_member_array,  // message members
  rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_type_support_handle = {
  0,
  &rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_members,
  get_message_typesupport_handle_function,
  &rosapi_msgs__srv__MessageDetails_Event__get_type_hash,
  &rosapi_msgs__srv__MessageDetails_Event__get_type_description,
  &rosapi_msgs__srv__MessageDetails_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Event)() {
  rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Request)();
  rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Response)();
  if (!rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosapi_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosapi_msgs/srv/detail/message_details__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_service_members = {
  "rosapi_msgs__srv",  // service namespace
  "MessageDetails",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_Request_message_type_support_handle,
  NULL,  // response message
  // rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_Response_message_type_support_handle
  NULL  // event_message
  // rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_Response_message_type_support_handle
};


static rosidl_service_type_support_t rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_service_type_support_handle = {
  0,
  &rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_service_members,
  get_service_typesupport_handle_function,
  &rosapi_msgs__srv__MessageDetails_Request__rosidl_typesupport_introspection_c__MessageDetails_Request_message_type_support_handle,
  &rosapi_msgs__srv__MessageDetails_Response__rosidl_typesupport_introspection_c__MessageDetails_Response_message_type_support_handle,
  &rosapi_msgs__srv__MessageDetails_Event__rosidl_typesupport_introspection_c__MessageDetails_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    rosapi_msgs,
    srv,
    MessageDetails
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    rosapi_msgs,
    srv,
    MessageDetails
  ),
  &rosapi_msgs__srv__MessageDetails__get_type_hash,
  &rosapi_msgs__srv__MessageDetails__get_type_description,
  &rosapi_msgs__srv__MessageDetails__get_type_description_sources,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Response)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Event)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosapi_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails)() {
  if (!rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_service_type_support_handle.typesupport_identifier) {
    rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosapi_msgs, srv, MessageDetails_Event)()->data;
  }

  return &rosapi_msgs__srv__detail__message_details__rosidl_typesupport_introspection_c__MessageDetails_service_type_support_handle;
}
