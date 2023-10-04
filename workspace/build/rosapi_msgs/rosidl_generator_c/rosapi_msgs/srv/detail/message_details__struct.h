// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosapi_msgs:srv/MessageDetails.idl
// generated code does not contain a copyright notice

#ifndef ROSAPI_MSGS__SRV__DETAIL__MESSAGE_DETAILS__STRUCT_H_
#define ROSAPI_MSGS__SRV__DETAIL__MESSAGE_DETAILS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MessageDetails in the package rosapi_msgs.
typedef struct rosapi_msgs__srv__MessageDetails_Request
{
  rosidl_runtime_c__String type;
} rosapi_msgs__srv__MessageDetails_Request;

// Struct for a sequence of rosapi_msgs__srv__MessageDetails_Request.
typedef struct rosapi_msgs__srv__MessageDetails_Request__Sequence
{
  rosapi_msgs__srv__MessageDetails_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosapi_msgs__srv__MessageDetails_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'typedefs'
#include "rosapi_msgs/msg/detail/type_def__struct.h"

/// Struct defined in srv/MessageDetails in the package rosapi_msgs.
typedef struct rosapi_msgs__srv__MessageDetails_Response
{
  rosapi_msgs__msg__TypeDef__Sequence typedefs;
} rosapi_msgs__srv__MessageDetails_Response;

// Struct for a sequence of rosapi_msgs__srv__MessageDetails_Response.
typedef struct rosapi_msgs__srv__MessageDetails_Response__Sequence
{
  rosapi_msgs__srv__MessageDetails_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosapi_msgs__srv__MessageDetails_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  rosapi_msgs__srv__MessageDetails_Event__request__MAX_SIZE = 1
};
// response
enum
{
  rosapi_msgs__srv__MessageDetails_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MessageDetails in the package rosapi_msgs.
typedef struct rosapi_msgs__srv__MessageDetails_Event
{
  service_msgs__msg__ServiceEventInfo info;
  rosapi_msgs__srv__MessageDetails_Request__Sequence request;
  rosapi_msgs__srv__MessageDetails_Response__Sequence response;
} rosapi_msgs__srv__MessageDetails_Event;

// Struct for a sequence of rosapi_msgs__srv__MessageDetails_Event.
typedef struct rosapi_msgs__srv__MessageDetails_Event__Sequence
{
  rosapi_msgs__srv__MessageDetails_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosapi_msgs__srv__MessageDetails_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSAPI_MSGS__SRV__DETAIL__MESSAGE_DETAILS__STRUCT_H_
