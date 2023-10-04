// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosbridge_test_msgs:srv/TestRequestOnly.idl
// generated code does not contain a copyright notice

#ifndef ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_REQUEST_ONLY__STRUCT_H_
#define ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_REQUEST_ONLY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/TestRequestOnly in the package rosbridge_test_msgs.
typedef struct rosbridge_test_msgs__srv__TestRequestOnly_Request
{
  int32_t data;
} rosbridge_test_msgs__srv__TestRequestOnly_Request;

// Struct for a sequence of rosbridge_test_msgs__srv__TestRequestOnly_Request.
typedef struct rosbridge_test_msgs__srv__TestRequestOnly_Request__Sequence
{
  rosbridge_test_msgs__srv__TestRequestOnly_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbridge_test_msgs__srv__TestRequestOnly_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/TestRequestOnly in the package rosbridge_test_msgs.
typedef struct rosbridge_test_msgs__srv__TestRequestOnly_Response
{
  uint8_t structure_needs_at_least_one_member;
} rosbridge_test_msgs__srv__TestRequestOnly_Response;

// Struct for a sequence of rosbridge_test_msgs__srv__TestRequestOnly_Response.
typedef struct rosbridge_test_msgs__srv__TestRequestOnly_Response__Sequence
{
  rosbridge_test_msgs__srv__TestRequestOnly_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbridge_test_msgs__srv__TestRequestOnly_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  rosbridge_test_msgs__srv__TestRequestOnly_Event__request__MAX_SIZE = 1
};
// response
enum
{
  rosbridge_test_msgs__srv__TestRequestOnly_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/TestRequestOnly in the package rosbridge_test_msgs.
typedef struct rosbridge_test_msgs__srv__TestRequestOnly_Event
{
  service_msgs__msg__ServiceEventInfo info;
  rosbridge_test_msgs__srv__TestRequestOnly_Request__Sequence request;
  rosbridge_test_msgs__srv__TestRequestOnly_Response__Sequence response;
} rosbridge_test_msgs__srv__TestRequestOnly_Event;

// Struct for a sequence of rosbridge_test_msgs__srv__TestRequestOnly_Event.
typedef struct rosbridge_test_msgs__srv__TestRequestOnly_Event__Sequence
{
  rosbridge_test_msgs__srv__TestRequestOnly_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosbridge_test_msgs__srv__TestRequestOnly_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSBRIDGE_TEST_MSGS__SRV__DETAIL__TEST_REQUEST_ONLY__STRUCT_H_
