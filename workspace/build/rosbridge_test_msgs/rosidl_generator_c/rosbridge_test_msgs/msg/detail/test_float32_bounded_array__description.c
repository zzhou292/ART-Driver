// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosbridge_test_msgs:msg/TestFloat32BoundedArray.idl
// generated code does not contain a copyright notice

#include "rosbridge_test_msgs/msg/detail/test_float32_bounded_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__msg__TestFloat32BoundedArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x41, 0xd7, 0x02, 0x5c, 0x5d, 0xfe, 0xf4, 0x59,
      0x63, 0x53, 0x91, 0x3e, 0xe0, 0x1f, 0x97, 0xbe,
      0x38, 0xb0, 0x77, 0x50, 0xe5, 0x2f, 0x81, 0x8b,
      0x27, 0xef, 0x2f, 0xc8, 0x52, 0x31, 0x43, 0x82,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosbridge_test_msgs__msg__TestFloat32BoundedArray__TYPE_NAME[] = "rosbridge_test_msgs/msg/TestFloat32BoundedArray";

// Define type names, field names, and default values
static char rosbridge_test_msgs__msg__TestFloat32BoundedArray__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__msg__TestFloat32BoundedArray__FIELDS[] = {
  {
    {rosbridge_test_msgs__msg__TestFloat32BoundedArray__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      16,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__msg__TestFloat32BoundedArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__msg__TestFloat32BoundedArray__TYPE_NAME, 47, 47},
      {rosbridge_test_msgs__msg__TestFloat32BoundedArray__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32[16] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__msg__TestFloat32BoundedArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__msg__TestFloat32BoundedArray__TYPE_NAME, 47, 47},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 17, 17},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__msg__TestFloat32BoundedArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__msg__TestFloat32BoundedArray__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
