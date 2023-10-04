// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosbridge_test_msgs:msg/TestUInt8FixedSizeArray16.idl
// generated code does not contain a copyright notice

#include "rosbridge_test_msgs/msg/detail/test_u_int8_fixed_size_array16__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6f, 0x8f, 0x3f, 0xf5, 0x69, 0x25, 0xb2, 0xc1,
      0xcc, 0x1e, 0xc7, 0xf2, 0x36, 0xfd, 0x08, 0xf7,
      0xae, 0xa0, 0x20, 0x85, 0x48, 0xe2, 0x67, 0x4a,
      0x8c, 0x68, 0x6d, 0xe2, 0x35, 0xe1, 0x36, 0x2d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__TYPE_NAME[] = "rosbridge_test_msgs/msg/TestUInt8FixedSizeArray16";

// Define type names, field names, and default values
static char rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__FIELDS[] = {
  {
    {rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      16,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__TYPE_NAME, 49, 49},
      {rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8[16] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__TYPE_NAME, 49, 49},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 15, 15},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__msg__TestUInt8FixedSizeArray16__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
