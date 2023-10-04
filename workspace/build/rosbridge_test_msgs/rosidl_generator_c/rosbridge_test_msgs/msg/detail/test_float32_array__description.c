// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosbridge_test_msgs:msg/TestFloat32Array.idl
// generated code does not contain a copyright notice

#include "rosbridge_test_msgs/msg/detail/test_float32_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__msg__TestFloat32Array__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbd, 0xa2, 0xda, 0x3c, 0xf5, 0xf0, 0xd2, 0x23,
      0xc6, 0x20, 0x34, 0xb8, 0xbe, 0xd3, 0xdb, 0xaa,
      0x7b, 0x1b, 0xbe, 0xf6, 0x8e, 0xcf, 0x49, 0x40,
      0x72, 0xd5, 0xcc, 0x22, 0x9b, 0x94, 0xfc, 0xaf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosbridge_test_msgs__msg__TestFloat32Array__TYPE_NAME[] = "rosbridge_test_msgs/msg/TestFloat32Array";

// Define type names, field names, and default values
static char rosbridge_test_msgs__msg__TestFloat32Array__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__msg__TestFloat32Array__FIELDS[] = {
  {
    {rosbridge_test_msgs__msg__TestFloat32Array__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__msg__TestFloat32Array__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__msg__TestFloat32Array__TYPE_NAME, 40, 40},
      {rosbridge_test_msgs__msg__TestFloat32Array__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32[] data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__msg__TestFloat32Array__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__msg__TestFloat32Array__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 15, 15},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__msg__TestFloat32Array__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__msg__TestFloat32Array__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
