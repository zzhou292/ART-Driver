// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosbridge_test_msgs:msg/Num.idl
// generated code does not contain a copyright notice

#include "rosbridge_test_msgs/msg/detail/num__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__msg__Num__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa4, 0x8f, 0x50, 0x4d, 0x57, 0x23, 0x16, 0xd3,
      0x90, 0x75, 0x44, 0xec, 0xe7, 0x29, 0xa1, 0xf0,
      0xc9, 0x7a, 0x24, 0x7d, 0x6f, 0x37, 0x68, 0xcc,
      0x2f, 0x44, 0xc2, 0x0c, 0x0d, 0x76, 0x13, 0xae,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosbridge_test_msgs__msg__Num__TYPE_NAME[] = "rosbridge_test_msgs/msg/Num";

// Define type names, field names, and default values
static char rosbridge_test_msgs__msg__Num__FIELD_NAME__num[] = "num";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__msg__Num__FIELDS[] = {
  {
    {rosbridge_test_msgs__msg__Num__FIELD_NAME__num, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__msg__Num__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__msg__Num__TYPE_NAME, 27, 27},
      {rosbridge_test_msgs__msg__Num__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int64 num";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__msg__Num__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__msg__Num__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 10, 10},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__msg__Num__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__msg__Num__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
