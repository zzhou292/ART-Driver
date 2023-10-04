// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosapi_msgs:msg/TypeDef.idl
// generated code does not contain a copyright notice

#include "rosapi_msgs/msg/detail/type_def__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosapi_msgs
const rosidl_type_hash_t *
rosapi_msgs__msg__TypeDef__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdc, 0xd2, 0x54, 0x91, 0x39, 0xd3, 0xfd, 0xb4,
      0x32, 0xe7, 0xf9, 0x25, 0xfe, 0x79, 0xe9, 0xfb,
      0x4f, 0x02, 0x88, 0x9c, 0x79, 0xbe, 0x45, 0xf7,
      0x48, 0x2c, 0x34, 0x67, 0x06, 0x73, 0x49, 0x28,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosapi_msgs__msg__TypeDef__TYPE_NAME[] = "rosapi_msgs/msg/TypeDef";

// Define type names, field names, and default values
static char rosapi_msgs__msg__TypeDef__FIELD_NAME__type[] = "type";
static char rosapi_msgs__msg__TypeDef__FIELD_NAME__fieldnames[] = "fieldnames";
static char rosapi_msgs__msg__TypeDef__FIELD_NAME__fieldtypes[] = "fieldtypes";
static char rosapi_msgs__msg__TypeDef__FIELD_NAME__fieldarraylen[] = "fieldarraylen";
static char rosapi_msgs__msg__TypeDef__FIELD_NAME__examples[] = "examples";
static char rosapi_msgs__msg__TypeDef__FIELD_NAME__constnames[] = "constnames";
static char rosapi_msgs__msg__TypeDef__FIELD_NAME__constvalues[] = "constvalues";

static rosidl_runtime_c__type_description__Field rosapi_msgs__msg__TypeDef__FIELDS[] = {
  {
    {rosapi_msgs__msg__TypeDef__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__FIELD_NAME__fieldnames, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__FIELD_NAME__fieldtypes, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__FIELD_NAME__fieldarraylen, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__FIELD_NAME__examples, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__FIELD_NAME__constnames, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__FIELD_NAME__constvalues, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosapi_msgs__msg__TypeDef__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosapi_msgs__msg__TypeDef__TYPE_NAME, 23, 23},
      {rosapi_msgs__msg__TypeDef__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string type\n"
  "string[] fieldnames\n"
  "string[] fieldtypes\n"
  "int32[] fieldarraylen\n"
  "string[] examples\n"
  "string[] constnames\n"
  "string[] constvalues";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosapi_msgs__msg__TypeDef__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosapi_msgs__msg__TypeDef__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 133, 133},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosapi_msgs__msg__TypeDef__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosapi_msgs__msg__TypeDef__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
