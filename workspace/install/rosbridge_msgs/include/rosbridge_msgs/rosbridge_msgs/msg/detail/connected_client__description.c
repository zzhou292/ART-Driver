// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosbridge_msgs:msg/ConnectedClient.idl
// generated code does not contain a copyright notice

#include "rosbridge_msgs/msg/detail/connected_client__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_msgs
const rosidl_type_hash_t *
rosbridge_msgs__msg__ConnectedClient__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb2, 0xa2, 0x48, 0x94, 0x97, 0xc8, 0x03, 0x9f,
      0xa7, 0x1d, 0x99, 0xfd, 0xce, 0xe6, 0x98, 0x1b,
      0x99, 0xa5, 0x8c, 0xba, 0x2b, 0x8f, 0xd3, 0x52,
      0x7d, 0x85, 0xa2, 0x05, 0xf0, 0x4f, 0xb1, 0xc8,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char rosbridge_msgs__msg__ConnectedClient__TYPE_NAME[] = "rosbridge_msgs/msg/ConnectedClient";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char rosbridge_msgs__msg__ConnectedClient__FIELD_NAME__ip_address[] = "ip_address";
static char rosbridge_msgs__msg__ConnectedClient__FIELD_NAME__connection_time[] = "connection_time";

static rosidl_runtime_c__type_description__Field rosbridge_msgs__msg__ConnectedClient__FIELDS[] = {
  {
    {rosbridge_msgs__msg__ConnectedClient__FIELD_NAME__ip_address, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_msgs__msg__ConnectedClient__FIELD_NAME__connection_time, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosbridge_msgs__msg__ConnectedClient__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_msgs__msg__ConnectedClient__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_msgs__msg__ConnectedClient__TYPE_NAME, 34, 34},
      {rosbridge_msgs__msg__ConnectedClient__FIELDS, 2, 2},
    },
    {rosbridge_msgs__msg__ConnectedClient__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string ip_address\n"
  "builtin_interfaces/Time connection_time";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_msgs__msg__ConnectedClient__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_msgs__msg__ConnectedClient__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 58, 58},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_msgs__msg__ConnectedClient__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_msgs__msg__ConnectedClient__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
