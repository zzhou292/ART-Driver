// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosbridge_test_msgs:msg/TestTimeArray.idl
// generated code does not contain a copyright notice

#include "rosbridge_test_msgs/msg/detail/test_time_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__msg__TestTimeArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5d, 0xed, 0xce, 0x51, 0x5d, 0x06, 0x24, 0x69,
      0x42, 0xca, 0xec, 0x3e, 0x85, 0x30, 0xa6, 0xe3,
      0x62, 0xfb, 0xa6, 0x3f, 0xdf, 0x04, 0xd9, 0xf7,
      0xdc, 0xce, 0x88, 0xc3, 0xc6, 0x11, 0x4e, 0xe0,
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

static char rosbridge_test_msgs__msg__TestTimeArray__TYPE_NAME[] = "rosbridge_test_msgs/msg/TestTimeArray";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char rosbridge_test_msgs__msg__TestTimeArray__FIELD_NAME__times[] = "times";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__msg__TestTimeArray__FIELDS[] = {
  {
    {rosbridge_test_msgs__msg__TestTimeArray__FIELD_NAME__times, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosbridge_test_msgs__msg__TestTimeArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__msg__TestTimeArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__msg__TestTimeArray__TYPE_NAME, 37, 37},
      {rosbridge_test_msgs__msg__TestTimeArray__FIELDS, 1, 1},
    },
    {rosbridge_test_msgs__msg__TestTimeArray__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "builtin_interfaces/Time[] times";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__msg__TestTimeArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__msg__TestTimeArray__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 32, 32},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__msg__TestTimeArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__msg__TestTimeArray__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
