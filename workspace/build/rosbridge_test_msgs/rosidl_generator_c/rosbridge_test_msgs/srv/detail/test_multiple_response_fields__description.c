// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosbridge_test_msgs:srv/TestMultipleResponseFields.idl
// generated code does not contain a copyright notice

#include "rosbridge_test_msgs/srv/detail/test_multiple_response_fields__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__srv__TestMultipleResponseFields__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x42, 0x82, 0x46, 0x3c, 0xbb, 0x12, 0x30, 0x43,
      0x3c, 0x2c, 0xa7, 0xbf, 0x4f, 0xb8, 0x6e, 0xe1,
      0x48, 0xaa, 0x3a, 0xd0, 0x28, 0x95, 0xfe, 0x05,
      0xa0, 0xca, 0x8f, 0x54, 0x2a, 0x99, 0x95, 0xa8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x21, 0x15, 0xd5, 0x4b, 0x57, 0xad, 0x0e, 0x31,
      0x24, 0xb6, 0xf8, 0xa6, 0x43, 0xc7, 0xc0, 0xc8,
      0xc1, 0x7e, 0x56, 0x7d, 0x08, 0x05, 0x1e, 0xb3,
      0x9b, 0x70, 0xbc, 0x43, 0x90, 0x3e, 0x9c, 0xa0,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x75, 0x00, 0xb0, 0x3a, 0xfe, 0x18, 0x49, 0xb1,
      0x52, 0xb5, 0x3b, 0x20, 0x3a, 0x76, 0x0c, 0x7e,
      0xf7, 0x36, 0x84, 0x3a, 0x50, 0xb0, 0x7b, 0x3f,
      0x5e, 0xcb, 0x15, 0xbf, 0xdd, 0x41, 0x25, 0x5b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_rosbridge_test_msgs
const rosidl_type_hash_t *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x26, 0x84, 0x92, 0xe2, 0x40, 0x48, 0xd9, 0xe3,
      0x1b, 0x4c, 0x9b, 0x0a, 0xcb, 0xba, 0x78, 0x85,
      0x98, 0x46, 0x3e, 0x59, 0x31, 0x0c, 0xc9, 0x91,
      0x0d, 0x68, 0xbd, 0xc4, 0x54, 0x0a, 0x9e, 0xb9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char rosbridge_test_msgs__srv__TestMultipleResponseFields__TYPE_NAME[] = "rosbridge_test_msgs/srv/TestMultipleResponseFields";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__TYPE_NAME[] = "rosbridge_test_msgs/srv/TestMultipleResponseFields_Event";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__TYPE_NAME[] = "rosbridge_test_msgs/srv/TestMultipleResponseFields_Request";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__TYPE_NAME[] = "rosbridge_test_msgs/srv/TestMultipleResponseFields_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELD_NAME__request_message[] = "request_message";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELD_NAME__response_message[] = "response_message";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELDS[] = {
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__TYPE_NAME, 58, 58},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__TYPE_NAME, 59, 59},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__TYPE_NAME, 56, 56},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosbridge_test_msgs__srv__TestMultipleResponseFields__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__TYPE_NAME, 56, 56},
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__TYPE_NAME, 58, 58},
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__TYPE_NAME, 59, 59},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__srv__TestMultipleResponseFields__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__srv__TestMultipleResponseFields__TYPE_NAME, 50, 50},
      {rosbridge_test_msgs__srv__TestMultipleResponseFields__FIELDS, 3, 3},
    },
    {rosbridge_test_msgs__srv__TestMultipleResponseFields__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__FIELDS[] = {
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__TYPE_NAME, 58, 58},
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__int_value[] = "int_value";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__float_value[] = "float_value";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__string[] = "string";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__bool_value[] = "bool_value";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELDS[] = {
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__int_value, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__float_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__string, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELD_NAME__bool_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__TYPE_NAME, 59, 59},
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELD_NAME__info[] = "info";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELD_NAME__request[] = "request";
static char rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELDS[] = {
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__TYPE_NAME, 58, 58},
    },
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__TYPE_NAME, 59, 59},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__TYPE_NAME, 58, 58},
    {NULL, 0, 0},
  },
  {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__TYPE_NAME, 59, 59},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__TYPE_NAME, 56, 56},
      {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__FIELDS, 3, 3},
    },
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "int32 int_value\n"
  "float32 float_value\n"
  "string string\n"
  "bool bool_value";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__srv__TestMultipleResponseFields__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields__TYPE_NAME, 50, 50},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 70, 70},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__TYPE_NAME, 58, 58},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__TYPE_NAME, 59, 59},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__TYPE_NAME, 56, 56},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__srv__TestMultipleResponseFields__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__srv__TestMultipleResponseFields__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__get_individual_type_description_source(NULL);
    sources[3] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_individual_type_description_source(NULL);
    sources[4] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Request__get_individual_type_description_source(NULL);
    sources[3] = *rosbridge_test_msgs__srv__TestMultipleResponseFields_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
