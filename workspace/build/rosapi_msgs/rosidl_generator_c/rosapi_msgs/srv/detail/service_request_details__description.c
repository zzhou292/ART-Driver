// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosapi_msgs:srv/ServiceRequestDetails.idl
// generated code does not contain a copyright notice

#include "rosapi_msgs/srv/detail/service_request_details__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosapi_msgs
const rosidl_type_hash_t *
rosapi_msgs__srv__ServiceRequestDetails__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xec, 0x8a, 0x46, 0xb5, 0x82, 0x97, 0xac, 0x8d,
      0x24, 0xaa, 0x82, 0x08, 0xad, 0x3b, 0x20, 0xeb,
      0xbe, 0x75, 0x37, 0x1d, 0xdd, 0xe5, 0x0e, 0x54,
      0xa6, 0x52, 0xd6, 0x65, 0xa0, 0x53, 0x4b, 0xa9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_rosapi_msgs
const rosidl_type_hash_t *
rosapi_msgs__srv__ServiceRequestDetails_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc0, 0x61, 0x6c, 0xc4, 0xe0, 0xb5, 0x08, 0x3f,
      0x30, 0x48, 0xb0, 0x28, 0xe7, 0x48, 0x6c, 0xd0,
      0x01, 0xc1, 0xe6, 0xe7, 0xa4, 0x1c, 0x90, 0xb1,
      0xae, 0xfa, 0x52, 0xf1, 0x73, 0xe9, 0x1f, 0x45,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_rosapi_msgs
const rosidl_type_hash_t *
rosapi_msgs__srv__ServiceRequestDetails_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3b, 0xf2, 0xc6, 0x24, 0xdb, 0x55, 0x8e, 0xde,
      0xe9, 0xfd, 0x94, 0x7c, 0x8b, 0x9e, 0x07, 0xa3,
      0xf9, 0xf3, 0xc3, 0x18, 0xaf, 0x19, 0x61, 0x08,
      0xe8, 0xd5, 0x37, 0xf3, 0x01, 0x90, 0xe1, 0xdd,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_rosapi_msgs
const rosidl_type_hash_t *
rosapi_msgs__srv__ServiceRequestDetails_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd9, 0xe4, 0x4f, 0x44, 0x01, 0xab, 0x56, 0x2c,
      0xe3, 0x9e, 0x3c, 0x1a, 0x3e, 0x35, 0x76, 0x98,
      0x76, 0x3e, 0x93, 0xa0, 0x1d, 0x61, 0xa3, 0xff,
      0xe4, 0xa2, 0xf9, 0x21, 0x70, 0xa8, 0xef, 0x20,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "rosapi_msgs/msg/detail/type_def__functions.h"
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
static const rosidl_type_hash_t rosapi_msgs__msg__TypeDef__EXPECTED_HASH = {1, {
    0xdc, 0xd2, 0x54, 0x91, 0x39, 0xd3, 0xfd, 0xb4,
    0x32, 0xe7, 0xf9, 0x25, 0xfe, 0x79, 0xe9, 0xfb,
    0x4f, 0x02, 0x88, 0x9c, 0x79, 0xbe, 0x45, 0xf7,
    0x48, 0x2c, 0x34, 0x67, 0x06, 0x73, 0x49, 0x28,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char rosapi_msgs__srv__ServiceRequestDetails__TYPE_NAME[] = "rosapi_msgs/srv/ServiceRequestDetails";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char rosapi_msgs__msg__TypeDef__TYPE_NAME[] = "rosapi_msgs/msg/TypeDef";
static char rosapi_msgs__srv__ServiceRequestDetails_Event__TYPE_NAME[] = "rosapi_msgs/srv/ServiceRequestDetails_Event";
static char rosapi_msgs__srv__ServiceRequestDetails_Request__TYPE_NAME[] = "rosapi_msgs/srv/ServiceRequestDetails_Request";
static char rosapi_msgs__srv__ServiceRequestDetails_Response__TYPE_NAME[] = "rosapi_msgs/srv/ServiceRequestDetails_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char rosapi_msgs__srv__ServiceRequestDetails__FIELD_NAME__request_message[] = "request_message";
static char rosapi_msgs__srv__ServiceRequestDetails__FIELD_NAME__response_message[] = "response_message";
static char rosapi_msgs__srv__ServiceRequestDetails__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field rosapi_msgs__srv__ServiceRequestDetails__FIELDS[] = {
  {
    {rosapi_msgs__srv__ServiceRequestDetails__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosapi_msgs__srv__ServiceRequestDetails_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosapi_msgs__srv__ServiceRequestDetails_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosapi_msgs__srv__ServiceRequestDetails_Event__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosapi_msgs__srv__ServiceRequestDetails__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Event__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosapi_msgs__srv__ServiceRequestDetails__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosapi_msgs__srv__ServiceRequestDetails__TYPE_NAME, 37, 37},
      {rosapi_msgs__srv__ServiceRequestDetails__FIELDS, 3, 3},
    },
    {rosapi_msgs__srv__ServiceRequestDetails__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosapi_msgs__msg__TypeDef__EXPECTED_HASH, rosapi_msgs__msg__TypeDef__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = rosapi_msgs__msg__TypeDef__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = rosapi_msgs__srv__ServiceRequestDetails_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = rosapi_msgs__srv__ServiceRequestDetails_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = rosapi_msgs__srv__ServiceRequestDetails_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char rosapi_msgs__srv__ServiceRequestDetails_Request__FIELD_NAME__type[] = "type";

static rosidl_runtime_c__type_description__Field rosapi_msgs__srv__ServiceRequestDetails_Request__FIELDS[] = {
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Request__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosapi_msgs__srv__ServiceRequestDetails_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosapi_msgs__srv__ServiceRequestDetails_Request__TYPE_NAME, 45, 45},
      {rosapi_msgs__srv__ServiceRequestDetails_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char rosapi_msgs__srv__ServiceRequestDetails_Response__FIELD_NAME__typedefs[] = "typedefs";

static rosidl_runtime_c__type_description__Field rosapi_msgs__srv__ServiceRequestDetails_Response__FIELDS[] = {
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Response__FIELD_NAME__typedefs, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosapi_msgs__msg__TypeDef__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosapi_msgs__srv__ServiceRequestDetails_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {rosapi_msgs__msg__TypeDef__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosapi_msgs__srv__ServiceRequestDetails_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosapi_msgs__srv__ServiceRequestDetails_Response__TYPE_NAME, 46, 46},
      {rosapi_msgs__srv__ServiceRequestDetails_Response__FIELDS, 1, 1},
    },
    {rosapi_msgs__srv__ServiceRequestDetails_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&rosapi_msgs__msg__TypeDef__EXPECTED_HASH, rosapi_msgs__msg__TypeDef__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = rosapi_msgs__msg__TypeDef__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char rosapi_msgs__srv__ServiceRequestDetails_Event__FIELD_NAME__info[] = "info";
static char rosapi_msgs__srv__ServiceRequestDetails_Event__FIELD_NAME__request[] = "request";
static char rosapi_msgs__srv__ServiceRequestDetails_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field rosapi_msgs__srv__ServiceRequestDetails_Event__FIELDS[] = {
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {rosapi_msgs__srv__ServiceRequestDetails_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {rosapi_msgs__srv__ServiceRequestDetails_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosapi_msgs__srv__ServiceRequestDetails_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__msg__TypeDef__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {rosapi_msgs__srv__ServiceRequestDetails_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosapi_msgs__srv__ServiceRequestDetails_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosapi_msgs__srv__ServiceRequestDetails_Event__TYPE_NAME, 43, 43},
      {rosapi_msgs__srv__ServiceRequestDetails_Event__FIELDS, 3, 3},
    },
    {rosapi_msgs__srv__ServiceRequestDetails_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosapi_msgs__msg__TypeDef__EXPECTED_HASH, rosapi_msgs__msg__TypeDef__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = rosapi_msgs__msg__TypeDef__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = rosapi_msgs__srv__ServiceRequestDetails_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = rosapi_msgs__srv__ServiceRequestDetails_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string type\n"
  "---\n"
  "TypeDef[] typedefs";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosapi_msgs__srv__ServiceRequestDetails__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosapi_msgs__srv__ServiceRequestDetails__TYPE_NAME, 37, 37},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 35, 35},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
rosapi_msgs__srv__ServiceRequestDetails_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosapi_msgs__srv__ServiceRequestDetails_Request__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
rosapi_msgs__srv__ServiceRequestDetails_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosapi_msgs__srv__ServiceRequestDetails_Response__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
rosapi_msgs__srv__ServiceRequestDetails_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosapi_msgs__srv__ServiceRequestDetails_Event__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosapi_msgs__srv__ServiceRequestDetails__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosapi_msgs__srv__ServiceRequestDetails__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *rosapi_msgs__msg__TypeDef__get_individual_type_description_source(NULL);
    sources[3] = *rosapi_msgs__srv__ServiceRequestDetails_Event__get_individual_type_description_source(NULL);
    sources[4] = *rosapi_msgs__srv__ServiceRequestDetails_Request__get_individual_type_description_source(NULL);
    sources[5] = *rosapi_msgs__srv__ServiceRequestDetails_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosapi_msgs__srv__ServiceRequestDetails_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosapi_msgs__srv__ServiceRequestDetails_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosapi_msgs__srv__ServiceRequestDetails_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosapi_msgs__srv__ServiceRequestDetails_Response__get_individual_type_description_source(NULL),
    sources[1] = *rosapi_msgs__msg__TypeDef__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosapi_msgs__srv__ServiceRequestDetails_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosapi_msgs__srv__ServiceRequestDetails_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *rosapi_msgs__msg__TypeDef__get_individual_type_description_source(NULL);
    sources[3] = *rosapi_msgs__srv__ServiceRequestDetails_Request__get_individual_type_description_source(NULL);
    sources[4] = *rosapi_msgs__srv__ServiceRequestDetails_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
