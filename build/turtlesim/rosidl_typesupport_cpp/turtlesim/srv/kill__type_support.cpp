// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from turtlesim:srv/Kill.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "turtlesim/srv/detail/kill__functions.h"
#include "turtlesim/srv/detail/kill__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace turtlesim
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Kill_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Kill_Request_type_support_ids_t;

static const _Kill_Request_type_support_ids_t _Kill_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Kill_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Kill_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Kill_Request_type_support_symbol_names_t _Kill_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlesim, srv, Kill_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, turtlesim, srv, Kill_Request)),
  }
};

typedef struct _Kill_Request_type_support_data_t
{
  void * data[2];
} _Kill_Request_type_support_data_t;

static _Kill_Request_type_support_data_t _Kill_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Kill_Request_message_typesupport_map = {
  2,
  "turtlesim",
  &_Kill_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Kill_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Kill_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Kill_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Kill_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &turtlesim__srv__Kill_Request__get_type_hash,
  &turtlesim__srv__Kill_Request__get_type_description,
  &turtlesim__srv__Kill_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace turtlesim

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<turtlesim::srv::Kill_Request>()
{
  return &::turtlesim::srv::rosidl_typesupport_cpp::Kill_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, turtlesim, srv, Kill_Request)() {
  return get_message_type_support_handle<turtlesim::srv::Kill_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlesim/srv/detail/kill__functions.h"
// already included above
// #include "turtlesim/srv/detail/kill__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlesim
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Kill_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Kill_Response_type_support_ids_t;

static const _Kill_Response_type_support_ids_t _Kill_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Kill_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Kill_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Kill_Response_type_support_symbol_names_t _Kill_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlesim, srv, Kill_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, turtlesim, srv, Kill_Response)),
  }
};

typedef struct _Kill_Response_type_support_data_t
{
  void * data[2];
} _Kill_Response_type_support_data_t;

static _Kill_Response_type_support_data_t _Kill_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Kill_Response_message_typesupport_map = {
  2,
  "turtlesim",
  &_Kill_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Kill_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Kill_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Kill_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Kill_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &turtlesim__srv__Kill_Response__get_type_hash,
  &turtlesim__srv__Kill_Response__get_type_description,
  &turtlesim__srv__Kill_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace turtlesim

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<turtlesim::srv::Kill_Response>()
{
  return &::turtlesim::srv::rosidl_typesupport_cpp::Kill_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, turtlesim, srv, Kill_Response)() {
  return get_message_type_support_handle<turtlesim::srv::Kill_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "turtlesim/srv/detail/kill__functions.h"
// already included above
// #include "turtlesim/srv/detail/kill__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlesim
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Kill_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Kill_Event_type_support_ids_t;

static const _Kill_Event_type_support_ids_t _Kill_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Kill_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Kill_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Kill_Event_type_support_symbol_names_t _Kill_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlesim, srv, Kill_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, turtlesim, srv, Kill_Event)),
  }
};

typedef struct _Kill_Event_type_support_data_t
{
  void * data[2];
} _Kill_Event_type_support_data_t;

static _Kill_Event_type_support_data_t _Kill_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Kill_Event_message_typesupport_map = {
  2,
  "turtlesim",
  &_Kill_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Kill_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Kill_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Kill_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Kill_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &turtlesim__srv__Kill_Event__get_type_hash,
  &turtlesim__srv__Kill_Event__get_type_description,
  &turtlesim__srv__Kill_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace turtlesim

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<turtlesim::srv::Kill_Event>()
{
  return &::turtlesim::srv::rosidl_typesupport_cpp::Kill_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, turtlesim, srv, Kill_Event)() {
  return get_message_type_support_handle<turtlesim::srv::Kill_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "turtlesim/srv/detail/kill__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace turtlesim
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _Kill_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Kill_type_support_ids_t;

static const _Kill_type_support_ids_t _Kill_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Kill_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Kill_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Kill_type_support_symbol_names_t _Kill_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlesim, srv, Kill)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, turtlesim, srv, Kill)),
  }
};

typedef struct _Kill_type_support_data_t
{
  void * data[2];
} _Kill_type_support_data_t;

static _Kill_type_support_data_t _Kill_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Kill_service_typesupport_map = {
  2,
  "turtlesim",
  &_Kill_service_typesupport_ids.typesupport_identifier[0],
  &_Kill_service_typesupport_symbol_names.symbol_name[0],
  &_Kill_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Kill_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Kill_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<turtlesim::srv::Kill_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<turtlesim::srv::Kill_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<turtlesim::srv::Kill_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<turtlesim::srv::Kill>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<turtlesim::srv::Kill>,
  &turtlesim__srv__Kill__get_type_hash,
  &turtlesim__srv__Kill__get_type_description,
  &turtlesim__srv__Kill__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace turtlesim

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<turtlesim::srv::Kill>()
{
  return &::turtlesim::srv::rosidl_typesupport_cpp::Kill_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, turtlesim, srv, Kill)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<turtlesim::srv::Kill>();
}

#ifdef __cplusplus
}
#endif
