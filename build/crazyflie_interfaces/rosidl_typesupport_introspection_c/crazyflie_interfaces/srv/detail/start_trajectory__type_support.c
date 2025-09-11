// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:srv/StartTrajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/srv/detail/start_trajectory__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/srv/detail/start_trajectory__functions.h"
#include "crazyflie_interfaces/srv/detail/start_trajectory__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__srv__StartTrajectory_Request__init(message_memory);
}

void crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_fini_function(void * message_memory)
{
  crazyflie_interfaces__srv__StartTrajectory_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_member_array[5] = {
  {
    "group_mask",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Request, group_mask),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trajectory_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Request, trajectory_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timescale",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Request, timescale),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reversed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Request, reversed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "relative",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Request, relative),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_members = {
  "crazyflie_interfaces__srv",  // message namespace
  "StartTrajectory_Request",  // message name
  5,  // number of fields
  sizeof(crazyflie_interfaces__srv__StartTrajectory_Request),
  false,  // has_any_key_member_
  crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_member_array,  // message members
  crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_members,
  get_message_typesupport_handle_function,
  &crazyflie_interfaces__srv__StartTrajectory_Request__get_type_hash,
  &crazyflie_interfaces__srv__StartTrajectory_Request__get_type_description,
  &crazyflie_interfaces__srv__StartTrajectory_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Request)() {
  if (!crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__rosidl_typesupport_introspection_c.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__functions.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__srv__StartTrajectory_Response__init(message_memory);
}

void crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_fini_function(void * message_memory)
{
  crazyflie_interfaces__srv__StartTrajectory_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_members = {
  "crazyflie_interfaces__srv",  // message namespace
  "StartTrajectory_Response",  // message name
  1,  // number of fields
  sizeof(crazyflie_interfaces__srv__StartTrajectory_Response),
  false,  // has_any_key_member_
  crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_member_array,  // message members
  crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_members,
  get_message_typesupport_handle_function,
  &crazyflie_interfaces__srv__StartTrajectory_Response__get_type_hash,
  &crazyflie_interfaces__srv__StartTrajectory_Response__get_type_description,
  &crazyflie_interfaces__srv__StartTrajectory_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Response)() {
  if (!crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__rosidl_typesupport_introspection_c.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__functions.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "crazyflie_interfaces/srv/start_trajectory.h"
// Member `request`
// Member `response`
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__srv__StartTrajectory_Event__init(message_memory);
}

void crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_fini_function(void * message_memory)
{
  crazyflie_interfaces__srv__StartTrajectory_Event__fini(message_memory);
}

size_t crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__size_function__StartTrajectory_Event__request(
  const void * untyped_member)
{
  const crazyflie_interfaces__srv__StartTrajectory_Request__Sequence * member =
    (const crazyflie_interfaces__srv__StartTrajectory_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_const_function__StartTrajectory_Event__request(
  const void * untyped_member, size_t index)
{
  const crazyflie_interfaces__srv__StartTrajectory_Request__Sequence * member =
    (const crazyflie_interfaces__srv__StartTrajectory_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_function__StartTrajectory_Event__request(
  void * untyped_member, size_t index)
{
  crazyflie_interfaces__srv__StartTrajectory_Request__Sequence * member =
    (crazyflie_interfaces__srv__StartTrajectory_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__fetch_function__StartTrajectory_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const crazyflie_interfaces__srv__StartTrajectory_Request * item =
    ((const crazyflie_interfaces__srv__StartTrajectory_Request *)
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_const_function__StartTrajectory_Event__request(untyped_member, index));
  crazyflie_interfaces__srv__StartTrajectory_Request * value =
    (crazyflie_interfaces__srv__StartTrajectory_Request *)(untyped_value);
  *value = *item;
}

void crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__assign_function__StartTrajectory_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  crazyflie_interfaces__srv__StartTrajectory_Request * item =
    ((crazyflie_interfaces__srv__StartTrajectory_Request *)
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_function__StartTrajectory_Event__request(untyped_member, index));
  const crazyflie_interfaces__srv__StartTrajectory_Request * value =
    (const crazyflie_interfaces__srv__StartTrajectory_Request *)(untyped_value);
  *item = *value;
}

bool crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__resize_function__StartTrajectory_Event__request(
  void * untyped_member, size_t size)
{
  crazyflie_interfaces__srv__StartTrajectory_Request__Sequence * member =
    (crazyflie_interfaces__srv__StartTrajectory_Request__Sequence *)(untyped_member);
  crazyflie_interfaces__srv__StartTrajectory_Request__Sequence__fini(member);
  return crazyflie_interfaces__srv__StartTrajectory_Request__Sequence__init(member, size);
}

size_t crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__size_function__StartTrajectory_Event__response(
  const void * untyped_member)
{
  const crazyflie_interfaces__srv__StartTrajectory_Response__Sequence * member =
    (const crazyflie_interfaces__srv__StartTrajectory_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_const_function__StartTrajectory_Event__response(
  const void * untyped_member, size_t index)
{
  const crazyflie_interfaces__srv__StartTrajectory_Response__Sequence * member =
    (const crazyflie_interfaces__srv__StartTrajectory_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_function__StartTrajectory_Event__response(
  void * untyped_member, size_t index)
{
  crazyflie_interfaces__srv__StartTrajectory_Response__Sequence * member =
    (crazyflie_interfaces__srv__StartTrajectory_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__fetch_function__StartTrajectory_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const crazyflie_interfaces__srv__StartTrajectory_Response * item =
    ((const crazyflie_interfaces__srv__StartTrajectory_Response *)
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_const_function__StartTrajectory_Event__response(untyped_member, index));
  crazyflie_interfaces__srv__StartTrajectory_Response * value =
    (crazyflie_interfaces__srv__StartTrajectory_Response *)(untyped_value);
  *value = *item;
}

void crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__assign_function__StartTrajectory_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  crazyflie_interfaces__srv__StartTrajectory_Response * item =
    ((crazyflie_interfaces__srv__StartTrajectory_Response *)
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_function__StartTrajectory_Event__response(untyped_member, index));
  const crazyflie_interfaces__srv__StartTrajectory_Response * value =
    (const crazyflie_interfaces__srv__StartTrajectory_Response *)(untyped_value);
  *item = *value;
}

bool crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__resize_function__StartTrajectory_Event__response(
  void * untyped_member, size_t size)
{
  crazyflie_interfaces__srv__StartTrajectory_Response__Sequence * member =
    (crazyflie_interfaces__srv__StartTrajectory_Response__Sequence *)(untyped_member);
  crazyflie_interfaces__srv__StartTrajectory_Response__Sequence__fini(member);
  return crazyflie_interfaces__srv__StartTrajectory_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Event, request),  // bytes offset in struct
    NULL,  // default value
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__size_function__StartTrajectory_Event__request,  // size() function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_const_function__StartTrajectory_Event__request,  // get_const(index) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_function__StartTrajectory_Event__request,  // get(index) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__fetch_function__StartTrajectory_Event__request,  // fetch(index, &value) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__assign_function__StartTrajectory_Event__request,  // assign(index, value) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__resize_function__StartTrajectory_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(crazyflie_interfaces__srv__StartTrajectory_Event, response),  // bytes offset in struct
    NULL,  // default value
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__size_function__StartTrajectory_Event__response,  // size() function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_const_function__StartTrajectory_Event__response,  // get_const(index) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__get_function__StartTrajectory_Event__response,  // get(index) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__fetch_function__StartTrajectory_Event__response,  // fetch(index, &value) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__assign_function__StartTrajectory_Event__response,  // assign(index, value) function pointer
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__resize_function__StartTrajectory_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_members = {
  "crazyflie_interfaces__srv",  // message namespace
  "StartTrajectory_Event",  // message name
  3,  // number of fields
  sizeof(crazyflie_interfaces__srv__StartTrajectory_Event),
  false,  // has_any_key_member_
  crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_member_array,  // message members
  crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_members,
  get_message_typesupport_handle_function,
  &crazyflie_interfaces__srv__StartTrajectory_Event__get_type_hash,
  &crazyflie_interfaces__srv__StartTrajectory_Event__get_type_description,
  &crazyflie_interfaces__srv__StartTrajectory_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Event)() {
  crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Request)();
  crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Response)();
  if (!crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_service_members = {
  "crazyflie_interfaces__srv",  // service namespace
  "StartTrajectory",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_type_support_handle,
  NULL,  // response message
  // crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_type_support_handle
  NULL  // event_message
  // crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_type_support_handle
};


static rosidl_service_type_support_t crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_service_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_service_members,
  get_service_typesupport_handle_function,
  &crazyflie_interfaces__srv__StartTrajectory_Request__rosidl_typesupport_introspection_c__StartTrajectory_Request_message_type_support_handle,
  &crazyflie_interfaces__srv__StartTrajectory_Response__rosidl_typesupport_introspection_c__StartTrajectory_Response_message_type_support_handle,
  &crazyflie_interfaces__srv__StartTrajectory_Event__rosidl_typesupport_introspection_c__StartTrajectory_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    crazyflie_interfaces,
    srv,
    StartTrajectory
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    crazyflie_interfaces,
    srv,
    StartTrajectory
  ),
  &crazyflie_interfaces__srv__StartTrajectory__get_type_hash,
  &crazyflie_interfaces__srv__StartTrajectory__get_type_description,
  &crazyflie_interfaces__srv__StartTrajectory__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory)(void) {
  if (!crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_service_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, StartTrajectory_Event)()->data;
  }

  return &crazyflie_interfaces__srv__detail__start_trajectory__rosidl_typesupport_introspection_c__StartTrajectory_service_type_support_handle;
}
