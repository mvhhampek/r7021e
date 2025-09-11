// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/SetGroupMask.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_interfaces/srv/set_group_mask.hpp"


#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__SET_GROUP_MASK__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__SET_GROUP_MASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/set_group_mask__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetGroupMask_Request_group_mask
{
public:
  Init_SetGroupMask_Request_group_mask()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crazyflie_interfaces::srv::SetGroupMask_Request group_mask(::crazyflie_interfaces::srv::SetGroupMask_Request::_group_mask_type arg)
  {
    msg_.group_mask = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::SetGroupMask_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::SetGroupMask_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_SetGroupMask_Request_group_mask();
}

}  // namespace crazyflie_interfaces


namespace crazyflie_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::SetGroupMask_Response>()
{
  return ::crazyflie_interfaces::srv::SetGroupMask_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace crazyflie_interfaces


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetGroupMask_Event_response
{
public:
  explicit Init_SetGroupMask_Event_response(::crazyflie_interfaces::srv::SetGroupMask_Event & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::srv::SetGroupMask_Event response(::crazyflie_interfaces::srv::SetGroupMask_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::SetGroupMask_Event msg_;
};

class Init_SetGroupMask_Event_request
{
public:
  explicit Init_SetGroupMask_Event_request(::crazyflie_interfaces::srv::SetGroupMask_Event & msg)
  : msg_(msg)
  {}
  Init_SetGroupMask_Event_response request(::crazyflie_interfaces::srv::SetGroupMask_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetGroupMask_Event_response(msg_);
  }

private:
  ::crazyflie_interfaces::srv::SetGroupMask_Event msg_;
};

class Init_SetGroupMask_Event_info
{
public:
  Init_SetGroupMask_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetGroupMask_Event_request info(::crazyflie_interfaces::srv::SetGroupMask_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetGroupMask_Event_request(msg_);
  }

private:
  ::crazyflie_interfaces::srv::SetGroupMask_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::SetGroupMask_Event>()
{
  return crazyflie_interfaces::srv::builder::Init_SetGroupMask_Event_info();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__SET_GROUP_MASK__BUILDER_HPP_
