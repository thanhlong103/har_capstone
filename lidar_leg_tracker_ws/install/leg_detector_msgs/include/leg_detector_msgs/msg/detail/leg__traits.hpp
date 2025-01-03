// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from leg_detector_msgs:msg/Leg.idl
// generated code does not contain a copyright notice

#ifndef LEG_DETECTOR_MSGS__MSG__DETAIL__LEG__TRAITS_HPP_
#define LEG_DETECTOR_MSGS__MSG__DETAIL__LEG__TRAITS_HPP_

#include "leg_detector_msgs/msg/detail/leg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<leg_detector_msgs::msg::Leg>()
{
  return "leg_detector_msgs::msg::Leg";
}

template<>
inline const char * name<leg_detector_msgs::msg::Leg>()
{
  return "leg_detector_msgs/msg/Leg";
}

template<>
struct has_fixed_size<leg_detector_msgs::msg::Leg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<leg_detector_msgs::msg::Leg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<leg_detector_msgs::msg::Leg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LEG_DETECTOR_MSGS__MSG__DETAIL__LEG__TRAITS_HPP_
