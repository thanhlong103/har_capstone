// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from leg_detector_msgs:msg/PersonArray.idl
// generated code does not contain a copyright notice

#ifndef LEG_DETECTOR_MSGS__MSG__DETAIL__PERSON_ARRAY__TRAITS_HPP_
#define LEG_DETECTOR_MSGS__MSG__DETAIL__PERSON_ARRAY__TRAITS_HPP_

#include "leg_detector_msgs/msg/detail/person_array__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<leg_detector_msgs::msg::PersonArray>()
{
  return "leg_detector_msgs::msg::PersonArray";
}

template<>
inline const char * name<leg_detector_msgs::msg::PersonArray>()
{
  return "leg_detector_msgs/msg/PersonArray";
}

template<>
struct has_fixed_size<leg_detector_msgs::msg::PersonArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<leg_detector_msgs::msg::PersonArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<leg_detector_msgs::msg::PersonArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LEG_DETECTOR_MSGS__MSG__DETAIL__PERSON_ARRAY__TRAITS_HPP_
