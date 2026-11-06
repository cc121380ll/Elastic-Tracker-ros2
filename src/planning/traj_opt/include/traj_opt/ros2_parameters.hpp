#pragma once
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <type_traits>
namespace elastic_tracker {
#ifndef ELASTIC_TRACKER_REQUIRED_PARAMETER_DEFINED
#define ELASTIC_TRACKER_REQUIRED_PARAMETER_DEFINED
template<class T> void required_parameter(rclcpp::Node& node, std::string name, T& out) {
  std::replace(name.begin(), name.end(), '/', '.');
  if (!node.has_parameter(name)) node.declare_parameter<std::decay_t<T>>(name);
  node.get_parameter(name, out);
}
#endif
}
