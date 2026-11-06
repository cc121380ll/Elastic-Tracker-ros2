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
inline bool optional_vector_parameter(rclcpp::Node& node, const std::string& name, std::vector<double>& out, size_t count) {
 if (!node.has_parameter(name)) node.declare_parameter<std::vector<double>>(name, std::vector<double>{});
 node.get_parameter(name, out); if (out.empty()) return false;
 if(out.size()!=count) throw std::runtime_error("Invalid extrinsic vector length: " + name);
 return true;
}
}

