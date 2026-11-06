#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <fstream>
namespace wr_msg {
template<class MSG, class FILENAME> void writeMsg(const MSG& msg, const FILENAME& name) {
 rclcpp::SerializedMessage bytes; rclcpp::Serialization<MSG> serializer;
 serializer.serialize_message(&msg, &bytes);
 std::ofstream out(name, std::ios::binary); if (!out) throw std::runtime_error("Cannot write ROS2 CDR debug file");
 auto& data=bytes.get_rcl_serialized_message(); out.write(reinterpret_cast<const char*>(data.buffer), data.buffer_length);
}
template<class MSG, class FILENAME> void readMsg(MSG& msg, const FILENAME& name) {
 std::ifstream in(name, std::ios::binary|std::ios::ate); if (!in) throw std::runtime_error("Cannot read ROS2 CDR debug file");
 auto size=in.tellg(); in.seekg(0); rclcpp::SerializedMessage bytes(static_cast<size_t>(size));
 auto& data=bytes.get_rcl_serialized_message(); in.read(reinterpret_cast<char*>(data.buffer),size); data.buffer_length=static_cast<size_t>(size);
 rclcpp::Serialization<MSG> serializer; serializer.deserialize_message(&bytes,&msg);
}
}
