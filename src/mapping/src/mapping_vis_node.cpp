#include <csignal>
#include <mapping/mapping.h>
#include <quadrotor_interfaces/msg/occ_map3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapping/ros2_parameters.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gridmap_vs_pub, gridmap_inflate_vs_pub;
bool remove_floor_ceil_ = false;

void gridmap_callback(const quadrotor_interfaces::msg::OccMap3d::ConstSharedPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  gridmap.from_msg(*msgPtr);
  sensor_msgs::msg::PointCloud2 pc;
  gridmap.occ2pc(pc);
  pc.header.frame_id = "world";
  gridmap_vs_pub->publish(pc);
}
void gridmap_inflate_callback(const quadrotor_interfaces::msg::OccMap3d::ConstSharedPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  gridmap.from_msg(*msgPtr);
  sensor_msgs::msg::PointCloud2 pc;
  if (remove_floor_ceil_) {
    gridmap.occ2pc(pc, 0.5, 2.5);
  } else {
    gridmap.occ2pc(pc);
  }
  pc.header.frame_id = "world";
  gridmap_inflate_vs_pub->publish(pc);
}

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
}

int main(int argc, char** argv) {
  // Keep the context valid until initialization and any executing callback finish.
  // The signal handler only writes an async-signal-safe flag; it calls no ROS API.
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<rclcpp::Node>("mapping_vis");
  auto& nh = *node;
  remove_floor_ceil_ = nh.declare_parameter("remove_floor_ceil", false);
  gridmap_vs_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("~/vs_gridmap", rclcpp::QoS(1).reliable().transient_local());
  gridmap_inflate_vs_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("~/vs_gridmap_inflate", rclcpp::QoS(1).reliable().transient_local());
  auto gridmap_sub = nh.create_subscription<quadrotor_interfaces::msg::OccMap3d>("~/gridmap", rclcpp::QoS(1).reliable().transient_local(), gridmap_callback);
  auto gridmap_inflate_sub = nh.create_subscription<quadrotor_interfaces::msg::OccMap3d>("~/gridmap_inflate", rclcpp::QoS(1).reliable().transient_local(), gridmap_inflate_callback);

  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    executor.remove_node(node);
  }
  // Release DDS entities before process-static RMW/plugin teardown.
  gridmap_sub.reset();
  gridmap_inflate_sub.reset();
  gridmap_vs_pub.reset();
  gridmap_inflate_vs_pub.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}

