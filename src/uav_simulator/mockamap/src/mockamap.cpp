#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"

void
optimizeMap(mocka::Maps::BasicInfo& in)
{
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width  = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (int i = 0; i < cloud->width; i++)
  {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

  for (int i = 0; i < cloud->width; i++)
  {
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
    {
      temp->push_back(i);
    }
  }
  for (int i = temp->size() - 1; i >= 0; i--)
  {
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i)); // erasing the enclosed points
  }
  in.cloud->width -= temp->size();

  pcl::toROSMsg(*in.cloud, *in.output);
  in.output->header.frame_id = "world";
  RCLCPP_INFO(rclcpp::get_logger("mockamap"), "finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

namespace { volatile std::sig_atomic_t stopping = 0; void stop_signal(int) { stopping=1; } }
int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal);
  auto node = std::make_shared<rclcpp::Node>("mockamap");

  auto pcl_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "mock_map", rclcpp::QoS(1).reliable().transient_local());
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::msg::PointCloud2       output;
  // Fill in the cloud data

  int seed;

  int sizeX;
  int sizeY;
  int sizeZ;

  double scale;
  double update_freq;

  int type;

  seed = node->declare_parameter("seed", 4546);
  update_freq = node->declare_parameter("update_freq", 1.0);
  scale = node->declare_parameter("resolution", 0.38);
  sizeX = node->declare_parameter("x_length", 100);
  sizeY = node->declare_parameter("y_length", 100);
  sizeZ = node->declare_parameter("z_length", 10);

  type = node->declare_parameter("type", 1);

  scale = 1 / scale;
  sizeX = sizeX * scale;
  sizeY = sizeY * scale;
  sizeZ = sizeZ * scale;

  mocka::Maps::BasicInfo info;
  info.nh_private = node.get();
  info.sizeX      = sizeX;
  info.sizeY      = sizeY;
  info.sizeZ      = sizeZ;
  info.seed       = seed;
  info.scale      = scale;
  info.output     = &output;
  info.cloud      = &cloud;

  mocka::Maps map;
  map.setInfo(info);
  map.generate(type);

  //  optimizeMap(info);

  //! @note publish loop
  if (update_freq <= 0.0) throw std::runtime_error("update_freq must be positive");
  output.header.stamp = node->now();
  pcl_pub->publish(output);
  auto timer = node->create_wall_timer(std::chrono::duration<double>(1.0 / update_freq), [&] {
    output.header.stamp = node->now();
    pcl_pub->publish(output);
  });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stopping && rclcpp::ok()) executor.spin_once(std::chrono::milliseconds(50));
  executor.remove_node(node);
  timer.reset(); pcl_pub.reset(); node.reset();
  rclcpp::shutdown();
  return 0;
}
