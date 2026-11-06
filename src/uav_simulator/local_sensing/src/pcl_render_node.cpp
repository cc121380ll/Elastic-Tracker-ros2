#include <fstream>
#include <iostream>
#include <vector>
//include ros dep.
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

//include pcl dep
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//include opencv and eigen
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "opencv2/highgui/highgui.hpp"

//#include <cloud_banchmark/cloud_banchmarkConfig.h>
#include "depth_render.cuh"

using namespace cv;
using namespace std;
using namespace Eigen;

int *depth_hostptr;
cv::Mat depth_mat;

//camera param
int width, height;
double fx, fy, cx, cy;

DepthRender depthrender;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_wolrd;

sensor_msgs::msg::PointCloud2 local_map_pcl;
sensor_msgs::msg::PointCloud2 local_depth_pcl;

rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub, local_map_sub;

rclcpp::TimerBase::SharedPtr local_sensing_timer, estimation_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

Matrix4d cam02body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::msg::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;

builtin_interfaces::msg::Time last_odom_stamp;
Eigen::Vector3d last_pose_world;

void render_currentpose();
void render_pcl_world();

void rcvOdometryCallbck(const nav_msgs::msg::Odometry &odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  //convert to cam pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);

  last_odom_stamp = odom.header.stamp;

  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;
}

void pubCameraPose() {
  if (!has_odom) return;
  //cout<<"pub cam pose"
  geometry_msgs::msg::PoseStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "world";
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  pub_pose->publish(camera_pose);
}

void renderSensedPoints() {
  //if(! has_global_map || ! has_odom) return;
  if (!has_global_map && !has_local_map) return;

  if (!has_odom) return;
  render_currentpose();
  render_pcl_world();
}

vector<float> cloud_data;
void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2 &pointcloud_map) {
  if (has_global_map)
    return;

  RCLCPP_WARN(rclcpp::get_logger("pcl_render"), "Global Pointcloud received..");
  //load global map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  //transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  for (int i = 0; i < int(cloudIn.points.size()); i++) {
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }
  printf("global map has points: %d.\n", (int)cloud_data.size() / 3);
  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  if (!depth_hostptr) depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_global_map = true;
  // RCLCPP_ERROR(rclcpp::get_logger("pcl_render"), "GLOBAL!!!!");
}

void rcvLocalPointCloudCallBack(const sensor_msgs::msg::PointCloud2 &pointcloud_map) {
  //RCLCPP_WARN(rclcpp::get_logger("pcl_render"), "Local Pointcloud received..");
  //load local map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  //transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  if (cloudIn.points.size() == 0) return;
  for (int i = 0; i < int(cloudIn.points.size()); i++) {
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  //printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );
  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  if (!depth_hostptr) depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_local_map = true;
}

void render_pcl_world() {
  //for debug purpose
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;

  Eigen::Vector4d pose_in_camera;
  Eigen::Vector4d pose_in_world;
  Eigen::Vector3d pose_pt;

  for (int u = 0; u < width; u++)
    for (int v = 0; v < height; v++) {
      float depth = depth_mat.at<float>(v, u);

      if (depth == 0.0)
        continue;

      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth;
      pose_in_camera(3) = 1.0;

      pose_in_world = cam2world * pose_in_camera;

      if ((pose_in_world.segment(0, 3) - last_pose_world).norm() > sensing_horizon)
        continue;

      pose_pt = pose_in_world.head(3);
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);

      localMap.points.push_back(pt_in);
    }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, local_map_pcl);
  local_map_pcl.header.frame_id = "world";
  local_map_pcl.header.stamp = last_odom_stamp;

  pub_pcl_wolrd->publish(local_map_pcl);
}

void render_currentpose() {
  

  Matrix4d cam_pose = cam2world.inverse();

  double pose[4 * 4];

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      pose[j + 4 * i] = cam_pose(i, j);

  depthrender.render_pose(pose, depth_hostptr);
  //depthrender.render_pose(cam_pose, depth_hostptr);

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) {
      float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
      depth = depth < 500.0f ? depth : 0;
      max = depth > max ? depth : max;
      depth_mat.at<float>(i, j) = depth;
    }
  //RCLCPP_INFO(rclcpp::get_logger("pcl_render"), "render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  //printf("max_depth %lf.\n", max);

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth->publish(*out_msg.toImageMsg());

  cv::Mat adjMap;
  // depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
  depth_mat.convertTo(adjMap, CV_8UC1, 255 / 13.0, -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color->publish(*cv_image_colored.toImageMsg());
  //cv::imshow("depth_image", adjMap);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("pcl_render");

  width = nh->declare_parameter<decltype(width)>("cam_width");
  height = nh->declare_parameter<decltype(height)>("cam_height");
  fx = nh->declare_parameter<decltype(fx)>("cam_fx");
  fy = nh->declare_parameter<decltype(fy)>("cam_fy");
  cx = nh->declare_parameter<decltype(cx)>("cam_cx");
  cy = nh->declare_parameter<decltype(cy)>("cam_cy");
  sensing_horizon = nh->declare_parameter<decltype(sensing_horizon)>("sensing_horizon");
  sensing_rate = nh->declare_parameter<decltype(sensing_rate)>("sensing_rate");
  estimation_rate = nh->declare_parameter<decltype(estimation_rate)>("estimation_rate");

  depthrender.set_para(fx, fy, cx, cy, width, height);

  // cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //               0.0, 0.0, 0.0, 1.0;

  cam02body << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  //init cam2world transformation
  cam2world = Matrix4d::Identity();
  //subscribe point cloud
  global_map_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/global_map", rclcpp::QoS(1).reliable().transient_local(), rcvGlobalPointCloudCallBack);
  local_map_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/local_map", rclcpp::QoS(1).reliable(), rcvLocalPointCloudCallBack);
  odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>(
    "~/odometry", rclcpp::QoS(50).reliable(), rcvOdometryCallbck);

  pub_depth = nh->create_publisher<sensor_msgs::msg::Image>("~/depth", rclcpp::QoS(5).reliable());
  pub_color = nh->create_publisher<sensor_msgs::msg::Image>("~/colordepth", rclcpp::QoS(5).reliable());
  pub_pose = nh->create_publisher<geometry_msgs::msg::PoseStamped>("~/camera_pose", rclcpp::QoS(5).reliable());
  pub_pcl_wolrd = nh->create_publisher<sensor_msgs::msg::PointCloud2>("~/rendered_pcl", rclcpp::QoS(1).reliable());

  if (width <= 0 || height <= 0 || sensing_rate <= 0 || estimation_rate <= 0)
    throw std::runtime_error("Invalid camera dimensions or sensing/estimation rate");
  local_sensing_timer = nh->create_wall_timer(std::chrono::duration<double>(1.0 / sensing_rate), renderSensedPoints);
  estimation_timer = nh->create_wall_timer(std::chrono::duration<double>(1.0 / estimation_rate), pubCameraPose);
  //cv::namedWindow("depth_image",1);

  // All callbacks are serialized: map upload, odometry and CUDA rendering share state.
  rclcpp::spin(nh);
  local_sensing_timer.reset();
  estimation_timer.reset();
  odom_sub.reset();
  global_map_sub.reset();
  local_map_sub.reset();
  pub_depth.reset();
  pub_color.reset();
  pub_pose.reset();
  pub_pcl_wolrd.reset();
  nh.reset();
  free(depth_hostptr);
  depth_hostptr = nullptr;
  rclcpp::shutdown();
  return 0;
}
