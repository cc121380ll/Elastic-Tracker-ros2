#include <string.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
#include <iostream>

#include "armadillo"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pose_utils.h"
#include "quadrotor_interfaces/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace arma;
using namespace std;

static string mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale;
bool cross_config = false;
bool tf45 = false;
bool cov_pos = false;
bool cov_vel = false;
bool cov_color = false;
bool origin = false;
bool isOriginSet = false;
colvec poseOrigin(6);
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr covVelPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensorPub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr heightPub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fov_pub_;
tf2_ros::TransformBroadcaster* broadcaster;
rclcpp::Node::SharedPtr node;
std::string tf_prefix;
std::string local_frame(std::string name) {
  while (!name.empty() && name.front() == '/') name.erase(0, 1);
  return tf_prefix.empty() ? name : tf_prefix + "/" + name;
}
void send_transform(const tf2::Transform& value, const builtin_interfaces::msg::Time& stamp,
                    const std::string& parent, const std::string& child) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent == "world" ? parent : local_frame(parent);
  msg.child_frame_id = local_frame(child);
  msg.transform = tf2::toMsg(value);
  broadcaster->sendTransform(msg);
}
geometry_msgs::msg::PoseStamped poseROS;
nav_msgs::msg::Path pathROS;
visualization_msgs::msg::Marker velROS;
visualization_msgs::msg::Marker covROS;
visualization_msgs::msg::Marker covVelROS;
visualization_msgs::msg::Marker trajROS;
visualization_msgs::msg::Marker sensorROS;
visualization_msgs::msg::Marker meshROS;
sensor_msgs::msg::Range heightROS;
string _frame_id;

// fov visualize
double max_dis_ = 4.0;
double x_max_dis_gain_ = 0.64;
double y_max_dis_gain_ = 0.82;
visualization_msgs::msg::Marker markerNode_fov;
visualization_msgs::msg::Marker markerEdge_fov;
visualization_msgs::msg::Marker marker_line, fast_marker_line;
std::vector<Eigen::Vector3d> fov_node;

void fov_visual_init(std::string msg_frame_id) {
  double x_max_dis = max_dis_ * x_max_dis_gain_;
  double y_max_dis = max_dis_ * y_max_dis_gain_;

  fov_node.resize(5);
  fov_node[0][0] = 0;
  fov_node[0][1] = 0;
  fov_node[0][2] = 0;

  fov_node[1][2] = x_max_dis;
  fov_node[1][1] = y_max_dis;
  fov_node[1][0] = max_dis_;

  fov_node[2][2] = x_max_dis;
  fov_node[2][1] = -y_max_dis;
  fov_node[2][0] = max_dis_;

  fov_node[3][2] = -x_max_dis;
  fov_node[3][1] = -y_max_dis;
  fov_node[3][0] = max_dis_;

  fov_node[4][2] = -x_max_dis;
  fov_node[4][1] = y_max_dis;
  fov_node[4][0] = max_dis_;

  markerNode_fov.header.frame_id = msg_frame_id;
  // markerNode_fov.header.stamp = msg_time;
  markerNode_fov.action = visualization_msgs::msg::Marker::ADD;
  markerNode_fov.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  markerNode_fov.ns = "fov_nodes";
  // markerNode_fov.id = 0;
  markerNode_fov.pose.orientation.w = 1;
  markerNode_fov.scale.x = 0.05;
  markerNode_fov.scale.y = 0.05;
  markerNode_fov.scale.z = 0.05;
  markerNode_fov.color.r = 0;
  markerNode_fov.color.g = 0.8;
  markerNode_fov.color.b = 1;
  markerNode_fov.color.a = 1;

  markerEdge_fov.header.frame_id = msg_frame_id;
  // markerEdge_fov.header.stamp = msg_time;
  markerEdge_fov.action = visualization_msgs::msg::Marker::ADD;
  markerEdge_fov.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge_fov.ns = "fov_edges";
  // markerEdge_fov.id = 0;
  markerEdge_fov.pose.orientation.w = 1;
  markerEdge_fov.scale.x = 0.05;
  markerEdge_fov.color.r = 0.5f;
  markerEdge_fov.color.g = 0.0;
  markerEdge_fov.color.b = 0.0;
  markerEdge_fov.color.a = 1;
}

void pub_fov_visual(Eigen::Vector3d& p, Eigen::Quaterniond& q) {
  visualization_msgs::msg::Marker clear_previous_msg;
  clear_previous_msg.action = visualization_msgs::msg::Marker::DELETEALL;

  visualization_msgs::msg::MarkerArray markerArray_fov;
  markerNode_fov.points.clear();
  markerEdge_fov.points.clear();

  std::vector<geometry_msgs::msg::Point> fov_node_marker;
  for (int i = 0; i < (int)fov_node.size(); i++) {
    Eigen::Vector3d vector_temp;
    vector_temp = q * fov_node[i] + p;
    geometry_msgs::msg::Point point_temp;
    point_temp.x = vector_temp[0];
    point_temp.y = vector_temp[1];
    point_temp.z = vector_temp[2];
    fov_node_marker.push_back(point_temp);
  }

  markerNode_fov.points.push_back(fov_node_marker[0]);
  markerNode_fov.points.push_back(fov_node_marker[1]);
  markerNode_fov.points.push_back(fov_node_marker[2]);
  markerNode_fov.points.push_back(fov_node_marker[3]);
  markerNode_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[0]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerEdge_fov.points.push_back(fov_node_marker[1]);
  markerEdge_fov.points.push_back(fov_node_marker[2]);

  markerEdge_fov.points.push_back(fov_node_marker[2]);
  markerEdge_fov.points.push_back(fov_node_marker[3]);

  markerEdge_fov.points.push_back(fov_node_marker[3]);
  markerEdge_fov.points.push_back(fov_node_marker[4]);

  markerEdge_fov.points.push_back(fov_node_marker[4]);
  markerEdge_fov.points.push_back(fov_node_marker[1]);

  markerArray_fov.markers.push_back(clear_previous_msg);
  markerArray_fov.markers.push_back(markerNode_fov);
  markerArray_fov.markers.push_back(markerEdge_fov);
  fov_pub_->publish(markerArray_fov);
}

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  if (msg->header.frame_id == string("null"))
    return;
  colvec pose(6);
  colvec q(4);
  colvec vel(3);

  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  // pose(2) = msg->pose.pose.position.z;
  pose(2) = 0;
  // q(0) = msg->pose.pose.orientation.w;
  // q(1) = msg->pose.pose.orientation.x;
  // q(2) = msg->pose.pose.orientation.y;
  // q(3) = msg->pose.pose.orientation.z;
  q(0) = msg->pose.pose.orientation.z;
  q(1) = 0;
  q(2) = 0;
  q(3) = -msg->pose.pose.orientation.w;

  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;

  // NOTE fov
  Eigen::Vector3d fov_p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Eigen::Quaterniond fov_q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  pub_fov_visual(fov_p, fov_q);

  if (origin && !isOriginSet) {
    isOriginSet = true;
    poseOrigin = pose;
  }
  if (origin) {
    vel = trans(ypr_to_R(pose.rows(3, 5))) * vel;
    pose = pose_update(pose_inverse(poseOrigin), pose);
    vel = ypr_to_R(pose.rows(3, 5)) * vel;
  }

  // Mesh model
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = msg->header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::msg::Marker::ADD;
  meshROS.mesh_use_embedded_materials = true;
  // meshROS.pose.position.x = msg->pose.pose.position.x;
  // meshROS.pose.position.y = msg->pose.pose.position.y;
  // meshROS.pose.position.z = msg->pose.pose.position.z;
  // q(0) = msg->pose.pose.orientation.w;
  // q(1) = msg->pose.pose.orientation.x;
  // q(2) = msg->pose.pose.orientation.y;
  // q(3) = msg->pose.pose.orientation.z;
  meshROS.pose.position.x = pose(0);
  meshROS.pose.position.y = pose(1);
  meshROS.pose.position.z = pose(2);

  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }

  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  // meshROS.color.a = color_a;
  // meshROS.color.r = color_r;
  // meshROS.color.g = color_g;
  // meshROS.color.b = color_b;
  meshROS.color.a = 0;
  meshROS.color.r = 0;
  meshROS.color.g = 0;
  meshROS.color.b = 0;
  meshROS.mesh_resource = mesh_resource;
  meshPub->publish(meshROS);

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = string("world");
  poseROS.pose.position.x = pose(0);
  poseROS.pose.position.y = pose(1);
  poseROS.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
  poseROS.pose.orientation.w = q(0);
  poseROS.pose.orientation.x = q(1);
  poseROS.pose.orientation.y = q(2);
  poseROS.pose.orientation.z = q(3);
  posePub->publish(poseROS);

  // Velocity
  colvec yprVel(3);
  yprVel(0) = atan2(vel(1), vel(0));
  yprVel(1) = -atan2(vel(2), norm(vel.rows(0, 1), 2));
  yprVel(2) = 0;
  q = R_to_quaternion(ypr_to_R(yprVel));
  velROS.header.frame_id = string("world");
  velROS.header.stamp = msg->header.stamp;
  velROS.ns = string("velocity");
  velROS.id = 0;
  velROS.type = visualization_msgs::msg::Marker::ARROW;
  velROS.action = visualization_msgs::msg::Marker::ADD;
  velROS.pose.position.x = pose(0);
  velROS.pose.position.y = pose(1);
  velROS.pose.position.z = pose(2);
  velROS.pose.orientation.w = q(0);
  velROS.pose.orientation.x = q(1);
  velROS.pose.orientation.y = q(2);
  velROS.pose.orientation.z = q(3);
  velROS.scale.x = norm(vel, 2);
  velROS.scale.y = 0.05;
  velROS.scale.z = 0.05;
  velROS.color.a = 1.0;
  velROS.color.r = color_r;
  velROS.color.g = color_g;
  velROS.color.b = color_b;
  velPub->publish(velROS);

  // Path
  static rclcpp::Time prevt = msg->header.stamp;
  if ((rclcpp::Time(msg->header.stamp) - prevt).seconds() > 0.1) {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub->publish(pathROS);
  }

  // Covariance color
  double r = 1;
  double g = 1;
  double b = 1;
  bool G = msg->twist.covariance[33];
  bool V = msg->twist.covariance[34];
  bool L = msg->twist.covariance[35];
  if (cov_color) {
    r = G;
    g = V;
    b = L;
  }

  // Covariance Position
  if (cov_pos) {
    mat P(6, 6);
    for (int j = 0; j < 6; j++)
      for (int i = 0; i < 6; i++)
        P(i, j) = msg->pose.covariance[i + j * 6];
    colvec eigVal;
    mat eigVec;
    eig_sym(eigVal, eigVec, P.submat(0, 0, 2, 2));
    if (det(eigVec) < 0) {
      for (int k = 0; k < 3; k++) {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0) {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covROS.header.frame_id = string("world");
    covROS.header.stamp = msg->header.stamp;
    covROS.ns = string("covariance");
    covROS.id = 0;
    covROS.type = visualization_msgs::msg::Marker::SPHERE;
    covROS.action = visualization_msgs::msg::Marker::ADD;
    covROS.pose.position.x = pose(0);
    covROS.pose.position.y = pose(1);
    covROS.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covROS.pose.orientation.w = q(0);
    covROS.pose.orientation.x = q(1);
    covROS.pose.orientation.y = q(2);
    covROS.pose.orientation.z = q(3);
    covROS.scale.x = sqrt(eigVal(0)) * cov_scale;
    covROS.scale.y = sqrt(eigVal(1)) * cov_scale;
    covROS.scale.z = sqrt(eigVal(2)) * cov_scale;
    covROS.color.a = 0.4;
    covROS.color.r = r * 0.5;
    covROS.color.g = g * 0.5;
    covROS.color.b = b * 0.5;
    covPub->publish(covROS);
  }

  // Covariance Velocity
  if (cov_vel) {
    mat P(3, 3);
    for (int j = 0; j < 3; j++)
      for (int i = 0; i < 3; i++)
        P(i, j) = msg->twist.covariance[i + j * 6];
    mat R = ypr_to_R(pose.rows(3, 5));
    P = R * P * trans(R);
    colvec eigVal;
    mat eigVec;
    eig_sym(eigVal, eigVec, P);
    if (det(eigVec) < 0) {
      for (int k = 0; k < 3; k++) {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0) {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covVelROS.header.frame_id = string("world");
    covVelROS.header.stamp = msg->header.stamp;
    covVelROS.ns = string("covariance_velocity");
    covVelROS.id = 0;
    covVelROS.type = visualization_msgs::msg::Marker::SPHERE;
    covVelROS.action = visualization_msgs::msg::Marker::ADD;
    covVelROS.pose.position.x = pose(0);
    covVelROS.pose.position.y = pose(1);
    covVelROS.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covVelROS.pose.orientation.w = q(0);
    covVelROS.pose.orientation.x = q(1);
    covVelROS.pose.orientation.y = q(2);
    covVelROS.pose.orientation.z = q(3);
    covVelROS.scale.x = sqrt(eigVal(0)) * cov_scale;
    covVelROS.scale.y = sqrt(eigVal(1)) * cov_scale;
    covVelROS.scale.z = sqrt(eigVal(2)) * cov_scale;
    covVelROS.color.a = 0.4;
    covVelROS.color.r = r;
    covVelROS.color.g = g;
    covVelROS.color.b = b;
    covVelPub->publish(covVelROS);
  }

  // Color Coded Trajectory
  static colvec ppose = pose;
  static rclcpp::Time pt = msg->header.stamp;
  rclcpp::Time t = msg->header.stamp;
  if ((t - pt).seconds() > 0.5) {
    trajROS.header.frame_id = string("world");
    trajROS.header.stamp = node->now();
    trajROS.ns = string("trajectory");
    trajROS.type = visualization_msgs::msg::Marker::LINE_LIST;
    trajROS.action = visualization_msgs::msg::Marker::ADD;
    trajROS.pose.position.x = 0;
    trajROS.pose.position.y = 0;
    trajROS.pose.position.z = 0;
    trajROS.pose.orientation.w = 1;
    trajROS.pose.orientation.x = 0;
    trajROS.pose.orientation.y = 0;
    trajROS.pose.orientation.z = 0;
    trajROS.scale.x = 0.1;
    trajROS.scale.y = 0;
    trajROS.scale.z = 0;
    trajROS.color.r = 0.0;
    trajROS.color.g = 1.0;
    trajROS.color.b = 0.0;
    trajROS.color.a = 0.8;
    geometry_msgs::msg::Point p;
    p.x = ppose(0);
    p.y = ppose(1);
    p.z = ppose(2);
    trajROS.points.push_back(p);
    p.x = pose(0);
    p.y = pose(1);
    p.z = pose(2);
    trajROS.points.push_back(p);
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    trajROS.colors.push_back(color);
    trajROS.colors.push_back(color);
    ppose = pose;
    pt = t;
    trajPub->publish(trajROS);
  }

  // Sensor availability
  sensorROS.header.frame_id = string("world");
  sensorROS.header.stamp = msg->header.stamp;
  sensorROS.ns = string("sensor");
  sensorROS.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  sensorROS.action = visualization_msgs::msg::Marker::ADD;
  sensorROS.pose.position.x = pose(0);
  sensorROS.pose.position.y = pose(1);
  sensorROS.pose.position.z = pose(2) + 1.0;
  sensorROS.pose.orientation.w = q(0);
  sensorROS.pose.orientation.x = q(1);
  sensorROS.pose.orientation.y = q(2);
  sensorROS.pose.orientation.z = q(3);
  string strG = G ? string(" GPS ") : string("");
  string strV = V ? string(" Vision ") : string("");
  string strL = L ? string(" Laser ") : string("");
  sensorROS.text = "| " + strG + strV + strL + " |";
  sensorROS.color.a = 1.0;
  sensorROS.color.r = 1.0;
  sensorROS.color.g = 1.0;
  sensorROS.color.b = 1.0;
  sensorROS.scale.z = 0.5;
  sensorPub->publish(sensorROS);

  // Laser height measurement
  double H = msg->twist.covariance[32];
  heightROS.header.frame_id = local_frame("height");
  heightROS.header.stamp = msg->header.stamp;
  heightROS.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  heightROS.field_of_view = 5.0 * M_PI / 180.0;
  heightROS.min_range = -100;
  heightROS.max_range = 100;
  heightROS.range = H;
  heightPub->publish(heightROS);

  // TF for raw sensor visualization
  if (1) {
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(pose(0), pose(1), pose(2)));
    transform.setRotation(tf2::Quaternion(q(1), q(2), q(3), q(0)));

    tf2::Transform transform45;
    transform45.setOrigin(tf2::Vector3(0, 0, 0));
    colvec y45 = zeros<colvec>(3);
    y45(0) = 45.0 * M_PI / 180;
    colvec q45 = R_to_quaternion(ypr_to_R(y45));
    transform45.setRotation(tf2::Quaternion(q45(1), q45(2), q45(3), q45(0)));

    tf2::Transform transform90;
    transform90.setOrigin(tf2::Vector3(0, 0, 0));
    colvec p90 = zeros<colvec>(3);
    p90(1) = 90.0 * M_PI / 180;
    colvec q90 = R_to_quaternion(ypr_to_R(p90));
    transform90.setRotation(tf2::Quaternion(q90(1), q90(2), q90(3), q90(0)));

    send_transform(transform, msg->header.stamp, string("world"), string("/base"));
    send_transform(transform45, msg->header.stamp, string("/base"), string("/laser"));
    send_transform(transform45, msg->header.stamp, string("/base"), string("/vision"));
    send_transform(transform90, msg->header.stamp, string("/base"), string("/height"));
  }
}

void cmd_callback(const quadrotor_interfaces::msg::PositionCommand& cmd) {
  if (cmd.header.frame_id == string("null"))
    return;

  colvec pose(6);
  pose(0) = cmd.position.x;
  pose(1) = cmd.position.y;
  pose(2) = cmd.position.z;
  colvec q(4);
  q(0) = 1.0;
  q(1) = 0.0;
  q(2) = 0.0;
  q(3) = 0.0;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

  // Mesh model
  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = cmd.header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::msg::Marker::ADD;
  meshROS.pose.position.x = cmd.position.x;
  meshROS.pose.position.y = cmd.position.y;
  meshROS.pose.position.z = cmd.position.z;

  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = 1.0;
  meshROS.scale.y = 1.0;
  meshROS.scale.z = 1.0;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub->publish(meshROS);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("odom_visualization");
  tf_prefix = node->get_namespace();
  while (!tf_prefix.empty() && tf_prefix.front() == '/') tf_prefix.erase(0, 1);

  // mesh_resource = node->declare_parameter("mesh_resource", std::string("package://odom_visualization/meshes/hummingbird.mesh"));
  mesh_resource = node->declare_parameter("mesh_resource", std::string("package://odom_visualization/meshes/f250.dae"));

  color_r = node->declare_parameter("color.r", 1.0);
  color_g = node->declare_parameter("color.g", 0.0);
  color_b = node->declare_parameter("color.b", 0.0);
  color_a = node->declare_parameter("color.a", 1.0);
  origin = node->declare_parameter("origin", false);
  scale = node->declare_parameter("robot_scale", 2.0);
  _frame_id = node->declare_parameter("frame_id", string("world"));

  cross_config = node->declare_parameter("cross_config", false);
  tf45 = node->declare_parameter("tf45", false);
  cov_scale = node->declare_parameter("covariance_scale", 100.0);
  cov_pos = node->declare_parameter("covariance_position", false);
  cov_vel = node->declare_parameter("covariance_velocity", false);
  cov_color = node->declare_parameter("covariance_color", false);

  auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(100).reliable(), odom_callback);
  auto sub_cmd = node->create_subscription<quadrotor_interfaces::msg::PositionCommand>("~/cmd", rclcpp::QoS(100).reliable(), cmd_callback);
  posePub = node->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", rclcpp::QoS(1).reliable().transient_local());
  pathPub = node->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).reliable().transient_local());
  velPub = node->create_publisher<visualization_msgs::msg::Marker>("~/velocity", rclcpp::QoS(1).reliable().transient_local());
  covPub = node->create_publisher<visualization_msgs::msg::Marker>("~/covariance", rclcpp::QoS(1).reliable().transient_local());
  covVelPub = node->create_publisher<visualization_msgs::msg::Marker>("~/covariance_velocity", rclcpp::QoS(1).reliable().transient_local());
  trajPub = node->create_publisher<visualization_msgs::msg::Marker>("~/trajectory", rclcpp::QoS(1).reliable().transient_local());
  sensorPub = node->create_publisher<visualization_msgs::msg::Marker>("~/sensor", rclcpp::QoS(1).reliable().transient_local());
  meshPub = node->create_publisher<visualization_msgs::msg::Marker>("~/robot", rclcpp::QoS(1).reliable().transient_local());
  heightPub = node->create_publisher<sensor_msgs::msg::Range>("~/height", rclcpp::QoS(1).reliable().transient_local());
  fov_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("~/fov_visual", rclcpp::QoS(5).reliable());
  auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  broadcaster = tf_broadcaster.get();
  fov_visual_init("world");

  rclcpp::spin(node);
  // Release DDS endpoints while their owning node/context is still in scope.
  // The publishers and node are globals; leaving them for static destruction
  // can run their teardown after the middleware's process-level cleanup.
  sub_odom.reset();
  sub_cmd.reset();
  posePub.reset();
  pathPub.reset();
  velPub.reset();
  covPub.reset();
  covVelPub.reset();
  trajPub.reset();
  sensorPub.reset();
  meshPub.reset();
  heightPub.reset();
  fov_pub_.reset();
  broadcaster = nullptr;
  tf_broadcaster.reset();
  node.reset();
  rclcpp::shutdown();

  return 0;
}
