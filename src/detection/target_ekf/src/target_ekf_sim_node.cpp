#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <target_ekf/target_ekf.hpp>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>
    YoloOdomSyncPolicy;
typedef message_filters::Synchronizer<YoloOdomSyncPolicy>
    YoloOdomSynchronizer;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_, yolo_odom_pub_;
rclcpp::Node::SharedPtr node_;
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_, width_, height_;
rclcpp::Time last_update_stamp_;
double pitch_thr_ = 30;
bool check_fov_ = false;

std::shared_ptr<Ekf> ekfPtr_;

void predict_state_callback() {
  double update_dt = (node_->now() - last_update_stamp_).seconds();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    RCLCPP_WARN(node_->get_logger(), "[ekf] too long time no update!");
    return;
  }
  // publish target odom
  nav_msgs::msg::Odometry target_odom;
  target_odom.header.stamp = node_->now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekfPtr_->pos().x();
  target_odom.pose.pose.position.y = ekfPtr_->pos().y();
  target_odom.pose.pose.position.z = ekfPtr_->pos().z();
  target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
  target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
  target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
  Eigen::Vector3d rpy = ekfPtr_->rpy();
  Eigen::Quaterniond q = euler2quaternion(rpy);
  target_odom.pose.pose.orientation.w = q.w();
  target_odom.pose.pose.orientation.x = q.x();
  target_odom.pose.pose.orientation.y = q.y();
  target_odom.pose.pose.orientation.z = q.z();
  target_odom_pub_->publish(target_odom);
}

void update_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& target_msg, const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
  // std::cout << "yolo stamp: " << bboxes_msg->header.stamp << std::endl;
  // std::cout << "odom stamp: " << odom_msg->header.stamp << std::endl;
  Eigen::Vector3d odom_p, p;
  Eigen::Quaterniond odom_q, q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;

  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

  p.x() = target_msg->pose.pose.position.x;
  p.y() = target_msg->pose.pose.position.y;
  p.z() = target_msg->pose.pose.position.z;
  q.w() = target_msg->pose.pose.orientation.w;
  q.x() = target_msg->pose.pose.orientation.x;
  q.y() = target_msg->pose.pose.orientation.y;
  q.z() = target_msg->pose.pose.orientation.z;

  Eigen::Vector3d rpy = quaternion2euler(q);

  // NOTE check whether it's in FOV
  if (check_fov_) {
    Eigen::Vector3d p_in_body = cam_q.inverse() * (p - cam_p);
    if (p_in_body.z() < 0.1 || p_in_body.z() > 5.0) {
      return;
    }
    double x = p_in_body.x() * fx_ / p_in_body.z() + cx_;
    if (x < 0 || x > height_) {
      return;
    }
    double y = p_in_body.y() * fy_ / p_in_body.z() + cy_;
    if (y < 0 || y > width_) {
      return;
    }
  }

  // update target odom
  double update_dt = (node_->now() - last_update_stamp_).seconds();
  if (update_dt > 5.0) {
    ekfPtr_->reset(p, rpy);
    RCLCPP_WARN(node_->get_logger(), "[ekf] reset!");
  } else if (ekfPtr_->update(p, rpy)) {
    // RCLCPP_WARN(node_->get_logger(), "[ekf] update!");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[ekf] update invalid!");
    return;
  }
  last_update_stamp_ = node_->now();
}

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
  // std::cout << "_now stamp: " << odom_msg->header.stamp << std::endl;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("target_ekf");
  last_update_stamp_ = node_->now() - rclcpp::Duration::from_seconds(10.0);

  auto rotation = node_->declare_parameter<std::vector<double>>("cam2body_R");
  auto translation = node_->declare_parameter<std::vector<double>>("cam2body_p");
  if (rotation.size() != 9 || translation.size() != 3)
    throw std::runtime_error("cam2body_R needs 9 values and cam2body_p needs 3 values");
  cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
  cam2body_p_ = Eigen::Map<const Eigen::Vector3d>(translation.data());
  fx_ = node_->declare_parameter<double>("cam_fx");
  fy_ = node_->declare_parameter<double>("cam_fy");
  cx_ = node_->declare_parameter<double>("cam_cx");
  cy_ = node_->declare_parameter<double>("cam_cy");
  pitch_thr_ = node_->declare_parameter<double>("pitch_thr", 30.0);
  width_ = static_cast<double>(node_->declare_parameter<int>("cam_width"));
  height_ = static_cast<double>(node_->declare_parameter<int>("cam_height"));
  check_fov_ = node_->declare_parameter<bool>("check_fov", false);
  message_filters::Subscriber<nav_msgs::msg::Odometry> yolo_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  rclcpp::TimerBase::SharedPtr ekf_predict_timer_;

  target_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("~/target_odom", rclcpp::QoS(1).reliable());
  yolo_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("~/yolo_odom", rclcpp::QoS(1).reliable());

  int ekf_rate = 20;
  ekf_rate = node_->declare_parameter<int>("ekf_rate", 20);
  if (ekf_rate <= 0) throw std::runtime_error("ekf_rate must be positive");
  ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);

  yolo_sub_.subscribe(node_.get(), "~/yolo", rclcpp::QoS(1).reliable().get_rmw_qos_profile());
  odom_sub_.subscribe(node_.get(), "~/odom", rclcpp::QoS(100).reliable().get_rmw_qos_profile());
  yolo_odom_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
  yolo_odom_sync_Ptr_->registerCallback(std::bind(&update_state_callback, std::placeholders::_1, std::placeholders::_2));
  ekf_predict_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / ekf_rate), &predict_state_callback);

  rclcpp::spin(node_);
  ekf_predict_timer_.reset();
  yolo_odom_sync_Ptr_.reset();
  yolo_sub_.unsubscribe();
  odom_sub_.unsubscribe();
  target_odom_pub_.reset();
  yolo_odom_pub_.reset();
  node_.reset();
  rclcpp::shutdown();
  return 0;
}
