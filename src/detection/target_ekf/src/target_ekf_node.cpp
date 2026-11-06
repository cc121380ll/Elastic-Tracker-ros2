#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <object_detection_interfaces/msg/bounding_boxes.hpp>

typedef message_filters::sync_policies::ApproximateTime<object_detection_interfaces::msg::BoundingBoxes, nav_msgs::msg::Odometry>
    YoloOdomSyncPolicy;
typedef message_filters::Synchronizer<YoloOdomSyncPolicy>
    YoloOdomSynchronizer;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr target_odom_pub_, yolo_odom_pub_;
rclcpp::Node::SharedPtr node_;
Eigen::Matrix3d cam2body_R_;
Eigen::Vector3d cam2body_p_;
double fx_, fy_, cx_, cy_;
rclcpp::Time last_update_stamp_;
double pitch_thr_ = 30;

struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 3);
    C.setZero(3, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 0) = dt;
    B(4, 1) = dt;
    B(5, 2) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    K = C;
    Qt.setIdentity(3, 3);
    Rt.setIdentity(3, 3);
    Qt(0, 0) = 4;
    Qt(1, 1) = 4;
    Qt(2, 2) = 1;
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    x.setZero(6);
  }
  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
    return;
  }
  inline void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
  }
  inline bool checkValid(const Eigen::Vector3d& z) const {
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    if (x_tmp.tail(3).norm() > vmax) {
      return false;
    } else {
      return true;
    }
  }
  inline void update(const Eigen::Vector3d& z) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
  }
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  inline const Eigen::Vector3d vel() const {
    return x.tail(3);
  }
};

std::shared_ptr<Ekf> ekfPtr_;

void predict_state_callback() {
  double update_dt = (node_->now() - last_update_stamp_).seconds();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    RCLCPP_WARN(node_->get_logger(), "too long time no update!");
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
  target_odom.pose.pose.orientation.w = 1.0;
  target_odom_pub_->publish(target_odom);
}

void update_state_callback(const object_detection_interfaces::msg::BoundingBoxes::ConstSharedPtr &bboxes_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
  // std::cout << "yolo stamp: " << bboxes_msg->header.stamp << std::endl;
  // std::cout << "odom stamp: " << odom_msg->header.stamp << std::endl;
  Eigen::Vector3d odom_p;
  Eigen::Quaterniond odom_q;
  odom_p(0) = odom_msg->pose.pose.position.x;
  odom_p(1) = odom_msg->pose.pose.position.y;
  odom_p(2) = odom_msg->pose.pose.position.z;
  odom_q.w() = odom_msg->pose.pose.orientation.w;
  odom_q.x() = odom_msg->pose.pose.orientation.x;
  odom_q.y() = odom_msg->pose.pose.orientation.y;
  odom_q.z() = odom_msg->pose.pose.orientation.z;

  // NOTE check pitch
  // Eigen::Vector3d eulerAngle = odom_q.matrix().eulerAngles(2, 1, 0);
  // double pitch = fabs( eulerAngle[1] / M_PI * 180 );
  // pitch = pitch > 90 ? 180 - pitch : pitch;
  // if (pitch > pitch_thr_) {
  //   RCLCPP_ERROR(node_->get_logger(), "pitch too large!");
  //   return;
  // }

  Eigen::Vector3d cam_p = odom_q.toRotationMatrix() * cam2body_p_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * Eigen::Quaterniond(cam2body_R_);

  if (bboxes_msg->bounding_boxes.empty()) return;
  auto yolo_bbox = bboxes_msg->bounding_boxes.front();
  double xmin, ymin, xmax, ymax;
  xmin = yolo_bbox.xmin;
  xmax = yolo_bbox.xmax;
  ymin = yolo_bbox.ymin;
  ymax = yolo_bbox.ymax;
  // NOTE check ymin ymax
  double pixel_thr = 30;
  if (ymin < pixel_thr || ymax > 480 - pixel_thr) {
    RCLCPP_ERROR(node_->get_logger(), "pitch out of range!");
    return;
  }
  // calculate target odom
  double height = ymax - ymin;
  double depth = 0.7 / height * fy_;
  double y = ((ymin + ymax) * 0.5 - cy_) * depth / fy_;
  double x = ((xmin + xmax) * 0.5 - cx_) * depth / fx_;
  Eigen::Vector3d p(x, y, depth);
  // std::cout << "p cam frame: " << p.transpose() << std::endl;
  p = cam_q * p + cam_p;
  // publish yolo odom
  nav_msgs::msg::Odometry yolo_odom;
  yolo_odom.header.stamp = bboxes_msg->header.stamp;
  yolo_odom.header.frame_id = "world";
  yolo_odom.pose.pose.orientation.w = 1.0;
  yolo_odom.pose.pose.position.x = p.x();
  yolo_odom.pose.pose.position.y = p.y();
  yolo_odom.pose.pose.position.z = p.z();
  yolo_odom_pub_->publish(yolo_odom);
  // update target odom
  double update_dt = (node_->now() - last_update_stamp_).seconds();
  if (update_dt > 3.0) {
    ekfPtr_->reset(p);
    RCLCPP_WARN(node_->get_logger(), "ekf reset!");
  } else if (ekfPtr_->checkValid(p)) {
    ekfPtr_->update(p);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "update invalid!");
    return;
  }
  last_update_stamp_ = node_->now();
}

void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
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
  message_filters::Subscriber<object_detection_interfaces::msg::BoundingBoxes> yolo_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_odom_sync_Ptr_;
  rclcpp::TimerBase::SharedPtr ekf_predict_timer_;

  target_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("~/target_odom", rclcpp::QoS(1).reliable());
  yolo_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("~/yolo_odom", rclcpp::QoS(1).reliable());

  int ekf_rate = 20;
  ekf_rate = node_->declare_parameter<int>("ekf_rate", 20);
  if (ekf_rate <= 0) throw std::runtime_error("ekf_rate must be positive");
  ekfPtr_ = std::make_shared<Ekf>(1.0/ekf_rate);

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


