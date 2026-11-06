#include "so3_controller/so3_controller.hpp"
#include <quadrotor_interfaces/msg/position_command.hpp>
#include <quadrotor_interfaces/msg/so3_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace so3_controller {
class Nodelet : public rclcpp::Node{
 private:
  std::shared_ptr<SO3Controller> so3ControlPtr_;
  rclcpp::Subscription<quadrotor_interfaces::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<quadrotor_interfaces::msg::SO3Command>::SharedPtr so3cmd_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  quadrotor_interfaces::msg::SO3Command so3cmd_;

  bool position_cmd_received_flag_ = false;
  Eigen::Vector3d des_pos_ = Eigen::Vector3d::Zero();
  double des_yaw_ = 0.0;
  bool odom_received_ = false;
  bool imu_received_ = false;
  quadrotor_interfaces::msg::PositionCommand::ConstSharedPtr pending_position_cmd_;

  bool feedback_ready() const { return odom_received_ && imu_received_; }
  void apply_pending_command() {
    if (feedback_ready() && pending_position_cmd_) {
      auto command = std::move(pending_position_cmd_);
      position_cmd_callback(command);
    }
  }

  void timer_callback() {
    // DDS discovery may finish after this timer starts. Until both feedback
    // streams arrive, keep the simulator's initialized equilibrium command.
    if (!feedback_ready()) return;
    if (position_cmd_received_flag_) {
      position_cmd_received_flag_ = false;
    } else {
      Eigen::Vector3d des_vel(0, 0, 0);
      Eigen::Vector3d des_acc(0, 0, 0);
      Eigen::Vector3d kx(5.7, 5.7, 6.2);
      Eigen::Vector3d kv(3.4, 3.4, 4.0);
      so3ControlPtr_->calculateControl(des_pos_, des_vel, des_acc, 
                                       des_yaw_, 0, kx, kv);
      const Eigen::Vector3d& f = so3ControlPtr_->getF();
      const Eigen::Quaterniond& q = so3ControlPtr_->getQ();
      so3cmd_.force.x = f(0);
      so3cmd_.force.y = f(1);
      so3cmd_.force.z = f(2);
      so3cmd_.orientation.x = q.x();
      so3cmd_.orientation.y = q.y();
      so3cmd_.orientation.z = q.z();
      so3cmd_.orientation.w = q.w();
      so3cmd_pub_->publish(so3cmd_);
    }
  }

  void position_cmd_callback(const quadrotor_interfaces::msg::PositionCommand::ConstSharedPtr& msg) {
    if (!feedback_ready()) {
      pending_position_cmd_ = msg;
      return;
    }
    position_cmd_received_flag_ = true;
    Eigen::Vector3d des_pos(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Vector3d des_vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    Eigen::Vector3d des_acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    Eigen::Vector3d kx(msg->kx[0], msg->kx[1], msg->kx[2]);
    Eigen::Vector3d kv(msg->kv[0], msg->kv[1], msg->kv[2]);
    if (msg->kx[0] == 0) {
      kx(0) = 5.7; kx(1) = 5.7; kx(2) = 6.2;
      kv(0) = 3.4; kv(1) = 3.4; kv(2) = 4.0;
    }
    double des_yaw = msg->yaw;
    double des_yaw_dot = msg->yaw_dot;
    so3ControlPtr_->calculateControl(des_pos, des_vel, des_acc, 
                                     des_yaw, des_yaw_dot, kx, kv);
    const Eigen::Vector3d& f = so3ControlPtr_->getF();
    const Eigen::Quaterniond& q = so3ControlPtr_->getQ();
    so3cmd_.force.x = f(0);
    so3cmd_.force.y = f(1);
    so3cmd_.force.z = f(2);
    so3cmd_.orientation.x = q.x();
    so3cmd_.orientation.y = q.y();
    so3cmd_.orientation.z = q.z();
    so3cmd_.orientation.w = q.w();
    so3cmd_pub_->publish(so3cmd_);
    // store last des_pos and des_yaw
    des_pos_ = des_pos;
    des_yaw_ = des_yaw;
  }
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    Eigen::Vector3d pos(msg->pose.pose.position.x, 
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Vector3d vel(msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z);
    so3ControlPtr_->setPos(pos);
    so3ControlPtr_->setVel(vel);
    so3cmd_.aux.current_yaw = tf2::getYaw(msg->pose.pose.orientation);
    des_pos_ = pos;
    des_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    odom_received_ = true;
    apply_pending_command();
  }
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    Eigen::Vector3d acc(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
    so3ControlPtr_->setAcc(acc);
    imu_received_ = true;
    apply_pending_command();
  }
 public:
  explicit Nodelet(const rclcpp::NodeOptions& options) : rclcpp::Node("so3_controller", options) {
    
    // parameters
    double mass, g;
    mass = declare_parameter<std::decay_t<decltype(mass)>>("mass");
    g = declare_parameter<std::decay_t<decltype(g)>>("g");
    so3ControlPtr_ = std::make_shared<SO3Controller>(mass, g);
    so3cmd_.header.frame_id = "world";
    so3cmd_.k_r[0] = declare_parameter<std::decay_t<decltype(so3cmd_.k_r[0])>>("gains.rot.x");
    so3cmd_.k_r[1] = declare_parameter<std::decay_t<decltype(so3cmd_.k_r[1])>>("gains.rot.y");
    so3cmd_.k_r[2] = declare_parameter<std::decay_t<decltype(so3cmd_.k_r[2])>>("gains.rot.z");
    so3cmd_.k_om[0] = declare_parameter<std::decay_t<decltype(so3cmd_.k_om[0])>>("gains.ang.x");
    so3cmd_.k_om[1] = declare_parameter<std::decay_t<decltype(so3cmd_.k_om[1])>>("gains.ang.y");
    so3cmd_.k_om[2] = declare_parameter<std::decay_t<decltype(so3cmd_.k_om[2])>>("gains.ang.z");
    so3cmd_.aux.kf_correction = declare_parameter<std::decay_t<decltype(so3cmd_.aux.kf_correction)>>("corrections.z");
    so3cmd_.aux.angle_corrections[0] = declare_parameter<std::decay_t<decltype(so3cmd_.aux.angle_corrections[0])>>("corrections.r");
    so3cmd_.aux.angle_corrections[1] = declare_parameter<std::decay_t<decltype(so3cmd_.aux.angle_corrections[1])>>("corrections.p");
    so3cmd_.aux.enable_motors = true;
    so3cmd_.aux.use_external_yaw = false;
    position_cmd_sub_ = create_subscription<quadrotor_interfaces::msg::PositionCommand>("~/position_cmd", rclcpp::QoS(10).reliable(), std::bind(&Nodelet::position_cmd_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(10).reliable(), std::bind(&Nodelet::odom_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("~/imu", rclcpp::QoS(10).reliable(), std::bind(&Nodelet::imu_callback, this, std::placeholders::_1));
    so3cmd_pub_ = create_publisher<quadrotor_interfaces::msg::SO3Command>("~/so3cmd", rclcpp::QoS(10).reliable());
    state_timer_ = create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&Nodelet::timer_callback, this));
  }
};

} // so3_controller


RCLCPP_COMPONENTS_REGISTER_NODE(so3_controller::Nodelet)
