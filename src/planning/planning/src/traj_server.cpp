#include <csignal>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_interfaces/msg/poly_traj.hpp>
#include <quadrotor_interfaces/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapping/ros2_parameters.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <traj_opt/poly_traj_utils.hpp>

rclcpp::Publisher<quadrotor_interfaces::msg::PositionCommand>::SharedPtr pos_cmd_pub_;
rclcpp::Node::SharedPtr node_;
rclcpp::Time heartbeat_time_;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_interfaces::msg::PolyTraj trajMsg_, trajMsg_last_;
Eigen::Vector3d last_p_;
double last_yaw_ = 0;

void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  quadrotor_interfaces::msg::PositionCommand cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_interfaces::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub_->publish(cmd);
  last_p_ = p;
}

bool exe_traj(const quadrotor_interfaces::msg::PolyTraj &trajMsg) {
  double t = (node_->now() - rclcpp::Time(trajMsg.start_time)).seconds();
  if (t > 0) {
    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("elastic_tracker"), "[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      publish_cmd(trajMsg.traj_id, p, v0, v0, last_yaw_, 0);  // TODO yaw
      return true;
    }
    if (trajMsg.order != 5) {
      RCLCPP_ERROR(rclcpp::get_logger("elastic_tracker"), "[traj_server] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("elastic_tracker"), "[traj_server] WRONG trajectory parameters!");
      return false;
    }
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      RCLCPP_ERROR(rclcpp::get_logger("elastic_tracker"), "[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t);
    v = traj.getVel(t);
    a = traj.getAcc(t);
    // NOTE yaw
    double yaw = trajMsg.yaw;
    double d_yaw = yaw - last_yaw_;
    d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
    d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
    double d_yaw_abs = fabs(d_yaw);
    if (d_yaw_abs >= 0.02) {
      yaw = last_yaw_ + d_yaw / d_yaw_abs * 0.02;
    }
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, 0);  // TODO yaw
    last_yaw_ = yaw;
    return true;
  }
  return false;
}

void heartbeatCallback(const std_msgs::msg::Empty::ConstSharedPtr &msg) {
  heartbeat_time_ = node_->now();
}

void polyTrajCallback(const quadrotor_interfaces::msg::PolyTraj::ConstSharedPtr &msgPtr) {
  trajMsg_ = *msgPtr;
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
  }
}

void cmdCallback() {
  if (!receive_traj_) {
    return;
  }
  rclcpp::Time time_now = node_->now();
  if ((time_now - heartbeat_time_).seconds() > 0.5) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("elastic_tracker"), "[traj_server] Lost heartbeat from the planner, is he dead?");
    publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 0);  // TODO yaw
    return;
  }
  if (exe_traj(trajMsg_)) {
    trajMsg_last_ = trajMsg_;
    return;
  } else if (exe_traj(trajMsg_last_)) {
    return;
  }
}

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
}

int main(int argc, char **argv) {
  // Keep the context valid until initialization and any executing callback finish.
  // The signal handler only writes an async-signal-safe flag; it calls no ROS API.
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  node_ = std::make_shared<rclcpp::Node>("traj_server");
  auto& nh = *node_;
  heartbeat_time_ = node_->now();

  auto poly_traj_sub = nh.create_subscription<quadrotor_interfaces::msg::PolyTraj>("~/trajectory", 10, polyTrajCallback);
  auto heartbeat_sub = nh.create_subscription<std_msgs::msg::Empty>("~/heartbeat", 10, heartbeatCallback);

  pos_cmd_pub_ = nh.create_publisher<quadrotor_interfaces::msg::PositionCommand>("~/position_cmd", 50);

  auto cmd_timer = nh.create_wall_timer(std::chrono::milliseconds(10), cmdCallback);

  

  RCLCPP_WARN(rclcpp::get_logger("elastic_tracker"), "[Traj server]: ready.");

  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    executor.remove_node(node_);
  }
  // Globals must not retain DDS entities until static destruction.
  cmd_timer.reset();
  poly_traj_sub.reset();
  heartbeat_sub.reset();
  pos_cmd_pub_.reset();
  node_.reset();
  rclcpp::shutdown();

  return 0;
}


