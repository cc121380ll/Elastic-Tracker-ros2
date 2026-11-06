#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mapping/mapping.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <quadrotor_interfaces/msg/occ_map3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mapping/ros2_parameters.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <atomic>
#include <thread>

namespace mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
    ImageOdomSyncPolicy;
typedef message_filters::Synchronizer<ImageOdomSyncPolicy>
    ImageOdomSynchronizer;

struct CamConfig {
  // camera paramters
  double rate;
  double range;
  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scaling_factor;
};

class Nodelet : public rclcpp::Node {
 private:
  // All subscriptions and timers use the node default mutually exclusive callback group.
  // Initialization completes synchronously before any executor can run callbacks.

  CamConfig camConfig_;  // just store the parameters of camera

  int down_sample_factor_;

  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;

  // just for depth filter
  Eigen::Vector3d last_cam_p_;
  Eigen::Quaterniond last_cam_q_;
  bool get_first_frame_ = false;
  cv::Mat last_depth_;
  double depth_filter_tolerance_;
  double depth_filter_mindist_;
  int depth_filter_margin_;

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<ImageOdomSynchronizer> depth_odom_sync_Ptr_;
  rclcpp::Publisher<quadrotor_interfaces::msg::OccMap3d>::SharedPtr gridmap_inflate_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pc_pub_, pcl_pub_;

  // NOTE just for global map in simulation
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_sub_;
  bool map_recieved_ = false;
  bool use_global_map_ = false;

  // NOTE for mask target
  bool use_mask_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
  Eigen::Vector3d target_odom_ = Eigen::Vector3d::Zero();
  bool target_received_ = false;

  OccGridMap gridmap_;
  int inflate_size_;

  void depth_odom_callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                           const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
    rclcpp::Time t1, t2;
    // t1 = now();
    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d cam_p = body_q.toRotationMatrix() * cam2body_p_ + body_p;
    Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (depth_ptr->image).convertTo(depth_ptr->image, CV_16UC1, camConfig_.depth_scaling_factor);
    }
    cv::Mat depth_img = depth_ptr->image;

    // pub target

    int nr = depth_img.rows;
    int nc = depth_img.cols;
    std::vector<Eigen::Vector3d> obs_pts;
    // put the points of the depth into the list of obs_points

    // TODO depth filter

    // t1 = now();
    for (int i = depth_filter_margin_; i < nr - depth_filter_margin_; i += down_sample_factor_) {
      for (int j = depth_filter_margin_; j < nc - depth_filter_margin_; j += down_sample_factor_) {
        // (x,y,z) in camera frame
        double z = (depth_img.at<uint16_t>(i, j)) / camConfig_.depth_scaling_factor;
        if (depth_img.at<uint16_t>(i, j) == 0) {
          z = camConfig_.range + 0.5;
        }
        if (std::isnan(z) || std::isinf(z))
          continue;
        if (z < depth_filter_mindist_) {
          continue;
        }
        double y = (i - camConfig_.cy) * z / camConfig_.fy;
        double x = (j - camConfig_.cx) * z / camConfig_.fx;
        Eigen::Vector3d p(x, y, z);
        p = cam_q * p + cam_p;
        bool good_point = true;
        if (get_first_frame_) {
          // NOTE depth filter:
          Eigen::Vector3d p_rev_proj =
              last_cam_q_.inverse().toRotationMatrix() * (p - last_cam_p_);
          double vv = p_rev_proj.y() * camConfig_.fy / p_rev_proj.z() + camConfig_.cy;
          double uu = p_rev_proj.x() * camConfig_.fx / p_rev_proj.z() + camConfig_.cx;
          if (vv >= 0 && vv < nr && uu >= 0 && uu < nc) {
            double drift_dis = fabs(last_depth_.at<uint16_t>((int)vv, (int)uu) / camConfig_.depth_scaling_factor - p_rev_proj.z());
            if (drift_dis > depth_filter_tolerance_) {
              good_point = false;
            }
          }
        }
        if (good_point) {
          obs_pts.push_back(p);
        }
      }
    }
    last_depth_ = depth_img;
    last_cam_p_ = cam_p;
    last_cam_q_ = cam_q;
    get_first_frame_ = true;
    gridmap_.updateMap(cam_p, obs_pts);

    // NOTE use mask
    if (use_mask_ && target_received_) {  // mask target
      Eigen::Vector3d ld = target_odom_;
      Eigen::Vector3d ru = target_odom_;
      ld.x() -= 0.5;
      ld.y() -= 0.5;
      ld.z() -= 1.0;
      ru.x() += 0.5;
      ru.y() += 0.5;
      ru.z() += 1.0;
      gridmap_.setFree(ld, ru);
    }

    // TODO pub local map
    // sensor_msgs::msg::PointCloud2 pc_msg;
    // pcl::PointCloud<pcl::PointXYZ> pcd;
    // pcl::PointXYZ pt;
    // for (const auto p : mask_pts_) {
    //   pt.x = p.x();
    //   pt.y = p.y();
    //   pt.z = p.z();
    //   pcd.push_back(pt);
    // }
    // pcd.width = pcd.points.size();
    // pcd.height = 1;
    // pcd.is_dense = true;
    // pcl::toROSMsg(pcd, pc_msg);
    // pc_msg.header.frame_id = "world";
    // local_pc_pub_->publish(pc_msg);

    quadrotor_interfaces::msg::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = now();
    gridmap_inflate_pub_->publish(gridmap_msg);
  }

  // NOTE
  void target_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msgPtr) {
    target_received_ = true;
    target_odom_.x() = msgPtr->pose.pose.position.x;
    target_odom_.y() = msgPtr->pose.pose.position.y;
    target_odom_.z() = msgPtr->pose.pose.position.z;
  }

  // NOTE just for global map in simulation
  void map_call_back(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msgPtr) {
    if (map_recieved_) {
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msgPtr, point_cloud);
    for (const auto& pt : point_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      gridmap_.setOcc(p);
    }
    gridmap_.inflate(inflate_size_);
    RCLCPP_WARN(rclcpp::get_logger("elastic_tracker"), "[mapping] GLOBAL MAP REVIEVED!");
    map_recieved_ = true;
    return;
  }
  void global_map_timer_callback() {
    if (!map_recieved_) {
      return;
    }
    quadrotor_interfaces::msg::OccMap3d gridmap_msg;
    gridmap_.to_msg(gridmap_msg);
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = now();
    gridmap_inflate_pub_->publish(gridmap_msg);
  }

  void init(rclcpp::Node& nh) {
    // set parameters of mapping
    // cam2body_R_ << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    // cam2body_p_.setZero();
    std::vector<double> tmp;
    if (elastic_tracker::optional_vector_parameter(nh, "cam2body_R", tmp, 9)) {
      cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
    }
    if (elastic_tracker::optional_vector_parameter(nh, "cam2body_p", tmp, 3)) {
      cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
    }
    // std::cout << "R: \n" << cam2body_R_ << std::endl;
    // std::cout << "p: \n" << cam2body_p_ << std::endl;
    double res;
    Eigen::Vector3d map_size;
    // NOTE whether to use global map
    use_global_map_ = nh.declare_parameter<bool>("use_global_map", false);
    if (use_global_map_) {
      double x, y, z, res;
      elastic_tracker::required_parameter(nh, "x_length", x);
      elastic_tracker::required_parameter(nh, "y_length", y);
      elastic_tracker::required_parameter(nh, "z_length", z);
      elastic_tracker::required_parameter(nh, "resolution", res);
      elastic_tracker::required_parameter(nh, "inflate_size", inflate_size_);
      gridmap_.setup(res, Eigen::Vector3d(x, y, z), 10, true);
    } else {
      // camera parameters
      elastic_tracker::required_parameter(nh, "camera_rate", camConfig_.rate);
      elastic_tracker::required_parameter(nh, "camera_range", camConfig_.range);
      elastic_tracker::required_parameter(nh, "cam_width", camConfig_.width);
      elastic_tracker::required_parameter(nh, "cam_height", camConfig_.height);
      elastic_tracker::required_parameter(nh, "cam_fx", camConfig_.fx);
      elastic_tracker::required_parameter(nh, "cam_fy", camConfig_.fy);
      elastic_tracker::required_parameter(nh, "cam_cx", camConfig_.cx);
      elastic_tracker::required_parameter(nh, "cam_cy", camConfig_.cy);
      elastic_tracker::required_parameter(nh, "depth_scaling_factor", camConfig_.depth_scaling_factor);
      // mapping parameters
      elastic_tracker::required_parameter(nh, "down_sample_factor", down_sample_factor_);
      elastic_tracker::required_parameter(nh, "resolution", res);
      elastic_tracker::required_parameter(nh, "local_x", map_size.x());
      elastic_tracker::required_parameter(nh, "local_y", map_size.y());
      elastic_tracker::required_parameter(nh, "local_z", map_size.z());
      elastic_tracker::required_parameter(nh, "inflate_size", inflate_size_);
      gridmap_.setup(res, map_size, camConfig_.range);
      // depth filter parameters
      elastic_tracker::required_parameter(nh, "depth_filter_tolerance", depth_filter_tolerance_);
      elastic_tracker::required_parameter(nh, "depth_filter_mindist", depth_filter_mindist_);
      elastic_tracker::required_parameter(nh, "depth_filter_margin", depth_filter_margin_);
      // raycasting parameters
      int p_min, p_max, p_hit, p_mis, p_occ, p_def;
      elastic_tracker::required_parameter(nh, "p_min", p_min);
      elastic_tracker::required_parameter(nh, "p_max", p_max);
      elastic_tracker::required_parameter(nh, "p_hit", p_hit);
      elastic_tracker::required_parameter(nh, "p_mis", p_mis);
      elastic_tracker::required_parameter(nh, "p_occ", p_occ);
      elastic_tracker::required_parameter(nh, "p_def", p_def);
      gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);
    }
    gridmap_.inflate_size = inflate_size_;
    // use mask parameter
    use_mask_ = nh.declare_parameter<bool>("use_mask", false);

    gridmap_inflate_pub_ = nh.create_publisher<quadrotor_interfaces::msg::OccMap3d>("~/gridmap_inflate", rclcpp::QoS(1).reliable().transient_local());

    local_pc_pub_ = nh.create_publisher<sensor_msgs::msg::PointCloud2>("~/local_pointcloud", rclcpp::QoS(1).reliable());
    pcl_pub_ = nh.create_publisher<sensor_msgs::msg::PointCloud2>("~/mask_cloud", rclcpp::QoS(10).reliable());

    if (use_global_map_) {
      map_pc_sub_ = nh.create_subscription<sensor_msgs::msg::PointCloud2>("~/global_map", rclcpp::QoS(1).reliable().transient_local(), std::bind(&Nodelet::map_call_back, this, std::placeholders::_1));
      global_map_timer_ = nh.create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&Nodelet::global_map_timer_callback, this));
    } else {
      depth_sub_.subscribe(&nh, "~/depth", rmw_qos_profile_sensor_data);
      odom_sub_.subscribe(&nh, "~/odom", rmw_qos_profile_sensor_data);
      depth_odom_sync_Ptr_ = std::make_shared<ImageOdomSynchronizer>(ImageOdomSyncPolicy(100), depth_sub_, odom_sub_);
      depth_odom_sync_Ptr_->registerCallback(std::bind(&Nodelet::depth_odom_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (use_mask_) {
      target_odom_sub_ = nh.create_subscription<nav_msgs::msg::Odometry>("~/target", rclcpp::QoS(1).reliable(), std::bind(&Nodelet::target_odom_callback, this, std::placeholders::_1));
    }
  }

 public:
  explicit Nodelet(const rclcpp::NodeOptions& options) : rclcpp::Node("mapping", options) { init(*this); }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace mapping


RCLCPP_COMPONENTS_REGISTER_NODE(mapping::Nodelet)

