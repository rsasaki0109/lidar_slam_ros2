// small_gicp scan-to-model odometry ROS2 node
// Uses IncrementalVoxelMap + GICP with KISS-ICP-style robustness
// MIT License (small_gicp: MIT, this wrapper: same as lidarslam)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <small_gicp/ann/incremental_voxelmap.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/icp_factor.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class SmallGicpOdomNode : public rclcpp::Node
{
public:
  SmallGicpOdomNode() : Node("small_gicp_odom")
  {
    downsampling_resolution_ = declare_parameter("downsampling_resolution", 0.25);
    voxel_resolution_ = declare_parameter("voxel_resolution", 1.0);
    max_correspondence_distance_ = declare_parameter("max_correspondence_distance", 1.0);
    num_neighbors_ = declare_parameter("num_neighbors", 20);
    num_threads_ = declare_parameter("num_threads", 4);
    max_range_ = declare_parameter("max_range", 100.0);
    min_range_ = declare_parameter("min_range", 1.0);
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    lidar_frame_ = declare_parameter("lidar_frame", std::string("lidar"));
    publish_tf_ = declare_parameter("publish_tf", true);
    min_motion_threshold_ = declare_parameter("min_motion_threshold", 0.1);
    use_gicp_ = declare_parameter("use_gicp", false);  // false = ICP (fast), true = GICP (needs covariance)

    RCLCPP_INFO(get_logger(),
      "small_gicp_odom: ds=%.2f voxel=%.2f corr=%.2f threads=%d range=[%.1f,%.1f] mode=%s",
      downsampling_resolution_, voxel_resolution_, max_correspondence_distance_,
      num_threads_, min_range_, max_range_, use_gicp_ ? "GICP" : "ICP");

    auto qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", qos,
      std::bind(&SmallGicpOdomNode::cloudCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    T_world_lidar_ = Eigen::Isometry3d::Identity();
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Non-monotonic timestamp handling
    // Note: Newer College bag has interleaved timestamps.
    // We don't skip frames but reset CV prediction on large backward jumps.
    rclcpp::Time cloud_stamp(msg->header.stamp);
    if (last_stamp_valid_) {
      double dt = (cloud_stamp - last_stamp_).seconds();
      if (dt < -1.0) {
        // Large backward timestamp jump: reset velocity prediction
        last_delta_ = Eigen::Isometry3d::Identity();
        last_delta_valid_ = false;
      }
    }
    last_stamp_ = cloud_stamp;
    last_stamp_valid_ = true;

    // Convert and filter
    auto points = std::make_shared<small_gicp::PointCloud>();
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *pcl_cloud);
      points->points.reserve(pcl_cloud->size());
      for (const auto& p : pcl_cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (r < min_range_ || r > max_range_) continue;
        points->points.emplace_back(p.x, p.y, p.z, 1.0);
      }
    }
    if (points->points.size() < 100) return;

    // Downsample
    auto downsampled = small_gicp::voxelgrid_sampling(*points, downsampling_resolution_);

    // Covariance estimation (only for GICP mode - expensive!)
    if (use_gicp_) {
      small_gicp::estimate_covariances_omp(*downsampled, num_neighbors_, num_threads_);
    }

    // First frame
    if (!voxelmap_) {
      voxelmap_ = std::make_shared<small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerCov>>(
        voxel_resolution_);
      voxelmap_->insert(*downsampled);
      publishOdom(msg->header);
      RCLCPP_INFO(get_logger(), "Map initialized with %zu points", downsampled->size());
      return;
    }

    // Prediction: constant velocity (with safety check)
    Eigen::Isometry3d prediction = T_world_lidar_;
    if (last_delta_valid_) {
      double delta_trans = last_delta_.translation().norm();
      if (std::isfinite(delta_trans) && delta_trans < 2.0) {
        prediction = T_world_lidar_ * last_delta_;
      }
    }

    // Adaptive correspondence distance (KISS-ICP style: 3*sigma)
    double max_dist = max_correspondence_distance_;
    if (adaptive_samples_ > 0) {
      double sigma = std::sqrt(adaptive_sse_ / adaptive_samples_);
      max_dist = std::clamp(3.0 * sigma, 0.3, max_correspondence_distance_);
    }

    // Registration (ICP or GICP)
    small_gicp::RegistrationResult result;
    double max_dist_sq = max_dist * max_dist;

    if (use_gicp_) {
      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> reg;
      reg.rejector.max_dist_sq = max_dist_sq;
      reg.optimizer.max_iterations = 20;
      reg.reduction.num_threads = num_threads_;
      result = reg.align(*voxelmap_, *downsampled, *voxelmap_, prediction);
    } else {
      small_gicp::Registration<small_gicp::ICPFactor, small_gicp::ParallelReductionOMP> reg;
      reg.rejector.max_dist_sq = max_dist_sq;
      reg.optimizer.max_iterations = 20;
      reg.reduction.num_threads = num_threads_;
      result = reg.align(*voxelmap_, *downsampled, *voxelmap_, prediction);
    }

    Eigen::Isometry3d new_pose = result.T_target_source;
    Eigen::Isometry3d delta = T_world_lidar_.inverse() * new_pose;
    double trans_jump = delta.translation().norm();

    // Validate: reject NaN, Inf, large jumps
    bool valid = std::isfinite(new_pose.translation().x()) &&
                 std::isfinite(new_pose.translation().y()) &&
                 std::isfinite(new_pose.translation().z()) &&
                 trans_jump < 5.0 &&
                 result.num_inliers >= 20;

    if (valid) {
      // Update adaptive threshold (only when motion exceeds noise floor)
      Eigen::Isometry3d model_deviation = prediction.inverse() * new_pose;
      double dev_trans = model_deviation.translation().norm();
      if (dev_trans > min_motion_threshold_) {
        adaptive_sse_ += dev_trans * dev_trans;
        adaptive_samples_++;
      }

      // Update state
      last_delta_ = delta;
      last_delta_valid_ = true;
      T_world_lidar_ = new_pose;

      // Insert into map (only on good registration)
      voxelmap_->insert(*downsampled, T_world_lidar_);
    } else {
      // Bad registration: fall back to prediction, don't update map
      T_world_lidar_ = prediction;
      last_delta_ = Eigen::Isometry3d::Identity();
      last_delta_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Rejected: inliers=%zu jump=%.2f valid=%d", result.num_inliers, trans_jump, valid);
    }

    publishOdom(msg->header);

    frame_count_++;
    if (frame_count_ % 200 == 0) {
      double sigma = adaptive_samples_ > 0 ? std::sqrt(adaptive_sse_ / adaptive_samples_) : 0;
      RCLCPP_INFO(get_logger(), "Frame %d: sigma=%.3f max_dist=%.3f inliers=%zu",
        frame_count_, sigma, max_dist, result.num_inliers);
    }
  }

  void publishOdom(const std_msgs::msg::Header& header)
  {
    Eigen::Vector3d t = T_world_lidar_.translation();
    if (!std::isfinite(t.x()) || !std::isfinite(t.y()) || !std::isfinite(t.z())) return;
    Eigen::Quaterniond q(T_world_lidar_.rotation());

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = header.frame_id.empty() ? lidar_frame_ : header.frame_id;
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = odom_msg.header;
      tf_msg.child_frame_id = odom_msg.child_frame_id;
      tf_msg.transform.translation.x = t.x();
      tf_msg.transform.translation.y = t.y();
      tf_msg.transform.translation.z = t.z();
      tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  // Parameters
  double downsampling_resolution_, voxel_resolution_, max_correspondence_distance_;
  int num_neighbors_, num_threads_;
  double max_range_, min_range_, min_motion_threshold_;
  std::string odom_frame_, lidar_frame_;
  bool publish_tf_, use_gicp_;

  // State
  small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerCov>::Ptr voxelmap_;
  Eigen::Isometry3d T_world_lidar_;
  Eigen::Isometry3d last_delta_ = Eigen::Isometry3d::Identity();
  bool last_delta_valid_ = false;
  double adaptive_sse_ = 0.0;
  int adaptive_samples_ = 0;
  int frame_count_ = 0;
  rclcpp::Time last_stamp_ {0, 0, RCL_ROS_TIME};
  bool last_stamp_valid_ = false;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmallGicpOdomNode>());
  rclcpp::shutdown();
  return 0;
}
