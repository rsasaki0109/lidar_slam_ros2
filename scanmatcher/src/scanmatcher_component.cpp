#include "scanmatcher/scanmatcher_component.h"
#include "scanmatcher/voxel_grid_safety.hpp"
#include "scanmatcher/odom_prior_utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

using namespace std::chrono_literals;

namespace
{
struct PointCloudExtractionResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud {
    new pcl::PointCloud<pcl::PointXYZI>()
  };
  std::vector<float> point_times {};
};

bool pointCloudHasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

bool checkedMultiply(size_t lhs, size_t rhs, size_t * product)
{
  if (lhs != 0U && rhs > std::numeric_limits<size_t>::max() / lhs) {
    return false;
  }
  *product = lhs * rhs;
  return true;
}

bool validateFloatField(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & name,
  bool required,
  std::string * error)
{
  const sensor_msgs::msg::PointField * match = nullptr;
  for (const auto & field : msg.fields) {
    if (field.name != name) {
      continue;
    }
    if (match != nullptr) {
      *error = "duplicate '" + name + "' field";
      return false;
    }
    match = &field;
  }
  if (match == nullptr) {
    if (required) {
      *error = "missing required '" + name + "' field";
      return false;
    }
    return true;
  }
  if (
    match->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
    match->count < 1U)
  {
    *error = "field '" + name + "' must be FLOAT32 with count >= 1";
    return false;
  }
  if (
    msg.point_step < sizeof(float) ||
    match->offset > msg.point_step - sizeof(float))
  {
    *error = "field '" + name + "' extends past point_step";
    return false;
  }
  return true;
}

bool validatePointCloudLayout(
  const sensor_msgs::msg::PointCloud2 & msg,
  size_t * point_count,
  std::string * error)
{
  *point_count = 0U;
  error->clear();
  if (msg.is_bigendian) {
    *error = "big-endian PointCloud2 fields are not supported";
    return false;
  }
  if (msg.width == 0U || msg.height == 0U) {
    *error = "width and height must both be greater than zero";
    return false;
  }
  if (msg.point_step == 0U) {
    *error = "point_step must be greater than zero";
    return false;
  }

  size_t packed_row_size = 0U;
  if (!checkedMultiply(msg.width, msg.point_step, &packed_row_size)) {
    *error = "width * point_step overflows size_t";
    return false;
  }
  if (msg.row_step < packed_row_size) {
    *error = "row_step must be at least width * point_step";
    return false;
  }

  size_t expected_data_size = 0U;
  if (!checkedMultiply(msg.row_step, msg.height, &expected_data_size)) {
    *error = "row_step * height overflows size_t";
    return false;
  }
  if (msg.data.size() != expected_data_size) {
    *error = "data size must equal row_step * height";
    return false;
  }
  if (!checkedMultiply(msg.width, msg.height, point_count)) {
    *error = "width * height overflows size_t";
    return false;
  }
  if (*point_count > std::numeric_limits<uint32_t>::max()) {
    *error = "flattened cloud exceeds the PCL uint32 width limit";
    return false;
  }

  const bool coordinates_valid =
    validateFloatField(msg, "x", true, error) &&
    validateFloatField(msg, "y", true, error) &&
    validateFloatField(msg, "z", true, error) &&
    validateFloatField(msg, "intensity", false, error) &&
    validateFloatField(msg, "time", false, error);
  if (!coordinates_valid) {
    return false;
  }

  const bool has_normal =
    pointCloudHasField(msg, "normal_x") ||
    pointCloudHasField(msg, "normal_y") ||
    pointCloudHasField(msg, "normal_z");
  return !has_normal ||
         (validateFloatField(msg, "normal_x", true, error) &&
         validateFloatField(msg, "normal_y", true, error) &&
         validateFloatField(msg, "normal_z", true, error));
}

sensor_msgs::msg::PointCloud2 packPointCloudRows(
  const sensor_msgs::msg::PointCloud2 & msg)
{
  const size_t packed_row_size =
    static_cast<size_t>(msg.width) * static_cast<size_t>(msg.point_step);
  if (msg.row_step == packed_row_size) {
    return msg;
  }

  sensor_msgs::msg::PointCloud2 packed = msg;
  packed.row_step = static_cast<uint32_t>(packed_row_size);
  packed.data.resize(packed_row_size * static_cast<size_t>(msg.height));
  for (size_t row = 0U; row < msg.height; ++row) {
    const auto source = msg.data.cbegin() + row * msg.row_step;
    const auto destination = packed.data.begin() + row * packed_row_size;
    std::copy_n(source, packed_row_size, destination);
  }
  return packed;
}

PointCloudExtractionResult extractPointCloudXYZIAndTimes(
  const sensor_msgs::msg::PointCloud2 & msg,
  size_t point_count)
{
  PointCloudExtractionResult result;
  const bool has_intensity_field = pointCloudHasField(msg, "intensity");
  const bool has_time_field = pointCloudHasField(msg, "time");
  result.cloud->points.reserve(point_count);
  if (has_time_field) {
    result.point_times.reserve(point_count);
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  if (has_intensity_field) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(msg, "intensity");
    if (has_time_field) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_time(msg, "time");
      for (size_t index = 0U; index < point_count;
        ++index,
        ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_time)
      {
        pcl::PointXYZI point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.intensity = *iter_intensity;
        result.cloud->points.push_back(point);
        result.point_times.push_back(*iter_time);
      }
    } else {
      for (size_t index = 0U; index < point_count;
        ++index,
        ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
      {
        pcl::PointXYZI point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.intensity = *iter_intensity;
        result.cloud->points.push_back(point);
      }
    }
  } else {
    if (has_time_field) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_time(msg, "time");
      for (size_t index = 0U; index < point_count;
        ++index, ++iter_x, ++iter_y, ++iter_z, ++iter_time)
      {
        pcl::PointXYZI point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.intensity = 0.0f;
        result.cloud->points.push_back(point);
        result.point_times.push_back(*iter_time);
      }
    } else {
      for (size_t index = 0U; index < point_count; ++index, ++iter_x, ++iter_y, ++iter_z) {
        pcl::PointXYZI point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.intensity = 0.0f;
        result.cloud->points.push_back(point);
      }
    }
  }

  result.cloud->width = static_cast<uint32_t>(result.cloud->points.size());
  result.cloud->height = 1;
  result.cloud->is_dense = msg.is_dense;
  return result;
}
}  // namespace

namespace graphslam
{
ScanMatcherComponent::ScanMatcherComponent(const rclcpp::NodeOptions & options)
: Node("scan_matcher", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  double ndt_resolution;
  int ndt_num_threads;
  double gicp_corr_dist_threshold;

  declare_parameter("global_frame_id", "map");
  get_parameter("global_frame_id", global_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("odom_frame_id", "odom");
  get_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method_);
  declare_parameter("ndt_resolution", 5.0);
  get_parameter("ndt_resolution", ndt_resolution);
  double ndt_step_size;
  declare_parameter("ndt_step_size", 0.1);
  get_parameter("ndt_step_size", ndt_step_size);
  declare_parameter("ndt_num_threads", 0);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("ndt_transformation_epsilon", 0.01);
  get_parameter("ndt_transformation_epsilon", ndt_transformation_epsilon_);
  declare_parameter("ndt_max_iterations", 35);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  declare_parameter("ndt_outlier_ratio", 0.55);
  get_parameter("ndt_outlier_ratio", ndt_outlier_ratio_);
  declare_parameter("gicp_corr_dist_threshold", 5.0);
  get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold);
  declare_parameter("trans_for_mapupdate", 1.5);
  get_parameter("trans_for_mapupdate", trans_for_mapupdate_);
  declare_parameter("vg_size_for_input", 0.2);
  get_parameter("vg_size_for_input", vg_size_for_input_);
  declare_parameter("vg_size_for_map", 0.1);
  get_parameter("vg_size_for_map", vg_size_for_map_);
  declare_parameter("min_points_for_scan", 100);
  get_parameter("min_points_for_scan", min_points_for_scan_);
  declare_parameter("use_min_max_filter", false);
  get_parameter("use_min_max_filter", use_min_max_filter_);
  declare_parameter("scan_min_range", 0.1);
  get_parameter("scan_min_range", scan_min_range_);
  declare_parameter("scan_max_range", 100.0);
  get_parameter("scan_max_range", scan_max_range_);
  declare_parameter("scan_period", 0.1);
  get_parameter("scan_period", scan_period_);
  declare_parameter("map_publish_period", 15.0);
  get_parameter("map_publish_period", map_publish_period_);  
  declare_parameter("num_targeted_cloud", 10);
  get_parameter("num_targeted_cloud", num_targeted_cloud_);
  if (num_targeted_cloud_ < 1) {
    std::cout << "num_tareged_cloud should be positive" << std::endl;
    num_targeted_cloud_ = 1;
  }
  declare_parameter("num_recovery_targeted_cloud", 40);
  get_parameter("num_recovery_targeted_cloud", num_recovery_targeted_cloud_);
  if (num_recovery_targeted_cloud_ < 1) {
    std::cout << "num_recovery_targeted_cloud should be positive" << std::endl;
    num_recovery_targeted_cloud_ = 1;
  }
  declare_parameter("use_spatial_local_map", false);
  get_parameter("use_spatial_local_map", use_spatial_local_map_);
  declare_parameter("spatial_local_map_radius", 30.0);
  get_parameter("spatial_local_map_radius", spatial_local_map_radius_);
  declare_parameter("use_voxel_hash_map", false);
  get_parameter("use_voxel_hash_map", use_voxel_hash_map_);
  declare_parameter("voxel_hash_map_voxel_size", 1.0);
  get_parameter("voxel_hash_map_voxel_size", voxel_hash_map_voxel_size_);
  declare_parameter("voxel_hash_map_max_distance", 100.0);
  get_parameter("voxel_hash_map_max_distance", voxel_hash_map_max_distance_);
  declare_parameter("voxel_hash_map_max_points_per_voxel", 20);
  get_parameter("voxel_hash_map_max_points_per_voxel", voxel_hash_map_max_points_per_voxel_);
  if (use_voxel_hash_map_) {
    voxel_hash_map_ = std::make_unique<VoxelHashMapPCL>(
      voxel_hash_map_voxel_size_, voxel_hash_map_max_distance_,
      voxel_hash_map_max_points_per_voxel_);
  }
  declare_parameter("adaptive_correspondence_threshold", false);
  get_parameter("adaptive_correspondence_threshold", adaptive_correspondence_threshold_);
  declare_parameter("adaptive_corr_dist_multiplier", 3.0);
  get_parameter("adaptive_corr_dist_multiplier", adaptive_corr_dist_multiplier_);
  declare_parameter("async_map_update", true);
  get_parameter("async_map_update", async_map_update_);
  declare_parameter("async_map_update_warmup_submaps", 1);
  get_parameter("async_map_update_warmup_submaps", async_map_update_warmup_submaps_);
  declare_parameter("recovery_clear_consecutive_accepted", 1);
  get_parameter("recovery_clear_consecutive_accepted", recovery_clear_consecutive_accepted_);
  if (recovery_clear_consecutive_accepted_ < 1) {
    recovery_clear_consecutive_accepted_ = 1;
  }
  declare_parameter("suspect_clear_consecutive_accepted", 2);
  get_parameter("suspect_clear_consecutive_accepted", suspect_clear_consecutive_accepted_);
  if (suspect_clear_consecutive_accepted_ < 1) {
    suspect_clear_consecutive_accepted_ = 1;
  }

  declare_parameter("initial_pose_x", 0.0);
  get_parameter("initial_pose_x", initial_pose_x_);
  declare_parameter("initial_pose_y", 0.0);
  get_parameter("initial_pose_y", initial_pose_y_);
  declare_parameter("initial_pose_z", 0.0);
  get_parameter("initial_pose_z", initial_pose_z_);
  declare_parameter("initial_pose_qx", 0.0);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  declare_parameter("initial_pose_qy", 0.0);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  declare_parameter("initial_pose_qz", 0.0);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  declare_parameter("initial_pose_qw", 1.0);
  get_parameter("initial_pose_qw", initial_pose_qw_);

  declare_parameter("set_initial_pose", false);
  get_parameter("set_initial_pose", set_initial_pose_);
  declare_parameter("publish_tf", true);
  get_parameter("publish_tf", publish_tf_);
  declare_parameter("use_odom", false);
  get_parameter("use_odom", use_odom_);
  declare_parameter("odom_prior_planar", false);
  get_parameter("odom_prior_planar", odom_prior_planar_);
  declare_parameter("odom_prior_translation_only", false);
  get_parameter("odom_prior_translation_only", odom_prior_translation_only_);
  declare_parameter("odom_prior_suspect_recovery_only", false);
  get_parameter("odom_prior_suspect_recovery_only", odom_prior_suspect_recovery_only_);
  declare_parameter("odom_prior_weight", 1.0);
  get_parameter("odom_prior_weight", odom_prior_weight_);
  odom_prior_weight_ = std::clamp(odom_prior_weight_, 0.0, 1.0);
  declare_parameter("use_imu", false);
  get_parameter("use_imu", use_imu_);
  declare_parameter("imu_translation_deskew", true);
  get_parameter("imu_translation_deskew", imu_translation_deskew_);
  declare_parameter("imu_rotation_deskew_use_orientation", true);
  get_parameter(
    "imu_rotation_deskew_use_orientation",
    imu_rotation_deskew_use_orientation_);
  declare_parameter("imu_pose_prediction_enable", true);
  get_parameter("imu_pose_prediction_enable", imu_pose_prediction_enable_);
  declare_parameter("imu_pose_prediction_max_age", 0.2);
  get_parameter("imu_pose_prediction_max_age", imu_pose_prediction_max_age_);
  declare_parameter("imu_pose_prediction_max_roll_pitch_deg", 12.0);
  get_parameter("imu_pose_prediction_max_roll_pitch_deg", imu_pose_prediction_max_roll_pitch_deg_);
  declare_parameter("imu_pose_prediction_max_yaw_deg", 20.0);
  get_parameter("imu_pose_prediction_max_yaw_deg", imu_pose_prediction_max_yaw_deg_);
  declare_parameter("imu_pose_prediction_weight", 0.0);
  get_parameter("imu_pose_prediction_weight", imu_pose_prediction_weight_);
  if (imu_pose_prediction_weight_ < 0.0) {imu_pose_prediction_weight_ = 0.0;}
  if (imu_pose_prediction_weight_ > 1.0) {imu_pose_prediction_weight_ = 1.0;}
  declare_parameter("imu_complementary_enable", false);
  get_parameter("imu_complementary_enable", imu_complementary_enable_);
  declare_parameter("imu_complementary_alpha", 0.0);
  get_parameter("imu_complementary_alpha", imu_complementary_alpha_);
  if (imu_complementary_alpha_ < 0.0) {imu_complementary_alpha_ = 0.0;}
  if (imu_complementary_alpha_ > 1.0) {imu_complementary_alpha_ = 1.0;}
  declare_parameter("imu_ndt_prior_enable", false);
  get_parameter("imu_ndt_prior_enable", imu_ndt_prior_enable_);
  declare_parameter("imu_ndt_prior_weight", 0.0);
  get_parameter("imu_ndt_prior_weight", imu_ndt_prior_weight_);
  declare_parameter("imu_ndt_prior_roll_pitch_only", true);
  get_parameter("imu_ndt_prior_roll_pitch_only", imu_ndt_prior_roll_pitch_only_);
  declare_parameter("imu_z_prior_enable", false);
  get_parameter("imu_z_prior_enable", imu_z_prior_enable_);
  declare_parameter("imu_z_prior_weight", 0.0);
  get_parameter("imu_z_prior_weight", imu_z_prior_weight_);
  declare_parameter("cloud_queue_depth", 5);
  get_parameter("cloud_queue_depth", cloud_queue_depth_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);
  declare_parameter("debug_cloud_dump_dir", "");
  get_parameter("debug_cloud_dump_dir", debug_cloud_dump_dir_);
  declare_parameter("debug_cloud_dump_max_frames", 0);
  get_parameter("debug_cloud_dump_max_frames", debug_cloud_dump_max_frames_);
  if (debug_cloud_dump_max_frames_ < 0) {
    debug_cloud_dump_max_frames_ = 0;
  }
  declare_parameter("diagnostic_warn_trans_jump", 0.75);
  get_parameter("diagnostic_warn_trans_jump", diagnostic_warn_trans_jump_);
  declare_parameter("diagnostic_warn_yaw_jump_deg", 12.0);
  get_parameter("diagnostic_warn_yaw_jump_deg", diagnostic_warn_yaw_jump_deg_);
  declare_parameter("reject_nonconverged_pose_update", true);
  get_parameter("reject_nonconverged_pose_update", reject_nonconverged_pose_update_);
  declare_parameter("reject_fitness_score", 0.0);
  get_parameter("reject_fitness_score", reject_fitness_score_);
  declare_parameter("reject_fitness_ratio", 2.5);
  get_parameter("reject_fitness_ratio", reject_fitness_ratio_);
  declare_parameter("reject_fitness_only_ratio", 8.0);
  get_parameter("reject_fitness_only_ratio", reject_fitness_only_ratio_);
  declare_parameter("reject_trans_only_ratio", 0.0);
  get_parameter("reject_trans_only_ratio", reject_trans_only_ratio_);
  declare_parameter("reject_trans_streak_scans", 0);
  get_parameter("reject_trans_streak_scans", reject_trans_streak_scans_);
  declare_parameter("reject_fitness_streak_ratio", 0.0);
  get_parameter("reject_fitness_streak_ratio", reject_fitness_streak_ratio_);
  declare_parameter("reject_hard_fitness_ratio", 0.0);
  get_parameter("reject_hard_fitness_ratio", reject_hard_fitness_ratio_);
  declare_parameter("reject_trans_jump", 1.0);
  get_parameter("reject_trans_jump", reject_trans_jump_);
  declare_parameter("reject_trans_jump_ratio", 3.0);
  get_parameter("reject_trans_jump_ratio", reject_trans_jump_ratio_);
  declare_parameter("reject_hard_trans_ratio", 0.0);
  get_parameter("reject_hard_trans_ratio", reject_hard_trans_ratio_);
  declare_parameter("reject_ema_alpha", 0.1);
  get_parameter("reject_ema_alpha", reject_ema_alpha_);
  declare_parameter("motion_gate_enable", true);
  get_parameter("motion_gate_enable", motion_gate_enable_);
  declare_parameter("motion_gate_max_linear_velocity", 8.0);
  get_parameter("motion_gate_max_linear_velocity", motion_gate_max_linear_velocity_);
  declare_parameter("motion_gate_max_yaw_rate_deg", 120.0);
  get_parameter("motion_gate_max_yaw_rate_deg", motion_gate_max_yaw_rate_deg_);
  declare_parameter("motion_gate_hard_multiplier", 4.0);
  get_parameter("motion_gate_hard_multiplier", motion_gate_hard_multiplier_);
  declare_parameter("reject_warmup_scans", 20);
  get_parameter("reject_warmup_scans", reject_warmup_scans_);
  declare_parameter("reject_map_update_cooldown_scans", 2);
  get_parameter("reject_map_update_cooldown_scans", reject_map_update_cooldown_scans_);
  declare_parameter("hard_reject_map_update_cooldown_scans", 4);
  get_parameter("hard_reject_map_update_cooldown_scans", hard_reject_map_update_cooldown_scans_);
  declare_parameter("reject_fitness_streak_scans", 0);
  get_parameter("reject_fitness_streak_scans", reject_fitness_streak_scans_);
  declare_parameter("reject_recovery_scans", 0);
  get_parameter("reject_recovery_scans", reject_recovery_scans_);
  declare_parameter("use_constant_velocity_model", false);
  get_parameter("use_constant_velocity_model", use_constant_velocity_model_);

  std::cout << "registration_method:" << registration_method_ << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_step_size:" << ndt_step_size << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "gicp_corr_dist_threshold[m]:" << gicp_corr_dist_threshold << std::endl;
  std::cout << "trans_for_mapupdate[m]:" << trans_for_mapupdate_ << std::endl;
  std::cout << "vg_size_for_input[m]:" << vg_size_for_input_ << std::endl;
  std::cout << "vg_size_for_map[m]:" << vg_size_for_map_ << std::endl;
  std::cout << "min_points_for_scan:" << min_points_for_scan_ << std::endl;
  std::cout << "use_min_max_filter:" << std::boolalpha << use_min_max_filter_ << std::endl;
  std::cout << "scan_min_range[m]:" << scan_min_range_ << std::endl;
  std::cout << "scan_max_range[m]:" << scan_max_range_ << std::endl;
  std::cout << "set_initial_pose:" << std::boolalpha << set_initial_pose_ << std::endl;
  std::cout << "publish_tf:" << std::boolalpha << publish_tf_ << std::endl;
  std::cout << "use_odom:" << std::boolalpha << use_odom_ << std::endl;
  std::cout << "odom_prior_planar:" << std::boolalpha << odom_prior_planar_ << std::endl;
  std::cout << "odom_prior_translation_only:" << std::boolalpha <<
    odom_prior_translation_only_ << std::endl;
  std::cout << "odom_prior_suspect_recovery_only:" << std::boolalpha <<
    odom_prior_suspect_recovery_only_ << std::endl;
  std::cout << "odom_prior_weight:" << odom_prior_weight_ << std::endl;
  std::cout << "use_imu:" << std::boolalpha << use_imu_ << std::endl;
  std::cout << "imu_translation_deskew:" << std::boolalpha << imu_translation_deskew_ <<
    std::endl;
  std::cout << "imu_rotation_deskew_use_orientation:" << std::boolalpha <<
    imu_rotation_deskew_use_orientation_ << std::endl;
  std::cout << "imu_pose_prediction_enable:" << std::boolalpha << imu_pose_prediction_enable_ <<
    std::endl;
  std::cout << "imu_pose_prediction_max_age[sec]:" << imu_pose_prediction_max_age_ << std::endl;
  std::cout << "imu_pose_prediction_max_roll_pitch_deg[deg]:" <<
    imu_pose_prediction_max_roll_pitch_deg_ << std::endl;
  std::cout << "imu_pose_prediction_max_yaw_deg[deg]:" <<
    imu_pose_prediction_max_yaw_deg_ << std::endl;
  std::cout << "imu_pose_prediction_weight:" << imu_pose_prediction_weight_ << std::endl;
  std::cout << "imu_complementary_enable:" << std::boolalpha << imu_complementary_enable_ <<
    std::endl;
  std::cout << "imu_complementary_alpha:" << imu_complementary_alpha_ << std::endl;
  std::cout << "imu_ndt_prior_enable:" << std::boolalpha << imu_ndt_prior_enable_ << std::endl;
  std::cout << "imu_ndt_prior_weight:" << imu_ndt_prior_weight_ << std::endl;
  std::cout << "imu_ndt_prior_roll_pitch_only:" << std::boolalpha <<
    imu_ndt_prior_roll_pitch_only_ << std::endl;
  std::cout << "use_constant_velocity_model:" << std::boolalpha <<
    use_constant_velocity_model_ << std::endl;
  std::cout << "diagnostic_warn_trans_jump[m]:" << diagnostic_warn_trans_jump_ << std::endl;
  std::cout << "diagnostic_warn_yaw_jump_deg[deg]:" << diagnostic_warn_yaw_jump_deg_ << std::endl;
  std::cout << "reject_nonconverged_pose_update:" << std::boolalpha <<
    reject_nonconverged_pose_update_ << std::endl;
  std::cout << "reject_fitness_score:" << reject_fitness_score_ << std::endl;
  std::cout << "reject_fitness_ratio:" << reject_fitness_ratio_ << std::endl;
  std::cout << "reject_fitness_only_ratio:" << reject_fitness_only_ratio_ << std::endl;
  std::cout << "reject_trans_only_ratio:" << reject_trans_only_ratio_ << std::endl;
  std::cout << "reject_trans_streak_scans:" << reject_trans_streak_scans_ << std::endl;
  std::cout << "reject_fitness_streak_ratio:" << reject_fitness_streak_ratio_ << std::endl;
  std::cout << "reject_hard_fitness_ratio:" << reject_hard_fitness_ratio_ << std::endl;
  std::cout << "reject_trans_jump[m]:" << reject_trans_jump_ << std::endl;
  std::cout << "reject_trans_jump_ratio:" << reject_trans_jump_ratio_ << std::endl;
  std::cout << "reject_hard_trans_ratio:" << reject_hard_trans_ratio_ << std::endl;
  std::cout << "reject_ema_alpha:" << reject_ema_alpha_ << std::endl;
  std::cout << "motion_gate_enable:" << std::boolalpha << motion_gate_enable_ << std::endl;
  std::cout << "motion_gate_max_linear_velocity[m/s]:" << motion_gate_max_linear_velocity_ <<
    std::endl;
  std::cout << "motion_gate_max_yaw_rate_deg[deg/s]:" << motion_gate_max_yaw_rate_deg_ <<
    std::endl;
  std::cout << "motion_gate_hard_multiplier:" << motion_gate_hard_multiplier_ << std::endl;
  std::cout << "reject_warmup_scans:" << reject_warmup_scans_ << std::endl;
  std::cout << "reject_map_update_cooldown_scans:" << reject_map_update_cooldown_scans_ << std::endl;
  std::cout << "hard_reject_map_update_cooldown_scans:" <<
    hard_reject_map_update_cooldown_scans_ << std::endl;
  std::cout << "reject_fitness_streak_scans:" << reject_fitness_streak_scans_ << std::endl;
  std::cout << "reject_recovery_scans:" << reject_recovery_scans_ << std::endl;
  std::cout << "scan_period[sec]:" << scan_period_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "debug_cloud_dump_dir:" << debug_cloud_dump_dir_ << std::endl;
  std::cout << "debug_cloud_dump_max_frames:" << debug_cloud_dump_max_frames_ << std::endl;
  std::cout << "map_publish_period[sec]:" << map_publish_period_ << std::endl;
  std::cout << "num_targeted_cloud:" << num_targeted_cloud_ << std::endl;
  std::cout << "num_recovery_targeted_cloud:" << num_recovery_targeted_cloud_ << std::endl;
  std::cout << "use_spatial_local_map:" << std::boolalpha << use_spatial_local_map_ << std::endl;
  std::cout << "spatial_local_map_radius[m]:" << spatial_local_map_radius_ << std::endl;
  std::cout << "use_voxel_hash_map:" << std::boolalpha << use_voxel_hash_map_ << std::endl;
  if (use_voxel_hash_map_) {
    std::cout << "voxel_hash_map_voxel_size[m]:" << voxel_hash_map_voxel_size_ << std::endl;
    std::cout << "voxel_hash_map_max_distance[m]:" << voxel_hash_map_max_distance_ << std::endl;
    std::cout << "voxel_hash_map_max_points_per_voxel:" << voxel_hash_map_max_points_per_voxel_ << std::endl;
  }
  std::cout << "adaptive_correspondence_threshold:" << std::boolalpha << adaptive_correspondence_threshold_ << std::endl;
  std::cout << "adaptive_corr_dist_multiplier:" << adaptive_corr_dist_multiplier_ << std::endl;
  std::cout << "async_map_update:" << std::boolalpha << async_map_update_ << std::endl;
  std::cout << "async_map_update_warmup_submaps:" << async_map_update_warmup_submaps_ << std::endl;
  std::cout << "recovery_clear_consecutive_accepted:" << recovery_clear_consecutive_accepted_ <<
    std::endl;
  std::cout << "suspect_clear_consecutive_accepted:" << suspect_clear_consecutive_accepted_ <<
    std::endl;
  std::cout << "------------------" << std::endl;

  if (registration_method_ == "NDT") {

    auto ndt = pcl::make_shared<
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt->setMaximumIterations(ndt_max_iterations_);
    ndt->setStepSize(ndt_step_size);
    ndt->setOulierRatio(ndt_outlier_ratio_);
    // ndt_omp
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}

    registration_ = ndt;

  } else if (registration_method_ == "GICP") {
    auto gicp = pcl::make_shared<
      pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
    gicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    gicp->setTransformationEpsilon(1e-8);
    registration_ = gicp;

  }
#ifdef HAS_FAST_GICP
  else if (registration_method_ == "FAST_GICP") {
    using FG = fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>;
    auto fgicp = pcl::make_shared<FG>();
    fgicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    fgicp->setTransformationEpsilon(1e-6);
    fgicp->setMaximumIterations(ndt_max_iterations_);
    if (ndt_num_threads > 0) { fgicp->setNumThreads(ndt_num_threads); }
    registration_ = fgicp;
  } else if (registration_method_ == "FAST_VGICP") {
    using FVG = fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>;
    auto fvgicp = pcl::make_shared<FVG>();
    fvgicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    fvgicp->setTransformationEpsilon(1e-6);
    fvgicp->setMaximumIterations(ndt_max_iterations_);
    fvgicp->setResolution(ndt_resolution);
    if (ndt_num_threads > 0) { fvgicp->setNumThreads(ndt_num_threads); }
    registration_ = fvgicp;
  }
#endif
#ifdef HAS_SMALL_GICP
  else if (registration_method_ == "SMALL_GICP" || registration_method_ == "SMALL_VGICP") {
    using SG = small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>;
    auto sg = pcl::make_shared<SG>();
    if (registration_method_ == "SMALL_VGICP") {
      sg->setRegistrationType("VGICP");
      sg->setVoxelResolution(ndt_resolution);
    } else {
      sg->setRegistrationType("GICP");
    }
    sg->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    sg->setTransformationEpsilon(1e-6);
    sg->setMaximumIterations(ndt_max_iterations_);
    if (ndt_num_threads > 0) { sg->setNumThreads(ndt_num_threads); }
    registration_ = sg;
  }
#endif
  else {
    RCLCPP_ERROR(get_logger(), "invalid registration method: %s", registration_method_.c_str());
    exit(1);
  }

  map_array_msg_.header.frame_id = global_frame_id_;
  map_array_msg_.cloud_coordinate = map_array_msg_.LOCAL;

  path_.header.frame_id = global_frame_id_;

  lidar_undistortion_.setScanPeriod(scan_period_);
  lidar_undistortion_.setUseTranslationDeskew(imu_translation_deskew_);
  lidar_undistortion_.setUseOrientationForRotationDeskew(
    imu_rotation_deskew_use_orientation_);

  initializePubSub();

  if (set_initial_pose_) {
    RCLCPP_INFO(get_logger(), "set initial pose");
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.position.x = initial_pose_x_;
    msg->pose.position.y = initial_pose_y_;
    msg->pose.position.z = initial_pose_z_;
    msg->pose.orientation.x = initial_pose_qx_;
    msg->pose.orientation.y = initial_pose_qy_;
    msg->pose.orientation.z = initial_pose_qz_;
    msg->pose.orientation.w = initial_pose_qw_;
    current_pose_stamped_ = *msg;
    pose_pub_->publish(current_pose_stamped_);
    initial_pose_received_ = true;

    path_.poses.push_back(*msg);
  }

  RCLCPP_INFO(get_logger(), "initialization end");
}

ScanMatcherComponent::~ScanMatcherComponent()
{
  // updateMap captures this; keep the node alive until its worker is done.
  if (mapping_thread_.joinable()) {
    mapping_thread_.join();
  }
}

void ScanMatcherComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
  // sub
  auto initial_pose_callback =
    [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
    {
      if (msg->header.frame_id != global_frame_id_) {
        RCLCPP_WARN(get_logger(), "This initial_pose is not in the global frame");
        return;
      }
      RCLCPP_INFO(get_logger(), "initial_pose is received");

      current_pose_stamped_ = *msg;
      previous_position_.x() = current_pose_stamped_.pose.position.x;
      previous_position_.y() = current_pose_stamped_.pose.position.y;
      previous_position_.z() = current_pose_stamped_.pose.position.z;
      initial_pose_received_ = true;

      pose_pub_->publish(current_pose_stamped_);
    };

  auto cloud_callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      if (!initial_pose_received_)
      {
        RCLCPP_WARN(get_logger(), "initial_pose is not received");
        return;
      }

      size_t point_count = 0U;
      std::string layout_error;
      if (!validatePointCloudLayout(*msg, &point_count, &layout_error)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[POINTCLOUD_LAYOUT_REJECTED] %s; the node remains active",
          layout_error.c_str());
        return;
      }

      sensor_msgs::msg::PointCloud2 transformed_msg;
      try {
        tf2::TimePoint time_point = tf2::TimePoint(
          std::chrono::seconds(msg->header.stamp.sec) +
          std::chrono::nanoseconds(msg->header.stamp.nanosec));
        const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
          robot_frame_id_, msg->header.frame_id, time_point);
        const sensor_msgs::msg::PointCloud2 packed_msg = packPointCloudRows(*msg);
        tf2::doTransform(packed_msg, transformed_msg, transform); // TODO:slow now(https://github.com/ros/geometry2/pull/432)
      } catch (tf2::TransformException & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        return;
      } catch (const std::runtime_error & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[POINTCLOUD_TRANSFORM_REJECTED] %s; the node remains active",
          e.what());
        return;
      }

      PointCloudExtractionResult extracted;
      try {
        extracted = extractPointCloudXYZIAndTimes(transformed_msg, point_count);
      } catch (const std::runtime_error & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[POINTCLOUD_CONVERSION_REJECTED] %s; the node remains active",
          e.what());
        return;
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr = extracted.cloud;
      std::vector<float> point_times = std::move(extracted.point_times);
      int debug_cloud_frame_index = -1;
      reserveDebugCloudDumpFrame(&debug_cloud_frame_index);
      const rclcpp::Time cloud_stamp(msg->header.stamp);
      auto point_times_ptr =
        [&point_times]() -> const std::vector<float> * {
          return point_times.empty() ? nullptr : &point_times;
        };

      dumpDebugCloudStage(
        tmp_ptr, point_times_ptr(), cloud_stamp, debug_cloud_frame_index, "pre_deskew");

      if (use_imu_) {
        double scan_time = msg->header.stamp.sec +
          msg->header.stamp.nanosec * 1e-9;
        if (!point_times.empty()) {
          RCLCPP_INFO_ONCE(
            get_logger(),
            "deskew uses PointCloud2 time field when available"
          );
          lidar_undistortion_.adjustDistortion(tmp_ptr, scan_time, &point_times);
        } else {
          RCLCPP_INFO_ONCE(
            get_logger(),
            "PointCloud2 has no time field; falling back to azimuth-based deskew"
          );
          lidar_undistortion_.adjustDistortion(tmp_ptr, scan_time);
        }
      }
      dumpDebugCloudStage(
        tmp_ptr, point_times_ptr(), cloud_stamp, debug_cloud_frame_index, "post_deskew");

      if (use_min_max_filter_) {
        double r;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr2(new pcl::PointCloud<pcl::PointXYZI>());
        std::vector<float> filtered_point_times;
        if (!point_times.empty()) {
          filtered_point_times.reserve(point_times.size());
        }
        size_t index = 0;
        for (const auto & p : tmp_ptr->points) {
          r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
          if (scan_min_range_ < r && r < scan_max_range_) {
            tmp_ptr2->points.push_back(p);
            if (!point_times.empty() && index < point_times.size()) {
              filtered_point_times.push_back(point_times[index]);
            }
          }
          ++index;
        }
        tmp_ptr = tmp_ptr2;
        tmp_ptr->width = static_cast<uint32_t>(tmp_ptr->points.size());
        tmp_ptr->height = 1;
        tmp_ptr->is_dense = false;
        if (!point_times.empty()) {
          point_times = std::move(filtered_point_times);
        }
      }
      dumpDebugCloudStage(
        tmp_ptr, point_times_ptr(), cloud_stamp, debug_cloud_frame_index, "post_filter");

      // Skip non-monotonic timestamps (e.g. corrupted bags with interleaved data)
      {
        if (last_cloud_stamp_valid_) {
          double dt = (cloud_stamp - last_cloud_stamp_).seconds();
          if (dt < -0.5) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "CLOUD_SKIP_NONMONOTONIC stamp=%.9f prev=%.9f dt=%.3f",
              cloud_stamp.seconds(), last_cloud_stamp_.seconds(), dt);
            return;
          }
        }
        last_cloud_stamp_ = cloud_stamp;
        last_cloud_stamp_valid_ = true;
      }

      if (!initial_cloud_received_) {
        RCLCPP_INFO(get_logger(), "initial_cloud is received");
        if (initializeMap(tmp_ptr, msg->header)) {
          initial_cloud_received_ = true;
          last_map_time_ = clock_.now();
        } else {
          RCLCPP_WARN(get_logger(), "initial_cloud skipped: filtered cloud is empty");
          return;
        }
      }

      receiveCloud(tmp_ptr, msg->header.stamp);

    };

  auto imu_callback =
    [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
    {
      if (initial_pose_received_) {receiveImu(*msg);}
    };

  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseStamped>(
    "initial_pose", rclcpp::QoS(10), initial_pose_callback);

  imu_sub_ =
    create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(), imu_callback);

  auto cloud_qos = rclcpp::SensorDataQoS();
  cloud_qos.keep_last(cloud_queue_depth_);
  input_cloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", cloud_qos, cloud_callback);

  // pub
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "current_pose",
    rclcpp::QoS(10));
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(10));
  map_array_pub_ =
    create_publisher<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(
      rclcpp::KeepLast(
        1)).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10));
}

bool ScanMatcherComponent::reserveDebugCloudDumpFrame(int * frame_index)
{
  if (debug_cloud_dump_max_frames_ <= 0 || debug_cloud_dump_dir_.empty()) {
    return false;
  }
  if (debug_cloud_dump_frame_count_ >= debug_cloud_dump_max_frames_) {
    return false;
  }
  *frame_index = debug_cloud_dump_frame_count_;
  ++debug_cloud_dump_frame_count_;
  return true;
}

void ScanMatcherComponent::dumpDebugCloudStage(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const std::vector<float> * point_times,
  const rclcpp::Time stamp,
  int frame_index,
  const std::string & stage)
{
  if (frame_index < 0 || cloud_ptr == nullptr || debug_cloud_dump_dir_.empty()) {
    return;
  }

  namespace fs = std::filesystem;
  fs::path dump_dir(debug_cloud_dump_dir_);
  std::error_code mkdir_error;
  fs::create_directories(dump_dir, mkdir_error);
  if (mkdir_error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "failed to create debug cloud dump dir %s: %s",
      dump_dir.string().c_str(), mkdir_error.message().c_str());
    return;
  }

  std::ostringstream base_name;
  base_name << std::setfill('0') << std::setw(4) << frame_index
            << "_" << std::fixed << std::setprecision(9)
            << stamp.seconds() << "_" << stage;
  const fs::path pcd_path = dump_dir / (base_name.str() + ".pcd");
  const fs::path json_path = dump_dir / (base_name.str() + ".json");

  pcl::PointCloud<pcl::PointXYZI> serializable_cloud = *cloud_ptr;
  if (
    serializable_cloud.width * serializable_cloud.height !=
    serializable_cloud.points.size())
  {
    serializable_cloud.width = static_cast<uint32_t>(serializable_cloud.points.size());
    serializable_cloud.height = 1;
  }

  if (pcl::io::savePCDFileASCII(pcd_path.string(), serializable_cloud) != 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "failed to save debug cloud %s",
      pcd_path.string().c_str());
    return;
  }

  double time_min = std::numeric_limits<double>::quiet_NaN();
  double time_max = std::numeric_limits<double>::quiet_NaN();
  size_t negative_time_count = 0;
  size_t finite_time_count = 0;
  if (point_times != nullptr) {
    for (float rel_time : *point_times) {
      if (!std::isfinite(rel_time)) {
        continue;
      }
      if (finite_time_count == 0) {
        time_min = rel_time;
        time_max = rel_time;
      } else {
        time_min = std::min(time_min, static_cast<double>(rel_time));
        time_max = std::max(time_max, static_cast<double>(rel_time));
      }
      if (rel_time < 0.0f) {
        ++negative_time_count;
      }
      ++finite_time_count;
    }
  }

  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  if (!cloud_ptr->empty()) {
    pcl::getMinMax3D(*cloud_ptr, min_point, max_point);
  } else {
    min_point = Eigen::Vector4f::Zero();
    max_point = Eigen::Vector4f::Zero();
  }

  std::ofstream json_stream(json_path);
  if (!json_stream.is_open()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "failed to write debug cloud metadata %s",
      json_path.string().c_str());
    return;
  }
  json_stream << "{\n"
              << "  \"frame_index\": " << frame_index << ",\n"
              << "  \"stage\": \"" << stage << "\",\n"
              << "  \"stamp_sec\": " << std::fixed << std::setprecision(9)
              << stamp.seconds() << ",\n"
              << "  \"point_count\": " << cloud_ptr->size() << ",\n"
              << "  \"point_times_present\": "
              << ((point_times != nullptr) ? "true" : "false") << ",\n"
              << "  \"finite_point_time_count\": " << finite_time_count << ",\n"
              << "  \"negative_point_time_count\": " << negative_time_count << ",\n"
              << "  \"point_time_min_sec\": " << time_min << ",\n"
              << "  \"point_time_max_sec\": " << time_max << ",\n"
              << "  \"min_xyz\": [" << min_point.x() << ", " << min_point.y()
              << ", " << min_point.z() << "],\n"
              << "  \"max_xyz\": [" << max_point.x() << ", " << max_point.y()
              << ", " << max_point.z() << "],\n"
              << "  \"pcd_path\": \"" << pcd_path.string() << "\"\n"
              << "}\n";
}

bool ScanMatcherComponent::initializeMap(const pcl::PointCloud <pcl::PointXYZI>::Ptr & tmp_ptr, const std_msgs::msg::Header & header)
{
  RCLCPP_INFO(get_logger(), "create a first map");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  if (!filterVoxelGridSafely(
      tmp_ptr, vg_size_for_map_, *cloud_ptr, "initial_map",
      "vg_size_for_map", false))
  {
    return false;
  }
  if (cloud_ptr->size() < static_cast<size_t>(min_points_for_scan_)) {
    RCLCPP_WARN(
      get_logger(),
      "initial map skipped: filtered cloud has %zu points (< %d)",
      cloud_ptr->size(),
      min_points_for_scan_);
    return false;
  }

  Eigen::Matrix4f sim_trans = getTransformation(current_pose_stamped_.pose);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
  if (transformed_cloud_ptr->empty()) {
    return false;
  }
  // Initialize voxel hash map if enabled
  if (use_voxel_hash_map_ && voxel_hash_map_) {
    Eigen::Vector3d init_pos(
      current_pose_stamped_.pose.position.x,
      current_pose_stamped_.pose.position.y,
      current_pose_stamped_.pose.position.z);
    voxel_hash_map_->update(transformed_cloud_ptr, init_pos);
  }

  registration_->setInputTarget(transformed_cloud_ptr);

  // map (global)
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*transformed_cloud_ptr, map_msg);
  map_msg.header.stamp = header.stamp;
  map_msg.header.frame_id = global_frame_id_;

  // map array (local clouds + global poses)
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_ptr, cloud_msg);
  cloud_msg.header.stamp = header.stamp;
  cloud_msg.header.frame_id = robot_frame_id_;

  lidarslam_msgs::msg::SubMap submap;
  submap.header.stamp = header.stamp;
  submap.header.frame_id = global_frame_id_;
  submap.distance = 0.0;
  submap.pose = current_pose_stamped_.pose;
  submap.cloud = cloud_msg;
  map_array_msg_.header.stamp = header.stamp;
  map_array_msg_.header.frame_id = global_frame_id_;
  map_array_msg_.submaps.push_back(submap);

  map_array_pub_->publish(map_array_msg_);
  map_pub_->publish(map_msg);
  return true;
}

void ScanMatcherComponent::receiveCloud(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const rclcpp::Time stamp)
{
  if (mapping_flag_ && mapping_future_.valid()) {
    auto status = mapping_future_.wait_for(0s);
    if (status == std::future_status::ready) {
      mapping_future_.get();
      pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_ptr;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (is_map_updated_ == true) {
          targeted_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(targeted_cloud_));
          is_map_updated_ = false;
        }
      }
      if (targeted_cloud_ptr) {
        if (!pose_acceptance_state_.recovery_target_active) {
          if (registration_method_ == "NDT") {
            registration_->setInputTarget(targeted_cloud_ptr);
          } else {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_targeted_cloud_ptr(
              new pcl::PointCloud<pcl::PointXYZI>());
            if (filterVoxelGridSafely(
                targeted_cloud_ptr, vg_size_for_input_, *filtered_targeted_cloud_ptr,
                "registration_target", "vg_size_for_input", true))
            {
              registration_->setInputTarget(filtered_targeted_cloud_ptr);
            }
          }
        }
      }
      mapping_flag_ = false;
      if (mapping_thread_.joinable()) {
        mapping_thread_.join();
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  if (!filterVoxelGridSafely(
      cloud_ptr, vg_size_for_input_, *filtered_cloud_ptr, "input_scan",
      "vg_size_for_input", true))
  {
    return;
  }
  if (filtered_cloud_ptr->size() < static_cast<size_t>(min_points_for_scan_)) {
    RCLCPP_WARN(
      get_logger(),
      "filtered input cloud has %zu points (< %d); skipping scan",
      filtered_cloud_ptr->size(),
      min_points_for_scan_);
    return;
  }
  registration_->setInputSource(filtered_cloud_ptr);

  Eigen::Matrix4f sim_trans = getTransformation(current_pose_stamped_.pose);

  const pose_prediction::ImuPredictionConfig imu_prediction_config = makeImuPredictionConfig();
  pose_prediction::ImuObservation imu_observation;
  imu_observation.latest_imu_orientation_valid = latest_imu_orientation_valid_;
  imu_observation.cloud_imu_reference_valid = cloud_imu_reference_valid_;
  imu_observation.cloud_imu_reference_quat = cloud_imu_reference_quat_;
  imu_observation.latest_imu_robot_quat = latest_imu_robot_quat_;
  imu_observation.imu_age_sec = latest_imu_orientation_valid_ ?
    std::abs((stamp - latest_imu_stamp_).seconds()) : 0.0;
  tf2::Quaternion current_orientation_quat;
  tf2::fromMsg(current_pose_stamped_.pose.orientation, current_orientation_quat);

  sim_trans = pose_prediction::applyConstantVelocityPrediction(
    sim_trans,
    use_constant_velocity_model_,
    pose_acceptance_state_.last_accepted_delta_valid,
    pose_acceptance_state_.tracking_state == TrackingState::Tracking,
    pose_acceptance_state_.last_accepted_delta_position);

  // IMU initial guess modification (only when complementary filter is disabled)
  if (!imu_complementary_enable_) {
    sim_trans = pose_prediction::applyImuRollPitchCorrection(
      sim_trans,
      imu_prediction_config,
      imu_observation,
      current_orientation_quat);
    sim_trans = pose_prediction::applyStateGatedImuPrediction(
      sim_trans,
      imu_prediction_config,
      imu_observation,
      current_orientation_quat,
      pose_acceptance_state_.tracking_state != TrackingState::Tracking ||
      pose_acceptance_state_.recovery_target_active);
  }

  if (use_odom_) {
    geometry_msgs::msg::TransformStamped odom_trans;
    bool odom_lookup_ok = true;
    try {
      odom_trans = tfbuffer_.lookupTransform(
        odom_frame_id_, robot_frame_id_, tf2_ros::fromMsg(
          stamp));
    } catch (tf2::TransformException & e) {
      odom_lookup_ok = false;
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    if (odom_lookup_ok) {
      Eigen::Affine3d odom_affine = tf2::transformToEigen(odom_trans);
      Eigen::Matrix4f odom_mat = odom_affine.matrix().cast<float>();
      if (previous_odom_valid_) {
        const bool odom_prior_active =
          !odom_prior_suspect_recovery_only_ ||
          pose_acceptance_state_.tracking_state != TrackingState::Tracking ||
          pose_acceptance_state_.recovery_target_active;
        if (odom_prior_active) {
          const Eigen::Matrix4f odom_delta = previous_odom_mat_.inverse() * odom_mat;
          const Eigen::Matrix4f filtered_odom_delta = odom_prior::filterAndBlendDelta(
            odom_delta,
            odom_prior_planar_,
            odom_prior_translation_only_,
            odom_prior_weight_);
          sim_trans = sim_trans * filtered_odom_delta;
        }
      }
      previous_odom_mat_ = odom_mat;
      previous_odom_valid_ = true;
    }
  }

  // Set IMU rotation prior for NDT cost function
  if (
    imu_ndt_prior_enable_ && imu_ndt_prior_weight_ > 0.0 &&
    use_imu_ && latest_imu_orientation_valid_ && cloud_imu_reference_valid_ &&
    registration_method_ == "NDT")
  {
    const double imu_age = std::abs((stamp - latest_imu_stamp_).seconds());
    if (imu_age <= imu_pose_prediction_max_age_) {
      // IMU-predicted rotation (previous pose + IMU delta) as Euler angles
      // matching NDT's internal convention (XYZ intrinsic)
      const Eigen::Vector3d prior_rpy = pose_prediction::imuPredictedRotationRpy(
        imu_observation,
        current_orientation_quat);
      auto ndt_ptr = pcl::dynamic_pointer_cast<
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
      if (ndt_ptr) {
        ndt_ptr->setRotationPrior(prior_rpy, imu_ndt_prior_weight_,
          imu_ndt_prior_roll_pitch_only_);
      }
    }
  }

  // Set IMU Z-translation prior: constrain z-drift using gravity direction
  if (imu_z_prior_enable_ && imu_z_prior_weight_ > 0.0 && registration_method_ == "NDT") {
    auto ndt_ptr = pcl::dynamic_pointer_cast<
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
    if (ndt_ptr) {
      // Use current z as prior (resist z changes between consecutive frames)
      Eigen::Vector3d z_prior(
        sim_trans(0, 3), sim_trans(1, 3), sim_trans(2, 3));
      Eigen::Vector3d weights(0.0, 0.0, imu_z_prior_weight_);  // z-only
      ndt_ptr->setTranslationPrior(z_prior, weights);
    }
  }

  // Set adaptive correspondence distance before alignment (all methods)
  if (adaptive_correspondence_threshold_ && adaptive_corr_dist_ema_ > 0.0) {
    double max_dist = map_update_policy::adaptiveMaxCorrespondenceDistance(
      adaptive_corr_dist_multiplier_, adaptive_corr_dist_ema_);
    if (registration_method_ == "NDT") {
      auto ndt_ptr = pcl::dynamic_pointer_cast<
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
      if (ndt_ptr) {
        ndt_ptr->setMaxCorrespondenceDistance(max_dist);
      }
    } else {
      registration_->setMaxCorrespondenceDistance(max_dist);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  registration_->align(*output_cloud, sim_trans);
  rclcpp::Time time_align_end = system_clock.now();

  // Clear rotation prior after alignment (NDT only)
  if (registration_method_ == "NDT") {
    auto ndt_ptr = pcl::dynamic_pointer_cast<
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
    if (ndt_ptr) {
      if (imu_ndt_prior_enable_) {
        ndt_ptr->clearRotationPrior();
      }
      if (imu_z_prior_enable_) {
        ndt_ptr->clearTranslationPrior();
      }
    }
  }

  // Update adaptive correspondence distance EMA after alignment (all methods)
  if (adaptive_correspondence_threshold_) {
    double mean_corr = 0.0;
    if (registration_method_ == "NDT") {
      auto ndt_ptr = pcl::dynamic_pointer_cast<
        pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
      if (ndt_ptr) {
        mean_corr = ndt_ptr->getLastMeanCorrespondenceDistance();
        ndt_ptr->setMaxCorrespondenceDistance(0.0);  // Reset for next frame
      }
    } else {
      // For GICP/VGICP methods: use sqrt(fitness) as proxy for mean correspondence distance
      double fitness = registration_->getFitnessScore();
      if (fitness > 0.0) {
        mean_corr = std::sqrt(fitness);
      }
      registration_->setMaxCorrespondenceDistance(
        std::numeric_limits<double>::max());  // Reset for next frame
    }
    adaptive_corr_dist_ema_ = map_update_policy::updateAdaptiveCorrespondenceEma(
      adaptive_corr_dist_ema_, mean_corr, adaptive_corr_dist_ema_alpha_);
  }

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();


  publishMapAndPose(cloud_ptr, final_transformation, stamp);
  if (use_imu_ && latest_imu_orientation_valid_) {
    cloud_imu_reference_quat_ = latest_imu_robot_quat_;
    cloud_imu_reference_stamp_ = latest_imu_stamp_;
    cloud_imu_reference_valid_ = true;
  }

  if (!debug_flag_) {return;}

  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(current_pose_stamped_.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
  std::cout << "trans: " << trans_ << std::endl;
  std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() << "s" <<
    std::endl;
  std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
  std::cout << "initial transformation:" << std::endl;
  std::cout << sim_trans << std::endl;
  std::cout << "has converged: " << registration_->hasConverged() << std::endl;
  std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
  std::cout << "final transformation:" << std::endl;
  std::cout << final_transformation << std::endl;
  std::cout << "rpy" << std::endl;
  std::cout << "roll:" << roll * 180 / M_PI << "," <<
    "pitch:" << pitch * 180 / M_PI << "," <<
    "yaw:" << yaw * 180 / M_PI << std::endl;
  int num_submaps;
  double latest_distance;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    num_submaps = map_array_msg_.submaps.size();
    latest_distance = latest_distance_;
  }
  std::cout << "num_submaps:" << num_submaps << std::endl;
  std::cout << "moving distance:" << latest_distance << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
}

void ScanMatcherComponent::publishMapAndPose(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp)
{

  Eigen::Vector3d position = final_transformation.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  const bool has_converged = registration_->hasConverged();
  const double fitness_score = registration_->getFitnessScore();
  pose_acceptance::Input acceptance_input;
  acceptance_input.position = position;
  acceptance_input.orientation = quat_msg;
  acceptance_input.previous_orientation = current_pose_stamped_.pose.orientation;
  acceptance_input.has_converged = has_converged;
  acceptance_input.fitness_score = fitness_score;
  acceptance_input.stamp_sec = stamp.seconds();
  acceptance_input.previous_stamp_sec = previous_pose_stamp_.seconds();
  // Duration::seconds() on the rclcpp::Time pair, exactly as the historical
  // inline block computed dt (double-double subtraction rounds differently).
  acceptance_input.dt = pose_acceptance_state_.previous_pose_diagnostic_valid ?
    (stamp - previous_pose_stamp_).seconds() : 0.0;
  acceptance_input.mapping_in_progress = mapping_flag_;

  pose_acceptance::Outcome acceptance = pose_acceptance::evaluate(
    makePoseAcceptanceConfig(), pose_acceptance_state_, acceptance_input);
  for (const auto & warn_line : acceptance.warn_lines) {
    RCLCPP_WARN(get_logger(), "%s", warn_line.c_str());
  }
  if (acceptance.request_recovery_target_refresh) {
    // The decision is pure; the registration-target side effect is not.
    pose_acceptance_state_.recovery_target_active = refreshRegistrationTargetFromTargetedCloud();
  }
  previous_pose_stamp_ = stamp;

  const Eigen::Vector3d accepted_position = acceptance.accepted_position;
  const geometry_msgs::msg::Quaternion accepted_quat_msg = acceptance.accepted_quat_msg;
  const bool suppress_map_update = acceptance.suppress_map_update;

  // Post-NDT complementary filter: blend roll/pitch with IMU for output only
  // current_pose_stamped_ keeps raw NDT result for next-frame initial guess
  // published_quat_msg is the blended version for TF/path output
  geometry_msgs::msg::Quaternion published_quat_msg = accepted_quat_msg;
  if (
    imu_complementary_enable_ && imu_complementary_alpha_ > 0.0 &&
    use_imu_ && latest_imu_orientation_valid_ && cloud_imu_reference_valid_ &&
    pose_acceptance_state_.previous_pose_diagnostic_valid)
  {
    const double imu_age = std::abs((stamp - latest_imu_stamp_).seconds());
    if (imu_age <= imu_pose_prediction_max_age_) {
      imu_processing::ComplementaryBlendInput blend_input;
      blend_input.imu_reference_quat = cloud_imu_reference_quat_;
      blend_input.latest_imu_robot_quat = latest_imu_robot_quat_;
      blend_input.accepted_quat_msg = accepted_quat_msg;
      blend_input.previous_published_rotation = ndt_pose_.block<3, 3>(0, 0);
      blend_input.alpha = imu_complementary_alpha_;
      published_quat_msg = imu_processing::blendComplementaryRollPitch(blend_input);
    }
  }
  // Store published rotation for next frame's complementary filter
  if (imu_complementary_enable_) {
    tf2::Quaternion pub_q;
    tf2::fromMsg(published_quat_msg, pub_q);
    Eigen::Quaterniond pub_q_eig(pub_q.w(), pub_q.x(), pub_q.y(), pub_q.z());
    ndt_pose_ = Eigen::Matrix4f::Identity();
    ndt_pose_.block<3, 3>(0, 0) = pub_q_eig.toRotationMatrix().cast<float>();
    ndt_pose_.block<3, 1>(0, 3) = accepted_position.cast<float>();
    ndt_pose_valid_ = true;
  }

  if(publish_tf_){
    geometry_msgs::msg::TransformStamped base_to_map_msg;
    base_to_map_msg.header.stamp = stamp;
    base_to_map_msg.header.frame_id = global_frame_id_;
    base_to_map_msg.child_frame_id = robot_frame_id_;
    base_to_map_msg.transform.translation.x = accepted_position.x();
    base_to_map_msg.transform.translation.y = accepted_position.y();
    base_to_map_msg.transform.translation.z = accepted_position.z();
    base_to_map_msg.transform.rotation = published_quat_msg;

    if(use_odom_){
        geometry_msgs::msg::TransformStamped odom_to_map_msg;
        odom_to_map_msg = calculateMaptoOdomTransform(base_to_map_msg, stamp);
        broadcaster_.sendTransform(odom_to_map_msg);
    }
    else{
      broadcaster_.sendTransform(base_to_map_msg);
    }
  }

  // current_pose_stamped_ stores raw NDT result (not filtered) for next-frame initial guess
  current_pose_stamped_.header.stamp = stamp;
  current_pose_stamped_.pose.position.x = accepted_position.x();
  current_pose_stamped_.pose.position.y = accepted_position.y();
  current_pose_stamped_.pose.position.z = accepted_position.z();
  current_pose_stamped_.pose.orientation = accepted_quat_msg;
  // Publish with complementary-filtered rotation (or raw NDT if filter disabled)
  geometry_msgs::msg::PoseStamped publish_pose = current_pose_stamped_;
  publish_pose.pose.orientation = published_quat_msg;
  pose_pub_->publish(publish_pose);

  path_.poses.push_back(publish_pose);
  path_pub_->publish(path_);

  trans_ = (accepted_position - previous_position_).norm();
  if (
    map_update_policy::shouldTriggerMapUpdate(
      trans_, trans_for_mapupdate_, mapping_flag_, suppress_map_update))
  {
    geometry_msgs::msg::PoseStamped current_pose_stamped;
    current_pose_stamped = current_pose_stamped_;
    previous_position_ = accepted_position;
    const double distance_increment = trans_;
    const bool use_async_map_update = map_update_policy::useAsyncMapUpdate(
      async_map_update_, async_map_update_warmup_submaps_,
      static_cast<int>(map_array_msg_.submaps.size()));
    if (use_async_map_update) {
      mapping_task_ =
        std::packaged_task<void()>(
        std::bind(
          &ScanMatcherComponent::updateMap, this, cloud_ptr,
          final_transformation, current_pose_stamped, distance_increment));
      mapping_future_ = mapping_task_.get_future();
      mapping_thread_ = std::thread(std::move(std::ref(mapping_task_)));
      mapping_flag_ = true;
    } else {
      updateMap(
        cloud_ptr, final_transformation, current_pose_stamped,
        distance_increment);
      mapping_flag_ = false;
    }
  }
}

void ScanMatcherComponent::updateMap(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr,
  const Eigen::Matrix4f final_transformation,
  const geometry_msgs::msg::PoseStamped current_pose_stamped,
  const double distance_increment)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  if (!filterVoxelGridSafely(
      cloud_ptr, vg_size_for_map_, *filtered_cloud_ptr, "map_update",
      "vg_size_for_map", true))
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, final_transformation);

  /* map array */
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*filtered_cloud_ptr, cloud_msg);
  cloud_msg.header.stamp = current_pose_stamped.header.stamp;
  cloud_msg.header.frame_id = robot_frame_id_;

  lidarslam_msgs::msg::SubMap submap;
  submap.header.stamp = current_pose_stamped.header.stamp;
  submap.header.frame_id = global_frame_id_;
  submap.pose = current_pose_stamped.pose;
  submap.cloud = cloud_msg;
  lidarslam_msgs::msg::MapArray map_array_snapshot;
  {
    std::lock_guard<std::mutex> lock(mtx_);

    // Attribute distance to the scan that launched this update, not a later scan.
    latest_distance_ += distance_increment;
    submap.distance = latest_distance_;

    if (use_voxel_hash_map_ && voxel_hash_map_) {
      // VoxelHashMap mode: add points and build target from nearby voxels
      Eigen::Vector3d current_pos(
        current_pose_stamped.pose.position.x,
        current_pose_stamped.pose.position.y,
        current_pose_stamped.pose.position.z);
      voxel_hash_map_->update(transformed_cloud_ptr, current_pos);
      // Use local points within spatial radius for registration target
      double local_radius = map_update_policy::voxelHashLocalRadius(voxel_hash_map_max_distance_);
      auto voxel_cloud = voxel_hash_map_->getLocalPoints(current_pos, local_radius);
      targeted_cloud_.clear();
      targeted_cloud_ += *voxel_cloud;
    } else if (use_spatial_local_map_) {
      // Spatial local map: select submaps within radius of current position
      targeted_cloud_.clear();
      targeted_cloud_ += *transformed_cloud_ptr;
      Eigen::Vector3d current_pos(
        current_pose_stamped.pose.position.x,
        current_pose_stamped.pose.position.y,
        current_pose_stamped.pose.position.z);
      std::vector<Eigen::Vector3d> submap_positions;
      submap_positions.reserve(map_array_msg_.submaps.size());
      for (const auto & submap_entry : map_array_msg_.submaps) {
        submap_positions.emplace_back(
          submap_entry.pose.position.x,
          submap_entry.pose.position.y,
          submap_entry.pose.position.z);
      }
      const std::vector<int> selected_indices =
        map_update_policy::selectSpatialSubmapIndices(
        submap_positions, current_pos, spatial_local_map_radius_, num_targeted_cloud_);
      appendTransformedSubmaps(selected_indices);
    } else {
      // Temporal local map: use N most recent submaps (original behavior)
      targeted_cloud_.clear();
      targeted_cloud_ += *transformed_cloud_ptr;
      const std::vector<int> selected_indices =
        map_update_policy::selectTemporalSubmapIndices(
        static_cast<int>(map_array_msg_.submaps.size()), num_targeted_cloud_);
      appendTransformedSubmaps(selected_indices);
    }

    map_array_msg_.header.stamp = current_pose_stamped.header.stamp;
    map_array_msg_.header.frame_id = global_frame_id_;
    map_array_msg_.submaps.push_back(submap);
    map_array_snapshot = map_array_msg_;
    is_map_updated_ = true;
  }
  map_array_pub_->publish(map_array_snapshot);

  rclcpp::Time map_time = clock_.now();
  double dt = map_time.seconds() - last_map_time_.seconds();
  if (map_update_policy::shouldPublishMap(dt, map_publish_period_)) {
    publishMap(map_array_snapshot, global_frame_id_);
    last_map_time_ = map_time;
  }
}

// Decode, transform into the map frame, and append the selected submaps to
// targeted_cloud_. Cloud I/O for the pure index selection in
// map_update_policy.hpp; expects mtx_ to be held by the caller.
void ScanMatcherComponent::appendTransformedSubmaps(const std::vector<int> & submap_indices)
{
  for (const int i : submap_indices) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg_.submaps[i].cloud, *tmp_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Affine3d submap_affine;
    tf2::fromMsg(map_array_msg_.submaps[i].pose, submap_affine);
    pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
    targeted_cloud_ += *transformed_tmp_ptr;
  }
}

bool ScanMatcherComponent::refreshRegistrationTargetFromTargetedCloud()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_ptr;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    targeted_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    int num_submaps = map_array_msg_.submaps.size();

    if (use_spatial_local_map_) {
      // Spatial recovery map: select submaps within larger radius
      Eigen::Vector3d current_pos(
        current_pose_stamped_.pose.position.x,
        current_pose_stamped_.pose.position.y,
        current_pose_stamped_.pose.position.z);
      int added = 0;
      double recovery_radius = spatial_local_map_radius_ * 2.0;
      for (int i = num_submaps - 1; i >= 0 && added < num_recovery_targeted_cloud_; i--) {
        Eigen::Vector3d submap_pos(
          map_array_msg_.submaps[i].pose.position.x,
          map_array_msg_.submaps[i].pose.position.y,
          map_array_msg_.submaps[i].pose.position.z);
        double dist = (submap_pos - current_pos).norm();
        if (dist <= recovery_radius) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
          pcl::fromROSMsg(map_array_msg_.submaps[i].cloud, *tmp_ptr);
          pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
          Eigen::Affine3d submap_affine;
          tf2::fromMsg(map_array_msg_.submaps[i].pose, submap_affine);
          pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
          *targeted_cloud_ptr += *transformed_tmp_ptr;
          added++;
        }
      }
    } else {
      for (int i = 0; i < num_recovery_targeted_cloud_; i++) {
        if (num_submaps - 1 - i < 0) {continue;}
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(map_array_msg_.submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Affine3d submap_affine;
        tf2::fromMsg(map_array_msg_.submaps[num_submaps - 1 - i].pose, submap_affine);
        pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
        *targeted_cloud_ptr += *transformed_tmp_ptr;
      }
    }
    if (targeted_cloud_ptr->empty() && !targeted_cloud_.empty()) {
      targeted_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(targeted_cloud_));
      RCLCPP_WARN(get_logger(), "POSE_REJECT_RECOVERY using tracking target fallback");
    }
    if (targeted_cloud_ptr->empty()) {
      RCLCPP_WARN(get_logger(), "POSE_REJECT_RECOVERY skipped: recovery target is empty");
      return false;
    }
  }

  if (targeted_cloud_ptr->empty()) {
    RCLCPP_WARN(get_logger(), "POSE_REJECT_RECOVERY skipped: recovery target is empty");
    return false;
  }
  if (registration_method_ == "NDT") {
    registration_->setInputTarget(targeted_cloud_ptr);
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_targeted_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    if (!filterVoxelGridSafely(
        targeted_cloud_ptr, vg_size_for_input_, *filtered_targeted_cloud_ptr,
        "recovery_target", "vg_size_for_input", true))
    {
      return false;
    }
    registration_->setInputTarget(filtered_targeted_cloud_ptr);
  }
  return true;
}

bool ScanMatcherComponent::filterVoxelGridSafely(
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr input,
  double leaf_size,
  pcl::PointCloud<pcl::PointXYZI> & output,
  const char * stage,
  const char * parameter_name,
  bool throttle_warning)
{
  const voxel_grid_safety::Report report =
    voxel_grid_safety::filter<pcl::PointXYZI>(input, leaf_size, output);
  if (report.safe()) {
    return true;
  }

  const std::string message = voxel_grid_safety::formatRejection(
    report, stage, parameter_name);
  if (throttle_warning) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "%s", message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
  return false;
}

const char * ScanMatcherComponent::trackingStateName(TrackingState state) const
{
  switch (state) {
    case TrackingState::Tracking:
      return "tracking";
    case TrackingState::Suspect:
      return "suspect";
    case TrackingState::Recovery:
      return "recovery";
  }
  return "unknown";
}

Eigen::Matrix4f ScanMatcherComponent::getTransformation(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f sim_trans = affine.matrix().cast<float>();
  return sim_trans;
}

pose_acceptance::Config ScanMatcherComponent::makePoseAcceptanceConfig() const
{
  pose_acceptance::Config config;
  config.diagnostic_warn_trans_jump = diagnostic_warn_trans_jump_;
  config.diagnostic_warn_yaw_jump_deg = diagnostic_warn_yaw_jump_deg_;
  config.reject_nonconverged_pose_update = reject_nonconverged_pose_update_;
  config.reject_fitness_score = reject_fitness_score_;
  config.reject_fitness_ratio = reject_fitness_ratio_;
  config.reject_fitness_only_ratio = reject_fitness_only_ratio_;
  config.reject_trans_only_ratio = reject_trans_only_ratio_;
  config.reject_trans_streak_scans = reject_trans_streak_scans_;
  config.reject_fitness_streak_ratio = reject_fitness_streak_ratio_;
  config.reject_fitness_streak_scans = reject_fitness_streak_scans_;
  config.reject_hard_fitness_ratio = reject_hard_fitness_ratio_;
  config.reject_trans_jump = reject_trans_jump_;
  config.reject_trans_jump_ratio = reject_trans_jump_ratio_;
  config.reject_hard_trans_ratio = reject_hard_trans_ratio_;
  config.reject_ema_alpha = reject_ema_alpha_;
  config.motion_gate_enable = motion_gate_enable_;
  config.motion_gate_max_linear_velocity = motion_gate_max_linear_velocity_;
  config.motion_gate_max_yaw_rate_deg = motion_gate_max_yaw_rate_deg_;
  config.motion_gate_hard_multiplier = motion_gate_hard_multiplier_;
  config.reject_warmup_scans = reject_warmup_scans_;
  config.reject_map_update_cooldown_scans = reject_map_update_cooldown_scans_;
  config.hard_reject_map_update_cooldown_scans = hard_reject_map_update_cooldown_scans_;
  config.reject_recovery_scans = reject_recovery_scans_;
  config.recovery_clear_consecutive_accepted = recovery_clear_consecutive_accepted_;
  config.suspect_clear_consecutive_accepted = suspect_clear_consecutive_accepted_;
  config.scan_period = scan_period_;
  config.robot_frame_id = robot_frame_id_;
  return config;
}

pose_prediction::ImuPredictionConfig ScanMatcherComponent::makeImuPredictionConfig() const
{
  pose_prediction::ImuPredictionConfig config;
  config.use_imu = use_imu_;
  config.imu_pose_prediction_enable = imu_pose_prediction_enable_;
  config.imu_pose_prediction_weight = imu_pose_prediction_weight_;
  config.imu_pose_prediction_max_age = imu_pose_prediction_max_age_;
  config.imu_pose_prediction_max_roll_pitch_deg = imu_pose_prediction_max_roll_pitch_deg_;
  config.imu_pose_prediction_max_yaw_deg = imu_pose_prediction_max_yaw_deg_;
  return config;
}

void ScanMatcherComponent::receiveImu(const sensor_msgs::msg::Imu msg)
{
  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu imu_msg = msg;

  // If the IMU frame differs from robot_frame_id_, rotate IMU vectors into robot_frame_id_.
  std::string imu_frame_id = imu_msg.header.frame_id;
  if (imu_frame_id.empty()) {imu_frame_id = robot_frame_id_;}

  tf2::Quaternion q_robot_imu(0.0, 0.0, 0.0, 1.0);
  bool have_imu_tf = (imu_frame_id == robot_frame_id_);

  if (!have_imu_tf) {
    // The TF lookup (with its stamped -> latest-static -> identity fallback
    // ladder) is a shell concern; the vector rotation itself is pure.
    bool rotate_into_robot_frame = false;
    try {
      tf2::TimePoint time_point;
      if (imu_msg.header.stamp.sec == 0 && imu_msg.header.stamp.nanosec == 0) {
        time_point = tf2::TimePointZero;
      } else {
        time_point = tf2::TimePoint(
          std::chrono::seconds(imu_msg.header.stamp.sec) +
          std::chrono::nanoseconds(imu_msg.header.stamp.nanosec));
      }
      const geometry_msgs::msg::TransformStamped tf = tfbuffer_.lookupTransform(
        robot_frame_id_, imu_frame_id, time_point);
      tf2::fromMsg(tf.transform.rotation, q_robot_imu);
      rotate_into_robot_frame = true;
      have_imu_tf = true;
    } catch (tf2::TransformException & e) {
      try {
        const geometry_msgs::msg::TransformStamped tf = tfbuffer_.lookupTransform(
          robot_frame_id_, imu_frame_id, tf2::TimePointZero);
        tf2::fromMsg(tf.transform.rotation, q_robot_imu);
        rotate_into_robot_frame = true;
        have_imu_tf = true;
        RCLCPP_WARN_ONCE(
          get_logger(),
          "IMU transform (%s -> %s) unavailable at stamp: %s. Falling back to latest available static TF.",
          imu_frame_id.c_str(), robot_frame_id_.c_str(), e.what());
      } catch (tf2::TransformException & latest_e) {
        RCLCPP_WARN_ONCE(
          get_logger(),
          "IMU transform (%s -> %s) unavailable: %s. Static fallback also failed: %s. Assuming IMU data is already in %s.",
          imu_frame_id.c_str(), robot_frame_id_.c_str(), e.what(), latest_e.what(),
          robot_frame_id_.c_str());
        q_robot_imu = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
        have_imu_tf = false;
      }
    }
    if (rotate_into_robot_frame) {
      imu_processing::ImuVectors imu_vectors;
      imu_vectors.angular_velocity = tf2::Vector3(
        imu_msg.angular_velocity.x,
        imu_msg.angular_velocity.y,
        imu_msg.angular_velocity.z);
      imu_vectors.linear_acceleration = tf2::Vector3(
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z);
      const imu_processing::ImuVectors robot_vectors =
        imu_processing::rotateImuVectorsIntoRobotFrame(imu_vectors, q_robot_imu);
      imu_msg.angular_velocity.x = robot_vectors.angular_velocity.x();
      imu_msg.angular_velocity.y = robot_vectors.angular_velocity.y();
      imu_msg.angular_velocity.z = robot_vectors.angular_velocity.z();
      imu_msg.linear_acceleration.x = robot_vectors.linear_acceleration.x();
      imu_msg.linear_acceleration.y = robot_vectors.linear_acceleration.y();
      imu_msg.linear_acceleration.z = robot_vectors.linear_acceleration.z();
    }
  }

  // Determine orientation (roll/pitch) in robot_frame_id_. If orientation is missing, estimate it from acceleration.
  tf2::Quaternion q_world_imu;
  tf2::fromMsg(imu_msg.orientation, q_world_imu);

  const double imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;

  imu_processing::OrientationInput orientation_input;
  orientation_input.q_world_imu = q_world_imu;
  orientation_input.orientation_covariance0 = imu_msg.orientation_covariance[0];
  orientation_input.linear_acceleration_x = imu_msg.linear_acceleration.x;
  orientation_input.linear_acceleration_y = imu_msg.linear_acceleration.y;
  orientation_input.linear_acceleration_z = imu_msg.linear_acceleration.z;
  orientation_input.angular_velocity_z = imu_msg.angular_velocity.z;
  orientation_input.imu_time = imu_time;
  orientation_input.have_imu_tf = have_imu_tf;
  orientation_input.frame_differs = imu_frame_id != robot_frame_id_;
  orientation_input.q_robot_imu = q_robot_imu;

  const imu_processing::OrientationResult orientation =
    imu_processing::resolveOrientation(orientation_input, imu_orientation_state_);

  latest_imu_robot_quat_ = orientation.q_world_robot;
  latest_imu_stamp_ = imu_msg.header.stamp;
  latest_imu_orientation_valid_ = true;

  Eigen::Vector3f angular_velo{
    static_cast<float>(imu_msg.angular_velocity.x),
    static_cast<float>(imu_msg.angular_velocity.y),
    static_cast<float>(imu_msg.angular_velocity.z)};
  Eigen::Vector3f acc = imu_processing::gravityCompensatedAcceleration(
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z,
    orientation.roll,
    orientation.pitch);
  Eigen::Quaternionf quat{
    static_cast<float>(orientation.q_world_robot.w()),
    static_cast<float>(orientation.q_world_robot.x()),
    static_cast<float>(orientation.q_world_robot.y()),
    static_cast<float>(orientation.q_world_robot.z())};

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);
}

void ScanMatcherComponent::publishMap(const lidarslam_msgs::msg::MapArray & map_array_msg , const std::string & map_frame_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto & submap : map_array_msg.submaps) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);

    Eigen::Affine3d affine;
    tf2::fromMsg(submap.pose, affine);
    pcl::transformPointCloud(
      *submap_cloud_ptr, *transformed_submap_cloud_ptr,
      affine.matrix().cast<float>());

    *map_ptr += *transformed_submap_cloud_ptr;
  }
  RCLCPP_INFO(get_logger(), "publish a map, number of points in the map : %zu", map_ptr->size());

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*map_ptr, map_msg);
  map_msg.header.frame_id = map_frame_id;
  map_msg.header.stamp = map_array_msg.header.stamp;
  map_pub_->publish(map_msg);
}

geometry_msgs::msg::TransformStamped ScanMatcherComponent::calculateMaptoOdomTransform(
  const geometry_msgs::msg::TransformStamped &base_to_map_msg,
  const rclcpp::Time stamp
)
{
  geometry_msgs::msg::TransformStamped odom_to_map_msg;
  try {
    geometry_msgs::msg::PoseStamped odom_to_map;
    geometry_msgs::msg::PoseStamped base_to_map;

    tf2::Transform odom_to_map_tf;
    tf2::Transform base_to_map_msg_tf;
    base_to_map.header.frame_id = robot_frame_id_;

    tf2::fromMsg(base_to_map_msg.transform, base_to_map_msg_tf);
    tf2::toMsg(base_to_map_msg_tf.inverse(), base_to_map.pose);
    tfbuffer_.transform(base_to_map, odom_to_map, odom_frame_id_);
    tf2::impl::Converter<true, false>::convert(odom_to_map.pose, odom_to_map_tf);
    tf2::impl::Converter<false, true>::convert(odom_to_map_tf.inverse(), odom_to_map_msg.transform);

    odom_to_map_msg.header.stamp = stamp;
    odom_to_map_msg.header.frame_id = global_frame_id_ ;
    odom_to_map_msg.child_frame_id = odom_frame_id_;
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform from base_link to odom failed: %s", e.what());
  }
  return odom_to_map_msg;
}

} // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)
