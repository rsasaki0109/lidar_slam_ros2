#pragma once

#include <algorithm>
#include <iostream>
#include <queue>
#include <memory>
#include <chrono>
#include <Eigen/Dense>

#include <discrete_transformation/discrete_transformation.hpp>

namespace cpu {
class VoxelMaps;

struct AngularInfo {
  Eigen::Vector3i num_division;
  Eigen::Vector3d rpy_res;
  Eigen::Vector3d min_rpy;
};

class BBS3D {
public:
  BBS3D();
  ~BBS3D();

  void set_tar_points(const std::vector<Eigen::Vector3d>& points, double min_level_res, int max_level);

  void set_src_points(const std::vector<Eigen::Vector3d>& points);

  void set_trans_search_range(const std::vector<Eigen::Vector3d>& points);

  void set_trans_search_range(const Eigen::Vector3d& min_xyz, const Eigen::Vector3d& max_xyz);

  void set_angular_search_range(const Eigen::Vector3d& min_rpy, const Eigen::Vector3d& max_rpy) {
    min_rpy_ = min_rpy;
    max_rpy_ = max_rpy;
  }

  void set_voxel_expantion_rate(const double rate) {
    v_rate_ = rate;
    inv_v_rate_ = 1.0 / rate;
  }

  void set_num_threads(const int num_threads) { num_threads_ = num_threads; }

  void set_score_threshold_percentage(double percentage) { score_threshold_percentage_ = percentage; }

  void enable_timeout() { use_timeout_ = true; }

  void disable_timeout() { use_timeout_ = false; }

  void set_timeout_duration_in_msec(const int msec);

  std::vector<Eigen::Vector3d> get_src_points() const { return src_points_; }

  bool set_voxelmaps_coords(const std::string& folder_path);

  std::pair<Eigen::Vector3d, Eigen::Vector3d> get_trans_search_range() const {
    return std::pair<Eigen::Vector3d, Eigen::Vector3d>{min_xyz_, max_xyz_};
  }

  std::vector<Eigen::Vector3d> get_angular_search_range() const { return std::vector<Eigen::Vector3d>{min_rpy_, max_rpy_}; }

  Eigen::Matrix4d get_global_pose() const { return global_pose_; }

  int get_best_score() const { return best_score_; }

  double get_elapsed_time() const { return elapsed_time_; }

  double get_best_score_percentage() {
    if (src_points_.size() == 0)
      return 0.0;
    else
      return static_cast<double>(best_score_) / src_points_.size();
  };

  bool has_timed_out() { return has_timed_out_; }

  bool has_localized() { return has_localized_; }

  void localize();

  // pcd iof
  bool load_voxel_params(const std::string& voxelmaps_folder_path);

  bool set_multi_buckets(const std::string& folder_path);

  bool save_voxelmaps_pcd(const std::string& folder_path);

  bool save_voxel_params(const std::string& folder_path);

private:
  void calc_angular_info(std::vector<AngularInfo>& ang_info_vec);

  std::vector<DiscreteTransformation<double>> create_init_transset(const AngularInfo& init_ang_info);

  void calc_score(
    DiscreteTransformation<double>& trans,
    const double trans_res,
    const Eigen::Vector3d& rpy_res,
    const Eigen::Vector3d& min_rpy,
    const std::vector<Eigen::Vector4i>& buckets,
    const int max_bucket_scan_count,
    const std::vector<Eigen::Vector3d>& points);

private:
  Eigen::Matrix4d global_pose_;
  bool has_timed_out_, has_localized_;
  double elapsed_time_;

  std::vector<Eigen::Vector3d> src_points_;

  std::unique_ptr<VoxelMaps> voxelmaps_ptr_;
  std::string voxelmaps_folder_name_;

  double v_rate_;  // voxel expansion rate
  double inv_v_rate_;
  int num_threads_;

  int best_score_;
  double score_threshold_percentage_;
  bool use_timeout_;
  std::chrono::milliseconds timeout_duration_;
  Eigen::Vector3d min_xyz_;
  Eigen::Vector3d max_xyz_;
  Eigen::Vector3d min_rpy_;
  Eigen::Vector3d max_rpy_;
  std::pair<int, int> init_tx_range_;
  std::pair<int, int> init_ty_range_;
  std::pair<int, int> init_tz_range_;
};

}  // namespace cpu