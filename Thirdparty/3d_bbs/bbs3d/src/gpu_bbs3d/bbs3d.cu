#include <gpu_bbs3d/voxelmaps.cuh>
#include <gpu_bbs3d/bbs3d.cuh>
#include <gpu_bbs3d/stream_manager/check_error.cuh>
#include <discrete_transformation/discrete_transformation.hpp>

namespace gpu {
BBS3D::BBS3D()
: v_rate_(2.0f),
  branch_copy_size_(10000),
  score_threshold_percentage_(0.0),
  use_timeout_(false),
  timeout_duration_(10000),
  has_timed_out_(false),
  has_localized_(false),
  voxelmaps_folder_name_("voxelmaps_coords") {
  check_error << cudaStreamCreate(&stream);

  inv_v_rate_ = 1.0f / v_rate_;
  min_rpy_ << -0.02f, -0.02f, 0.0f;
  max_rpy_ << 0.02f, 0.02f, 2 * M_PI;
}

BBS3D::~BBS3D() {
  check_error << cudaStreamDestroy(stream);
}

void BBS3D::set_timeout_duration_in_msec(const int msec) {
  timeout_duration_ = std::chrono::milliseconds(msec);
}

void BBS3D::set_tar_points(const std::vector<Eigen::Vector3f>& points, float min_level_res, int max_level) {
  voxelmaps_ptr_.reset(new VoxelMaps);
  voxelmaps_ptr_->set_min_res(min_level_res);
  voxelmaps_ptr_->set_max_level(max_level);
  voxelmaps_ptr_->create_voxelmaps(points, v_rate_, stream);
}

void BBS3D::set_src_points(const std::vector<Eigen::Vector3f>& points) {
  int src_size = points.size();
  src_points_.clear();
  src_points_.shrink_to_fit();
  src_points_.resize(src_size);
  std::copy(points.begin(), points.end(), src_points_.begin());

  d_src_points_.clear();
  d_src_points_.shrink_to_fit();
  d_src_points_.resize(src_size);
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_src_points_.data()),
    points.data(),
    sizeof(Eigen::Vector3f) * src_size,
    cudaMemcpyHostToDevice,
    stream);
}

void BBS3D::set_trans_search_range(const std::vector<Eigen::Vector3f>& points) {
  // The minimum and maximum x, y, z values are selected from the 3D coordinate vector.
  Eigen::Vector3f min_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f max_xyz = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

  for (const auto& point : points) {
    min_xyz = min_xyz.cwiseMin(point);
    max_xyz = max_xyz.cwiseMax(point);
  }

  set_trans_search_range(min_xyz, max_xyz);
}

void BBS3D::set_trans_search_range(const Eigen::Vector3f& min_xyz, const Eigen::Vector3f& max_xyz) {
  min_xyz_ = min_xyz;
  max_xyz_ = max_xyz;
  const int max_level = voxelmaps_ptr_->get_max_level();
  const float top_res = voxelmaps_ptr_->voxelmaps_info_[max_level].res;
  init_tx_range_ = std::make_pair<int, int>(std::floor(min_xyz.x() / top_res), std::ceil(max_xyz.x() / top_res));
  init_ty_range_ = std::make_pair<int, int>(std::floor(min_xyz.y() / top_res), std::ceil(max_xyz.y() / top_res));
  init_tz_range_ = std::make_pair<int, int>(std::floor(min_xyz.z() / top_res), std::ceil(max_xyz.z() / top_res));
}

void BBS3D::calc_angular_info(std::vector<AngularInfo>& ang_info_vec) {
  float max_norm = src_points_[0].norm();
  for (const auto& point : src_points_) {
    float norm = point.norm();
    if (norm > max_norm) {
      max_norm = norm;
    }
  }

  const int max_level = voxelmaps_ptr_->get_max_level();
  for (int i = max_level; i >= 0; i--) {
    const float cosine = 1 - (std::pow(voxelmaps_ptr_->voxelmaps_info_[i].res, 2) / std::pow(max_norm, 2)) * 0.5f;
    float ori_res = std::acos(max(cosine, -1.0f));
    ori_res = std::floor(ori_res * 10000) / 10000;
    Eigen::Vector3f rpy_res_temp;
    rpy_res_temp.x() = ori_res <= std::abs(max_rpy_.x() - min_rpy_.x()) ? ori_res : 0.0f;
    rpy_res_temp.y() = ori_res <= std::abs(max_rpy_.y() - min_rpy_.y()) ? ori_res : 0.0f;
    rpy_res_temp.z() = ori_res <= std::abs(max_rpy_.z() - min_rpy_.z()) ? ori_res : 0.0f;

    Eigen::Vector3f max_rpy_piece;
    if (i == max_level) {
      max_rpy_piece = max_rpy_ - min_rpy_;
    } else {
      max_rpy_piece.x() = ang_info_vec[i + 1].rpy_res.x() != 0.0f ? ang_info_vec[i + 1].rpy_res.x() : max_rpy_.x() - min_rpy_.x();
      max_rpy_piece.y() = ang_info_vec[i + 1].rpy_res.y() != 0.0f ? ang_info_vec[i + 1].rpy_res.y() : max_rpy_.y() - min_rpy_.y();
      max_rpy_piece.z() = ang_info_vec[i + 1].rpy_res.z() != 0.0f ? ang_info_vec[i + 1].rpy_res.z() : max_rpy_.z() - min_rpy_.z();
    }

    // Angle division number
    Eigen::Vector3i num_division;
    num_division.x() = rpy_res_temp.x() == 0.0f ? 1 : std::ceil(max_rpy_piece.x() / rpy_res_temp.x());
    num_division.y() = rpy_res_temp.y() == 0.0f ? 1 : std::ceil(max_rpy_piece.y() / rpy_res_temp.y());
    num_division.z() = rpy_res_temp.z() == 0.0f ? 1 : std::ceil(max_rpy_piece.z() / rpy_res_temp.z());
    ang_info_vec[i].num_division = num_division;

    // Bisect an angle
    ang_info_vec[i].rpy_res.x() = num_division.x() == 1 ? 0.0f : max_rpy_piece.x() / num_division.x();
    ang_info_vec[i].rpy_res.y() = num_division.y() == 1 ? 0.0f : max_rpy_piece.y() / num_division.y();
    ang_info_vec[i].rpy_res.z() = num_division.z() == 1 ? 0.0f : max_rpy_piece.z() / num_division.z();

    ang_info_vec[i].min_rpy.x() = ang_info_vec[i].rpy_res.x() == 0.0f ? 0.0f : min_rpy_.x();
    ang_info_vec[i].min_rpy.y() = ang_info_vec[i].rpy_res.y() == 0.0f ? 0.0f : min_rpy_.y();
    ang_info_vec[i].min_rpy.z() = ang_info_vec[i].rpy_res.z() == 0.0f ? 0.0f : min_rpy_.z();
  }
}

std::vector<DiscreteTransformation<float>> BBS3D::create_init_transset(const AngularInfo& init_ang_info) {
  const int init_transset_size = (init_tx_range_.second - init_tx_range_.first + 1) * (init_ty_range_.second - init_ty_range_.first + 1) *
                                 (init_tz_range_.second - init_tz_range_.first + 1) * (init_ang_info.num_division.x()) *
                                 (init_ang_info.num_division.y()) * (init_ang_info.num_division.z());

  const int max_level = voxelmaps_ptr_->get_max_level();

  std::vector<DiscreteTransformation<float>> transset;
  transset.reserve(init_transset_size);
  for (int tx = init_tx_range_.first; tx <= init_tx_range_.second; tx++) {
    for (int ty = init_ty_range_.first; ty <= init_ty_range_.second; ty++) {
      for (int tz = init_tz_range_.first; tz <= init_tz_range_.second; tz++) {
        for (int roll = 0; roll < init_ang_info.num_division.x(); roll++) {
          for (int pitch = 0; pitch < init_ang_info.num_division.y(); pitch++) {
            for (int yaw = 0; yaw < init_ang_info.num_division.z(); yaw++) {
              transset.emplace_back(DiscreteTransformation<float>(0, max_level, tx, ty, tz, roll, pitch, yaw));
            }
          }
        }
      }
    }
  }
  return transset;
}

void BBS3D::localize() {
  // Calc localize time limit
  const auto start_time = std::chrono::system_clock::now();
  const auto time_limit = start_time + timeout_duration_;
  has_timed_out_ = false;

  // Initialize best score
  best_score_ = 0;
  const int score_threshold = std::floor(src_points_.size() * score_threshold_percentage_);
  int best_score = score_threshold;
  DiscreteTransformation<float> best_trans(best_score);

  // Preapre initial transset
  const int max_level = voxelmaps_ptr_->get_max_level();
  std::vector<AngularInfo> ang_info_vec(max_level + 1);
  calc_angular_info(ang_info_vec);
  thrust::device_vector<AngularInfo> d_ang_info_vec(ang_info_vec.size());
  check_error << cudaMemcpyAsync(
    thrust::raw_pointer_cast(d_ang_info_vec.data()),
    ang_info_vec.data(),
    sizeof(AngularInfo) * ang_info_vec.size(),
    cudaMemcpyHostToDevice,
    stream);

  const auto init_transset = create_init_transset(ang_info_vec[max_level]);

  // Calc initial transset scores
  const auto init_transset_output = calc_scores(init_transset, d_ang_info_vec);

  std::priority_queue<DiscreteTransformation<float>> trans_queue(init_transset_output.begin(), init_transset_output.end());

  std::vector<DiscreteTransformation<float>> branch_stock;
  branch_stock.reserve(branch_copy_size_);
  while (!trans_queue.empty()) {
    if (use_timeout_ && std::chrono::system_clock::now() > time_limit) {
      has_timed_out_ = true;
      break;
    }

    auto trans = trans_queue.top();
    trans_queue.pop();

    // Calculate remaining branch_stock when queue is empty
    if (trans_queue.empty() && !branch_stock.empty()) {
      const auto transset_output = calc_scores(branch_stock, d_ang_info_vec);
      for (const auto& output : transset_output) {
        if (output.score < best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }

    // pruning
    if (trans.score < best_score) {
      continue;
    }

    if (trans.is_leaf()) {
      best_trans = trans;
      best_score = trans.score;
    } else {
      const int child_level = trans.level - 1;
      trans.branch(branch_stock, child_level, static_cast<int>(v_rate_), ang_info_vec[child_level].num_division);
    }

    if (branch_stock.size() >= branch_copy_size_) {
      const auto transset_output = calc_scores(branch_stock, d_ang_info_vec);
      for (const auto& output : transset_output) {
        if (output.score < best_score) continue;  // pruning
        trans_queue.push(output);
      }
      branch_stock.clear();
    }
  }

  // Calc localization elapsed time
  const auto end_time = std::chrono::system_clock::now();
  elapsed_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e6;

  // Not localized
  if (best_score == score_threshold || has_timed_out_) {
    has_localized_ = false;
    return;
  }

  float min_res = voxelmaps_ptr_->get_min_res();
  global_pose_ = best_trans.create_matrix(min_res, ang_info_vec[0].rpy_res, ang_info_vec[0].min_rpy);
  best_score_ = best_score;
  has_timed_out_ = false;
  has_localized_ = true;
}
}  // namespace gpu