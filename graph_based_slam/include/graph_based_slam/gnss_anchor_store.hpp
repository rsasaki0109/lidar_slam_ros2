// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// GNSS anchors of the pose graph, without ROS: what graph_based_slam's live node does with
// NavSatFix messages, shared by the live node and the offline pipeline (offline_pipeline.hpp).
//
// A fix is used when it has a fix (status >= 0) and usable coordinates. The first fixes set the
// ENU origin (GnssOriginAccumulator); each later fix becomes an ENU sample with information from
// its covariance (gnss_weighting.hpp) and a stamp checked against sensor time: the latest
// odometry stamp. A fix that comes before the first odometry waits for it (with the odometry
// input on) and is then checked against the first odometry stamp. At a pose adjustment each
// submap stamp is matched against the samples (gnss_anchor_matching.hpp), and the yaw alignment
// can move the graph into the ENU frame (gnss_alignment.hpp). The store does no locking and no
// logging; its results carry what the callers log.

#ifndef GRAPH_BASED_SLAM__GNSS_ANCHOR_STORE_HPP_
#define GRAPH_BASED_SLAM__GNSS_ANCHOR_STORE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include "graph_based_slam/gnss_alignment.hpp"
#include "graph_based_slam/gnss_anchor_matching.hpp"
#include "graph_based_slam/gnss_geometry.hpp"
#include "graph_based_slam/gnss_origin_accumulator.hpp"
#include "graph_based_slam/gnss_weighting.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"

namespace graphslam
{
namespace gnss
{

// sensor_msgs/NavSatFix, the fields the store reads.
struct Fix
{
  double header_stamp_sec {0.0};  // rclcpp::Time(header.stamp).seconds()
  std::int8_t status {-1};  // NavSatStatus.status; STATUS_FIX is 0
  double latitude {0.0};
  double longitude {0.0};
  double altitude {0.0};
  std::array<double, 9> position_covariance{};
  std::uint8_t position_covariance_type {0};
};

// The GNSS parameters of graph_based_slam (graph_slam_config.hpp: gnss_*, as normalized there).
struct AnchorStoreConfig
{
  detail::GnssWeightingConfig weighting;
  int origin_min_samples {3};
  double origin_consistency_threshold_m {20.0};
  double header_stamp_max_skew_sec {30.0};
  double interpolation_max_gap_sec {0.0};
  Eigen::Vector3d antenna_offset {Eigen::Vector3d::Zero()};
  double robust_kernel_delta {0.0};
  bool align_yaw {true};
  int yaw_alignment_min_anchors {10};
  double yaw_alignment_min_baseline_m {5.0};
};

// A stored fix in the ENU frame at the origin.
struct EnuSample
{
  double stamp {0.0};
  double x {0.0};
  double y {0.0};
  double z {0.0};
  double info_x {0.0};
  double info_y {0.0};
  double info_z {0.0};
  bool covariance_valid {false};
  bool rtk_like {false};
  double horizontal_stddev_m {0.0};
};

enum class FixOutcome
{
  kNoFix,  // status below STATUS_FIX
  kUnusable,  // coordinates out of range or zero
  kOriginPending,  // used for the origin, which is not set yet
  kWaitingForOdometry,  // kept until the first odometry stamp is known
  kStored,
};

struct FixResult
{
  FixOutcome outcome {FixOutcome::kNoFix};
  // The origin accumulator's step, when the fix went to it (before the origin was set).
  bool origin_update_valid {false};
  detail::GnssOriginUpdate origin_update;
  bool origin_set_now {false};  // this fix completed the origin
  // kStored: the stamp check, the weights and the sensor time compared with.
  detail::GnssTimestampResolution stamp;
  detail::GnssConstraintWeights weights;
  double header_stamp_sec {0.0};
  double sensor_time_sec {0.0};
  // Fixes from before the first odometry moved into the samples by this call.
  std::size_t flushed {0};
  int flushed_used_fallback {0};
};

struct FlushResult
{
  std::size_t flushed {0};
  int used_fallback {0};
};

struct Anchors
{
  std::vector<pose_graph::GnssConstraint> constraints;
  int rtk_like {0};
  int interpolated {0};
};

struct YawAlignment
{
  bool attempted {false};
  bool applied {false};
  std::size_t pairs {0};
  gnss_alignment::PlanarAlignment alignment;
};

class AnchorStore
{
public:
  explicit AnchorStore(const AnchorStoreConfig & config = AnchorStoreConfig())
  : config_(config)
  {
    accumulator_.configure(config_.origin_min_samples, config_.origin_consistency_threshold_m);
  }

  const AnchorStoreConfig & config() const {return config_;}

  // One NavSatFix. latest_odom_stamp_sec and first_odom_stamp_sec are 0 before the first
  // odometry; without the odometry input (use_odom_input false) clock_now_sec stands in for
  // sensor time.
  FixResult add(
    const Fix & fix, double latest_odom_stamp_sec, double first_odom_stamp_sec,
    bool use_odom_input, double clock_now_sec)
  {
    FixResult result;
    if (fix.status < 0) {
      result.outcome = FixOutcome::kNoFix;
      return result;
    }
    if (!detail::isUsableGeodeticFix(fix.latitude, fix.longitude, fix.altitude)) {
      result.outcome = FixOutcome::kUnusable;
      return result;
    }
    if (!origin_set_) {
      result.origin_update_valid = true;
      result.origin_update = accumulator_.add(fix.latitude, fix.longitude, fix.altitude);
      if (!result.origin_update.initialized) {
        result.outcome = FixOutcome::kOriginPending;
        return result;
      }
      origin_ = result.origin_update.origin;
      origin_set_ = true;
      result.origin_set_now = true;
    }

    const Eigen::Vector3d enu = detail::geodeticToEnu(
      fix.latitude, fix.longitude, fix.altitude, origin_);
    result.weights = detail::computeGnssConstraintWeights(fix, config_.weighting);
    result.header_stamp_sec = fix.header_stamp_sec;
    EnuSample sample;
    sample.stamp = fix.header_stamp_sec;
    sample.x = enu.x();
    sample.y = enu.y();
    sample.z = enu.z();
    sample.info_x = result.weights.info_x;
    sample.info_y = result.weights.info_y;
    sample.info_z = result.weights.info_z;
    sample.covariance_valid = result.weights.covariance_valid;
    sample.rtk_like = result.weights.rtk_like;
    sample.horizontal_stddev_m = result.weights.horizontal_stddev_m;

    double sensor_time_sec = latest_odom_stamp_sec;
    if (sensor_time_sec <= 0.0) {
      if (use_odom_input) {
        pre_odometry_.push_back(sample);
        if (pre_odometry_.size() > kMaxSamples) {  // no odometry for hours: keep the newest
          pre_odometry_.erase(pre_odometry_.begin(), pre_odometry_.begin() + kTrimSamples);
        }
        result.outcome = FixOutcome::kWaitingForOdometry;
        return result;
      }
      sensor_time_sec = clock_now_sec;
    }
    const FlushResult flush = flushPreOdometry(first_odom_stamp_sec);
    result.flushed = flush.flushed;
    result.flushed_used_fallback = flush.used_fallback;
    result.sensor_time_sec = sensor_time_sec;
    result.stamp = detail::resolveGnssMeasurementStamp(
      fix.header_stamp_sec, sensor_time_sec, config_.header_stamp_max_skew_sec);
    sample.stamp = result.stamp.stamp_sec;
    samples_.push_back(sample);
    if (samples_.size() > kMaxSamples) {
      samples_.erase(samples_.begin(), samples_.begin() + kTrimSamples);
    }
    result.outcome = FixOutcome::kStored;
    return result;
  }

  // Fixes that arrived before the first odometry: their headers checked against the first
  // odometry stamp, the earliest sensor time. Nothing happens before that stamp is known.
  FlushResult flushPreOdometry(double first_odom_stamp_sec)
  {
    FlushResult result;
    if (pre_odometry_.empty() || first_odom_stamp_sec <= 0.0) {
      return result;
    }
    for (EnuSample sample : pre_odometry_) {
      const detail::GnssTimestampResolution resolution = detail::resolveGnssMeasurementStamp(
        sample.stamp, first_odom_stamp_sec, config_.header_stamp_max_skew_sec);
      result.used_fallback += resolution.used_fallback ? 1 : 0;
      sample.stamp = resolution.stamp_sec;
      samples_.push_back(sample);
    }
    result.flushed = pre_odometry_.size();
    pre_odometry_.clear();
    return result;
  }

  bool originSet() const {return origin_set_;}
  const detail::GeodeticOrigin & origin() const {return origin_;}
  const std::vector<EnuSample> & samples() const {return samples_;}
  std::size_t pendingBeforeOdometry() const {return pre_odometry_.size();}

  // The anchor of each submap stamp (seconds) that matches a sample, in submap order.
  Anchors anchors(const std::vector<double> & submap_stamps) const
  {
    std::vector<detail::GnssSample> sorted;
    sorted.reserve(samples_.size());
    for (const auto & g : samples_) {
      detail::GnssSample sample;
      sample.stamp = g.stamp;
      sample.position = Eigen::Vector3d(g.x, g.y, g.z);
      sample.info = Eigen::Vector3d(g.info_x, g.info_y, g.info_z);
      sample.rtk_like = g.rtk_like;
      sorted.push_back(sample);
    }
    std::stable_sort(sorted.begin(), sorted.end(), detail::gnssSampleStampLess);
    detail::GnssAnchorMatchConfig match_config;
    match_config.max_interpolation_gap_sec = config_.interpolation_max_gap_sec;
    Anchors result;
    for (std::size_t i = 0; i < submap_stamps.size(); ++i) {
      const detail::GnssAnchorMatch match =
        detail::matchGnssAnchor(sorted, submap_stamps[i], match_config);
      if (!match.valid) {
        continue;
      }
      pose_graph::GnssConstraint constraint;
      constraint.submap_index = static_cast<int>(i);
      constraint.position = match.position;
      constraint.info_diag = match.info;
      constraint.antenna_offset = config_.antenna_offset;
      constraint.robust_kernel_delta = config_.robust_kernel_delta;
      result.constraints.push_back(constraint);
      result.rtk_like += match.rtk_like ? 1 : 0;
      result.interpolated += match.interpolated ? 1 : 0;
    }
    return result;
  }

  // With align_yaw and anchors: the planar odometry -> ENU transform of the anchors; when it is
  // valid, every node moves into the ENU frame (the vertex-0 gauge is then released).
  YawAlignment alignYaw(
    std::vector<pose_graph::SubmapNode> & nodes,
    const std::vector<pose_graph::GnssConstraint> & constraints) const
  {
    YawAlignment result;
    if (!config_.align_yaw || constraints.empty()) {
      return result;
    }
    result.attempted = true;
    std::vector<Eigen::Vector2d> odom_xy;
    std::vector<Eigen::Vector2d> enu_xy;
    odom_xy.reserve(constraints.size());
    enu_xy.reserve(constraints.size());
    for (const auto & constraint : constraints) {
      const Eigen::Vector3d p = nodes[constraint.submap_index].pose * constraint.antenna_offset;
      odom_xy.emplace_back(p.x(), p.y());
      enu_xy.emplace_back(constraint.position.x(), constraint.position.y());
    }
    result.pairs = odom_xy.size();
    result.alignment = gnss_alignment::estimatePlanarAlignment(
      odom_xy, enu_xy, config_.yaw_alignment_min_anchors, config_.yaw_alignment_min_baseline_m);
    if (!result.alignment.valid) {
      return result;
    }
    Eigen::Isometry3d enu_from_odom = Eigen::Isometry3d::Identity();
    enu_from_odom.linear() =
      Eigen::AngleAxisd(result.alignment.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    enu_from_odom.translation() = Eigen::Vector3d(
      result.alignment.translation.x(), result.alignment.translation.y(), 0.0);
    for (auto & node : nodes) {
      node.pose = enu_from_odom * node.pose;
    }
    result.applied = true;
    return result;
  }

private:
  static constexpr std::size_t kMaxSamples = 100000;
  static constexpr std::size_t kTrimSamples = 25000;

  AnchorStoreConfig config_;
  detail::GnssOriginAccumulator accumulator_;
  bool origin_set_ {false};
  detail::GeodeticOrigin origin_;
  std::vector<EnuSample> samples_;
  std::vector<EnuSample> pre_odometry_;
};

}  // namespace gnss
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GNSS_ANCHOR_STORE_HPP_
