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

// gnss::AnchorStore against the live node's GNSS path before the store existed
// (graph_slam_ros_adapter.cpp receiveNavSatFix / flushPreOdometryGnssLocked / the GNSS block of
// doPoseAdjustment, kept here verbatim but for the ROS types): the same fix and odometry
// sequences must give the same samples, anchors and yaw alignment.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "graph_based_slam/gnss_anchor_store.hpp"

namespace graphslam
{
namespace
{

// The previous inline implementation.
struct OldGnssPath
{
  struct GnssEnu
  {
    double stamp;
    double x;
    double y;
    double z;
    double info_x;
    double info_y;
    double info_z;
    bool covariance_valid;
    bool rtk_like;
    double horizontal_stddev_m;
  };

  explicit OldGnssPath(const gnss::AnchorStoreConfig & c)
  : config(c)
  {
    accumulator.configure(config.origin_min_samples, config.origin_consistency_threshold_m);
  }

  void receive(
    const gnss::Fix & msg, double latest_odom_stamp_sec, double first_odom_stamp_sec,
    bool use_odom_input, double now_sec)
  {
    if (msg.status < 0) {
      return;
    }
    if (!detail::isUsableGeodeticFix(msg.latitude, msg.longitude, msg.altitude)) {
      return;
    }
    if (!origin_set) {
      const detail::GnssOriginUpdate update =
        accumulator.add(msg.latitude, msg.longitude, msg.altitude);
      if (!update.initialized) {
        return;
      }
      origin = update.origin;
      origin_set = true;
    }
    Eigen::Vector3d enu =
      detail::geodeticToEnu(msg.latitude, msg.longitude, msg.altitude, origin);
    const detail::GnssConstraintWeights gnss_weights =
      detail::computeGnssConstraintWeights(msg, config.weighting);
    const double header_time_sec = msg.header_stamp_sec;
    GnssEnu g;
    g.stamp = header_time_sec;
    g.x = enu.x();
    g.y = enu.y();
    g.z = enu.z();
    g.info_x = gnss_weights.info_x;
    g.info_y = gnss_weights.info_y;
    g.info_z = gnss_weights.info_z;
    g.covariance_valid = gnss_weights.covariance_valid;
    g.rtk_like = gnss_weights.rtk_like;
    g.horizontal_stddev_m = gnss_weights.horizontal_stddev_m;
    double sensor_time_sec = latest_odom_stamp_sec;
    if (sensor_time_sec <= 0.0) {
      if (use_odom_input) {
        pre_odom.push_back(g);
        if (pre_odom.size() > 100000) {
          pre_odom.erase(pre_odom.begin(), pre_odom.begin() + 25000);
        }
        return;
      }
      sensor_time_sec = now_sec;
    }
    flush(first_odom_stamp_sec);
    const detail::GnssTimestampResolution stamp_resolution =
      detail::resolveGnssMeasurementStamp(
      header_time_sec, sensor_time_sec, config.header_stamp_max_skew_sec);
    g.stamp = stamp_resolution.stamp_sec;
    buffer.push_back(g);
    if (buffer.size() > 100000) {
      buffer.erase(buffer.begin(), buffer.begin() + 25000);
    }
  }

  void flush(double first_odom_stamp_sec)
  {
    if (pre_odom.empty()) {
      return;
    }
    if (first_odom_stamp_sec <= 0.0) {
      return;
    }
    for (GnssEnu g : pre_odom) {
      const detail::GnssTimestampResolution resolution = detail::resolveGnssMeasurementStamp(
        g.stamp, first_odom_stamp_sec, config.header_stamp_max_skew_sec);
      g.stamp = resolution.stamp_sec;
      buffer.push_back(g);
    }
    pre_odom.clear();
  }

  // doPoseAdjustment's GNSS block; nodes are moved as it moved them.
  std::vector<pose_graph::GnssConstraint> anchorAndAlign(
    const std::vector<double> & submap_stamps, double first_odom_stamp_sec,
    std::vector<pose_graph::SubmapNode> & submap_nodes, bool * fix_first_vertex)
  {
    std::vector<pose_graph::GnssConstraint> gnss_constraints;
    *fix_first_vertex = true;
    if (!origin_set) {
      return gnss_constraints;
    }
    std::vector<detail::GnssSample> gnss_samples;
    flush(first_odom_stamp_sec);
    gnss_samples.reserve(buffer.size());
    for (const auto & g : buffer) {
      detail::GnssSample sample;
      sample.stamp = g.stamp;
      sample.position = Eigen::Vector3d(g.x, g.y, g.z);
      sample.info = Eigen::Vector3d(g.info_x, g.info_y, g.info_z);
      sample.rtk_like = g.rtk_like;
      gnss_samples.push_back(sample);
    }
    std::stable_sort(gnss_samples.begin(), gnss_samples.end(), detail::gnssSampleStampLess);
    detail::GnssAnchorMatchConfig match_config;
    match_config.max_interpolation_gap_sec = config.interpolation_max_gap_sec;
    for (std::size_t i = 0; i < submap_stamps.size(); i++) {
      const detail::GnssAnchorMatch match =
        detail::matchGnssAnchor(gnss_samples, submap_stamps[i], match_config);
      if (!match.valid) {continue;}
      pose_graph::GnssConstraint gnss_constraint;
      gnss_constraint.submap_index = static_cast<int>(i);
      gnss_constraint.position = match.position;
      gnss_constraint.info_diag = match.info;
      gnss_constraint.antenna_offset = config.antenna_offset;
      gnss_constraint.robust_kernel_delta = config.robust_kernel_delta;
      gnss_constraints.push_back(gnss_constraint);
    }
    if (config.align_yaw && !gnss_constraints.empty()) {
      std::vector<Eigen::Vector2d> odom_xy;
      std::vector<Eigen::Vector2d> enu_xy;
      for (const auto & gnss_constraint : gnss_constraints) {
        const Eigen::Vector3d p =
          submap_nodes[gnss_constraint.submap_index].pose * gnss_constraint.antenna_offset;
        odom_xy.emplace_back(p.x(), p.y());
        enu_xy.emplace_back(gnss_constraint.position.x(), gnss_constraint.position.y());
      }
      const auto alignment = gnss_alignment::estimatePlanarAlignment(
        odom_xy, enu_xy, config.yaw_alignment_min_anchors, config.yaw_alignment_min_baseline_m);
      if (alignment.valid) {
        Eigen::Isometry3d enu_from_odom = Eigen::Isometry3d::Identity();
        enu_from_odom.linear() =
          Eigen::AngleAxisd(alignment.yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        enu_from_odom.translation() =
          Eigen::Vector3d(alignment.translation.x(), alignment.translation.y(), 0.0);
        for (auto & node : submap_nodes) {
          node.pose = enu_from_odom * node.pose;
        }
        *fix_first_vertex = false;
      }
    }
    return gnss_constraints;
  }

  gnss::AnchorStoreConfig config;
  detail::GnssOriginAccumulator accumulator;
  bool origin_set {false};
  detail::GeodeticOrigin origin;
  std::vector<GnssEnu> buffer;
  std::vector<GnssEnu> pre_odom;
};

// The new path, driven as the live adapter now drives it.
struct NewGnssPath
{
  explicit NewGnssPath(const gnss::AnchorStoreConfig & c)
  : store(c) {}

  void receive(
    const gnss::Fix & fix, double latest, double first, bool use_odom_input, double now_sec)
  {
    store.add(fix, latest, first, use_odom_input, now_sec);
  }

  std::vector<pose_graph::GnssConstraint> anchorAndAlign(
    const std::vector<double> & submap_stamps, double first_odom_stamp_sec,
    std::vector<pose_graph::SubmapNode> & submap_nodes, bool * fix_first_vertex)
  {
    *fix_first_vertex = true;
    if (!store.originSet()) {
      return {};
    }
    store.flushPreOdometry(first_odom_stamp_sec);
    gnss::Anchors anchors = store.anchors(submap_stamps);
    if (store.config().align_yaw && !anchors.constraints.empty()) {
      const gnss::YawAlignment yaw = store.alignYaw(submap_nodes, anchors.constraints);
      *fix_first_vertex = !yaw.applied;
    }
    return anchors.constraints;
  }

  gnss::AnchorStore store;
};

gnss::Fix makeFix(double stamp, double east_m, double north_m, double var_xy, int8_t status = 2)
{
  gnss::Fix fix;
  fix.header_stamp_sec = stamp;
  fix.status = status;
  fix.latitude = 48.78 + north_m / 111132.0;
  fix.longitude = 9.17 + east_m / (111320.0 * std::cos(48.78 * M_PI / 180.0));
  fix.altitude = 300.0 + 0.01 * stamp;
  fix.position_covariance[0] = var_xy;
  fix.position_covariance[4] = var_xy;
  fix.position_covariance[8] = 4.0 * var_xy;
  fix.position_covariance_type = 2;  // diagonal known
  return fix;
}

struct Event
{
  enum Kind {kFix, kOdometry} kind;
  gnss::Fix fix;
  double odom_stamp;
};

// A walk east with RTK and float fixes, fixes before the first odometry, a stale header, a zero
// header, a no-fix message, an unusable one and an origin jump among the first fixes.
std::vector<Event> scenario(double start)
{
  std::vector<Event> events;
  auto fix = [&events](gnss::Fix f) {events.push_back({Event::kFix, f, 0.0});};
  fix(makeFix(start - 0.5, 0.0, 0.0, 0.0004));
  fix(makeFix(start - 0.4, 500.0, 0.0, 0.0004));  // 500 m jump: the origin restarts
  for (int k = 0; k < 5; ++k) {
    fix(makeFix(start - 0.3 + 0.1 * k, 0.01 * k, 0.0, 0.0004));  // before the first odometry
  }
  for (int k = 0; k < 300; ++k) {
    const double t = start + 0.1 * k;
    events.push_back({Event::kOdometry, gnss::Fix(), t});
    if (k % 2 == 0) {
      const double var = k < 150 ? 0.0004 : 0.25;  // RTK, then float
      fix(makeFix(t + 0.03, 0.5 * k * 0.1 * 10.0, 0.02 * k, var));
    }
    if (k == 40) {
      fix(makeFix(t - 100.0, 0.0, 0.0, 0.0004));  // header 100 s off: sensor time is used
    }
    if (k == 60) {
      fix(makeFix(0.0, 1.0, 1.0, 0.0004));  // no header stamp
    }
    if (k == 80) {
      fix(makeFix(t, 1.0, 1.0, 0.0004, -1));  // no fix
      gnss::Fix unusable = makeFix(t, 0.0, 0.0, 0.0004);
      unusable.latitude = 0.0;
      unusable.longitude = 0.0;
      fix(unusable);
    }
    if (k == 100) {
      gnss::Fix unknown = makeFix(t, 2.0 * k, 0.0, 0.0004);
      unknown.position_covariance_type = 0;  // unknown covariance: fixed weights
      fix(unknown);
    }
  }
  return events;
}

template<typename Path>
void play(Path & path, const std::vector<Event> & events, bool use_odom_input)
{
  double first = 0.0, latest = 0.0;
  double now = 1700000000.0;
  for (const Event & event : events) {
    now += 0.05;
    if (event.kind == Event::kOdometry) {
      latest = event.odom_stamp;
      if (first <= 0.0) {
        first = latest;
      }
    } else {
      path.receive(event.fix, latest, first, use_odom_input, now);
    }
  }
}

std::vector<pose_graph::SubmapNode> walkNodes(double start, std::vector<double> * stamps)
{
  std::vector<pose_graph::SubmapNode> nodes;
  for (int k = 0; k < 300; k += 10) {
    pose_graph::SubmapNode node;
    // the odometry frame is rotated 30 deg from ENU
    const double yaw = 30.0 * M_PI / 180.0;
    const Eigen::Vector2d enu(0.5 * k, 0.02 * k);
    const Eigen::Vector2d odom = Eigen::Rotation2Dd(-yaw) * enu;
    node.pose = Eigen::Isometry3d::Identity();
    node.pose.translation() = Eigen::Vector3d(odom.x(), odom.y(), 0.1);
    nodes.push_back(node);
    stamps->push_back(start + 0.1 * k);
  }
  return nodes;
}

void expectSame(const gnss::AnchorStoreConfig & config, bool use_odom_input)
{
  const double start = 1762269335.6;
  const std::vector<Event> events = scenario(start);
  OldGnssPath old_path(config);
  NewGnssPath new_path(config);
  play(old_path, events, use_odom_input);
  play(new_path, events, use_odom_input);

  ASSERT_EQ(old_path.origin_set, new_path.store.originSet());
  ASSERT_TRUE(old_path.origin_set);
  EXPECT_EQ(old_path.origin.latitude_deg, new_path.store.origin().latitude_deg);
  EXPECT_EQ(old_path.origin.longitude_deg, new_path.store.origin().longitude_deg);
  EXPECT_EQ(old_path.origin.altitude_m, new_path.store.origin().altitude_m);

  std::vector<double> stamps_old, stamps_new;
  std::vector<pose_graph::SubmapNode> nodes_old = walkNodes(start, &stamps_old);
  std::vector<pose_graph::SubmapNode> nodes_new = walkNodes(start, &stamps_new);
  bool fix_old = true, fix_new = true;
  const auto anchors_old = old_path.anchorAndAlign(stamps_old, start, nodes_old, &fix_old);
  const auto anchors_new = new_path.anchorAndAlign(stamps_new, start, nodes_new, &fix_new);

  ASSERT_EQ(old_path.buffer.size(), new_path.store.samples().size());
  for (std::size_t i = 0; i < old_path.buffer.size(); ++i) {
    const auto & a = old_path.buffer[i];
    const auto & b = new_path.store.samples()[i];
    EXPECT_EQ(a.stamp, b.stamp) << i;
    EXPECT_EQ(a.x, b.x) << i;
    EXPECT_EQ(a.y, b.y) << i;
    EXPECT_EQ(a.z, b.z) << i;
    EXPECT_EQ(a.info_x, b.info_x) << i;
    EXPECT_EQ(a.info_y, b.info_y) << i;
    EXPECT_EQ(a.info_z, b.info_z) << i;
    EXPECT_EQ(a.rtk_like, b.rtk_like) << i;
    EXPECT_EQ(a.covariance_valid, b.covariance_valid) << i;
  }
  ASSERT_EQ(anchors_old.size(), anchors_new.size());
  ASSERT_GT(anchors_old.size(), 10U);
  for (std::size_t i = 0; i < anchors_old.size(); ++i) {
    EXPECT_EQ(anchors_old[i].submap_index, anchors_new[i].submap_index);
    EXPECT_EQ(anchors_old[i].position, anchors_new[i].position);
    EXPECT_EQ(anchors_old[i].info_diag, anchors_new[i].info_diag);
    EXPECT_EQ(anchors_old[i].antenna_offset, anchors_new[i].antenna_offset);
    EXPECT_EQ(anchors_old[i].robust_kernel_delta, anchors_new[i].robust_kernel_delta);
  }
  EXPECT_EQ(fix_old, fix_new);
  EXPECT_FALSE(fix_new);  // the walk is long enough: the graph moved into the ENU frame
  EXPECT_LT(old_path.buffer.size() + 2U, events.size());  // some fixes were refused
  for (std::size_t i = 0; i < nodes_old.size(); ++i) {
    EXPECT_TRUE(nodes_old[i].pose.matrix() == nodes_new[i].pose.matrix()) << i;
  }
}

TEST(GnssAnchorStore, MatchesTheLiveNodesPreviousPathWithDefaults)
{
  gnss::AnchorStoreConfig config;
  config.yaw_alignment_min_anchors = 5;
  expectSame(config, true);
}

TEST(GnssAnchorStore, MatchesTheLiveNodesPreviousPathWithTheRtkPreset)
{
  // lidarslam_rtk_gnss.yaml's settings
  gnss::AnchorStoreConfig config;
  config.interpolation_max_gap_sec = 0.5;
  config.antenna_offset = Eigen::Vector3d(0.034, 0.0, 0.046);
  config.robust_kernel_delta = 2.0;
  config.weighting.vertical_weight_scale = 0.3;
  config.yaw_alignment_min_anchors = 5;
  expectSame(config, true);
}

TEST(GnssAnchorStore, MatchesTheLiveNodesPreviousPathWithoutOdometryInput)
{
  gnss::AnchorStoreConfig config;
  config.yaw_alignment_min_anchors = 5;
  config.header_stamp_max_skew_sec = 1e9;
  expectSame(config, false);
}

TEST(GnssAnchorStore, ReportsTheFixOutcomes)
{
  gnss::AnchorStoreConfig config;
  config.origin_min_samples = 2;
  gnss::AnchorStore store(config);
  EXPECT_EQ(store.add(makeFix(10.0, 0, 0, 1e-4, -1), 0, 0, true, 0).outcome,
    gnss::FixOutcome::kNoFix);
  gnss::Fix unusable = makeFix(10.0, 0, 0, 1e-4);
  unusable.latitude = 91.0;
  EXPECT_EQ(store.add(unusable, 0, 0, true, 0).outcome, gnss::FixOutcome::kUnusable);
  EXPECT_EQ(store.add(makeFix(10.0, 0, 0, 1e-4), 0, 0, true, 0).outcome,
    gnss::FixOutcome::kOriginPending);
  const gnss::FixResult origin = store.add(makeFix(10.1, 0, 0, 1e-4), 0, 0, true, 0);
  EXPECT_TRUE(origin.origin_set_now);
  EXPECT_EQ(origin.outcome, gnss::FixOutcome::kWaitingForOdometry);
  EXPECT_EQ(store.pendingBeforeOdometry(), 1U);
  const gnss::FixResult stored = store.add(makeFix(10.3, 1, 0, 1e-4), 10.2, 10.2, true, 0);
  EXPECT_EQ(stored.outcome, gnss::FixOutcome::kStored);
  EXPECT_EQ(stored.flushed, 1U);
  EXPECT_EQ(store.samples().size(), 2U);
  EXPECT_TRUE(stored.weights.rtk_like);
  const gnss::FixResult stale = store.add(makeFix(40.3, 1, 0, 1e-4), 10.2, 10.2, true, 0);
  EXPECT_TRUE(stale.stamp.used_fallback);
  EXPECT_EQ(store.samples().back().stamp, 10.2);
}

}  // namespace
}  // namespace graphslam
