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

// graph_slam_multisession_join: joins several sessions of one site into one pose graph by place
// recognition, with no prior on their relative poses (bench_report/data/grandtour/
// multisession_2026-10-29/plan.txt, step 2). Each session is its backend input bag (the frontend's
// odometry and deskewed scans) and the output of graph_slam_offline_runner on it
// (trajectory_raw.tum, loop_edges.csv). The runner's submaps are the keyframes; a keyframe's map
// is the scans within keyframe_window_s of its stamp. Every session is joined to the first
// (multisession_join.hpp); the joined ones form one graph: each session's odometry chain, its own
// loop edges, and one inter-session edge per joined place, optimized with the first submap fixed.
// Writes, in output_dir: join_report.yaml, inter_session_edges.csv and, per joined session i,
// trajectory_session<i>.tum (its optimized submap poses in the first session's frame). With
// multisession_cache_dir, each session's self-similarity is stored there under its fingerprint
// and used instead of its self-join in later runs (a stored map is checked once; step 6).

#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)

#include <algorithm>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/points/eigen.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/downsampling.hpp>

#include "graph_based_slam/cloud_input_extrinsic.hpp"
#include "graph_based_slam/multisession_join.hpp"
#include "graph_based_slam/pose_graph_optimization.hpp"

namespace
{

using graphslam::multisession::Keyframe;

struct Session
{
  std::vector<double> stamps;  // submap stamps (s)
  std::vector<Eigen::Isometry3d> raw;  // submap odometry poses
  std::vector<std::array<double, 36>> covariance;
  std::vector<graphslam::pose_graph::LoopConstraint> loops;
  std::vector<Keyframe> keyframes;
  // every odometry message of the bag (the GNSS gate pairs the fixes with them)
  std::vector<double> odom_stamps;
  std::vector<Eigen::Isometry3d> odom_poses;
};

Eigen::Isometry3d isometry(const double * t, const double * q_xyzw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::Quaterniond(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2])
    .normalized().toRotationMatrix();
  pose.translation() = Eigen::Vector3d(t[0], t[1], t[2]);
  return pose;
}

void readRunnerOutput(const std::string & dir, Session * session)
{
  std::ifstream tum(dir + "/trajectory_raw.tum");
  if (!tum.is_open()) {
    throw std::runtime_error("cannot open " + dir + "/trajectory_raw.tum");
  }
  std::string line;
  while (std::getline(tum, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream in(line);
    double v[8];
    for (double & x : v) {
      in >> x;
    }
    session->stamps.push_back(v[0]);
    session->raw.push_back(isometry(v + 1, v + 4));
  }
  std::ifstream csv(dir + "/loop_edges.csv");
  if (!csv.is_open()) {
    throw std::runtime_error("cannot open " + dir + "/loop_edges.csv");
  }
  std::getline(csv, line);  // header
  while (std::getline(csv, line)) {
    if (line.empty()) {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream in(line);
    double v[10];
    for (double & x : v) {
      in >> x;
    }
    graphslam::pose_graph::LoopConstraint c;
    c.from = static_cast<int>(v[0]);
    c.to = static_cast<int>(v[1]);
    c.fitness_score = v[2];
    c.relative_pose = isometry(v + 3, v + 6);
    session->loops.push_back(c);
  }
}

// Keyframe maps from the backend input bag: every scan within window_s of a submap stamp, moved
// into that submap's gravity-aligned frame, cropped and thinned.
void readKeyframes(
  const std::string & bag_path, const std::string & odom_topic, const std::string & cloud_topic,
  const graphslam::cloud_input::LidarExtrinsic & extrinsic, double window_s, double voxel_m,
  double range_m, Session * session)
{
  const std::size_t n = session->stamps.size();
  session->keyframes.assign(n, Keyframe());
  session->covariance.assign(n, std::array<double, 36>{});
  std::vector<std::vector<Eigen::Vector3f>> gathered(n);
  for (std::size_t k = 0; k < n; ++k) {
    session->keyframes[k].id = static_cast<int>(k);
    session->keyframes[k].frame = graphslam::multisession::gravityAlignedFrame(session->raw[k]);
  }
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serialization;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  std::map<uint64_t, nav_msgs::msg::Odometry> odoms;
  std::map<uint64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::size_t first = 0;  // keyframes before it are complete
  auto stamp_of = [](const builtin_interfaces::msg::Time & t) {
      return static_cast<uint64_t>(t.sec) * 1000000000ull + t.nanosec;
    };
  auto use_pair = [&](const nav_msgs::msg::Odometry & odom, sensor_msgs::msg::PointCloud2 cloud) {
      const double t = rclcpp::Time(odom.header.stamp).seconds();
      while (first < n && session->stamps[first] + window_s < t) {
        ++first;
      }
      std::vector<std::size_t> targets;
      for (std::size_t k = first; k < n && session->stamps[k] - window_s <= t; ++k) {
        targets.push_back(k);
        if (std::abs(session->stamps[k] - t) < 1e-6) {
          session->covariance[k] = odom.pose.covariance;
        }
      }
      if (targets.empty()) {
        return;
      }
      std::string error;
      if (!graphslam::cloud_input::transformCloudToBase(extrinsic, cloud, &error)) {
        throw std::runtime_error(error);
      }
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(cloud, pcl_cloud);
      const auto & p = odom.pose.pose;
      Eigen::Isometry3d world_base = Eigen::Isometry3d::Identity();
      world_base.linear() = Eigen::Quaterniond(
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).normalized()
        .toRotationMatrix();
      world_base.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
      for (std::size_t k : targets) {
        const Eigen::Isometry3d frame_base = session->keyframes[k].frame.inverse() * world_base;
        for (const auto & q : pcl_cloud.points) {
          if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) {
            continue;
          }
          const Eigen::Vector3d v = frame_base * Eigen::Vector3d(q.x, q.y, q.z);
          if (v.norm() < range_m) {
            gathered[k].push_back(v.cast<float>());
          }
        }
      }
    };
  while (reader.has_next()) {
    auto message = reader.read_next();
    rclcpp::SerializedMessage serialized(*message->serialized_data);
    if (message->topic_name == odom_topic) {
      nav_msgs::msg::Odometry odom;
      odom_serialization.deserialize_message(&serialized, &odom);
      const auto & p = odom.pose.pose;
      const double t[3] = {p.position.x, p.position.y, p.position.z};
      const double q[4] = {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
      session->odom_stamps.push_back(rclcpp::Time(odom.header.stamp).seconds());
      session->odom_poses.push_back(isometry(t, q));
      const auto it = clouds.find(stamp_of(odom.header.stamp));
      if (it != clouds.end()) {
        use_pair(odom, it->second);
        clouds.erase(it);
      } else {
        odoms[stamp_of(odom.header.stamp)] = odom;
      }
    } else if (message->topic_name == cloud_topic) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud_serialization.deserialize_message(&serialized, &cloud);
      const auto it = odoms.find(stamp_of(cloud.header.stamp));
      if (it != odoms.end()) {
        use_pair(it->second, cloud);
        odoms.erase(it);
      } else {
        clouds[stamp_of(cloud.header.stamp)] = cloud;
      }
    }
  }
  for (std::size_t k = 0; k < n; ++k) {
    if (gathered[k].empty()) {
      continue;
    }
    const auto thinned = small_gicp::voxelgrid_sampling<
      std::vector<Eigen::Vector3f>, small_gicp::PointCloud>(gathered[k], voxel_m);
    auto & points = session->keyframes[k].points;
    points.reserve(thinned->size());
    for (std::size_t i = 0; i < thinned->size(); ++i) {
      points.push_back(thinned->point(i).head<3>().cast<float>());
    }
    std::vector<Eigen::Vector3f>().swap(gathered[k]);
  }
}

// GNSS gate: a kept fix (status >= 0, horizontal standard deviation at most 0.3 m).
struct GnssFix
{
  double stamp;
  double latitude;
  double longitude;
  double altitude;
};

std::vector<GnssFix> readGnssFixes(const std::string & bag_path, const std::string & topic)
{
  std::vector<GnssFix> fixes;
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  bool has_topic = false;
  for (const auto & t : reader.get_all_topics_and_types()) {
    has_topic = has_topic || t.name == topic;
  }
  if (!has_topic) {
    return fixes;
  }
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader.set_filter(filter);
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serialization;
  while (reader.has_next()) {
    auto message = reader.read_next();
    if (message->topic_name != topic) {
      continue;
    }
    rclcpp::SerializedMessage serialized(*message->serialized_data);
    sensor_msgs::msg::NavSatFix fix;
    serialization.deserialize_message(&serialized, &fix);
    const auto & c = fix.position_covariance;
    if (fix.status.status < 0 || c[0] > 0.3 * 0.3 || c[4] > 0.3 * 0.3) {
      continue;
    }
    fixes.push_back(
      {fix.header.stamp.sec + 1e-9 * fix.header.stamp.nanosec, fix.latitude, fix.longitude,
        fix.altitude});
  }
  return fixes;
}

Eigen::Vector3d toEcef(const GnssFix & f)
{
  const double a = 6378137.0, flat = 1.0 / 298.257223563, e2 = 2.0 * flat - flat * flat;
  const double la = f.latitude * M_PI / 180.0, lo = f.longitude * M_PI / 180.0;
  const double n = a / std::sqrt(1.0 - e2 * std::sin(la) * std::sin(la));
  return Eigen::Vector3d(
    (n + f.altitude) * std::cos(la) * std::cos(lo), (n + f.altitude) * std::cos(la) * std::sin(lo),
    (n * (1.0 - e2) + f.altitude) * std::sin(la));
}

// The fixes with stamps in (t_min, t_max), each paired with the session's odometry pose nearest in
// time (within 0.05 s): the antenna's position in the odometry frame, and the fix's in the local
// ENU frame at origin.
graphslam::multisession::GnssPairs makeGnssPairs(
  const std::vector<GnssFix> & fixes, const GnssFix & origin, const Session & session,
  const Eigen::Vector3d & antenna, double t_min, double t_max)
{
  graphslam::multisession::GnssPairs out;
  const std::vector<double> & t = session.odom_stamps;
  if (t.size() < 2) {
    return out;
  }
  const double lat0 = origin.latitude * M_PI / 180.0, lon0 = origin.longitude * M_PI / 180.0;
  Eigen::Matrix3d enu;
  enu << -std::sin(lon0), std::cos(lon0), 0.0,
    -std::sin(lat0) * std::cos(lon0), -std::sin(lat0) * std::sin(lon0), std::cos(lat0),
    std::cos(lat0) * std::cos(lon0), std::cos(lat0) * std::sin(lon0), std::sin(lat0);
  const Eigen::Vector3d ecef0 = toEcef(origin);
  for (const GnssFix & f : fixes) {
    if (!(f.stamp > t_min && f.stamp < t_max)) {
      continue;
    }
    std::size_t i = std::lower_bound(t.begin(), t.end(), f.stamp) - t.begin();
    i = std::min(std::max<std::size_t>(i, 1), t.size() - 1);
    const std::size_t j = std::abs(t[i - 1] - f.stamp) <= std::abs(t[i] - f.stamp) ? i - 1 : i;
    if (std::abs(t[j] - f.stamp) > 0.05) {
      continue;
    }
    out.odom.push_back(session.odom_poses[j] * antenna);
    out.enu.push_back(enu * (toEcef(f) - ecef0));
  }
  return out;
}

std::string gnssFields(const graphslam::multisession::SetAttempt & t)
{
  if (t.gnss_places < 0) {
    return "";
  }
  std::ostringstream out;
  out << ", gnss: {places: " << t.gnss_places << ", median_m: " << t.gnss_median_m <<
    ", median_heading_deg: " << t.gnss_median_heading_deg << ", passed: " <<
    (t.gnss_passed ? "true" : "false") << "}";
  return out.str();
}

void writePoses(
  const std::string & path, const std::vector<double> & stamps,
  const std::vector<Eigen::Isometry3d> & poses)
{
  std::ofstream out(path);
  char line[256];
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const Eigen::Vector3d t = poses[i].translation();
    const Eigen::Quaterniond q(poses[i].rotation());
    std::snprintf(
      line, sizeof(line), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f",
      stamps[i], t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    out << line << "\n";
  }
}

// Test only: one session cut into parts of equal duration, with gap_s left out between them. Each
// part becomes a session: its submaps, odometry, own loop edges and keyframe maps.
std::vector<Session> splitSession(Session all, int parts, double gap_s)
{
  std::vector<Session> out(parts);
  const double t0 = all.stamps.front();
  const double part_s = (all.stamps.back() - t0) / parts;
  std::vector<int> part_of(all.stamps.size(), -1), local(all.stamps.size(), -1);
  for (std::size_t k = 0; k < all.stamps.size(); ++k) {
    const double t = all.stamps[k] - t0;
    const int p = std::min(parts - 1, static_cast<int>(t / part_s));
    if ((p > 0 && t - p * part_s <= 0.5 * gap_s) ||
      (p < parts - 1 && (p + 1) * part_s - t <= 0.5 * gap_s))
    {
      continue;
    }
    Session & part = out[p];
    part_of[k] = p;
    local[k] = static_cast<int>(part.stamps.size());
    part.stamps.push_back(all.stamps[k]);
    part.raw.push_back(all.raw[k]);
    part.covariance.push_back(all.covariance[k]);
    part.keyframes.push_back(std::move(all.keyframes[k]));
    part.keyframes.back().id = local[k];
  }
  for (auto c : all.loops) {
    if (part_of[c.from] >= 0 && part_of[c.from] == part_of[c.to]) {
      const int p = part_of[c.from];
      c.from = local[c.from];
      c.to = local[c.to];
      out[p].loops.push_back(c);
    }
  }
  return out;
}

// The self-similarity stored in dir for a session fingerprint: 1 or 0, or -1 if none is stored.
int readSelfSimilarity(const std::string & dir, const std::string & fingerprint)
{
  std::ifstream in(dir + "/self_similarity_" + fingerprint + ".yaml");
  std::string line;
  while (std::getline(in, line)) {
    if (line == "self_similar: true") {
      return 1;
    }
    if (line == "self_similar: false") {
      return 0;
    }
  }
  return -1;
}

void writeSelfSimilarity(
  const std::string & dir, const std::string & fingerprint, bool self_similar,
  std::size_t keyframes, const std::string & bag, const std::string & run)
{
  std::filesystem::create_directories(dir);
  const std::string path = dir + "/self_similarity_" + fingerprint + ".yaml";
  {
    std::ofstream out(path + ".tmp");
    out << "fingerprint: " << fingerprint << "\nself_similar: " <<
      (self_similar ? "true" : "false") << "\nkeyframes: " << keyframes << "\n";
    out << "session_bag_path: '" << bag << "'\nsession_backend_dir: '" << run << "'\n";
  }
  std::filesystem::rename(path + ".tmp", path);
}

std::string matrixRow(const Eigen::Isometry3d & t)
{
  std::ostringstream out;
  out.precision(9);
  out << "[";
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      out << t.matrix()(r, c) << (r == 2 && c == 3 ? "" : ", ");
    }
  }
  out << "]";
  return out.str();
}

// Test only (debug_check_transforms): the whole-map check of session b in session a at each given
// transform (12 values each, a 3x4 matrix row by row). Decides nothing; written to path.
void writeDebugChecks(
  const std::string & path, const std::vector<graphslam::multisession::Keyframe> & a,
  const std::vector<graphslam::multisession::Keyframe> & b, const std::vector<double> & values,
  const graphslam::multisession::JoinConfig & join,
  const graphslam::multisession::GnssPairs * gnss_a = nullptr,
  const graphslam::multisession::GnssPairs * gnss_b = nullptr)
{
  std::unique_ptr<graphslam::multisession::GnssLocalFits> fits_a, fits_b;
  if (join.gnss_gate && gnss_a != nullptr && gnss_b != nullptr && !gnss_a->odom.empty() &&
    !gnss_b->odom.empty())
  {
    fits_a = std::make_unique<graphslam::multisession::GnssLocalFits>(a, *gnss_a, join);
    fits_b = std::make_unique<graphslam::multisession::GnssLocalFits>(b, *gnss_b, join);
  }
  std::ofstream out(path);
  out << "debug_checks:\n  place_min_fraction: " << join.place_min_fraction << "\n  checks:\n";
  for (std::size_t i = 0; i + 12 <= values.size(); i += 12) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    for (int j = 0; j < 12; ++j) {
      t.matrix()(j / 4, j % 4) = values[i + j];
    }
    auto check = graphslam::multisession::wholeMapCheck(a, b, t, join);
    if (fits_a) {
      graphslam::multisession::gnssCheck(a, b, t, *fits_a, *fits_b, join, &check);
    }
    out << "    - {index: " << i / 12 << ", places: " << check.places << ", share_below: " <<
      check.share_below << ", median: " << check.median << ", passed: " <<
      (check.passed ? "true" : "false") << ", transform: " << matrixRow(t) << gnssFields(check) <<
      "}\n";
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "graph_based_slam",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();
  std::vector<std::string> bags, runs;
  std::string output_dir, odom_topic, cloud_topic;
  node->get_parameter_or("session_bag_paths", bags, std::vector<std::string>{});
  node->get_parameter_or("session_backend_dirs", runs, std::vector<std::string>{});
  node->get_parameter_or("output_dir", output_dir, std::string("multisession"));
  node->get_parameter_or("offline_odom_topic", odom_topic, std::string("/rko_lio/odometry"));
  node->get_parameter_or("offline_cloud_topic", cloud_topic, std::string("/rko_lio/frame"));
  // Test only: one session split in two at its middle, with split_gap_s left out between the
  // halves; the second half is joined to the first (the truth is the identity).
  double split_gap_s;
  node->get_parameter_or("debug_split_gap_s", split_gap_s, -1.0);
  // A session's self-similarity is stored in cache_dir under its fingerprint and used instead of
  // its self-join next time. With precheck every session is checked first (a map to be joined
  // later: one session is enough then).
  // Test only, with debug_split_gap_s: 3 or more parts become sessions of their own and are joined
  // like sessions (2 parts: the second half is joined to the first, as in steps 3-5).
  int split_parts;
  node->get_parameter_or("debug_split_parts", split_parts, 2);
  // "map": each session is joined to every session joined before it (the map; step 7); "first":
  // to the first session only (steps 2-6). Sessions that do not join are tried again once the map
  // has grown.
  std::string join_to;
  node->get_parameter_or("multisession_join_to", join_to, std::string("map"));
  // After each join, while sessions are left to join, optimize the graph so far and place the map's
  // keyframes at their optimized submap poses (step 8); otherwise each session is placed by its
  // join's transform alone.
  bool reoptimize = true;
  node->get_parameter_or("multisession_reoptimize_map", reoptimize, true);
  std::string cache_dir;
  bool precheck = false;
  node->get_parameter_or("multisession_cache_dir", cache_dir, std::string());
  node->get_parameter_or("multisession_precheck_self_similarity", precheck, false);
  const std::size_t min_sessions = split_gap_s >= 0.0 || precheck ? 1U : 2U;
  if (bags.size() < min_sessions || bags.size() != runs.size()) {
    RCLCPP_ERROR(
      logger, "need at least two sessions (one with debug_split_gap_s or "
      "multisession_precheck_self_similarity): session_bag_paths and session_backend_dirs, the "
      "same number of each");
    rclcpp::shutdown();
    return 1;
  }
  std::vector<double> extrinsic_values {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  node->get_parameter_or("extrinsic_lidar2base_quat_xyzw_xyz", extrinsic_values, extrinsic_values);
  graphslam::cloud_input::LidarExtrinsic extrinsic;
  std::string error;
  if (!graphslam::cloud_input::parseLidarExtrinsic(extrinsic_values, &extrinsic, &error)) {
    RCLCPP_ERROR(logger, "%s", error.c_str());
    rclcpp::shutdown();
    return 1;
  }
  double window_s, voxel_m, range_m;
  node->get_parameter_or("multisession_keyframe_window_s", window_s, 1.0);
  node->get_parameter_or("multisession_keyframe_voxel_m", voxel_m, 0.2);
  node->get_parameter_or("multisession_keyframe_range_m", range_m, 40.0);
  graphslam::multisession::JoinConfig join;
  node->get_parameter_or("multisession_num_threads", join.num_threads, join.num_threads);
  node->get_parameter_or(
    "multisession_self_gate_fraction", join.self_gate_fraction, join.self_gate_fraction);
  // the stress test of the plan only: drop the matches that agree with this transform of session 1
  std::vector<double> exclude_values;
  double exclude_radius_m;
  node->get_parameter_or("debug_exclude_transform", exclude_values, std::vector<double>{});
  node->get_parameter_or("debug_exclude_radius_m", exclude_radius_m, 0.3);
  // Test only: with two sessions, write the whole-map check of session 1 at these transforms to
  // output_dir/debug_checks.yaml and stop (bench_report/data/multisession_heldout_2026-11-07)
  std::vector<double> check_values;
  node->get_parameter_or("debug_check_transforms", check_values, std::vector<double>{});
  // A candidate, off: the GNSS gate (JoinConfig::gnss_gate). GNSS (NavSatFix) is read from each
  // session's bag, or from multisession_gnss_bag_paths (one per session).
  node->get_parameter_or("multisession_gnss_gate", join.gnss_gate, join.gnss_gate);
  std::string gnss_topic;
  std::vector<std::string> gnss_bags;
  std::vector<double> antenna_values {0.0, 0.0, 0.0};
  node->get_parameter_or("multisession_gnss_topic", gnss_topic, std::string("/gnss/fix"));
  node->get_parameter_or("multisession_gnss_bag_paths", gnss_bags, std::vector<std::string>{});
  node->get_parameter_or("multisession_gnss_antenna_xyz", antenna_values, antenna_values);
  if (join.gnss_gate && ((!gnss_bags.empty() && gnss_bags.size() != bags.size()) ||
    antenna_values.size() != 3))
  {
    RCLCPP_ERROR(
      logger, "multisession_gnss_bag_paths: one per session; multisession_gnss_antenna_xyz: 3 "
      "values");
    rclcpp::shutdown();
    return 1;
  }
  const Eigen::Vector3d antenna(antenna_values[0], antenna_values[1], antenna_values[2]);

  // pose graph settings, the runner's parameter names
  int num_adjacent;
  double adjacent_weight, adjacent_trans, adjacent_rot, loop_weight, loop_trans, loop_rot, delta;
  bool split = false;
  std::string kernel;
  node->get_parameter_or("num_adjacent_pose_cnstraints", num_adjacent, 5);
  node->get_parameter_or("adjacent_edge_info_weight", adjacent_weight, 1000.0);
  node->get_parameter_or("adjacent_edge_info_auto_scale_split_trans_rot", split, false);
  node->get_parameter_or("adjacent_edge_info_weight_trans", adjacent_trans, 0.0);
  node->get_parameter_or("adjacent_edge_info_weight_rot", adjacent_rot, 0.0);
  node->get_parameter_or("loop_edge_info_weight", loop_weight, 100.0);
  node->get_parameter_or("loop_edge_info_constant_trans", loop_trans, 0.0);
  node->get_parameter_or("loop_edge_info_constant_rot", loop_rot, 0.0);
  node->get_parameter_or("loop_edge_robust_kernel_type", kernel, std::string("huber"));
  node->get_parameter_or("loop_edge_robust_kernel_delta", delta, 1.0);
  graphslam::pose_graph::AdjacentEdgeConfig adjacent;
  adjacent.num_adjacent_pose_constraints = num_adjacent;
  adjacent.info_weight = adjacent_weight;
  adjacent.split_trans_rot = split;
  adjacent.info_weight_trans = adjacent_trans > 0.0 ? adjacent_trans : adjacent_weight;
  adjacent.info_weight_rot = adjacent_rot > 0.0 ? adjacent_rot : adjacent_weight;
  graphslam::pose_graph::LoopEdgeConfig loop;
  loop.info_weight = loop_weight;
  loop.constant_info_trans = std::max(0.0, loop_trans);
  loop.constant_info_rot = std::max(0.0, loop_rot);
  loop.robust_kernel_type = kernel;
  loop.robust_kernel_delta = delta;

  std::filesystem::create_directories(output_dir);
  std::vector<Session> sessions(bags.size());
  try {
    for (std::size_t s = 0; s < bags.size(); ++s) {
      readRunnerOutput(runs[s], &sessions[s]);
      readKeyframes(
        bags[s], odom_topic, cloud_topic, extrinsic, window_s, voxel_m, range_m, &sessions[s]);
      RCLCPP_INFO(
        logger, "session %zu: %zu submaps, %zu loop edges", s, sessions[s].raw.size(),
        sessions[s].loops.size());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "reading the sessions failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // GNSS gate: each session's kept fixes, in one local ENU frame at session 0's first fix
  const bool parts_as_sessions = split_gap_s >= 0.0 && split_parts >= 3;
  if (join.gnss_gate && parts_as_sessions) {
    RCLCPP_WARN(logger, "multisession_gnss_gate: not used with debug_split_parts >= 3");
    join.gnss_gate = false;
  }
  std::vector<std::vector<GnssFix>> fixes(sessions.size());
  std::vector<graphslam::multisession::GnssPairs> gnss_pairs(sessions.size());
  GnssFix origin {0.0, 0.0, 0.0, 0.0};
  if (join.gnss_gate) {
    for (std::size_t s = 0; s < sessions.size(); ++s) {
      fixes[s] = readGnssFixes(gnss_bags.empty() ? bags[s] : gnss_bags[s], gnss_topic);
    }
    if (!fixes[0].empty()) {
      origin = fixes[0].front();
      const double inf = std::numeric_limits<double>::infinity();
      for (std::size_t s = 0; s < sessions.size(); ++s) {
        gnss_pairs[s] = makeGnssPairs(fixes[s], origin, sessions[s], antenna, -inf, inf);
      }
    }
    for (std::size_t s = 0; s < sessions.size(); ++s) {
      RCLCPP_INFO(
        logger, "session %zu: %zu GNSS fixes kept, %zu paired with the odometry", s,
        fixes[s].size(), gnss_pairs[s].odom.size());
    }
  }

  if (!check_values.empty()) {
    if (sessions.size() != 2 || split_gap_s >= 0.0 || check_values.size() % 12 != 0) {
      RCLCPP_ERROR(
        logger, "debug_check_transforms: two sessions, no split, 12 values per transform");
      rclcpp::shutdown();
      return 1;
    }
    writeDebugChecks(
      output_dir + "/debug_checks.yaml", sessions[0].keyframes, sessions[1].keyframes,
      check_values, join, &gnss_pairs[0], &gnss_pairs[1]);
    RCLCPP_INFO(
      logger, "debug_check_transforms: %zu transforms checked", check_values.size() / 12);
    rclcpp::shutdown();
    return 0;
  }

  if (parts_as_sessions) {
    sessions = splitSession(std::move(sessions[0]), split_parts, split_gap_s);
    for (int p = 0; p < split_parts; ++p) {
      RCLCPP_INFO(
        logger, "part %d: %zu submaps, %zu loop edges", p, sessions[p].raw.size(),
        sessions[p].loops.size());
    }
    bags.assign(split_parts, bags[0]);
    runs.assign(split_parts, runs[0]);
  }

  std::function<bool(const graphslam::multisession::Match &)> exclude = nullptr;
  auto make_exclude = [&](const std::vector<graphslam::multisession::Keyframe> & b) {
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      for (int i = 0; i < 12; ++i) {
        t.matrix()(i / 4, i % 4) = exclude_values[i];
      }
      std::vector<Eigen::Vector3d> positions;
      for (const auto & k : b) {
        positions.push_back(k.frame.translation());
      }
      return std::function<bool(const graphslam::multisession::Match &)>(
        [t, positions, exclude_radius_m](const graphslam::multisession::Match & m) {
          const Eigen::Vector3d & p = positions[m.b];
          return (m.session * p - t * p).norm() <= exclude_radius_m;
        });
    };

  std::ofstream report(output_dir + "/join_report.yaml");
  if (split_gap_s >= 0.0 && !parts_as_sessions) {
    const auto & all = sessions[0];
    const double middle = 0.5 * (all.stamps.front() + all.stamps.back());
    std::vector<graphslam::multisession::Keyframe> a, b;
    for (std::size_t k = 0; k < all.keyframes.size(); ++k) {
      if (all.stamps[k] < middle - 0.5 * split_gap_s) {
        a.push_back(all.keyframes[k]);
      } else if (all.stamps[k] > middle + 0.5 * split_gap_s) {
        b.push_back(all.keyframes[k]);
      }
    }
    if (exclude_values.size() == 12) {
      exclude = make_exclude(b);
    }
    graphslam::multisession::SessionCache cache_a(a, join), cache_b(b, join);
    graphslam::multisession::GnssPairs gnss_a, gnss_b;
    if (join.gnss_gate && !fixes[0].empty()) {  // the halves' fixes, as their keyframes
      const double inf = std::numeric_limits<double>::infinity();
      gnss_a = makeGnssPairs(fixes[0], origin, all, antenna, -inf, middle - 0.5 * split_gap_s);
      gnss_b = makeGnssPairs(fixes[0], origin, all, antenna, middle + 0.5 * split_gap_s, inf);
      cache_a.gnss = &gnss_a;
      cache_b.gnss = &gnss_b;
    }
    const auto result = graphslam::multisession::joinSessions(cache_a, cache_b, join, exclude);
    double sq = 0.0;
    for (const auto & k : b) {
      const Eigen::Vector3d p = k.frame.translation();
      sq += (result.transform * p - p).squaredNorm();
    }
    report << "split:\n  gap_s: " << split_gap_s << "\n  keyframes_a: " << a.size() <<
      "\n  keyframes_b: " << b.size() << "\n  candidates: " << result.candidates <<
      "\n  accepted: " << result.accepted.size() << "\n  excluded_by_debug_hook: " <<
      (exclude ? "true" : "false") << "\n  joined: " << (result.joined ? "true" : "false") <<
      "\n  ambiguous: " << (result.ambiguous ? "true" : "false") << "\n  self_similar: '" <<
      result.self_similar << "'\n";
    if (join.gnss_gate) {
      report << "  gnss: {pairs_a: " << gnss_a.odom.size() << ", pairs_b: " << gnss_b.odom.size() <<
        ", decided: " << (result.gnss_decided ? "true" : "false") << "}\n";
    }
    report << "  sets_tried:\n";
    for (const auto & t : result.attempts) {
      report << "    - {size: " << t.size << ", places: " << t.places << ", share_below: " <<
        t.share_below << ", median: " << t.median << ", passed: " <<
        (t.passed ? "true" : "false") << ", transform: " << matrixRow(t.transform) <<
        gnssFields(t) << "}\n";
    }
    if (result.joined) {
      report << "  transform: " << matrixRow(result.transform) << "\n  error_to_identity_rms_m: " <<
        std::sqrt(sq / std::max<std::size_t>(1, b.size())) << "\n";
    }
    RCLCPP_INFO(
      logger, "split session: %zu + %zu keyframes, %d candidates, %zu accepted, %zu sets tried, %s",
      a.size(), b.size(), result.candidates, result.accepted.size(), result.attempts.size(),
      result.joined ? "joined" : "not joined");
    rclcpp::shutdown();
    return 0;
  }
  std::ofstream edges_csv(output_dir + "/inter_session_edges.csv");
  edges_csv << "session,map_session,submap_a,submap_b,inlier_fraction,tx,ty,tz,qx,qy,qz,qw\n";
  report << "sessions:\n";
  std::vector<graphslam::pose_graph::SubmapNode> nodes;
  std::vector<graphslam::pose_graph::LoopConstraint> constraints;
  std::vector<int> offsets(sessions.size(), -1);
  std::vector<Eigen::Isometry3d> placement(sessions.size(), Eigen::Isometry3d::Identity());
  // Each session's work (Scan Context, small_gicp clouds, self-similarity) is kept for its joins;
  // session 0's is shared by every join. A stored self-similarity replaces the self-join.
  std::vector<std::unique_ptr<graphslam::multisession::SessionCache>> caches;
  std::vector<std::string> fingerprints, checks;
  auto release = [&caches](std::size_t s) {  // session 0's clouds are used by every join
      if (s > 0) {
        caches[s]->prepared.assign(caches[s]->prepared.size(), nullptr);
      }
    };
  for (std::size_t s = 0; s < sessions.size(); ++s) {
    caches.push_back(
      std::make_unique<graphslam::multisession::SessionCache>(sessions[s].keyframes, join));
    if (join.gnss_gate) {
      caches[s]->gnss = &gnss_pairs[s];
    }
    fingerprints.push_back(
      graphslam::multisession::sessionFingerprint(sessions[s].keyframes, join));
    checks.push_back("none");
    if (!cache_dir.empty()) {
      const int stored = readSelfSimilarity(cache_dir, fingerprints[s]);
      if (stored >= 0) {
        caches[s]->self_similar = stored;
        checks[s] = "stored";
      }
    }
    if (precheck && caches[s]->self_similar < 0) {
      graphslam::multisession::selfSimilar(*caches[s], join);
      release(s);
    }
    RCLCPP_INFO(
      logger, "session %zu: fingerprint %s, self-similarity %s", s, fingerprints[s].c_str(),
      caches[s]->self_similar < 0 ? "not checked yet" :
      (checks[s] == "stored" ? "stored" : "checked"));
  }
  auto add_session = [&](std::size_t s) {
      offsets[s] = static_cast<int>(nodes.size());
      for (std::size_t k = 0; k < sessions[s].raw.size(); ++k) {
        graphslam::pose_graph::SubmapNode node_k;
        node_k.pose = placement[s] * sessions[s].raw[k];
        node_k.odometry_covariance = sessions[s].covariance[k];
        node_k.session_start = (s > 0 && k == 0);
        nodes.push_back(node_k);
      }
      for (auto c : sessions[s].loops) {
        c.from += offsets[s];
        c.to += offsets[s];
        constraints.push_back(c);
      }
    };
  add_session(0);
  const std::size_t n = sessions.size();
  std::vector<int> map_sessions {0};  // the sessions joined so far, in order
  std::vector<graphslam::multisession::JoinResult> results(n);
  std::vector<std::vector<int>> tried_with(n);  // the map's sessions at each session's last try
  std::vector<int> tries(n, 0), pass_joined(n, 0);
  std::vector<bool> excluded(n, false);
  std::vector<std::vector<Eigen::Isometry3d>> map_pose(n);  // optimized submap poses, if any
  bool grew = true;
  for (int pass = 1; grew && (pass == 1 || join_to == "map"); ++pass) {
    grew = false;
    for (std::size_t s = 1; s < n; ++s) {
      if (offsets[s] >= 0 || (pass > 1 && tried_with[s] == map_sessions)) {
        continue;
      }
      // The map: the keyframes of its sessions in session 0's frame, each moved by its session's
      // placement (a keyframe's map stays in its own frame). origin: (session, keyframe).
      const std::vector<int> members = join_to == "map" ? map_sessions : std::vector<int>{0};
      std::vector<graphslam::multisession::Keyframe> map_keyframes;
      std::vector<std::pair<int, int>> origin;
      for (int j : members) {
        for (std::size_t k = 0; k < sessions[j].keyframes.size(); ++k) {
          origin.emplace_back(j, static_cast<int>(k));
        }
      }
      graphslam::multisession::SessionCache * map_cache = caches[0].get();
      std::unique_ptr<graphslam::multisession::SessionCache> joined_map;
      graphslam::multisession::GnssPairs map_gnss;  // the map's GNSS, each session placed rigidly
      if (members.size() > 1) {
        for (const auto & o : origin) {
          const Session & member = sessions[o.first];
          map_keyframes.push_back(member.keyframes[o.second]);
          auto & frame = map_keyframes.back().frame;
          frame = map_pose[o.first].empty() ? placement[o.first] * frame :
            map_pose[o.first][o.second] * member.raw[o.second].inverse() * frame;
        }
        joined_map = std::make_unique<graphslam::multisession::SessionCache>(map_keyframes, join);
        if (join.gnss_gate) {
          for (int j : members) {
            for (std::size_t i = 0; i < gnss_pairs[j].odom.size(); ++i) {
              map_gnss.odom.push_back(placement[j] * gnss_pairs[j].odom[i]);
              map_gnss.enu.push_back(gnss_pairs[j].enu[i]);
            }
          }
          joined_map->gnss = &map_gnss;
        }
        // Every session in the map was checked when it joined: none is self-similar.
        joined_map->self_similar = 0;
        for (int j : members) {
          joined_map->self_similar = std::max(
            joined_map->self_similar,
            graphslam::multisession::selfSimilar(*caches[j], join) ? 1 : 0);
        }
        map_cache = joined_map.get();
      }
      exclude = nullptr;
      if ((s == 1 || parts_as_sessions) && exclude_values.size() == 12) {
        exclude = make_exclude(sessions[s].keyframes);
      }
      excluded[s] = exclude != nullptr;
      results[s] = graphslam::multisession::joinSessions(*map_cache, *caches[s], join, exclude);
      tried_with[s] = members;
      ++tries[s];
      release(s);
      const auto & result = results[s];
      std::string with;
      for (int j : members) {
        with += (with.empty() ? "" : ", ") + std::to_string(j);
      }
      RCLCPP_INFO(
        logger, "session %zu: %d candidates, %zu accepted, %zu sets tried, %s (pass %d, map of "
        "sessions %s)", s, result.candidates, result.accepted.size(), result.attempts.size(),
        result.joined ? "joined" : "not joined", pass, with.c_str());
      if (!result.joined) {
        continue;
      }
      placement[s] = result.transform;
      pass_joined[s] = pass;
      map_sessions.push_back(static_cast<int>(s));
      grew = true;
      offsets[s] = static_cast<int>(nodes.size());  // its nodes follow its inter-session edges
      // one inter-session edge per joined place of the session: its best match
      std::map<int, const graphslam::multisession::Match *> best;
      for (int i : result.set) {
        const auto & m = result.accepted[i];
        if (!best.count(m.b) || m.inlier_fraction > best[m.b]->inlier_fraction) {
          best[m.b] = &m;
        }
      }
      for (const auto & entry : best) {
        const auto & m = *entry.second;
        const int j = origin[m.a].first;
        const auto & ka = sessions[j].keyframes[origin[m.a].second];  // in its own frame
        const auto & kb = sessions[s].keyframes[m.b];
        graphslam::pose_graph::LoopConstraint c;
        c.from = offsets[j] + ka.id;
        c.to = offsets[s] + kb.id;
        c.relative_pose = sessions[j].raw[ka.id].inverse() * ka.frame * m.relative *
          kb.frame.inverse() * sessions[s].raw[kb.id];
        c.fitness_score = 1.0 - m.inlier_fraction;
        constraints.push_back(c);
        const Eigen::Vector3d t = c.relative_pose.translation();
        const Eigen::Quaterniond q(c.relative_pose.rotation());
        edges_csv << s << "," << j << "," << ka.id << "," << kb.id << "," << m.inlier_fraction <<
          "," << t.x() << "," << t.y() << "," << t.z() << "," << q.x() << "," << q.y() << "," <<
          q.z() << "," << q.w() << "\n";
      }
      add_session(s);
      if (reoptimize && join_to == "map" && map_sessions.size() < n) {
        const auto current = graphslam::pose_graph::optimizePoseGraph(
          nodes, constraints, {}, {}, adjacent, loop, graphslam::pose_graph::ImuEdgeConfig{},
          graphslam::pose_graph::Chi2Collection::NONE, true, 10);
        for (int j : map_sessions) {
          map_pose[j].assign(
            current.poses.begin() + offsets[j],
            current.poses.begin() + offsets[j] + sessions[j].raw.size());
        }
      }
    }
  }
  report << "  - session: 0\n    joined: true\n";
  for (std::size_t s = 1; s < n; ++s) {
    const auto & result = results[s];
    report << "  - session: " << s << "\n    joined: " << (result.joined ? "true" : "false") <<
      "\n    candidates: " << result.candidates << "\n    accepted: " << result.accepted.size() <<
      "\n    excluded_by_debug_hook: " << (excluded[s] ? "true" : "false") << "\n    ambiguous: " <<
      (result.ambiguous ? "true" : "false") << "\n    self_similar: '" << result.self_similar <<
      "'\n";
    if (join.gnss_gate) {
      report << "    gnss: {pairs: " << gnss_pairs[s].odom.size() << ", decided: " <<
        (result.gnss_decided ? "true" : "false") << "}\n";
    }
    report << "    sets_tried:\n";
    for (const auto & a : result.attempts) {
      report << "      - {size: " << a.size << ", places: " << a.places << ", share_below: " <<
        a.share_below << ", median: " << a.median << ", passed: " <<
        (a.passed ? "true" : "false") << ", transform: " << matrixRow(a.transform) <<
        gnssFields(a) << "}\n";
    }
    if (result.joined) {
      report << "    transform_to_session_0: " << matrixRow(result.transform) << "\n";
    }
  }
  // how each session was joined: tries, the pass, the map's sessions at its last try
  report << "map_joins:\n";
  for (std::size_t s = 1; s < n; ++s) {
    report << "  - {session: " << s << ", tries: " << tries[s] << ", joined_in_pass: " <<
      pass_joined[s] << ", map_sessions: [";
    for (std::size_t i = 0; i < tried_with[s].size(); ++i) {
      report << (i ? ", " : "") << tried_with[s][i];
    }
    report << "]";
    if (parts_as_sessions && offsets[s] >= 0) {  // the truth is the identity
      double sq = 0.0;
      for (const auto & k : sessions[s].keyframes) {
        const Eigen::Vector3d p = k.frame.translation();
        sq += (placement[s] * p - p).squaredNorm();
      }
      report << ", error_to_identity_rms_m: " <<
        std::sqrt(sq / std::max<std::size_t>(1, sessions[s].keyframes.size()));
    }
    report << "}\n";
  }
  // every session's self-similarity: stored, checked in this run (then stored), or not needed
  report << "self_similarity:\n";
  for (std::size_t s = 0; s < sessions.size(); ++s) {
    const int result = caches[s]->self_similar;
    if (result >= 0 && checks[s] == "none") {
      checks[s] = "checked";
      if (!cache_dir.empty()) {
        writeSelfSimilarity(
          cache_dir, fingerprints[s], result == 1, sessions[s].keyframes.size(), bags[s], runs[s]);
      }
    }
    report << "  - {session: " << s << ", fingerprint: " << fingerprints[s] << ", check: " <<
      checks[s];
    if (result >= 0) {
      report << ", self_similar: " << (result == 1 ? "true" : "false");
    }
    report << "}\n";
  }
  const auto optimized = graphslam::pose_graph::optimizePoseGraph(
    nodes, constraints, {}, {}, adjacent, loop, graphslam::pose_graph::ImuEdgeConfig{},
    graphslam::pose_graph::Chi2Collection::NONE, true, 10);
  for (std::size_t s = 0; s < sessions.size(); ++s) {
    if (offsets[s] < 0) {
      continue;
    }
    std::vector<Eigen::Isometry3d> poses(
      optimized.poses.begin() + offsets[s],
      optimized.poses.begin() + offsets[s] + sessions[s].raw.size());
    writePoses(output_dir + "/trajectory_session" + std::to_string(s) + ".tum", sessions[s].stamps,
      poses);
  }
  RCLCPP_INFO(
    logger, "Multi-session join complete: %zu nodes, %zu constraints, written to %s", nodes.size(),
    constraints.size(), output_dir.c_str());
  rclcpp::shutdown();
  return 0;
}
