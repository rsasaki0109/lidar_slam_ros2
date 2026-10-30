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

  std::function<bool(const graphslam::multisession::Match &)> exclude = nullptr;
  auto make_exclude = [&](const std::vector<graphslam::multisession::Keyframe> & b) {
      Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
      for (int i = 0; i < 12; ++i) {
        t.matrix()(i / 4, i % 4) = exclude_values[i];
      }
      return std::function<bool(const graphslam::multisession::Match &)>(
        [t, b, exclude_radius_m](const graphslam::multisession::Match & m) {
          const Eigen::Vector3d p = b[m.b].frame.translation();
          return (m.session * p - t * p).norm() <= exclude_radius_m;
        });
    };

  std::ofstream report(output_dir + "/join_report.yaml");
  if (split_gap_s >= 0.0) {
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
      result.self_similar << "'\n  sets_tried:\n";
    for (const auto & t : result.attempts) {
      report << "    - {size: " << t.size << ", places: " << t.places << ", share_below: " <<
        t.share_below << ", median: " << t.median << ", passed: " <<
        (t.passed ? "true" : "false") << "}\n";
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
  edges_csv << "session,submap_a,submap_b,inlier_fraction,tx,ty,tz,qx,qy,qz,qw\n";
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
  for (std::size_t s = 0; s < sessions.size(); ++s) {
    if (s > 0) {
      exclude = nullptr;
      if (s == 1 && exclude_values.size() == 12) {
        exclude = make_exclude(sessions[s].keyframes);
      }
      const auto result = graphslam::multisession::joinSessions(
        *caches[0], *caches[s], join, exclude);
      release(s);
      report << "  - session: " << s << "\n    joined: " << (result.joined ? "true" : "false") <<
        "\n    candidates: " << result.candidates << "\n    accepted: " << result.accepted.size() <<
        "\n    excluded_by_debug_hook: " << (exclude ? "true" : "false") << "\n    ambiguous: " <<
        (result.ambiguous ? "true" : "false") << "\n    self_similar: '" << result.self_similar <<
        "'\n    sets_tried:\n";
      for (const auto & a : result.attempts) {
        report << "      - {size: " << a.size << ", places: " << a.places << ", share_below: " <<
          a.share_below << ", median: " << a.median << ", passed: " <<
          (a.passed ? "true" : "false") <<
          "}\n";
      }
      RCLCPP_INFO(
        logger, "session %zu: %d candidates, %zu accepted, %zu sets tried, %s", s,
        result.candidates, result.accepted.size(), result.attempts.size(),
        result.joined ? "joined" : "not joined");
      if (!result.joined) {
        continue;
      }
      report << "    transform_to_session_0: " << matrixRow(result.transform) << "\n";
      placement[s] = result.transform;
      // one inter-session edge per joined place of B: its best match
      std::map<int, const graphslam::multisession::Match *> best;
      for (int i : result.set) {
        const auto & m = result.accepted[i];
        if (!best.count(m.b) || m.inlier_fraction > best[m.b]->inlier_fraction) {
          best[m.b] = &m;
        }
      }
      offsets[s] = static_cast<int>(nodes.size());
      for (const auto & entry : best) {
        const auto & m = *entry.second;
        const auto & ka = sessions[0].keyframes[m.a];
        const auto & kb = sessions[s].keyframes[m.b];
        graphslam::pose_graph::LoopConstraint c;
        c.from = offsets[0] + ka.id;
        c.to = offsets[s] + kb.id;
        c.relative_pose = sessions[0].raw[ka.id].inverse() * ka.frame * m.relative *
          kb.frame.inverse() * sessions[s].raw[kb.id];
        c.fitness_score = 1.0 - m.inlier_fraction;
        constraints.push_back(c);
        const Eigen::Vector3d t = c.relative_pose.translation();
        const Eigen::Quaterniond q(c.relative_pose.rotation());
        edges_csv << s << "," << ka.id << "," << kb.id << "," << m.inlier_fraction << "," <<
          t.x() <<
          "," << t.y() << "," << t.z() << "," << q.x() << "," << q.y() << "," << q.z() << "," <<
          q.w() << "\n";
      }
    } else {
      offsets[0] = 0;
      report << "  - session: 0\n    joined: true\n";
    }
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
