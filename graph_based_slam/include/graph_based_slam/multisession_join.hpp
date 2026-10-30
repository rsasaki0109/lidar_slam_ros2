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

#ifndef GRAPH_BASED_SLAM__MULTISESSION_JOIN_HPP_
#define GRAPH_BASED_SLAM__MULTISESSION_JOIN_HPP_

// Joining two sessions of one site by place recognition, with no prior on their relative pose
// (bench_report/data/grandtour/multisession_2026-10-29/plan.txt, steps 1b and 2):
//   Scan Context candidates between keyframe maps; small_gicp GICP from the Scan Context yaw; a
//   match is kept if most of B's points land on A's; the largest set of matches that agree on one
//   session transform (exact maximum clique) is the join if it spans several places and if the
//   transform also holds over the whole map (B's keyframes that land near A's keyframes must lie on
//   A's maps). A site's repeated structure makes wrong matches that agree with one another; the
//   whole-map check is what rejects them. Needs small_gicp.
// Each registration runs on one thread and the candidates run in parallel (num_threads), so the
// result does not depend on the number of threads (plan.txt, step 5).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)
#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)
#include <small_gicp/ann/kdtree.hpp>  // NOLINT(build/include_order)
#include <small_gicp/points/eigen.hpp>  // NOLINT(build/include_order)
#include <small_gicp/points/point_cloud.hpp>  // NOLINT(build/include_order)
#include <small_gicp/registration/registration_helper.hpp>  // NOLINT(build/include_order)

#include "graph_based_slam/scan_context.hpp"

namespace graphslam
{
namespace multisession
{

struct JoinConfig
{
  double sc_range_m {30.0};
  // Scan Context keeps the highest point of a cell, from 0: the map is raised by this much first.
  double sc_z_offset_m {2.0};
  int sc_candidates {3};
  double sc_max_distance {0.4};
  double registration_voxel_m {0.25};
  std::vector<double> registration_distances_m {2.0, 1.0, 0.5};
  int registration_max_iterations {30};
  double inlier_distance_m {0.3};
  double min_inlier_fraction {0.6};
  double agree_m {0.3};
  double agree_deg {1.0};
  int min_set {3};
  double min_spread_m {5.0};
  double place_radius_m {3.0};
  double place_min_fraction {0.5};
  int min_places {5};
  double max_bad_share {0.10};
  int max_sets {3};
  // Refuse when two of the sets tried pass the whole-map check with transforms that differ (a
  // symmetric room fits turned by 180 deg as well as it fits as it is).
  bool refuse_ambiguous {true};
  double ambiguity_m {1.0};
  double ambiguity_deg {2.0};
  // Refuse when either session joins itself under a transform other than the identity (its
  // place repeats itself, so no join to it can be told apart from a wrong one).
  bool refuse_self_similar {true};
  // In the self-join: Scan Context candidates per keyframe, and keyframes this close in the
  // sequence left out (a keyframe and its neighbours always match themselves).
  int self_candidates {10};
  int self_skip_neighbors {10};
  // In the self-join: a candidate with less than this share of B's points within
  // self_gate_distance_m of A's after the first registration stage is not refined further. With
  // a share at most min_inlier_fraction and a distance at least inlier_distance_m it is then
  // rejected. Most self-join candidates are wrong, and they take most of its time (plan.txt,
  // step 5). 0: off.
  double self_gate_fraction {0.48};
  double self_gate_distance_m {1.0};
  int num_threads {4};
};

// A keyframe: its gravity-aligned frame in the session's odometry frame (position and yaw) and
// its map in that frame.
struct Keyframe
{
  int id {-1};
  Eigen::Isometry3d frame {Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Vector3f> points;
};

struct Match
{
  int a {-1};  // index into A's keyframes
  int b {-1};  // index into B's keyframes
  double sc_distance {1.0};
  double inlier_fraction {0.0};
  Eigen::Isometry3d relative {Eigen::Isometry3d::Identity()};  // T_{frame a <- frame b}
  Eigen::Isometry3d session {Eigen::Isometry3d::Identity()};  // T_{A <- B}
};

struct SetAttempt
{
  int size {0};
  int places {0};
  double share_below {1.0};
  double median {0.0};
  bool passed {false};
};

struct JoinResult
{
  bool joined {false};
  Eigen::Isometry3d transform {Eigen::Isometry3d::Identity()};  // T_{A <- B}
  int candidates {0};
  std::vector<Match> accepted;
  std::vector<int> set;  // indices into accepted: the joined set
  std::vector<SetAttempt> attempts;
  // A set passed, but the join was refused: two passing sets disagree, or a session is
  // self-similar ("a" or "b").
  bool ambiguous {false};
  std::string self_similar;
};

inline Eigen::Isometry3d gravityAlignedFrame(const Eigen::Isometry3d & pose)
{
  const Eigen::Matrix3d r = pose.rotation();
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
  frame.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  frame.translation() = pose.translation();
  return frame;
}

inline double rotationDeg(const Eigen::Matrix3d & r)
{
  const double c = std::max(-1.0, std::min(1.0, (r.trace() - 1.0) / 2.0));
  return std::acos(c) * 180.0 / M_PI;
}

// Bron-Kerbosch with pivoting; with stop_at > 0 it returns once a clique of that size is found.
inline std::vector<int> maxClique(const std::vector<std::set<int>> & adj, int stop_at = 0)
{
  std::vector<int> best;
  std::function<void(std::vector<int> &, std::set<int>, std::set<int>)> bk =
    [&](std::vector<int> & r, std::set<int> p, std::set<int> x) {
      if (stop_at > 0 && static_cast<int>(best.size()) >= stop_at) {
        return;
      }
      if ((p.empty() && x.empty()) || (stop_at > 0 && static_cast<int>(r.size()) >= stop_at)) {
        if (r.size() > best.size()) {
          best = r;
        }
        return;
      }
      if (r.size() + p.size() <= best.size()) {
        return;
      }
      int pivot = -1;
      std::size_t pivot_degree = 0;
      for (const std::set<int> * s : {&p, &x}) {
        for (int u : *s) {
          std::size_t d = 0;
          for (int v : adj[u]) {
            d += p.count(v);
          }
          if (pivot < 0 || d > pivot_degree) {
            pivot = u;
            pivot_degree = d;
          }
        }
      }
      std::vector<int> candidates;
      for (int v : p) {
        if (!adj[pivot].count(v)) {
          candidates.push_back(v);
        }
      }
      for (int v : candidates) {
        std::set<int> p2, x2;
        for (int w : adj[v]) {
          if (p.count(w)) {p2.insert(w);}
          if (x.count(w)) {x2.insert(w);}
        }
        r.push_back(v);
        bk(r, p2, x2);
        r.pop_back();
        p.erase(v);
        x.insert(v);
      }
    };
  std::vector<int> r;
  std::set<int> p;
  for (int i = 0; i < static_cast<int>(adj.size()); ++i) {
    p.insert(i);
  }
  bk(r, p, {});
  return best;
}

namespace detail
{

inline ScanContext::Descriptor scanContext(const Keyframe & keyframe, const JoinConfig & config)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->reserve(keyframe.points.size());
  for (const auto & p : keyframe.points) {
    pcl::PointXYZI q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z() + static_cast<float>(config.sc_z_offset_m);
    q.intensity = 0.0F;
    cloud->push_back(q);
  }
  return ScanContext::computeDescriptor(cloud, config.sc_range_m);
}

struct Prepared
{
  small_gicp::PointCloud::Ptr cloud;
  std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> tree;
};

inline Prepared prepare(const Keyframe & keyframe, const JoinConfig & config)
{
  auto pre = small_gicp::preprocess_points(keyframe.points, config.registration_voxel_m, 10, 1);
  return {pre.first, pre.second};
}

// The share of the source's points within distance of the target's, the source moved by t.
inline double overlap(
  const Prepared & target, const Prepared & source, const Eigen::Isometry3d & t, double distance)
{
  const double max_sq = distance * distance;
  std::size_t inside = 0;
  for (std::size_t i = 0; i < source.cloud->size(); ++i) {
    std::size_t index = 0;
    double sq = 0.0;
    const Eigen::Vector4d q = t.matrix() * source.cloud->point(i);
    if (target.tree->nearest_neighbor_search(q, &index, &sq) && sq < max_sq) {
      ++inside;
    }
  }
  return source.cloud->size() > 0 ?
         static_cast<double>(inside) / static_cast<double>(source.cloud->size()) : 0.0;
}

// Moving B's keyframe positions by two session transforms: the larger distance between them.
inline double disagreement(
  const Match & m, const Match & n, const std::vector<Keyframe> & b)
{
  double worst = 0.0;
  for (const Match * k : {&m, &n}) {
    const Eigen::Vector3d p = b[k->b].frame.translation();
    worst = std::max(worst, (m.session * p - n.session * p).norm());
  }
  return worst;
}

}  // namespace detail

// The work on one session that does not depend on the session it is joined to: Scan Context
// descriptors, small_gicp clouds (made when a join first needs them) and the self-similarity
// check. A join and its self-joins share it, and so do the joins of one session to several
// others. All of them must use the JoinConfig it was made with.
struct SessionCache
{
  SessionCache(const std::vector<Keyframe> & session, const JoinConfig & config)
  : keyframes(session), descriptors(session.size()), prepared(session.size())
  {
    const int n = static_cast<int>(session.size());
#pragma omp parallel for num_threads(config.num_threads) schedule(dynamic)
    for (int i = 0; i < n; ++i) {
      descriptors[i] = detail::scanContext(session[i], config);
    }
  }

  // Makes the small_gicp clouds of these keyframes that are not made yet.
  void prepare(std::vector<int> indices, const JoinConfig & config)
  {
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    indices.erase(
      std::remove_if(
        indices.begin(), indices.end(), [this](int i) {return prepared[i] != nullptr;}),
      indices.end());
    const int n = static_cast<int>(indices.size());
#pragma omp parallel for num_threads(config.num_threads) schedule(dynamic)
    for (int k = 0; k < n; ++k) {
      prepared[indices[k]] = std::make_shared<const detail::Prepared>(
        detail::prepare(keyframes[indices[k]], config));
    }
  }

  const std::vector<Keyframe> & keyframes;
  std::vector<ScanContext::Descriptor> descriptors;
  std::vector<std::shared_ptr<const detail::Prepared>> prepared;
  int self_similar {-1};  // -1: not checked yet
};

// The whole-map check of a session transform: B's keyframes that land within place_radius_m of
// one of A's, and the share of them with less than place_min_fraction of their points within
// inlier_distance_m of those A keyframes' maps.
inline SetAttempt wholeMapCheck(
  const std::vector<Keyframe> & a, const std::vector<Keyframe> & b,
  const Eigen::Isometry3d & session, const JoinConfig & config)
{
  SetAttempt out;
  const double max_sq = config.inlier_distance_m * config.inlier_distance_m;
  const int nb = static_cast<int>(b.size());
  std::vector<double> placed(nb, -1.0);  // the fraction of each placed B keyframe
#pragma omp parallel for num_threads(config.num_threads) schedule(dynamic)
  for (int k = 0; k < nb; ++k) {
    const Keyframe & kb = b[k];
    const Eigen::Isometry3d g = session * kb.frame;
    std::vector<Eigen::Vector3f> near_points;
    for (const Keyframe & ka : a) {
      if ((ka.frame.translation() - g.translation()).norm() > config.place_radius_m) {
        continue;
      }
      for (const auto & p : ka.points) {
        near_points.push_back((ka.frame * p.cast<double>()).cast<float>());
      }
    }
    if (near_points.empty() || kb.points.empty()) {
      continue;
    }
    auto target = std::make_shared<small_gicp::PointCloud>(near_points);
    small_gicp::KdTree<small_gicp::PointCloud> tree(target);
    std::size_t inside = 0;
    for (const auto & p : kb.points) {
      const Eigen::Vector3d q = g * p.cast<double>();
      std::size_t index = 0;
      double sq = 0.0;
      if (tree.nearest_neighbor_search(Eigen::Vector4d(q.x(), q.y(), q.z(), 1.0), &index, &sq) &&
        sq < max_sq)
      {
        ++inside;
      }
    }
    placed[k] = static_cast<double>(inside) / static_cast<double>(kb.points.size());
  }
  std::vector<double> fractions;
  for (double f : placed) {
    if (f >= 0.0) {
      fractions.push_back(f);
    }
  }
  out.places = static_cast<int>(fractions.size());
  if (fractions.empty()) {
    return out;
  }
  const double below = static_cast<double>(std::count_if(
      fractions.begin(), fractions.end(),
      [&](double f) {return f < config.place_min_fraction;}));
  out.share_below = below / static_cast<double>(fractions.size());
  std::nth_element(fractions.begin(), fractions.begin() + fractions.size() / 2, fractions.end());
  out.median = fractions[fractions.size() / 2];
  out.passed = out.places >= config.min_places && out.share_below <= config.max_bad_share;
  return out;
}

inline bool differentTransforms(
  const Eigen::Isometry3d & s, const Eigen::Isometry3d & t, const std::vector<Keyframe> & b,
  const JoinConfig & config);
inline bool selfSimilar(SessionCache & s, const JoinConfig & config);

// Joins session B to session A. exclude (optional) drops matches before the agreement step; it
// is a test hook (the stress test of the plan), not part of the method. In a self-join cache_a
// and cache_b are the same session.
inline JoinResult joinSessions(
  SessionCache & cache_a, SessionCache & cache_b, const JoinConfig & config,
  const std::function<bool(const Match &)> & exclude = nullptr, bool self_join = false)
{
  JoinResult result;
  const std::vector<Keyframe> & a = cache_a.keyframes;
  const std::vector<Keyframe> & b = cache_b.keyframes;
  if (a.empty() || b.empty()) {
    return result;
  }
  // the Scan Context candidates of every B keyframe, in B's order
  struct Candidate
  {
    int a;
    int b;
    double distance;
    int shift;
  };
  const int nb = static_cast<int>(b.size());
  std::vector<std::vector<Candidate>> per_b(nb);
#pragma omp parallel for num_threads(config.num_threads) schedule(dynamic)
  for (int bi = 0; bi < nb; ++bi) {
    if (b[bi].points.empty()) {
      continue;
    }
    std::vector<std::pair<double, int>> ranked;  // (distance, a index), with the shift kept apart
    std::vector<int> shifts(a.size(), 0);
    for (int ai = 0; ai < static_cast<int>(a.size()); ++ai) {
      if (self_join && std::abs(ai - bi) <= config.self_skip_neighbors) {
        continue;
      }
      const auto aligned = ScanContext::distanceWithAlignment(
        cache_b.descriptors[bi], cache_a.descriptors[ai]);
      ranked.emplace_back(aligned.distance, ai);
      shifts[ai] = aligned.shift;
    }
    const int keep = std::min<int>(
      self_join ? config.self_candidates : config.sc_candidates, static_cast<int>(ranked.size()));
    std::partial_sort(ranked.begin(), ranked.begin() + keep, ranked.end());
    for (int r = 0; r < keep; ++r) {
      const int ai = ranked[r].second;
      if (ranked[r].first <= config.sc_max_distance && !a[ai].points.empty()) {
        per_b[bi].push_back({ai, bi, ranked[r].first, shifts[ai]});
      }
    }
  }
  std::vector<Candidate> candidates;
  std::vector<int> need_a, need_b;
  for (const auto & list : per_b) {
    for (const Candidate & c : list) {
      candidates.push_back(c);
      need_a.push_back(c.a);
      need_b.push_back(c.b);
    }
  }
  result.candidates = static_cast<int>(candidates.size());
  cache_a.prepare(need_a, config);
  cache_b.prepare(need_b, config);

  // the registration of every candidate, one thread each
  const int nc = static_cast<int>(candidates.size());
  std::vector<Match> matches(nc);
#pragma omp parallel for num_threads(config.num_threads) schedule(dynamic)
  for (int c = 0; c < nc; ++c) {
    const int ai = candidates[c].a;
    const int bi = candidates[c].b;
    const detail::Prepared & target = *cache_a.prepared[ai];
    const detail::Prepared & source = *cache_b.prepared[bi];
    // B's azimuth theta is seen in A at theta + shift * sector width.
    const double yaw = candidates[c].shift * 2.0 * M_PI / ScanContext::NUM_SECTORS;
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto & stages = config.registration_distances_m;
    for (std::size_t stage = 0; stage < stages.size(); ++stage) {
      small_gicp::RegistrationSetting setting;
      setting.type = small_gicp::RegistrationSetting::GICP;
      setting.max_correspondence_distance = stages[stage];
      setting.num_threads = 1;
      setting.max_iterations = config.registration_max_iterations;
      t = small_gicp::align(*target.cloud, *source.cloud, *target.tree, t,
            setting).T_target_source;
      if (stage == 0 && self_join && config.self_gate_fraction > 0.0 &&
        detail::overlap(target, source, t, config.self_gate_distance_m) <
        config.self_gate_fraction)
      {
        break;
      }
    }
    Match & m = matches[c];
    m.a = ai;
    m.b = bi;
    m.sc_distance = candidates[c].distance;
    m.inlier_fraction = detail::overlap(target, source, t, config.inlier_distance_m);
    m.relative = t;
    m.session = a[ai].frame * t * b[bi].frame.inverse();
  }
  for (const Match & m : matches) {
    if (m.inlier_fraction >= config.min_inlier_fraction && !(exclude && exclude(m))) {
      result.accepted.push_back(m);
    }
  }

  const int n = static_cast<int>(result.accepted.size());
  std::vector<std::set<int>> adj(n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const Match & m = result.accepted[i];
      const Match & k = result.accepted[j];
      if (detail::disagreement(m, k, b) <= config.agree_m &&
        rotationDeg(m.session.rotation().transpose() * k.session.rotation()) <= config.agree_deg)
      {
        adj[i].insert(j);
        adj[j].insert(i);
      }
    }
  }
  std::set<int> remaining;
  for (int i = 0; i < n; ++i) {
    remaining.insert(i);
  }
  for (int attempt = 0; attempt < config.max_sets && !remaining.empty(); ++attempt) {
    std::vector<int> ids(remaining.begin(), remaining.end());
    std::vector<int> local(n, -1);
    for (int k = 0; k < static_cast<int>(ids.size()); ++k) {
      local[ids[k]] = k;
    }
    std::vector<std::set<int>> sub(ids.size());
    for (int k = 0; k < static_cast<int>(ids.size()); ++k) {
      for (int v : adj[ids[k]]) {
        if (local[v] >= 0) {
          sub[k].insert(local[v]);
        }
      }
    }
    std::vector<int> clique;
    for (int k : maxClique(sub)) {
      clique.push_back(ids[k]);
    }
    // the set's distinct B keyframes: at least min_set of them all min_spread_m apart?
    std::vector<int> places;
    for (int i : clique) {
      if (std::find(places.begin(), places.end(), result.accepted[i].b) == places.end()) {
        places.push_back(result.accepted[i].b);
      }
    }
    std::vector<std::set<int>> far(places.size());
    for (std::size_t i = 0; i < places.size(); ++i) {
      for (std::size_t j = 0; j < places.size(); ++j) {
        if (i != j &&
          (b[places[i]].frame.translation() - b[places[j]].frame.translation()).norm() >=
          config.min_spread_m)
        {
          far[i].insert(static_cast<int>(j));
        }
      }
    }
    if (static_cast<int>(maxClique(far, config.min_set).size()) < config.min_set) {
      break;
    }
    int medoid = clique.front();
    double best = std::numeric_limits<double>::max();
    for (int i : clique) {
      double sum = 0.0;
      for (int j : clique) {
        sum += detail::disagreement(result.accepted[i], result.accepted[j], b);
      }
      if (sum < best) {
        best = sum;
        medoid = i;
      }
    }
    SetAttempt check = wholeMapCheck(a, b, result.accepted[medoid].session, config);
    check.size = static_cast<int>(clique.size());
    result.attempts.push_back(check);
    if (check.passed) {
      if (!result.joined) {
        result.joined = true;
        result.transform = result.accepted[medoid].session;
        result.set = clique;
        if (!config.refuse_ambiguous) {
          break;
        }
      } else {
        const Eigen::Isometry3d & other = result.accepted[medoid].session;
        if (differentTransforms(result.transform, other, b, config)) {
          result.ambiguous = true;
        }
      }
    }
    for (int i : clique) {
      remaining.erase(i);
    }
  }
  if (result.ambiguous) {
    result.joined = false;
  }
  if (result.joined && config.refuse_self_similar) {
    if (selfSimilar(cache_a, config)) {
      result.self_similar = "a";
    } else if (selfSimilar(cache_b, config)) {
      result.self_similar = "b";
    }
    if (!result.self_similar.empty()) {
      result.joined = false;
    }
  }
  return result;
}

// Two transforms of B differ if they move B's keyframes apart by more than ambiguity_m (RMS) or
// turn by more than ambiguity_deg.
inline bool differentTransforms(
  const Eigen::Isometry3d & s, const Eigen::Isometry3d & t, const std::vector<Keyframe> & b,
  const JoinConfig & config)
{
  double sq = 0.0;
  for (const Keyframe & k : b) {
    const Eigen::Vector3d p = k.frame.translation();
    sq += (s * p - t * p).squaredNorm();
  }
  const double rms = std::sqrt(sq / static_cast<double>(std::max<std::size_t>(1, b.size())));
  return rms > config.ambiguity_m ||
         rotationDeg(s.rotation().transpose() * t.rotation()) > config.ambiguity_deg;
}

// A session is self-similar if it joins itself, with the matches near the identity left out,
// under the same method (without these two refusals). Checked once per session cache.
inline bool selfSimilar(SessionCache & s, const JoinConfig & config)
{
  if (s.self_similar < 0) {
    JoinConfig inner = config;
    inner.refuse_ambiguous = false;
    inner.refuse_self_similar = false;
    const double radius = config.ambiguity_m;
    const std::vector<Keyframe> & k = s.keyframes;
    s.self_similar = joinSessions(
      s, s, inner, [&k, radius](const Match & m) {
        const Eigen::Vector3d p = k[m.b].frame.translation();
        return (m.session * p - p).norm() <= radius;
      }, true).joined ? 1 : 0;
  }
  return s.self_similar == 1;
}

// A fingerprint of a session for its self-similarity check. It covers the keyframes (their ids,
// frames and maps) and every setting of the check except the number of threads, which does not
// change the result. Equal fingerprints give the same result, so a stored result can be used
// instead of a new self-join (plan.txt, step 6). Raise kSelfSimilarityVersion when the method
// changes in a way the settings do not show.
constexpr std::uint32_t kSelfSimilarityVersion = 1;

inline std::string sessionFingerprint(
  const std::vector<Keyframe> & session, const JoinConfig & config)
{
  std::uint64_t h = 14695981039346656037ULL;  // FNV-1a, 64 bit
  auto add = [&h](const void * data, std::size_t size) {
      const auto * bytes = static_cast<const unsigned char *>(data);
      for (std::size_t i = 0; i < size; ++i) {
        h = (h ^ bytes[i]) * 1099511628211ULL;
      }
    };
  auto value = [&add](auto v) {add(&v, sizeof(v));};
  value(kSelfSimilarityVersion);
  for (double v : {config.sc_range_m, config.sc_z_offset_m, config.sc_max_distance,
      config.registration_voxel_m, config.inlier_distance_m, config.min_inlier_fraction,
      config.agree_m, config.agree_deg, config.min_spread_m, config.place_radius_m,
      config.place_min_fraction, config.max_bad_share, config.ambiguity_m, config.ambiguity_deg,
      config.self_gate_fraction, config.self_gate_distance_m})
  {
    value(v);
  }
  for (int v : {config.sc_candidates, config.registration_max_iterations, config.min_set,
      config.min_places, config.max_sets, config.self_candidates, config.self_skip_neighbors})
  {
    value(v);
  }
  value(config.registration_distances_m.size());
  for (double v : config.registration_distances_m) {
    value(v);
  }
  value(session.size());
  for (const Keyframe & k : session) {
    value(k.id);
    add(k.frame.matrix().data(), 16 * sizeof(double));
    value(k.points.size());
    for (const auto & p : k.points) {
      add(p.data(), 3 * sizeof(float));
    }
  }
  std::ostringstream out;
  out << std::hex << std::setw(16) << std::setfill('0') << h;
  return out.str();
}

// The same for two sessions without caches of their own.
inline JoinResult joinSessions(
  const std::vector<Keyframe> & a, const std::vector<Keyframe> & b, const JoinConfig & config,
  const std::function<bool(const Match &)> & exclude = nullptr)
{
  SessionCache cache_a(a, config);
  SessionCache cache_b(b, config);
  return joinSessions(cache_a, cache_b, config, exclude);
}

}  // namespace multisession
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MULTISESSION_JOIN_HPP_
