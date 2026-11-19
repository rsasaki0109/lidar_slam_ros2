// Copyright 2026 lidarslam_ros2 contributors.
// SPDX-License-Identifier: BSD-2-Clause
#ifndef LIDARSLAM_CORE__PYTHON_MAP_CHANGES_HPP_
#define LIDARSLAM_CORE__PYTHON_MAP_CHANGES_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "graph_based_slam/map_change_evidence.hpp"
#include "graph_based_slam/map_store.hpp"

namespace lidarslam_core
{
namespace changes = graphslam::map_change;

using ChangeScan = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

inline std::vector<changes::Point> scanEndpoints(
  const changes::Point & origin, const Eigen::Ref<const ChangeScan> & endpoints)
{
  if (!changes::usable(origin)) {
    throw std::invalid_argument("invalid change-evidence sensor origin");
  }
  std::unordered_set<changes::Voxel, changes::VoxelHash> occupied;
  std::vector<changes::Point> rays;
  for (Eigen::Index i = 0; i < endpoints.rows(); ++i) {
    const changes::Point point{endpoints(i, 0), endpoints(i, 1), endpoints(i, 2)};
    if (changes::usable(point) && changes::squaredDistance(origin, point) <= 1600.0 &&
      occupied.insert(changes::voxel(point)).second)
    {
      rays.push_back(point);
    }
  }
  return rays;
}

struct VoxelCentroid
{
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  std::size_t count{0};
};

inline std::unordered_map<changes::Voxel, VoxelCentroid, changes::VoxelHash> submapCentroids(
  const graphslam::map_store::StoredSubmap & stored)
{
  std::unordered_map<changes::Voxel, VoxelCentroid, changes::VoxelHash> centers;
  const auto cloud = graphslam::map_store::submapCloud(stored);
  for (const auto & point : *cloud) {
    const changes::Point local{point.x, point.y, point.z};
    if (changes::usable(local)) {
      auto & center = centers[changes::voxel(local)];
      center.sum += point.getVector3fMap().cast<double>();
      ++center.count;
    }
  }
  return centers;
}

class MapChangeEvidence
{
public:
  using Poses = std::vector<std::vector<Eigen::Matrix4d>>;
  using Scan = ChangeScan;
  // session, submap, local voxel xyz, hits, misses, separated views, decision
  using EvidenceRow = std::array<std::int64_t, 9>;
  MapChangeEvidence(const std::vector<std::string> & directories, const Poses & poses)
  {
    if (directories.size() != poses.size()) {
      throw std::invalid_argument("expected one absolute pose list per reference session");
    }
    for (std::size_t session = 0; session < directories.size(); ++session) {
      graphslam::map_store::SessionMap stored;
      std::string error;
      if (!graphslam::map_store::readSession(directories[session], stored, error)) {
        throw std::runtime_error(error);
      }
      if (poses[session].size() != stored.submaps.size()) {
        throw std::invalid_argument("change evidence poses do not match the stored submaps");
      }
      for (std::size_t submap = 0; submap < stored.submaps.size(); ++submap) {
        addSubmap(session, submap, stored.submaps[submap], poses[session][submap]);
      }
    }
  }

  void observe(const changes::Point & origin, const Eigen::Ref<const Scan> & endpoints)
  {
    observeRays(origin, scanEndpoints(origin, endpoints));
  }

  std::vector<EvidenceRow> snapshot() const
  {
    std::vector<EvidenceRow> rows;
    for (const auto & state : states_) {
      if (!state.marked) {
        continue;
      }
      const bool remove = state.separated &&
        state.misses >= 8 * std::max(std::int64_t{1}, state.hits);
      const bool keep = state.hits >= 2 && state.hits >= state.misses;
      rows.push_back({state.session, state.submap, state.local[0], state.local[1],
        state.local[2], state.hits, state.misses, state.separated, remove ? 2 : (keep ? 1 : 0)});
    }
    return rows;
  }

private:
  friend class RefinedNewPointEvidence;

  void observeRays(const changes::Point & origin, const std::vector<changes::Point> & rays)
  {
    ++frame_;
    const double hit_radius = refined_ ? .10 : .09;
    // Finish counting hits first so the same frame cannot also count a miss.
    for (const auto & point : rays) {
      const auto cell = changes::voxel(point);
      for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
          for (int z = -1; z <= 1; ++z) {
            visitCell({cell[0] + x, cell[1] + y, cell[2] + z}, [&](State & state) {
              if (state.last_hit != frame_ &&
                changes::squaredDistance(point, state.center) <= hit_radius * hit_radius)
              {
                state.last_hit = frame_;
                ++state.hits;
                state.marked = true;
              }
            });
          }
        }
      }
    }
    for (const auto & point : rays) {
      const Eigen::Vector3d ray(point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]);
      const double length = refined_ ? ray.norm() : 0.;
      changes::traceFree(origin, point, [&](const auto & cell) {
        visitCell(cell, [&](State & state) {
          if (state.last_hit == frame_ || state.last_miss == frame_) {
            return;
          }
          if (refined_) {
            const Eigen::Vector3d delta(state.center[0] - origin[0],
              state.center[1] - origin[1], state.center[2] - origin[2]);
            const double depth = ray.dot(delta) / length;
            if (depth <= 0. || depth > length - changes::ray_margin_m ||
              delta.squaredNorm() - depth * depth > .085 * .085)
            {
              return;
            }
          }
          state.last_miss = frame_;
          if (state.misses++ == 0) {
            state.first_view = origin;
          } else if (changes::squaredDistance(origin, state.first_view) >= .5 * .5) {
            state.separated = true;
          }
          state.marked = true;
        });
      });
    }
  }
  struct State
  {
    std::int64_t session, submap;
    changes::Voxel local;
    changes::Point center;
    changes::Point first_view{};
    std::int64_t hits{0}, misses{0}, last_hit{-1}, last_miss{-1};
    bool separated{false}, marked{false};
  };

  void addSubmap(
    std::size_t session, std::size_t submap,
    const graphslam::map_store::StoredSubmap & stored, const Eigen::Matrix4d & pose,
    const Eigen::Vector3d * range_origin = nullptr)
  {
    for (const auto & entry : submapCentroids(stored)) {
      const Eigen::Vector3d local = entry.second.sum / entry.second.count;
      if (range_origin && (local - *range_origin).squaredNorm() > 1600.) {
        continue;
      }
      const Eigen::Vector3d center = pose.topLeftCorner<3, 3>() * local + pose.topRightCorner<3, 1>();
      const changes::Point world{center.x(), center.y(), center.z()};
      if (changes::usable(world)) {
        cells_[changes::voxel(world)].push_back(states_.size());
        states_.push_back({static_cast<std::int64_t>(session), static_cast<std::int64_t>(submap),
          entry.first, world});
      }
    }
  }

  template<typename Visit>
  void visitCell(const changes::Voxel & key, Visit visit)
  {
    const auto found = cells_.find(key);
    if (found != cells_.end()) {
      for (const auto id : found->second) {
        visit(states_[id]);
      }
    }
  }

  std::unordered_map<changes::Voxel, std::vector<std::size_t>, changes::VoxelHash> cells_;
  std::vector<State> states_;
  std::int64_t frame_{0};
  bool refined_{false};
};

class RefinedNewPointEvidence
{
public:
  using MaskRow = std::array<std::int64_t, 4>;

  void observe(const changes::Point & origin, const Eigen::Ref<const ChangeScan> & endpoints)
  {
    auto rays = scanEndpoints(origin, endpoints);
    if (!rays.empty()) {
      scans_.push_back({origin, std::move(rays)});
    }
  }

  std::vector<MaskRow> suppressionMask(
    const std::string & directory, const std::map<std::size_t, Eigen::Matrix4d> & capture_poses,
    const Eigen::Vector3d & sensor_in_base) const
  {
    if (scans_.size() < 3) {
      return {};
    }
    graphslam::map_store::SessionMap session;
    std::string error;
    if (!graphslam::map_store::readSession(directory, session, error)) {
      throw std::runtime_error(error);
    }
    MapChangeEvidence current({}, {});
    current.refined_ = true;
    for (const auto & [index, pose] : capture_poses) {
      if (index >= session.submaps.size()) {
        throw std::invalid_argument("new-point evidence refers to a missing submap");
      }
      current.addSubmap(0, index, session.submaps[index], pose, &sensor_in_base);
    }
    // Build the entire candidate index before voting, so early free space counts.
    for (const auto & scan : scans_) {
      current.observeRays(scan.origin, scan.rays);
    }
    std::vector<MaskRow> mask;
    for (const auto & state : current.states_) {
      if (state.hits < 2 || state.hits < state.misses) {
        mask.push_back({state.submap, state.local[0], state.local[1], state.local[2]});
      }
    }
    return mask;
  }

private:
  struct ScanRays
  {
    changes::Point origin;
    std::vector<changes::Point> rays;
  };

  std::vector<ScanRays> scans_;
};

// Per-session support for the new scan endpoints. Free-space votes apply from
// the first hit onward, and a hit takes precedence within the same scan.
class NewPointEvidence
{
public:
  using MaskRow = std::array<std::int64_t, 4>;

  void observe(const changes::Point & origin, const Eigen::Ref<const ChangeScan> & endpoints)
  {
    const auto rays = scanEndpoints(origin, endpoints);
    ++frame_;
    for (const auto & point : rays) {
      auto & cell = cells_[changes::voxel(point)];
      if (cell.hits++ == 0) {
        cell.representative = point;
      }
      cell.last_hit = frame_;
    }
    for (const auto & point : rays) {
      changes::traceFree(origin, point, [&](const auto & key) {
        const auto found = cells_.find(key);
        if (found == cells_.end()) {
          return;
        }
        auto & cell = found->second;
        if (cell.last_hit != frame_ && cell.last_miss != frame_) {
          ++cell.misses;
          cell.last_miss = frame_;
        }
      });
    }
  }

  std::vector<MaskRow> suppressionMask(
    const std::string & directory, const std::map<std::size_t, Eigen::Matrix4d> & capture_poses,
    const Eigen::Vector3d & sensor_in_base) const
  {
    graphslam::map_store::SessionMap session;
    std::string error;
    if (!graphslam::map_store::readSession(directory, session, error)) {
      throw std::runtime_error(error);
    }
    std::vector<MaskRow> mask;
    // A too-short session cannot support a persistence decision.
    if (frame_ < 3) {
      return mask;
    }
    for (const auto & [index, pose] : capture_poses) {
      if (index >= session.submaps.size()) {
        throw std::invalid_argument("new-point evidence refers to a missing submap");
      }
      for (const auto & [key, centroid] : submapCentroids(session.submaps[index])) {
        const Eigen::Vector3d local = centroid.sum / centroid.count;
        if ((local - sensor_in_base).squaredNorm() > 1600.0) {
          continue;
        }
        const Eigen::Vector3d world = pose.topLeftCorner<3, 3>() * local +
          pose.topRightCorner<3, 1>();
        const changes::Point point{world.x(), world.y(), world.z()};
        if (changes::usable(point) && !supported(point)) {
          mask.push_back({static_cast<std::int64_t>(index), key[0], key[1], key[2]});
        }
      }
    }
    return mask;
  }

private:
  bool supported(const changes::Point & point) const
  {
    const auto key = changes::voxel(point);
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          const auto found = cells_.find({key[0] + x, key[1] + y, key[2] + z});
          if (found != cells_.end()) {
            const auto & cell = found->second;
            if (cell.hits >= 3 && cell.hits >= cell.misses &&
              changes::squaredDistance(point, cell.representative) <= .15 * .15)
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  struct State
  {
    changes::Point representative{};
    std::int64_t hits{0}, misses{0}, last_hit{-1}, last_miss{-1};
  };

  std::unordered_map<changes::Voxel, State, changes::VoxelHash> cells_;
  std::int64_t frame_{0};
};
}  // namespace lidarslam_core
#endif  // LIDARSLAM_CORE__PYTHON_MAP_CHANGES_HPP_
