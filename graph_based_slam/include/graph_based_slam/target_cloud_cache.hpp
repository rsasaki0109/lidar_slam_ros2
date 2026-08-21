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
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
// OF SUCH DAMAGE.

#ifndef GRAPH_BASED_SLAM__TARGET_CLOUD_CACHE_HPP_
#define GRAPH_BASED_SLAM__TARGET_CLOUD_CACHE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

namespace graphslam
{
namespace backend_core
{

using TargetCloud = pcl::PointCloud<pcl::PointXYZI>;
using TargetCloudPtr = TargetCloud::Ptr;

// A candidate source determines which target representation is needed.  Keep
// this in the cache key: a ScanContext target must never be mistaken for the
// regular distance target merely because its neighboring revisions match.
enum class TargetCloudCacheVariant : std::uint8_t
{
  kRegular = 0,
  kScanContext = 1,
  kScanContextWithThreeDBbs = 2,
};

// A revision is supplied by the shell together with each submap snapshot.
// The pose is stored as well as the revision because the live optimizer can
// update a pose without changing the cloud revision.  A zero cloud revision
// is deliberately non-cacheable: callers that cannot provide a stable
// content revision must retain the historical rebuild path.
struct TargetCloudCacheRevision
{
  int submap_index{-1};
  std::uint64_t content_revision{0};
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};

  bool operator==(const TargetCloudCacheRevision & other) const
  {
    return submap_index == other.submap_index &&
           content_revision == other.content_revision &&
           pose == other.pose;
  }
};

// The key covers every input that affects the two target aggregates used by
// loop verification.  It intentionally contains the complete neighboring
// pose/revision list instead of only a hash, so a hash collision cannot reuse
// stale geometry.  Equality is exact, matching the existing deterministic
// Eigen/PCL path.
struct TargetCloudCacheKey
{
  int candidate_index{-1};
  TargetCloudCacheVariant variant{TargetCloudCacheVariant::kRegular};
  int neighbor_radius{0};
  int bbs_neighbor_radius{0};
  double voxel_leaf_size{0.0};
  double bbs_voxel_leaf_size{0.0};
  std::vector<TargetCloudCacheRevision> revisions;

  bool cacheable() const
  {
    if (candidate_index < 0 ||
      (variant != TargetCloudCacheVariant::kRegular &&
      variant != TargetCloudCacheVariant::kScanContext &&
      variant != TargetCloudCacheVariant::kScanContextWithThreeDBbs) ||
      neighbor_radius < 0 || bbs_neighbor_radius < 0 ||
      !std::isfinite(voxel_leaf_size) || voxel_leaf_size <= 0.0 ||
      !std::isfinite(bbs_voxel_leaf_size) || bbs_voxel_leaf_size <= 0.0 || revisions.empty())
    {
      return false;
    }
    for (const auto & revision : revisions) {
      if (revision.submap_index < 0 || revision.content_revision == 0 ||
        !revision.pose.allFinite())
      {
        return false;
      }
    }
    return true;
  }

  bool operator==(const TargetCloudCacheKey & other) const
  {
    return candidate_index == other.candidate_index &&
           variant == other.variant &&
           neighbor_radius == other.neighbor_radius &&
           bbs_neighbor_radius == other.bbs_neighbor_radius &&
           voxel_leaf_size == other.voxel_leaf_size &&
           bbs_voxel_leaf_size == other.bbs_voxel_leaf_size &&
           revisions == other.revisions;
  }
};

struct TargetCloudCacheValue
{
  // Keep only the representation required by the key's candidate variant.
  // Regular candidates retain their final filtered target; ScanContext
  // candidates retain the BBS-radius filtered target; only the 3D-BBS path
  // additionally retains its unfiltered aggregate for the BBS verifier.
  TargetCloudPtr aggregate;
  TargetCloudPtr bbs_aggregate;
  TargetCloudPtr filtered;
  TargetCloudPtr filtered_bbs;

  std::size_t pointCount() const
  {
    return (aggregate ? aggregate->size() : 0U) +
           (bbs_aggregate ? bbs_aggregate->size() : 0U) +
           (filtered ? filtered->size() : 0U) +
           (filtered_bbs ? filtered_bbs->size() : 0U);
  }

  bool completeFor(const TargetCloudCacheVariant variant) const
  {
    switch (variant) {
      case TargetCloudCacheVariant::kRegular:
        return static_cast<bool>(filtered);
      case TargetCloudCacheVariant::kScanContext:
        return static_cast<bool>(filtered_bbs);
      case TargetCloudCacheVariant::kScanContextWithThreeDBbs:
        return static_cast<bool>(bbs_aggregate) && static_cast<bool>(filtered_bbs);
    }
    return false;
  }
};

// Deterministic FNV-1a revision for an already converted cloud.  Offline
// records use this to make replacement of a cloud fail-safe.  This is kept
// here, rather than in the plugin API, because it is only a cache key aid and
// does not impose a cloud/revision policy on external registration plugins.
inline std::uint64_t targetCloudContentRevision(const TargetCloud & cloud)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](const void * data, std::size_t size) {
      const auto * bytes = static_cast<const std::uint8_t *>(data);
      for (std::size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
      }
    };
  mix(&cloud.width, sizeof(cloud.width));
  mix(&cloud.height, sizeof(cloud.height));
  mix(&cloud.is_dense, sizeof(cloud.is_dense));
  for (const auto & point : cloud.points) {
    mix(&point.x, sizeof(point.x));
    mix(&point.y, sizeof(point.y));
    mix(&point.z, sizeof(point.z));
    mix(&point.intensity, sizeof(point.intensity));
  }
  return hash == 0 ? 1 : hash;
}

class TargetCloudCache
{
public:
  struct Config
  {
    // Eight entries cover the observed candidate working set while keeping
    // the cache bounded for a live node that receives untrusted maps.
    std::size_t capacity{8U};
    // The budget counts points across all four retained clouds.  With the
    // default PointXYZI layout this is a hard geometry-memory ceiling of
    // roughly 32 MiB before allocator/container overhead.
    std::size_t max_points{2U * 1000U * 1000U};
  };

  TargetCloudCache() = default;
  explicit TargetCloudCache(const Config & config)
  : config_(config)
  {
  }

  bool lookup(const TargetCloudCacheKey & key, TargetCloudCacheValue * value)
  {
    if (value == nullptr || !key.cacheable()) {
      return false;
    }
    for (auto & entry : entries_) {
      if (entry.key == key) {
        entry.last_use = next_sequence_++;
        *value = entry.value;
        ++hits_;
        return true;
      }
    }
    ++misses_;
    return false;
  }

  bool insert(const TargetCloudCacheKey & key, const TargetCloudCacheValue & value)
  {
    if (!key.cacheable() || !value.completeFor(key.variant) || config_.capacity == 0U) {
      return false;
    }
    const std::size_t points = value.pointCount();
    if (points == 0U || points > config_.max_points) {
      return false;
    }

    for (auto & entry : entries_) {
      if (entry.key == key) {
        total_points_ -= entry.points;
        entry.value = value;
        entry.points = points;
        entry.last_use = next_sequence_++;
        total_points_ += points;
        evictToBudget();
        return contains(key);
      }
    }

    while (entries_.size() >= config_.capacity ||
      total_points_ + points > config_.max_points)
    {
      if (entries_.empty()) {
        return false;
      }
      evictLeastRecentlyUsed();
    }

    entries_.push_back(Entry{key, value, points, next_sequence_++});
    total_points_ += points;
    return true;
  }

  void clear()
  {
    entries_.clear();
    total_points_ = 0U;
    hits_ = 0U;
    misses_ = 0U;
    next_sequence_ = 0U;
  }

  std::size_t size() const {return entries_.size();}
  std::size_t totalPoints() const {return total_points_;}
  std::size_t hits() const {return hits_;}
  std::size_t misses() const {return misses_;}
  const Config & config() const {return config_;}

private:
  struct Entry
  {
    TargetCloudCacheKey key;
    TargetCloudCacheValue value;
    std::size_t points{0U};
    std::uint64_t last_use{0U};
  };

  bool contains(const TargetCloudCacheKey & key) const
  {
    return std::any_of(
      entries_.begin(), entries_.end(),
      [&key](const Entry & entry) {return entry.key == key;});
  }

  void evictLeastRecentlyUsed()
  {
    if (entries_.empty()) {
      return;
    }
    auto victim = entries_.begin();
    for (auto it = entries_.begin() + 1; it != entries_.end(); ++it) {
      // sequence numbers are strictly increasing, so this also gives a
      // deterministic insertion-order tie break if the counter wraps.
      if (it->last_use < victim->last_use) {
        victim = it;
      }
    }
    total_points_ -= victim->points;
    entries_.erase(victim);
  }

  void evictToBudget()
  {
    while (total_points_ > config_.max_points && !entries_.empty()) {
      evictLeastRecentlyUsed();
    }
  }

  Config config_;
  std::vector<Entry> entries_;
  std::size_t total_points_{0U};
  std::size_t hits_{0U};
  std::size_t misses_{0U};
  std::uint64_t next_sequence_{0U};
};

}  // namespace backend_core
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__TARGET_CLOUD_CACHE_HPP_
