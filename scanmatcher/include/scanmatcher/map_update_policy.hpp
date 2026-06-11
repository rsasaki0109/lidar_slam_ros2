#ifndef SCANMATCHER_MAP_UPDATE_POLICY_HPP_
#define SCANMATCHER_MAP_UPDATE_POLICY_HPP_

#include <algorithm>
#include <vector>

#include <Eigen/Core>

namespace graphslam
{
namespace map_update_policy
{

// Keyframe trigger: a map update starts once the robot moved far enough,
// no async update is in flight, and pose acceptance did not suppress it.
inline bool shouldTriggerMapUpdate(
  const double trans,
  const double trans_for_mapupdate,
  const bool mapping_in_flight,
  const bool suppress_map_update)
{
  return trans >= trans_for_mapupdate && !mapping_in_flight && !suppress_map_update;
}

// Async warmup gate: run the map update on a worker thread only after the
// map holds enough submaps (warmup_submaps <= 0 disables the warmup).
inline bool useAsyncMapUpdate(
  const bool async_map_update,
  const int async_map_update_warmup_submaps,
  const int num_submaps)
{
  return async_map_update &&
         (async_map_update_warmup_submaps <= 0 ||
         num_submaps >= async_map_update_warmup_submaps);
}

// VoxelHashMap mode: local points within a spatial radius form the
// registration target, capped at 50 m.
inline double voxelHashLocalRadius(const double voxel_hash_map_max_distance)
{
  return std::min(voxel_hash_map_max_distance, 50.0);
}

// Spatial local map: walk submaps newest-first and keep those within the
// radius of the current position, up to num_targeted_cloud - 1 entries
// (the live scan itself is the remaining one). Returns indices in the
// historical iteration order (newest to oldest).
inline std::vector<int> selectSpatialSubmapIndices(
  const std::vector<Eigen::Vector3d> & submap_positions,
  const Eigen::Vector3d & current_position,
  const double spatial_local_map_radius,
  const int num_targeted_cloud)
{
  std::vector<int> indices;
  const int num_submaps = static_cast<int>(submap_positions.size());
  int added = 0;
  for (int i = num_submaps - 1; i >= 0 && added < num_targeted_cloud - 1; i--) {
    const double dist = (submap_positions[i] - current_position).norm();
    if (dist <= spatial_local_map_radius) {
      indices.push_back(i);
      added++;
    }
  }
  return indices;
}

// Temporal local map: the num_targeted_cloud - 1 most recent submaps,
// newest first (original behavior).
inline std::vector<int> selectTemporalSubmapIndices(
  const int num_submaps,
  const int num_targeted_cloud)
{
  std::vector<int> indices;
  for (int i = 0; i < num_targeted_cloud - 1; i++) {
    if (num_submaps - 1 - i < 0) {continue;}
    indices.push_back(num_submaps - 1 - i);
  }
  return indices;
}

// Periodic full-map publish gate.
inline bool shouldPublishMap(const double dt_sec, const double map_publish_period)
{
  return dt_sec > map_publish_period;
}

// Adaptive correspondence-distance EMA update after alignment. A
// non-positive mean correspondence distance leaves the EMA unchanged; the
// first positive sample seeds it.
inline double updateAdaptiveCorrespondenceEma(
  const double current_ema,
  const double mean_correspondence_distance,
  const double ema_alpha)
{
  if (mean_correspondence_distance > 0.0) {
    if (current_ema <= 0.0) {
      return mean_correspondence_distance;  // Initialize
    }
    return ema_alpha * mean_correspondence_distance + (1.0 - ema_alpha) * current_ema;
  }
  return current_ema;
}

// Max correspondence distance applied before alignment when the adaptive
// threshold is active (the shell only applies it while the EMA is seeded).
inline double adaptiveMaxCorrespondenceDistance(
  const double adaptive_corr_dist_multiplier,
  const double adaptive_corr_dist_ema)
{
  return adaptive_corr_dist_multiplier * adaptive_corr_dist_ema;
}

}  // namespace map_update_policy
}  // namespace graphslam

#endif  // SCANMATCHER_MAP_UPDATE_POLICY_HPP_
