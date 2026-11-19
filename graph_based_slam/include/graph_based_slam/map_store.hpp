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
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
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

// One session of a long-term map store (docs/roadmap/long-term-mapping-2026-11.md): the
// backend's submaps (stamp, odometry pose, optimized pose, the scan kept compressed as the backend
// keeps it) with the session's loop edge count, GNSS origin and pose graph, saved to and loaded
// from a session directory; and the Autoware map bundle exported from stored sessions. The
// export of one session is the bundle the backend wrote for it, byte for byte, because it runs
// the same map saving (map_export::writeBundle) on the same clouds and poses.
//
// Files of a session directory:
//   submaps.bin      the submaps (format below, little-endian)
//   pose_graph.g2o   the session's pose graph, when the backend kept one (offline_map_bundle)
//
// submaps.bin: "LSSUBMAP", u32 version (1), u8 origin set, f64 latitude, f64 longitude,
// u64 loop edges, u64 submaps; per submap: f64 stamp, f64[12] odometry pose and f64[12] pose
// (3x4, row-major), the scan header (i32 sec, u32 nanosec, str frame_id, u32 height, u32 width,
// u32 fields, per field str name, u32 offset, u8 datatype, u32 count; u8 is_bigendian,
// u32 point_step, u32 row_step, u8 is_dense), f64 quantization step, bytes compressed,
// bytes data (a str or bytes is a u64 length and its bytes).

#ifndef GRAPH_BASED_SLAM__MAP_STORE_HPP_
#define GRAPH_BASED_SLAM__MAP_STORE_HPP_

#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "graph_based_slam/cloud_compression_core.hpp"
#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/map_export.hpp"

namespace graphslam
{
namespace map_store
{

using StoredCloud = cloud_compression::BasicStoredCloud<msg::PointCloud2>;

struct StoredSubmap
{
  double stamp_sec {0.0};
  // The submap's odometry pose and its pose after the session's pose-graph optimization.
  Eigen::Isometry3d odometry_pose {Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d pose {Eigen::Isometry3d::Identity()};
  // The scan, packed and compressed as the backend keeps it (cloud_compression).
  std::shared_ptr<const StoredCloud> cloud;
};

struct SessionMap
{
  std::vector<StoredSubmap> submaps;
  std::size_t loop_edge_count {0U};
  map_export::ProjectorOrigin origin;
  std::string pose_graph_g2o;
};

// Writes the session to directory (created if needed). Each file is written next to its final
// name and renamed, so a reader never sees half a file. Returns false and sets error on failure.
bool writeSession(const SessionMap & session, const std::string & directory, std::string & error);

// Reads a session written by writeSession. Returns false and sets error on failure.
bool readSession(const std::string & directory, SessionMap & session, std::string & error);

// The submap's cloud in the submap frame, as the backend restores it for its map.
pcl::PointCloud<pcl::PointXYZI>::Ptr submapCloud(const StoredSubmap & submap);

// Writes the Autoware map bundle of the sessions' submaps at their poses, in session order, to
// save_dir. The GNSS origin is the first session's; pose_graph.g2o is written for a single
// session only (a store graph over several sessions comes with relocalization, L3).
void exportBundle(
  const std::vector<SessionMap> & sessions, const map_export::MapExportConfig & config,
  const std::string & save_dir, const map_export::MapExportLog & log);

// Session order, then submap index and suppressed submap-local voxels.
using RemovedVoxels = std::vector<std::map<std::size_t, std::set<std::array<int, 3>>>>;

// Reviewed edits of a person or an agent (lidarslam.map_edits), folded to their final state:
// store-frame voxels (map_change::voxel of a point at its exported pose). A point in a `keep`
// voxel stays even when the change evidence removed it; a point in a `remove` voxel is dropped.
// A voxel is in at most one of the two sets.
struct StoreFrameEdits
{
  std::set<std::array<int, 3>> remove;
  std::set<std::array<int, 3>> keep;
  bool empty() const {return remove.empty() && keep.empty();}
};

// What the export does with one stored point.
enum class PointState : std::uint8_t
{
  kept = 0,  // kept (no evidence or edit removes it)
  removed_by_evidence = 1,  // the change evidence removed it
  removed_by_edit = 2,  // an edit removed it
  kept_by_edit = 3,  // the change evidence removed it and an edit keeps it
};

// The state of a point given in its submap frame, with its submap's mask (may be null), its
// exported pose and the edits.
PointState pointState(
  const pcl::PointXYZI & point, const std::set<std::array<int, 3>> * mask,
  const Eigen::Isometry3d & pose, const StoreFrameEdits & edits);

void exportBundle(
  const std::vector<SessionMap> & sessions, const map_export::MapExportConfig & config,
  const std::string & save_dir, const map_export::MapExportLog & log,
  const RemovedVoxels & removed, const StoreFrameEdits & edits = StoreFrameEdits());

// The store-frame voxel of a point given in its submap frame at its exported pose, as edits
// address it.
std::array<int, 3> storeFrameVoxel(const pcl::PointXYZI & point, const Eigen::Isometry3d & pose);

// Every stored point at its exported pose with its state, in session, submap and point order:
// visit(session, submap, store-frame position, store-frame voxel, state).
void forEachPoint(
  const std::vector<SessionMap> & sessions, const RemovedVoxels & removed,
  const StoreFrameEdits & edits,
  const std::function<void(
    std::size_t, std::size_t, const Eigen::Vector3f &, const std::array<int, 3> &,
    PointState)> & visit);

}  // namespace map_store
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MAP_STORE_HPP_
