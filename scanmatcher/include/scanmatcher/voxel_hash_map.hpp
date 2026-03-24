// Voxel Hash Map for scan matching local map management
// Inspired by KISS-ICP (Vizzo et al., RAL 2023, MIT License)
// Reimplemented without Sophus dependency for PCL registration compatibility.
#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <unordered_map>
#include <vector>

namespace graphslam {

struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &v) const {
    return static_cast<std::size_t>(
      v.x * 73856093 ^ v.y * 19349669 ^ v.z * 83492791);
  }
};

class VoxelHashMapPCL {
public:
  VoxelHashMapPCL(double voxel_size, double max_distance, int max_points_per_voxel)
    : voxel_size_(voxel_size),
      max_distance_(max_distance),
      max_points_per_voxel_(max_points_per_voxel),
      map_resolution_(voxel_size / std::sqrt(static_cast<double>(max_points_per_voxel))) {}

  // Add transformed points to the map
  void addPoints(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) {
    for (const auto &p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      VoxelKey key = pointToVoxel(p);
      auto &voxel = map_[key];
      if (static_cast<int>(voxel.size()) >= max_points_per_voxel_) continue;
      // Check minimum distance to existing points in voxel
      bool too_close = false;
      for (const auto &existing : voxel) {
        double dx = p.x - existing.x, dy = p.y - existing.y, dz = p.z - existing.z;
        if (dx*dx + dy*dy + dz*dz < map_resolution_ * map_resolution_) {
          too_close = true;
          break;
        }
      }
      if (!too_close) {
        voxel.push_back(p);
      }
    }
  }

  // Remove points far from the given position
  void removePointsFarFrom(const Eigen::Vector3d &position) {
    double max_dist_sq = max_distance_ * max_distance_;
    for (auto it = map_.begin(); it != map_.end(); ) {
      if (it->second.empty()) {
        it = map_.erase(it);
        continue;
      }
      const auto &first_pt = it->second.front();
      double dx = first_pt.x - position.x();
      double dy = first_pt.y - position.y();
      double dz = first_pt.z - position.z();
      if (dx*dx + dy*dy + dz*dz > max_dist_sq) {
        it = map_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Update: add points then prune
  void update(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud,
              const Eigen::Vector3d &position) {
    addPoints(cloud);
    removePointsFarFrom(position);
  }

  // Extract all points as a PCL point cloud (for registration target)
  pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() const {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::size_t total = 0;
    for (const auto &kv : map_) total += kv.second.size();
    cloud->points.reserve(total);
    for (const auto &kv : map_) {
      for (const auto &p : kv.second) {
        cloud->points.push_back(p);
      }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  // Get points near a position (for local registration target)
  pcl::PointCloud<pcl::PointXYZI>::Ptr getLocalPoints(
    const Eigen::Vector3d &position, double radius) const {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    double radius_sq = radius * radius;
    for (const auto &kv : map_) {
      if (kv.second.empty()) continue;
      const auto &first_pt = kv.second.front();
      double dx = first_pt.x - position.x();
      double dy = first_pt.y - position.y();
      double dz = first_pt.z - position.z();
      if (dx*dx + dy*dy + dz*dz <= radius_sq) {
        for (const auto &p : kv.second) {
          cloud->points.push_back(p);
        }
      }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  std::size_t numVoxels() const { return map_.size(); }
  std::size_t numPoints() const {
    std::size_t total = 0;
    for (const auto &kv : map_) total += kv.second.size();
    return total;
  }

  void clear() { map_.clear(); }

private:
  VoxelKey pointToVoxel(const pcl::PointXYZI &p) const {
    return {
      static_cast<int>(std::floor(p.x / voxel_size_)),
      static_cast<int>(std::floor(p.y / voxel_size_)),
      static_cast<int>(std::floor(p.z / voxel_size_))
    };
  }

  double voxel_size_;
  double max_distance_;
  int max_points_per_voxel_;
  double map_resolution_;
  std::unordered_map<VoxelKey, std::vector<pcl::PointXYZI>, VoxelKeyHash> map_;
};

}  // namespace graphslam
