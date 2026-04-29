#pragma once

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <Eigen/Core>

namespace pciof {
template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

// Declare T before using it in UnorderedVoxelMap
template <typename T>
struct VectorHash {
  size_t operator()(const Eigen::Vector3i& x) const {
    size_t seed = 0;
    boost::hash_combine(seed, x[0]);
    boost::hash_combine(seed, x[1]);
    boost::hash_combine(seed, x[2]);
    return seed;
  }
};

template <typename T>
struct VctorEqual {
  bool operator()(const Eigen::Vector3i& v1, const Eigen::Vector3i& v2) const { return v1 == v2; }
};

template <typename T>
using UnorderedVoxelMap = std::unordered_map<Eigen::Vector3i, std::vector<Vector3<T>>, VectorHash<T>, VctorEqual<T>>;

template <typename T>
pciof::UnorderedVoxelMap<T> create_voxel_map(const std::vector<Vector3<T>>& points3d, const T& voxel_width) {
  pciof::UnorderedVoxelMap<T> voxel_map;
  const auto inv_voxel_width = 1 / voxel_width;
  for (const auto& point : points3d) {
    Eigen::Vector3i voxel = (point.array() * inv_voxel_width).template cast<int>().array().floor();
    voxel_map[voxel].push_back(point);
  }
  return voxel_map;
}

template <typename T>
std::vector<Vector3<T>> filter(const std::vector<Vector3<T>>& points3d, const T& voxel_width) {
  // if width  == 0, return the original point cloud
  if (voxel_width == 0) {
    return points3d;
  }

  // Create a map of voxels
  const auto voxel_map = create_voxel_map(points3d, voxel_width);

  std::vector<Vector3<T>> filtered;
  filtered.reserve(voxel_map.size());

  // To avoid numerical errors, we subtract the first point from all the points
  const Vector3<T> offset = points3d[0];

  // Filter the point cloud
  for (const auto& voxel : voxel_map) {
    Vector3<T> average_point = Vector3<T>::Zero();
    for (const auto& point : voxel.second) {
      average_point += point - offset;
    }
    average_point /= static_cast<T>(voxel.second.size());

    average_point += offset;
    filtered.push_back(average_point);
  }

  return filtered;
}

template <typename T>
std::vector<Vector3<T>> narrow_scan_range(const std::vector<Vector3<T>>& points3d, const T& min_range, const T& max_range) {
  if (min_range == max_range) {
    return points3d;
  }

  std::vector<Vector3<T>> filtered;
  filtered.reserve(points3d.size());
  for (const auto& point : points3d) {
    const auto range = point.norm();
    if (range < min_range || range > max_range) {
      continue;
    }
    filtered.push_back(point);
  }
  return filtered;
}
}  // namespace pciof
