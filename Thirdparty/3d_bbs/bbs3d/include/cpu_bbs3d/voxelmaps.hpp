#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include <boost/functional/hash/hash.hpp>

namespace cpu {
class VoxelMaps {
public:
  VoxelMaps();
  ~VoxelMaps();

  struct VectorHash {
    size_t operator()(const Eigen::Vector3i& x) const {
      size_t seed = 0;
      boost::hash_combine(seed, x[0]);
      boost::hash_combine(seed, x[1]);
      boost::hash_combine(seed, x[2]);
      return seed;
    }
  };

  struct VctorEqual {
    bool operator()(const Eigen::Vector3i& v1, const Eigen::Vector3i& v2) const { return v1 == v2; }
  };

  using UnorderedVoxelMap = std::unordered_map<Eigen::Vector3i, int, VectorHash, VctorEqual>;
  using Buckets = std::vector<Eigen::Vector4i>;

  void set_min_res(double min_level_res) { min_level_res_ = min_level_res; }

  void set_max_level(int max_level) { max_level_ = max_level; }

  void set_max_bucket_scan_count(int max_bucket_scan_count) { max_bucket_scan_count_ = max_bucket_scan_count; }

  double get_min_res() const { return min_level_res_; }

  int get_max_level() const { return max_level_; }

  int get_max_bucket_scan_count() const { return max_bucket_scan_count_; }

  void create_voxelmaps(const std::vector<Eigen::Vector3d>& points, const int v_rate);

private:
  std::vector<Eigen::Vector3i> create_neighbor_coords(const Eigen::Vector3i& vec);

  Buckets create_hash_buckets(const UnorderedVoxelMap& unordered_voxelmap);

public:
  std::vector<Buckets> multi_buckets_;

  std::vector<double> voxelmaps_res_;

private:
  double min_level_res_;
  int max_level_, max_bucket_scan_count_;
};
}  // namespace cpu
