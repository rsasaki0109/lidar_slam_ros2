#include <gpu_bbs3d/voxelmaps.cuh>
#include <gpu_bbs3d/stream_manager/check_error.cuh>

namespace gpu {

VoxelMaps::VoxelMaps() : min_level_res_(1.0f), max_level_(6), max_bucket_scan_count_(10) {}

VoxelMaps::~VoxelMaps() {}

void VoxelMaps::create_voxelmaps(const std::vector<Eigen::Vector3f>& points, const int v_rate, cudaStream_t stream) {
  const int multi_buckets_size = max_level_ + 1;
  std::vector<Buckets> multi_buckets(multi_buckets_size);

  float resolution = min_level_res_;
  for (int i = 0; i < multi_buckets_size; i++) {
    UnorderedVoxelMap unordered_voxelmap;
    std::vector<Eigen::Vector3i> coords(points.size());
    std::transform(points.begin(), points.end(), coords.begin(), [&](const Eigen::Vector3f& point) {
      return (point.array() / resolution).floor().cast<int>();
    });

    // 0: empty, 1: voxel, -1: neighbor
    for (const auto& coord : coords) {
      if (unordered_voxelmap.count(coord) == 0) {
        // if the coord is not in the map
        unordered_voxelmap[coord] = 1;
      } else if (unordered_voxelmap[coord] == -1) {
        // if the coord is exist as a neighbor
        unordered_voxelmap[coord] = 1;
      } else if (unordered_voxelmap[coord] == 1) {
        // if the coord is exist as a voxel
        continue;
      }

      const auto neighbor_coords = create_neighbor_coords(coord);
      for (const auto& neighbor_coord : neighbor_coords) {
        if (unordered_voxelmap.count(neighbor_coord) == 0) {
          // add neighbor coords
          unordered_voxelmap[neighbor_coord] = -1;
        }
      }
    }

    const Buckets& buckets = create_hash_buckets(unordered_voxelmap);
    multi_buckets[i] = buckets;

    resolution = resolution * v_rate;
  }

  // host to device
  set_buckets_on_device(multi_buckets, v_rate, stream);
}

std::vector<Eigen::Vector3i> VoxelMaps::create_neighbor_coords(const Eigen::Vector3i& vec) {
  std::vector<Eigen::Vector3i> neighbor_coord;
  neighbor_coord.reserve(7);
  neighbor_coord.emplace_back(vec.x() - 1, vec.y() - 1, vec.z());
  neighbor_coord.emplace_back(vec.x() - 1, vec.y(), vec.z());
  neighbor_coord.emplace_back(vec.x(), vec.y() - 1, vec.z());
  neighbor_coord.emplace_back(vec.x() - 1, vec.y() - 1, vec.z() - 1);
  neighbor_coord.emplace_back(vec.x() - 1, vec.y(), vec.z() - 1);
  neighbor_coord.emplace_back(vec.x(), vec.y() - 1, vec.z() - 1);
  neighbor_coord.emplace_back(vec.x(), vec.y(), vec.z() - 1);
  return neighbor_coord;
}

VoxelMaps::Buckets VoxelMaps::create_hash_buckets(const UnorderedVoxelMap& unordered_voxelmap) {
  // Loop until the success rate exceeds the threshold
  std::vector<Eigen::Vector4i> buckets;
  for (int num_buckets = unordered_voxelmap.size(); num_buckets <= unordered_voxelmap.size() * 16; num_buckets *= 2) {
    buckets.resize(num_buckets);
    std::fill(buckets.begin(), buckets.end(), Eigen::Vector4i::Zero());

    int success_count = 0;
    for (const auto& voxel : unordered_voxelmap) {
      Eigen::Vector4i coord;
      coord << voxel.first.x(), voxel.first.y(), voxel.first.z(), 1;
      const std::uint32_t hash = (coord[0] * 73856093) ^ (coord[1] * 19349669) ^ (coord[2] * 83492791);

      // Find empty bucket
      for (int i = 0; i < max_bucket_scan_count_; i++) {
        const std::uint32_t bucket_index = (hash + i) % num_buckets;

        if (buckets[bucket_index].w() == 0) {
          buckets[bucket_index] = coord;
          success_count++;
          break;
        }
      }
    }

    const double success_rate = static_cast<double>(success_count) / unordered_voxelmap.size();
    if (success_rate > 0.999) {
      break;
    }
  }

  return buckets;
}

void VoxelMaps::set_buckets_on_device(const std::vector<Buckets>& multi_buckets, const int v_rate, cudaStream_t stream) {
  d_multi_buckets_.resize(multi_buckets.size());
  voxelmaps_info_.resize(multi_buckets.size());

  float resolution = min_level_res_;
  for (int i = 0; i < multi_buckets.size(); i++) {
    const int buckets_size = multi_buckets[i].size();
    const int buckets_datasize = sizeof(Eigen::Vector4i) * buckets_size;
    DeviceBuckets d_buckets(buckets_size);
    check_error
      << cudaMemcpyAsync(thrust::raw_pointer_cast(d_buckets.data()), multi_buckets[i].data(), buckets_datasize, cudaMemcpyHostToDevice, stream);
    d_multi_buckets_[i] = d_buckets;

    // create voxel map info
    VoxelMapInfo info;
    info.res = resolution;
    info.inv_res = 1.0f / resolution;
    info.max_bucket_scan_count = max_bucket_scan_count_;
    info.num_buckets = buckets_size;
    voxelmaps_info_[i] = info;

    resolution = resolution * v_rate;
  }

  // Extract d_multi_buckets_ pointers
  std::vector<Eigen::Vector4i*> ptrs;
  ptrs.reserve(multi_buckets.size());
  for (int i = 0; i < multi_buckets.size(); i++) {
    ptrs.emplace_back(thrust::raw_pointer_cast(d_multi_buckets_[i].data()));
  }

  // Copy host to device (ptrs)
  d_multi_buckets_ptrs_.resize(multi_buckets.size());
  const int ptrs_datasize = sizeof(Eigen::Vector4i*) * multi_buckets.size();
  check_error << cudaMemcpyAsync(thrust::raw_pointer_cast(d_multi_buckets_ptrs_.data()), ptrs.data(), ptrs_datasize, cudaMemcpyHostToDevice, stream);

  // Copy host to device (voxel map info)
  d_voxelmaps_info_.resize(voxelmaps_info_.size());
  const int info_datasize = sizeof(VoxelMapInfo) * voxelmaps_info_.size();
  check_error
    << cudaMemcpyAsync(thrust::raw_pointer_cast(d_voxelmaps_info_.data()), voxelmaps_info_.data(), info_datasize, cudaMemcpyHostToDevice, stream);

  check_error << cudaStreamSynchronize(stream);
}
}  // namespace gpu