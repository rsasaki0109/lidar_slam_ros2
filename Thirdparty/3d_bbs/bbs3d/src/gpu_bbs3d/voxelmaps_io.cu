#include <gpu_bbs3d/bbs3d.cuh>
#include <gpu_bbs3d/voxelmaps.cuh>
#include <pointcloud_iof/pcd_loader_without_pcl.hpp>
#include <boost/filesystem.hpp>

namespace gpu {

bool BBS3D::set_voxelmaps_coords(const std::string& folder_path) {
  const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
  if (!boost::filesystem::exists(voxelmaps_folder_path)) {
    return false;
  }

  if (!load_voxel_params(voxelmaps_folder_path)) return false;

  const auto multi_buckets = set_multi_buckets(voxelmaps_folder_path);
  if (multi_buckets.empty()) return false;

  voxelmaps_ptr_->set_buckets_on_device(multi_buckets, v_rate_, stream);

  return true;
}

bool BBS3D::load_voxel_params(const std::string& voxelmaps_folder_path) {
  std::ifstream ifs(voxelmaps_folder_path + "/voxel_params.txt");
  if (!ifs) return false;
  std::string str;
  std::getline(ifs, str);
  const float min_level_res = std::stof(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int max_level = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  set_voxel_expantion_rate(std::stof(str.substr(str.find(" ") + 1)));

  voxelmaps_ptr_.reset(new VoxelMaps);
  voxelmaps_ptr_->set_min_res(min_level_res);
  voxelmaps_ptr_->set_max_level(max_level);
  return true;
}

std::vector<std::vector<Eigen::Vector4i>> BBS3D::set_multi_buckets(const std::string& voxelmaps_folder_path) {
  const auto pcd_files = pciof::load_pcd_file_paths(voxelmaps_folder_path);

  std::vector<std::vector<Eigen::Vector4i>> multi_buckets;
  multi_buckets.reserve(pcd_files.size());

  for (const auto& file : pcd_files) {
    const auto coords = pciof::read_pcd<int>(file.first);
    if (coords.empty()) return {};
    std::vector<Eigen::Vector4i> coords4i(coords.size());
    for (int i = 0; i < coords.size(); i++) {
      coords4i[i] << coords[i], 1;
    }

    multi_buckets.emplace_back(coords4i);
  }

  // The minimum and maximum x, y, z values are selected from the 3D coordinate vector.
  Eigen::Vector4i min_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::max());
  Eigen::Vector4i max_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::lowest());

  int max_level = voxelmaps_ptr_->get_max_level();
  const auto& top_buckets = multi_buckets[max_level];
  for (const auto& point : top_buckets) {
    min_xyz = min_xyz.cwiseMin(point);
    max_xyz = max_xyz.cwiseMax(point);
  }

  init_tx_range_ = std::make_pair(min_xyz.x(), max_xyz.x());
  init_ty_range_ = std::make_pair(min_xyz.y(), max_xyz.y());
  init_tz_range_ = std::make_pair(min_xyz.z(), max_xyz.z());

  return multi_buckets;
}
}  // namespace gpu