#include <cpu_bbs3d/bbs3d.hpp>
#include <cpu_bbs3d/voxelmaps.hpp>
#include <pointcloud_iof/pcd_loader_without_pcl.hpp>

#include <boost/filesystem.hpp>

namespace cpu {

bool BBS3D::set_voxelmaps_coords(const std::string& folder_path) {
  const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
  if (!boost::filesystem::exists(voxelmaps_folder_path)) {
    return false;
  }

  if (!load_voxel_params(voxelmaps_folder_path)) return false;

  if (!set_multi_buckets(voxelmaps_folder_path)) return false;
  return true;
}

bool BBS3D::load_voxel_params(const std::string& voxelmaps_folder_path) {
  std::ifstream ifs(voxelmaps_folder_path + "/voxel_params.txt");
  if (!ifs) return false;
  std::string str;
  std::getline(ifs, str);
  const double min_level_res = std::stod(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  const int max_level = std::stoi(str.substr(str.find(" ") + 1));

  std::getline(ifs, str);
  set_voxel_expantion_rate(std::stod(str.substr(str.find(" ") + 1)));

  voxelmaps_ptr_.reset(new VoxelMaps);
  voxelmaps_ptr_->set_min_res(min_level_res);
  voxelmaps_ptr_->set_max_level(max_level);
  return true;
}

bool BBS3D::set_multi_buckets(const std::string& voxelmaps_folder_path) {
  const auto pcd_files = pciof::load_pcd_file_paths(voxelmaps_folder_path);
  voxelmaps_ptr_->multi_buckets_.resize(pcd_files.size());
  voxelmaps_ptr_->voxelmaps_res_.resize(pcd_files.size());

  for (int i = 0; i < pcd_files.size(); i++) {
    const auto& file = pcd_files[i];
    const auto coords = pciof::read_pcd<int>(file.first);
    if (coords.empty()) return false;
    std::vector<Eigen::Vector4i> coords4i(coords.size());
    for (int j = 0; j < coords.size(); j++) {
      coords4i[j] << coords[j], 1;
    }

    voxelmaps_ptr_->multi_buckets_[i] = coords4i;
    voxelmaps_ptr_->voxelmaps_res_[i] = voxelmaps_ptr_->get_min_res() * std::pow(v_rate_, i);
  }

  // The minimum and maximum x, y, z values are selected from the 3D coordinate vector.
  Eigen::Vector4i min_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::max());
  Eigen::Vector4i max_xyz = Eigen::Vector4i::Constant(std::numeric_limits<int>::lowest());

  int max_level = voxelmaps_ptr_->get_max_level();
  const auto& top_buckets = voxelmaps_ptr_->multi_buckets_[max_level];
  for (const auto& point : top_buckets) {
    min_xyz = min_xyz.cwiseMin(point);
    max_xyz = max_xyz.cwiseMax(point);
  }

  init_tx_range_ = std::make_pair(min_xyz.x(), max_xyz.x());
  init_ty_range_ = std::make_pair(min_xyz.y(), max_xyz.y());
  init_tz_range_ = std::make_pair(min_xyz.z(), max_xyz.z());

  return true;
}

bool BBS3D::save_voxel_params(const std::string& folder_path) {
  boost::filesystem::path dir(folder_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return false;
  }

  double min_level_res = voxelmaps_ptr_->get_min_res();
  int max_level = voxelmaps_ptr_->get_max_level();

  // create voxelmaps coords folder
  const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
  boost::filesystem::path voxelmaps_coords_dir(voxelmaps_folder_path);
  if (!boost::filesystem::exists(voxelmaps_coords_dir)) {
    boost::filesystem::create_directory(voxelmaps_coords_dir);
  }

  std::ofstream ofs(voxelmaps_folder_path + "/voxel_params.txt");
  ofs << "min_level_res " << min_level_res << std::endl;
  ofs << "max_level " << max_level << std::endl;
  ofs << "v_rate " << v_rate_ << std::endl;
  ofs.close();

  return true;
}

bool BBS3D::save_voxelmaps_pcd(const std::string& folder_path) {
  boost::filesystem::path dir(folder_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return false;
  }

  // create voxelmaps coords folder
  const std::string voxelmaps_folder_path = folder_path + "/" + voxelmaps_folder_name_;
  boost::filesystem::path voxelmaps_coords_dir(voxelmaps_folder_path);
  if (!boost::filesystem::exists(voxelmaps_coords_dir)) {
    boost::filesystem::create_directory(voxelmaps_coords_dir);
  }

  for (int i = 0; i < voxelmaps_ptr_->get_max_level() + 1; ++i) {
    const std::string file_path = voxelmaps_folder_path + "/" + std::to_string(i) + ".pcd";
    const auto& coords = voxelmaps_ptr_->multi_buckets_[i];
    if (coords.empty()) {
      std::cout << "[ERROR] Can not get coords" << std::endl;
      return false;
    }

    std::vector<Eigen::Vector3i> coords3i(coords.size());
    for (int i = 0; i < coords.size(); ++i) {
      coords3i[i] = coords[i].head<3>();
    }

    if (!pciof::save_pcd<int>(file_path, coords3i)) {
      std::cout << "[ERROR] Can not save pcd file: " << file_path << std::endl;
      return false;
    }
  }
  return true;
}
}  // namespace cpu