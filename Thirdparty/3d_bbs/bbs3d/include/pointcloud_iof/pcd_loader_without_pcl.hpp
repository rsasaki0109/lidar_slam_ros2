#pragma once
#include <pointcloud_iof/pcd_io.hpp>
#include <pointcloud_iof/filter.hpp>
#include <boost/filesystem.hpp>

namespace pciof {
template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
bool load_tar_points(const std::string& tar_folder_path, const T& filter_voxel_width, std::vector<Vector3<T>>& points) {
  boost::filesystem::path dir(tar_folder_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return false;
  }

  for (const auto& file : boost::filesystem::directory_iterator(tar_folder_path)) {
    const std::string file_name = file.path().c_str();
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }

    const auto loaded_points = read_pcd<T>(file_name);

    if (loaded_points.empty()) {
      std::cout << "[ERROR] Failed to read point cloud from " << file_name << std::endl;
      return false;
    }

    const auto filtered_points = filter<T>(loaded_points, filter_voxel_width);

    points.reserve(points.size() + filtered_points.size());
    points.insert(points.end(), filtered_points.begin(), filtered_points.end());
  }
  return true;
}

bool can_convert_to_int(const std::vector<std::pair<std::string, std::string>>& name_vec) {
  for (const auto& str : name_vec) {
    try {
      std::stoi(str.second);
    } catch (const std::invalid_argument& e) {
      return false;
    } catch (const std::out_of_range& e) {
      return false;
    }
  }
  return true;
}

std::vector<std::pair<std::string, std::string>> load_pcd_file_paths(const std::string& folder_path) {
  boost::filesystem::path dir(folder_path);
  if (!boost::filesystem::exists(dir)) {
    return {};  // output error
  }

  std::vector<std::pair<std::string, std::string>> pcd_files;
  for (const auto& file : boost::filesystem::directory_iterator(folder_path)) {
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }
    pcd_files.emplace_back(file.path().string(), file.path().stem().string());
  }

  if (can_convert_to_int(pcd_files)) {
    std::sort(pcd_files.begin(), pcd_files.end(), [](const std::pair<std::string, std::string>& a, const std::pair<std::string, std::string>& b) {
      return std::stoi(a.second) < std::stoi(b.second);
    });
  }

  return pcd_files;
}

template <typename T>
bool load_src_points(std::string src_folder_path, T min_range, T max_range, T voxel_width, std::vector<std::vector<Vector3<T>>>& src_points_set) {
  const auto pcd_files = load_pcd_file_paths(src_folder_path);
  if (pcd_files.empty()) {
    std::cout << "[ERROR] No pcd files in the folder" << std::endl;
    return false;
  }

  src_points_set.resize(pcd_files.size());

  for (int i = 0; i < pcd_files.size(); i++) {
    const auto& pcd_file = pcd_files[i];
    const auto loaded_points = read_pcd<T>(pcd_file.first);
    if (loaded_points.empty()) {
      std::cout << "[ERROR] Failed to read point cloud from " << pcd_file.first << std::endl;
      return false;
    }

    const auto narrow_scan_range_points = narrow_scan_range<T>(loaded_points, min_range, max_range);
    const auto filtered_points = filter<T>(narrow_scan_range_points, voxel_width);

    src_points_set[i].reserve(filtered_points.size());
    src_points_set[i].insert(src_points_set[i].end(), filtered_points.begin(), filtered_points.end());
  }
}

template <typename T>
bool load_src_points_with_filename(
  std::string src_folder_path,
  T min_range,
  T max_range,
  T voxel_width,
  std::pair<std::string, std::vector<Vector3<T>>>& src_points_set) {
  const auto pcd_files = load_pcd_file_paths(src_folder_path);
  if (pcd_files.empty()) {
    std::cout << "[ERROR] No pcd files in the folder" << std::endl;
    return false;
  }

  src_points_set.resize(pcd_files.size());

  for (int i = 0; i < pcd_files.size(); i++) {
    const auto& pcd_file = pcd_files[i];
    const auto loaded_points = read_pcd<T>(pcd_file.first);
    if (loaded_points.empty()) {
      std::cout << "[ERROR] Failed to read point cloud from " << pcd_file.first << std::endl;
      return false;
    }

    const auto narrow_scan_range_points = narrow_scan_range<T>(loaded_points, min_range, max_range);
    const auto filtered_points = filter<T>(narrow_scan_range_points, voxel_width);

    src_points_set[i].first = pcd_file.second;
    src_points_set[i].second.reserve(filtered_points.size());
    src_points_set[i].second.insert(src_points_set[i].second.end(), filtered_points.begin(), filtered_points.end());
  }
}
}  // namespace pciof