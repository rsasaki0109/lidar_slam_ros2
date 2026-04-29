#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/distances.h>
#include <boost/filesystem.hpp>

namespace pciof {
bool load_tar_clouds(const std::string& tar_path, const float tar_leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr& tar_cloud_ptr) {
  // Load pcd file
  boost::filesystem::path dir(tar_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& file : boost::filesystem::directory_iterator(tar_path)) {
    const std::string filename = file.path().c_str();
    const std::string extension = file.path().extension().string();
    if (extension != ".pcd" && extension != ".PCD") {
      continue;
    }

    // Check load pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(filename, *cloud_temp_ptr) == -1) {
      std::cout << "[WARN] Can not open pcd file: " << filename << std::endl;
      continue;
    }
    *cloud_ptr += *cloud_temp_ptr;
  }

  // Downsample
  if (tar_leaf_size != 0.0f) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(tar_leaf_size, tar_leaf_size, tar_leaf_size);
    filter.setInputCloud(cloud_ptr);
    filter.filter(*filtered_cloud_ptr);
    *tar_cloud_ptr = *filtered_cloud_ptr;
  } else {
    *tar_cloud_ptr = *cloud_ptr;
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

bool load_src_points_with_filename(
  const std::string& src_path,
  const double min_scan_range,
  const double max_scan_range,
  const float src_leaf_size,
  std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>>& cloud_set) {
  boost::filesystem::path dir(src_path);
  if (!boost::filesystem::exists(dir)) {
    std::cout << "[ERROR] Can not open folder" << std::endl;
    return false;
  }

  std::vector<std::pair<std::string, std::string>> pcd_files;
  for (const auto& file : boost::filesystem::directory_iterator(src_path)) {
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

  cloud_set.reserve(pcd_files.size());

  for (const auto& file : pcd_files) {
    // check load pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(file.first, *cloud_ptr) == -1) {
      std::cout << "[WARN] Can not open pcd file: " << file.first << std::endl;
      continue;
    }

    // Cut scan range
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (!(min_scan_range == 0.0 && max_scan_range == 0.0)) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      for (size_t i = 0; i < cloud_ptr->points.size(); ++i) {
        pcl::PointXYZ point = cloud_ptr->points[i];
        double norm = pcl::euclideanDistance(point, pcl::PointXYZ(0, 0, 0));

        if (norm >= min_scan_range && norm <= max_scan_range) {
          cropped_cloud_ptr->points.push_back(point);
        }
      }
      *src_cloud_ptr = *cropped_cloud_ptr;
    } else {
      *src_cloud_ptr = *cloud_ptr;
    }

    // Downsample
    if (src_leaf_size != 0.0f) {
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      filter.setLeafSize(src_leaf_size, src_leaf_size, src_leaf_size);
      filter.setInputCloud(src_cloud_ptr);
      filter.filter(*src_cloud_ptr);
    }

    cloud_set.emplace_back(file.second, src_cloud_ptr);
  }
  return true;
}
}  // namespace pciof
