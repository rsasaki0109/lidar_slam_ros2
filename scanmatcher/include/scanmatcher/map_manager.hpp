#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lidarslam_msgs/msg/map_array.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

namespace graphslam
{

class MapManager
{
public:
  using PointT = pcl::PointXYZI;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  struct Config
  {
    std::string global_frame_id = "map";
    double vg_size_for_map = 0.1;
    int num_targeted_cloud = 10;
  };

  explicit MapManager(const Config& config)
    : config_(config)
  {
    map_array_msg_.header.frame_id = config_.global_frame_id;
    map_array_msg_.cloud_coordinate = lidarslam_msgs::msg::MapArray::LOCAL;
  }

  void initializeMap(
    const PointCloudConstPtr& cloud_ptr,
    const geometry_msgs::msg::PoseStamped& pose,
    const std_msgs::msg::Header& header)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    PointCloudPtr filtered_cloud_ptr(new PointCloud());
    filterCloud(cloud_ptr, filtered_cloud_ptr, config_.vg_size_for_map);

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

    lidarslam_msgs::msg::SubMap submap;
    submap.header = header;
    submap.distance = 0;
    submap.pose = pose.pose;
    submap.cloud = *cloud_msg_ptr;

    map_array_msg_.header = header;
    map_array_msg_.submaps.push_back(submap);

    latest_distance_ = 0;
  }

  void addSubmap(
    const PointCloudConstPtr& cloud_ptr,
    const Eigen::Matrix4f& transformation,
    const geometry_msgs::msg::PoseStamped& pose,
    double trans)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    PointCloudPtr filtered_cloud_ptr(new PointCloud());
    filterCloud(cloud_ptr, filtered_cloud_ptr, config_.vg_size_for_map);

    PointCloudPtr transformed_cloud_ptr(new PointCloud());
    pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, transformation);

    updateTargetedCloud(transformed_cloud_ptr);

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

    lidarslam_msgs::msg::SubMap submap;
    submap.header.frame_id = config_.global_frame_id;
    submap.header.stamp = pose.header.stamp;
    latest_distance_ += trans;
    submap.distance = latest_distance_;
    submap.pose = pose.pose;
    submap.cloud = *cloud_msg_ptr;
    submap.cloud.header.frame_id = config_.global_frame_id;

    map_array_msg_.header.stamp = pose.header.stamp;
    map_array_msg_.submaps.push_back(submap);

    is_map_updated_ = true;
  }

  PointCloud getTargetedCloud() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return targeted_cloud_;
  }

  bool isMapUpdated() const
  {
    return is_map_updated_;
  }

  void clearMapUpdatedFlag()
  {
    is_map_updated_ = false;
  }

  lidarslam_msgs::msg::MapArray getMapArray() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return map_array_msg_;
  }

  double getLatestDistance() const
  {
    return latest_distance_;
  }

  const Config& getConfig() const
  {
    return config_;
  }

  PointCloudPtr buildFullMap() const
  {
    std::lock_guard<std::mutex> lock(mtx_);

    PointCloudPtr map_ptr(new PointCloud);
    for (const auto& submap : map_array_msg_.submaps) {
      PointCloudPtr submap_cloud_ptr(new PointCloud);
      PointCloudPtr transformed_submap_cloud_ptr(new PointCloud);
      pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);

      Eigen::Affine3d affine;
      tf2::fromMsg(submap.pose, affine);
      pcl::transformPointCloud(
        *submap_cloud_ptr, *transformed_submap_cloud_ptr,
        affine.matrix().cast<float>());

      *map_ptr += *transformed_submap_cloud_ptr;
    }
    return map_ptr;
  }

private:
  void filterCloud(const PointCloudConstPtr& input, PointCloudPtr& output, double leaf_size)
  {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(input);
    voxel_grid.filter(*output);
  }

  void updateTargetedCloud(const PointCloudPtr& new_cloud)
  {
    targeted_cloud_.clear();
    targeted_cloud_ += *new_cloud;

    int num_submaps = map_array_msg_.submaps.size();
    for (int i = 0; i < config_.num_targeted_cloud - 1; i++) {
      if (num_submaps - 1 - i < 0) {
        continue;
      }
      PointCloudPtr tmp_ptr(new PointCloud());
      pcl::fromROSMsg(map_array_msg_.submaps[num_submaps - 1 - i].cloud, *tmp_ptr);

      PointCloudPtr transformed_tmp_ptr(new PointCloud());
      Eigen::Affine3d submap_affine;
      tf2::fromMsg(map_array_msg_.submaps[num_submaps - 1 - i].pose, submap_affine);
      pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix().cast<float>());

      targeted_cloud_ += *transformed_tmp_ptr;
    }
  }

  Config config_;
  mutable std::mutex mtx_;

  PointCloud targeted_cloud_;
  lidarslam_msgs::msg::MapArray map_array_msg_;

  std::atomic<bool> is_map_updated_{false};
  double latest_distance_{0};
};

}  // namespace graphslam

#endif  // MAP_MANAGER_HPP_
