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

// ROS-free point cloud and odometry messages for the backend, and the PCL conversion of a cloud.
//
// The structs mirror sensor_msgs/PointCloud2 and nav_msgs/Odometry member for member (same names,
// same types), so the backend helpers are templates that take either. fromCloudMsg() is
// pcl::fromROSMsg (pcl_conversions) for any such cloud: the same metadata copy and the same PCL
// call, so both give the same points.

#ifndef GRAPH_BASED_SLAM__CLOUD_MESSAGE_HPP_
#define GRAPH_BASED_SLAM__CLOUD_MESSAGE_HPP_

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>

#include <Eigen/Geometry>  // NOLINT(build/include_order)

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace graphslam
{
namespace msg
{

struct Time
{
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
};

struct Header
{
  Time stamp;
  std::string frame_id;
};

struct PointField
{
  static constexpr std::uint8_t INT8 = 1;
  static constexpr std::uint8_t UINT8 = 2;
  static constexpr std::uint8_t INT16 = 3;
  static constexpr std::uint8_t UINT16 = 4;
  static constexpr std::uint8_t INT32 = 5;
  static constexpr std::uint8_t UINT32 = 6;
  static constexpr std::uint8_t FLOAT32 = 7;
  static constexpr std::uint8_t FLOAT64 = 8;

  std::string name;
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
  std::uint32_t count = 0;
};

struct PointCloud2
{
  Header header;
  std::uint32_t height = 0;
  std::uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  std::vector<std::uint8_t> data;
  bool is_dense = false;
};

struct Point
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose
{
  Point position;
  Quaternion orientation;
};

struct PoseWithCovariance
{
  Pose pose;
  std::array<double, 36> covariance{};
};

struct Vector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

struct TwistWithCovariance
{
  Twist twist;
  std::array<double, 36> covariance{};
};

struct Odometry
{
  Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};

}  // namespace msg

namespace cloud_msg
{

constexpr std::uint8_t kFloat32 = 7;  // sensor_msgs/PointField FLOAT32

// rclcpp::Time(stamp).seconds() for a builtin_interfaces/Time-like stamp.
template<typename Stamp>
inline double stampSeconds(const Stamp & stamp)
{
  const std::int64_t nanoseconds =
    static_cast<std::int64_t>(stamp.sec) * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
  return static_cast<double>(nanoseconds) / 1e9;
}

// tf2::fromMsg(geometry_msgs::msg::Pose, Eigen::Affine3d) for any Pose-like message.
template<typename Pose>
inline void poseToAffine(const Pose & pose, Eigen::Affine3d & out)
{
  const auto & q = pose.orientation;
  const auto & p = pose.position;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  quat.normalize();
  out = Eigen::Affine3d(Eigen::Translation3d(p.x, p.y, p.z) * quat);
}

// pcl_conversions::copyPointCloud2MetaData: everything but the point data.
template<typename Cloud>
inline void copyMetaData(const Cloud & cloud, pcl::PCLPointCloud2 & pcl_cloud)
{
  const std::int64_t nanoseconds =
    static_cast<std::int64_t>(cloud.header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(cloud.header.stamp.nanosec);
  pcl_cloud.header.stamp = static_cast<std::uint64_t>(nanoseconds) / 1000ULL;
  pcl_cloud.header.seq = 0;
  pcl_cloud.header.frame_id = cloud.header.frame_id;
  pcl_cloud.height = cloud.height;
  pcl_cloud.width = cloud.width;
  pcl_cloud.fields.resize(cloud.fields.size());
  for (std::size_t i = 0; i < cloud.fields.size(); ++i) {
    pcl_cloud.fields[i].name = cloud.fields[i].name;
    pcl_cloud.fields[i].offset = cloud.fields[i].offset;
    pcl_cloud.fields[i].datatype = cloud.fields[i].datatype;
    pcl_cloud.fields[i].count = cloud.fields[i].count;
  }
  pcl_cloud.is_bigendian = cloud.is_bigendian;
  pcl_cloud.point_step = cloud.point_step;
  pcl_cloud.row_step = cloud.row_step;
  pcl_cloud.is_dense = cloud.is_dense;
}

// pcl::fromROSMsg (pcl_conversions), for sensor_msgs/PointCloud2 or msg::PointCloud2.
template<typename Cloud, typename PointT>
inline void fromCloudMsg(const Cloud & cloud, pcl::PointCloud<PointT> & pcl_cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
#if PCL_VERSION_COMPARE(>=, 1, 13, 1)
  copyMetaData(cloud, pcl_pc2);
  pcl::MsgFieldMap field_map;
  pcl::createMapping<PointT>(pcl_pc2.fields, field_map);
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud, field_map, &cloud.data[0]);
#else
  copyMetaData(cloud, pcl_pc2);
  pcl_pc2.data = cloud.data;
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
#endif
}

}  // namespace cloud_msg
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__CLOUD_MESSAGE_HPP_
