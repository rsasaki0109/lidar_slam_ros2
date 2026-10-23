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
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following disclaimer
//    in the documentation and/or other materials provided with the
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

#ifndef GRAPH_BASED_SLAM__CLOUD_INPUT_EXTRINSIC_HPP_
#define GRAPH_BASED_SLAM__CLOUD_INPUT_EXTRINSIC_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace graphslam
{
namespace cloud_input
{

// The LiDAR-to-base extrinsic of the odom/cloud input, in the layout of
// RKO-LIO's extrinsic_lidar2base_quat_xyzw_xyz: [qx, qy, qz, qw, x, y, z].
// The maintained RKO-LIO fork and official releases before 0.4.1 publish the
// deskewed scan in the LiDAR frame but the odometry pose of the base frame,
// so such a scan has to pass through this extrinsic before the odometry pose
// can place it in the map.
struct LidarExtrinsic
{
  Eigen::Isometry3d base_from_lidar {Eigen::Isometry3d::Identity()};
  // Exactly the identity: the input already is in the base frame and is
  // used unchanged.
  bool identity {true};
};

inline bool parseLidarExtrinsic(
  const std::vector<double> & quat_xyzw_xyz, LidarExtrinsic * extrinsic, std::string * error)
{
  if (quat_xyzw_xyz.size() != 7) {
    if (error != nullptr) {
      *error = "extrinsic_lidar2base_quat_xyzw_xyz must have 7 values [qx, qy, qz, qw, x, y, z]";
    }
    return false;
  }
  for (const double value : quat_xyzw_xyz) {
    if (!std::isfinite(value)) {
      if (error != nullptr) {
        *error = "extrinsic_lidar2base_quat_xyzw_xyz must be finite";
      }
      return false;
    }
  }
  const Eigen::Quaterniond rotation(
    quat_xyzw_xyz[3], quat_xyzw_xyz[0], quat_xyzw_xyz[1], quat_xyzw_xyz[2]);
  if (!(rotation.norm() > 1e-9)) {
    if (error != nullptr) {
      *error = "extrinsic_lidar2base_quat_xyzw_xyz must have a non-zero quaternion";
    }
    return false;
  }
  LidarExtrinsic result;
  result.base_from_lidar.linear() = rotation.normalized().toRotationMatrix();
  result.base_from_lidar.translation() =
    Eigen::Vector3d(quat_xyzw_xyz[4], quat_xyzw_xyz[5], quat_xyzw_xyz[6]);
  result.identity = quat_xyzw_xyz[0] == 0.0 && quat_xyzw_xyz[1] == 0.0 &&
    quat_xyzw_xyz[2] == 0.0 && quat_xyzw_xyz[4] == 0.0 && quat_xyzw_xyz[5] == 0.0 &&
    quat_xyzw_xyz[6] == 0.0;
  *extrinsic = result;
  return true;
}

// Official RKO-LIO 0.4.1 and later already moves the deskewed scan into the
// base frame and stamps it with that frame, the odometry's child frame. Such
// a scan must not pass through the extrinsic a second time.
inline bool scanIsInBaseFrame(const std::string & scan_frame_id, const std::string & base_frame_id)
{
  return !scan_frame_id.empty() && scan_frame_id == base_frame_id;
}

// Moves the FLOAT32 x, y and z fields of cloud from the LiDAR frame into the
// base frame in place; every other field (intensity, time, ...) is untouched
// and non-finite points stay as they are. An identity extrinsic leaves the
// cloud unchanged.
inline bool transformCloudToBase(
  const LidarExtrinsic & extrinsic, sensor_msgs::msg::PointCloud2 & cloud, std::string * error)
{
  if (extrinsic.identity) {
    return true;
  }
  std::size_t offsets[3] = {0, 0, 0};
  const char * names[3] = {"x", "y", "z"};
  for (int axis = 0; axis < 3; ++axis) {
    bool found = false;
    for (const auto & field : cloud.fields) {
      if (field.name == names[axis]) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1) {
          if (error != nullptr) {
            *error = std::string("cloud field ") + names[axis] + " must be a single FLOAT32";
          }
          return false;
        }
        offsets[axis] = field.offset;
        found = true;
      }
    }
    if (!found || offsets[axis] + sizeof(float) > cloud.point_step) {
      if (error != nullptr) {
        *error = std::string("cloud has no usable field ") + names[axis];
      }
      return false;
    }
  }
  const std::uint32_t one = 1;
  std::uint8_t host_byte = 0;
  std::memcpy(&host_byte, &one, 1);
  const bool host_little_endian = host_byte == 1;
  if (cloud.is_bigendian == host_little_endian) {
    if (error != nullptr) {
      *error = "cloud byte order differs from the host";
    }
    return false;
  }
  if (cloud.point_step == 0 || cloud.row_step < cloud.width * cloud.point_step ||
    cloud.data.size() < static_cast<std::size_t>(cloud.height) * cloud.row_step)
  {
    if (error != nullptr) {
      *error = "cloud data is shorter than its layout";
    }
    return false;
  }
  const Eigen::Matrix3d rotation = extrinsic.base_from_lidar.linear();
  const Eigen::Vector3d translation = extrinsic.base_from_lidar.translation();
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    std::uint8_t * point = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step;
    for (std::uint32_t column = 0; column < cloud.width; ++column, point += cloud.point_step) {
      float xyz[3];
      for (int axis = 0; axis < 3; ++axis) {
        std::memcpy(&xyz[axis], point + offsets[axis], sizeof(float));
      }
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
        continue;
      }
      const Eigen::Vector3d moved =
        rotation * Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) + translation;
      for (int axis = 0; axis < 3; ++axis) {
        const float value = static_cast<float>(moved[axis]);
        std::memcpy(point + offsets[axis], &value, sizeof(float));
      }
    }
  }
  return true;
}

}  // namespace cloud_input
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__CLOUD_INPUT_EXTRINSIC_HPP_
