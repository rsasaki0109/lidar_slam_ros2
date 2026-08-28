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
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SCANMATCHER__OFFLINE_MAP_EXPORT_HPP_
#define SCANMATCHER__OFFLINE_MAP_EXPORT_HPP_

// Deterministic, frontend-only map export for the offline evidence runner.
// A MapArray submap stores points in the robot frame and its world pose in
// submap.pose. Keep this transform-and-append order identical to
// ScanMatcherComponent::publishMap(); the optional export must not alter the
// trajectory or submap stream used by the existing frontend gate.

#include <Eigen/Core>  // NOLINT(build/include_order)

#include <pcl/common/transforms.h>  // NOLINT(build/include_order)
#include <pcl/io/pcd_io.h>  // NOLINT(build/include_order)
#include <pcl/point_cloud.h>  // NOLINT(build/include_order)
#include <pcl/point_types.h>  // NOLINT(build/include_order)

#include <string>

#include <lidarslam_msgs/msg/map_array.hpp>
#include <pcl_conversions/pcl_conversions.h>  // NOLINT(build/include_order)
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace graphslam
{
namespace offline_map_export
{

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

inline PointCloud mergeSubmaps(const lidarslam_msgs::msg::MapArray & map_array)
{
  PointCloud map;
  for (const auto & submap : map_array.submaps) {
    PointCloud local;
    pcl::fromROSMsg(submap.cloud, local);

    PointCloud transformed;
    Eigen::Affine3d affine;
    tf2::fromMsg(submap.pose, affine);
    pcl::transformPointCloud(local, transformed, affine.matrix().cast<float>());
    map += transformed;
  }
  return map;
}

inline bool saveBinaryCompressed(const std::string & path, const PointCloud & map)
{
  return pcl::io::savePCDFileBinaryCompressed(path, map) == 0;
}

}  // namespace offline_map_export
}  // namespace graphslam

#endif  // SCANMATCHER__OFFLINE_MAP_EXPORT_HPP_
