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

#ifndef GRAPH_BASED_SLAM__MAP_POINT_HPP_
#define GRAPH_BASED_SLAM__MAP_POINT_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

namespace graphslam
{

// A saved-map point in 16 bytes. pcl::PointXYZI keeps x, y, z and a padding float in one
// 16-byte block and intensity in a second one (32 bytes); here intensity takes the padding
// slot. The registered fields are the same (x, y, z, intensity, all float, in that order), so
// VoxelGrid, getMinMax3D and the PCD writer, which work field by field, compute and write the
// same values for a MapPoint cloud as for the PointXYZI cloud with the same field values.
// getArray4fMap() covers intensity as its fourth lane; the PCL code used on it only reads
// lanes 0 to 2 of the results. Not for pcl::transformPointCloud, which overwrites lane 3.
struct EIGEN_ALIGN16 MapPoint
{
  union EIGEN_ALIGN16 {
    float data[4];
    struct
    {
      float x;
      float y;
      float z;
      float intensity;
    };
  };
  PCL_ADD_EIGEN_MAPS_POINT4D

  inline MapPoint()
  {
    x = y = z = intensity = 0.0f;
  }
  inline explicit MapPoint(const pcl::PointXYZI & p)
  {
    x = p.x;
    y = p.y;
    z = p.z;
    intensity = p.intensity;
  }

  inline pcl::PointXYZI toPointXYZI() const
  {
    return pcl::PointXYZI(x, y, z, intensity);
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
static_assert(sizeof(MapPoint) == 16, "MapPoint must stay 16 bytes");

using MapCloud = pcl::PointCloud<MapPoint>;

// Copies of a cloud in the other point type: same points, width, height, is_dense, header and
// sensor pose.
template<typename OutT, typename InT>
inline void copyCloudProperties(const pcl::PointCloud<InT> & in, pcl::PointCloud<OutT> & out)
{
  out.header = in.header;
  out.width = in.width;
  out.height = in.height;
  out.is_dense = in.is_dense;
  out.sensor_origin_ = in.sensor_origin_;
  out.sensor_orientation_ = in.sensor_orientation_;
}

inline MapCloud::Ptr toMapCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  MapCloud::Ptr out(new MapCloud);
  out->points.reserve(cloud.size());
  for (const auto & p : cloud.points) {
    out->points.emplace_back(p);
  }
  copyCloudProperties(cloud, *out);
  return out;
}

inline pcl::PointCloud<pcl::PointXYZI>::Ptr toPointXYZICloud(const MapCloud & cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
  out->points.reserve(cloud.size());
  for (const auto & p : cloud.points) {
    out->points.push_back(p.toPointXYZI());
  }
  copyCloudProperties(cloud, *out);
  return out;
}

namespace map_saver
{

// pcl::VoxelGrid<MapPoint> (all data downsampled, no filter field, min 0 points per voxel),
// instantiated from PCL's own implementation in map_point.cpp.
void voxelGridFilter(const MapCloud::ConstPtr & input, float leaf_size, MapCloud & output);

}  // namespace map_saver
}  // namespace graphslam

POINT_CLOUD_REGISTER_POINT_STRUCT(
  graphslam::MapPoint,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

#endif  // GRAPH_BASED_SLAM__MAP_POINT_HPP_
