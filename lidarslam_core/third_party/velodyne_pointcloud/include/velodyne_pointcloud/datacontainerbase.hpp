// lidarslam: the part of velodyne_pointcloud's DataContainerBase that RawData::unpack calls, without
// ROS 2. The package's container (lidarslam_core/src/velodyne_decoder.cpp) is velodyne_pointcloud's
// PointcloudXYZIRT with no fixed or target frame: transformPoint leaves the point as it is, and
// pointInRange compares the float range with the double limits, as upstream does.

#ifndef VELODYNE_POINTCLOUD__DATACONTAINERBASE_HPP_
#define VELODYNE_POINTCLOUD__DATACONTAINERBASE_HPP_

#include <cstdint>

namespace velodyne_rawdata
{
class DataContainerBase
{
public:
  virtual ~DataContainerBase() {}

  virtual void addPoint(
    float x, float y, float z, const uint16_t ring, const float distance,
    const float intensity, const float time) = 0;
  virtual void newLine() = 0;
};
}  // namespace velodyne_rawdata

#endif  // VELODYNE_POINTCLOUD__DATACONTAINERBASE_HPP_
