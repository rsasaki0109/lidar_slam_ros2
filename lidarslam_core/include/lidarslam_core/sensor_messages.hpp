// Copyright 2026 lidarslam_ros2 contributors. BSD-2-Clause.
//
// Plain sensor-message types for the ROS-free core. Each mirrors the ROS 2 message of the same
// name field for field (sensor_msgs/PointCloud2, PointField, Imu, NavSatFix), so a ROS adapter can
// convert with a copy and the bytes that reach the mapping code are the same with or without ROS.

#ifndef LIDARSLAM_CORE__SENSOR_MESSAGES_HPP_
#define LIDARSLAM_CORE__SENSOR_MESSAGES_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace lidarslam_core
{

struct Stamp
{
  int32_t sec = 0;
  uint32_t nanosec = 0;
};

struct Header
{
  Stamp stamp;
  std::string frame_id;
};

// sensor_msgs/PointField; datatype uses the same codes (INT8 = 1 ... FLOAT64 = 8).
struct PointField
{
  enum : uint8_t
  {
    INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4, INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8
  };
  std::string name;
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t count = 0;
};

// sensor_msgs/PointCloud2: a packed point buffer described by `fields`.
struct PackedCloud
{
  Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  std::vector<uint8_t> data;
  bool is_dense = false;
};

// sensor_msgs/Imu.
struct ImuSample
{
  Header header;
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
  std::array<double, 9> orientation_covariance{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 3> linear_acceleration{};
  std::array<double, 9> linear_acceleration_covariance{};
};

// sensor_msgs/NavSatFix (status flattened).
struct NavSatFix
{
  Header header;
  int8_t status = 0;
  uint16_t service = 0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  std::array<double, 9> position_covariance{};
  uint8_t position_covariance_type = 0;
};

// nav_msgs/Odometry (pose and twist flattened).
struct Odometry
{
  Header header;
  std::string child_frame_id;
  std::array<double, 3> position{};
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
  std::array<double, 36> pose_covariance{};
  std::array<double, 3> linear_velocity{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 36> twist_covariance{};
};

}  // namespace lidarslam_core

#endif  // LIDARSLAM_CORE__SENSOR_MESSAGES_HPP_
