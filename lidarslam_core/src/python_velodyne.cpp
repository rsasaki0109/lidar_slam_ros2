// Copyright 2026 lidarslam_ros2 contributors. BSD-2-Clause.
//
// lidarslam._core.VelodyneDecoder: velodyne_msgs/VelodyneScan -> PointCloud2 without ROS 2.
//
// The decoding is velodyne_pointcloud's (third_party/velodyne_pointcloud, ros-drivers/velodyne
// 56fc178d): RawData::unpack per packet into the unorganized XYZIRT container, as
// velodyne_transform_node runs it with organize_cloud false and no fixed or target frame. The
// container below is velodyne_pointcloud's PointcloudXYZIRT and DataContainerBase without the
// tf2 parts: fields x, y, z, intensity (FLOAT32), ring (UINT16), time (FLOAT32) packed with no
// padding (point_step 22), height 1, is_dense true, the scan's stamp and frame_id.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "lidarslam_core/sensor_messages.hpp"
#include "velodyne_pointcloud/datacontainerbase.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace py = pybind11;
using lidarslam_core::PackedCloud;
using lidarslam_core::PointField;

namespace
{

class PointcloudXYZIRT final : public velodyne_rawdata::DataContainerBase
{
public:
  PointcloudXYZIRT(double min_range, double max_range, unsigned int scans_per_packet)
  : min_range_(min_range), max_range_(max_range), scans_per_packet_(scans_per_packet)
  {
    // DataContainerBase(..., 0, 1, true, ...) with the six fields of PointcloudXYZIRT;
    // sensor_msgs::addPointField packs them without alignment.
    const std::pair<const char *, uint8_t> fields[] = {
      {"x", PointField::FLOAT32}, {"y", PointField::FLOAT32}, {"z", PointField::FLOAT32},
      {"intensity", PointField::FLOAT32}, {"ring", PointField::UINT16},
      {"time", PointField::FLOAT32}};
    uint32_t offset = 0;
    for (const auto & [name, datatype] : fields) {
      cloud_.fields.push_back(PointField{name, offset, datatype, 1});
      offset += datatype == PointField::UINT16 ? 2 : 4;
    }
    cloud_.point_step = offset;
    cloud_.width = 0;
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.row_step = cloud_.width * cloud_.point_step;
  }

  void setup(const lidarslam_core::Header & header, size_t packets)
  {
    cloud_.header.stamp = header.stamp;
    sensor_frame_ = header.frame_id;
    cloud_.width = 0;
    cloud_.height = 1;
    cloud_.is_dense = true;
    cloud_.row_step = cloud_.width * cloud_.point_step;
    cloud_.data.assign(packets * scans_per_packet_ * cloud_.point_step, 0);
  }

  void newLine() override {}

  void addPoint(
    float x, float y, float z, const uint16_t ring, const float distance,
    const float intensity, const float time) override
  {
    if (!(distance >= min_range_ && distance <= max_range_)) {  // pointInRange
      return;
    }
    // transformPoint: no fixed or target frame, the point is unchanged.
    uint8_t * point = cloud_.data.data() + static_cast<size_t>(cloud_.width) * cloud_.point_step;
    std::memcpy(point + 0, &x, sizeof(float));
    std::memcpy(point + 4, &y, sizeof(float));
    std::memcpy(point + 8, &z, sizeof(float));
    std::memcpy(point + 12, &intensity, sizeof(float));
    std::memcpy(point + 16, &ring, sizeof(uint16_t));
    std::memcpy(point + 18, &time, sizeof(float));
    ++cloud_.width;
  }

  PackedCloud finishCloud()
  {
    cloud_.data.resize(static_cast<size_t>(cloud_.point_step) * cloud_.width * cloud_.height);
    cloud_.row_step = cloud_.point_step * cloud_.width;
    cloud_.header.frame_id = sensor_frame_;
    return std::move(cloud_);
  }

private:
  double min_range_;
  double max_range_;
  unsigned int scans_per_packet_;
  std::string sensor_frame_;
  PackedCloud cloud_;
};

velodyne_pointcloud::CalibrationDocument toDocument(const py::dict & calibration)
{
  velodyne_pointcloud::CalibrationDocument document;
  for (const auto & [key, value] : calibration) {
    const std::string name = py::str(key);
    if (name == "lasers") {
      for (const auto & laser : value.cast<py::list>()) {
        std::map<std::string, std::string> fields;
        for (const auto & [field, scalar] : laser.cast<py::dict>()) {
          fields[py::str(field)] = py::str(scalar);
        }
        document.lasers.push_back(std::move(fields));
      }
    } else if (py::isinstance<py::str>(value)) {
      document.scalars[name] = py::str(value);
    }
  }
  return document;
}

class VelodyneDecoder
{
public:
  VelodyneDecoder(
    const py::dict & calibration, const std::string & model, double min_range,
    double max_range, double view_direction, double view_width)
  : min_range_(min_range), max_range_(max_range)
  {
    data_ = std::make_unique<velodyne_rawdata::RawData>(toDocument(calibration), model);
    // Transform: data_->setParameters(...) after the container, then container configure().
    data_->setParameters(min_range, max_range, view_direction, view_width);
  }

  int num_lasers() const {return data_->numLasers();}
  int scans_per_packet() const {return data_->scansPerPacket();}

  // packets: (sec, nanosec, 1206 bytes) per packet of the scan, in order.
  PackedCloud decode(
    const lidarslam_core::Header & header,
    const std::vector<std::tuple<int32_t, uint32_t, py::bytes>> & packets)
  {
    PointcloudXYZIRT container(min_range_, max_range_, data_->scansPerPacket());
    container.setup(header, packets.size());
    builtin_interfaces::msg::Time scan_stamp;
    scan_stamp.sec = header.stamp.sec;
    scan_stamp.nanosec = header.stamp.nanosec;
    const rclcpp::Time scan_start_time(scan_stamp);
    velodyne_msgs::msg::VelodynePacket packet;
    for (const auto & [sec, nanosec, data] : packets) {
      const std::string_view bytes(data);
      if (bytes.size() != packet.data.size()) {
        throw std::invalid_argument(
                "VelodynePacket data must be 1206 bytes, got " + std::to_string(bytes.size()));
      }
      packet.stamp.sec = sec;
      packet.stamp.nanosec = nanosec;
      std::memcpy(packet.data.data(), bytes.data(), packet.data.size());
      data_->unpack(packet, container, scan_start_time);
    }
    return container.finishCloud();
  }

private:
  double min_range_;
  double max_range_;
  std::unique_ptr<velodyne_rawdata::RawData> data_;
};

}  // namespace

void bind_velodyne(py::module_ & m)
{
  py::class_<VelodyneDecoder>(m, "VelodyneDecoder")
  .def(
    py::init<const py::dict &, const std::string &, double, double, double, double>(),
    py::arg("calibration"), py::arg("model"), py::arg("min_range"), py::arg("max_range"),
    py::arg("view_direction"), py::arg("view_width"))
  .def_property_readonly("num_lasers", &VelodyneDecoder::num_lasers)
  .def_property_readonly("scans_per_packet", &VelodyneDecoder::scans_per_packet)
  .def("decode", &VelodyneDecoder::decode, py::arg("header"), py::arg("packets"));
}
