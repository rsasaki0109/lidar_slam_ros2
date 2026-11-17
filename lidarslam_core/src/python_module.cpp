// Copyright 2026 lidarslam_ros2 contributors. BSD-2-Clause.
//
// lidarslam._core: Python bindings of the ROS-free core. M2 binds the plain sensor-message types
// and the point-field reader; M3 adds the RKO-LIO odometry driver (python_frontend.cpp).

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "lidarslam_core/sensor_messages.hpp"

namespace py = pybind11;
void bind_velodyne(py::module_ & m);  // python_velodyne.cpp
#ifdef LIDARSLAM_CORE_HAVE_FRONTEND
void bind_frontend(py::module_ & m);  // python_frontend.cpp
#endif
#ifdef LIDARSLAM_CORE_HAVE_BACKEND
void bind_backend(py::module_ & m);  // python_backend.cpp
#endif
using lidarslam_core::ImuSample;
using lidarslam_core::NavSatFix;
using lidarslam_core::PackedCloud;
using lidarslam_core::PointField;

namespace
{

size_t fieldSize(uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8: case PointField::UINT8: return 1;
    case PointField::INT16: case PointField::UINT16: return 2;
    case PointField::INT32: case PointField::UINT32: case PointField::FLOAT32: return 4;
    case PointField::FLOAT64: return 8;
    default: throw std::invalid_argument("unknown PointField datatype " + std::to_string(datatype));
  }
}

template<typename T>
double readAs(const uint8_t * p)
{
  T value;
  std::memcpy(&value, p, sizeof(T));
  return static_cast<double>(value);
}

double readScalar(const uint8_t * p, uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8: return readAs<int8_t>(p);
    case PointField::UINT8: return readAs<uint8_t>(p);
    case PointField::INT16: return readAs<int16_t>(p);
    case PointField::UINT16: return readAs<uint16_t>(p);
    case PointField::INT32: return readAs<int32_t>(p);
    case PointField::UINT32: return readAs<uint32_t>(p);
    case PointField::FLOAT32: return readAs<float>(p);
    case PointField::FLOAT64: return readAs<double>(p);
    default: throw std::invalid_argument("unknown PointField datatype");
  }
}

// One named field of every point, as float64 (little-endian clouds only).
py::array_t<double> fieldValues(const PackedCloud & cloud, const std::string & name)
{
  if (cloud.is_bigendian) {
    throw std::invalid_argument("big-endian PointCloud2 is not supported");
  }
  const PointField * field = nullptr;
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      field = &f;
      break;
    }
  }
  if (field == nullptr) {
    throw py::key_error("no point field named '" + name + "'");
  }
  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  if (field->offset + fieldSize(field->datatype) > cloud.point_step ||
    cloud.data.size() < static_cast<size_t>(cloud.row_step) * cloud.height ||
    static_cast<size_t>(cloud.point_step) * cloud.width > cloud.row_step)
  {
    throw std::invalid_argument("PointCloud2 layout does not fit its data");
  }
  py::array_t<double> out(static_cast<py::ssize_t>(n));
  auto view = out.mutable_unchecked<1>();
  size_t k = 0;
  for (uint32_t row = 0; row < cloud.height; ++row) {
    const uint8_t * base = cloud.data.data() + static_cast<size_t>(row) * cloud.row_step;
    for (uint32_t col = 0; col < cloud.width; ++col, ++k) {
      view(k) = readScalar(base + static_cast<size_t>(col) * cloud.point_step + field->offset,
          field->datatype);
    }
  }
  return out;
}

py::bytes dataBytes(const PackedCloud & cloud)
{
  return py::bytes(reinterpret_cast<const char *>(cloud.data.data()), cloud.data.size());
}

void setData(PackedCloud & cloud, const py::buffer & buffer)
{
  const py::buffer_info info = buffer.request();
  const auto * begin = static_cast<const uint8_t *>(info.ptr);
  cloud.data.assign(begin, begin + info.size * info.itemsize);
}

}  // namespace

PYBIND11_MODULE(_core, m)
{
  m.doc() = "ROS-free core of lidarslam (sensor-message types; drivers follow).";

  py::class_<lidarslam_core::Stamp>(m, "Stamp")
  .def(py::init<>())
  .def_readwrite("sec", &lidarslam_core::Stamp::sec)
  .def_readwrite("nanosec", &lidarslam_core::Stamp::nanosec);

  py::class_<lidarslam_core::Header>(m, "Header")
  .def(py::init<>())
  .def_readwrite("stamp", &lidarslam_core::Header::stamp)
  .def_readwrite("frame_id", &lidarslam_core::Header::frame_id);

  py::class_<PointField>(m, "PointField")
  .def(py::init<>())
  .def(py::init([](std::string name, uint32_t offset, uint8_t datatype, uint32_t count) {
      return PointField{std::move(name), offset, datatype, count};
    }), py::arg("name"), py::arg("offset"), py::arg("datatype"), py::arg("count"))
  .def_readwrite("name", &PointField::name)
  .def_readwrite("offset", &PointField::offset)
  .def_readwrite("datatype", &PointField::datatype)
  .def_readwrite("count", &PointField::count);

  py::class_<PackedCloud>(m, "PackedCloud")
  .def(py::init<>())
  .def_readwrite("header", &PackedCloud::header)
  .def_readwrite("height", &PackedCloud::height)
  .def_readwrite("width", &PackedCloud::width)
  .def_readwrite("fields", &PackedCloud::fields)
  .def_readwrite("is_bigendian", &PackedCloud::is_bigendian)
  .def_readwrite("point_step", &PackedCloud::point_step)
  .def_readwrite("row_step", &PackedCloud::row_step)
  .def_readwrite("is_dense", &PackedCloud::is_dense)
  .def_property("data", &dataBytes, &setData)
  .def("field", &fieldValues, py::arg("name"),
    "Values of one point field for every point, as float64.")
  .def("__len__", [](const PackedCloud & c) {return static_cast<size_t>(c.width) * c.height;});

  py::class_<ImuSample>(m, "ImuSample")
  .def(py::init<>())
  .def_readwrite("header", &ImuSample::header)
  .def_readwrite("orientation", &ImuSample::orientation)
  .def_readwrite("orientation_covariance", &ImuSample::orientation_covariance)
  .def_readwrite("angular_velocity", &ImuSample::angular_velocity)
  .def_readwrite("angular_velocity_covariance", &ImuSample::angular_velocity_covariance)
  .def_readwrite("linear_acceleration", &ImuSample::linear_acceleration)
  .def_readwrite("linear_acceleration_covariance", &ImuSample::linear_acceleration_covariance);

  py::class_<NavSatFix>(m, "NavSatFix")
  .def(py::init<>())
  .def_readwrite("header", &NavSatFix::header)
  .def_readwrite("status", &NavSatFix::status)
  .def_readwrite("service", &NavSatFix::service)
  .def_readwrite("latitude", &NavSatFix::latitude)
  .def_readwrite("longitude", &NavSatFix::longitude)
  .def_readwrite("altitude", &NavSatFix::altitude)
  .def_readwrite("position_covariance", &NavSatFix::position_covariance)
  .def_readwrite("position_covariance_type", &NavSatFix::position_covariance_type);

  py::class_<lidarslam_core::Odometry>(m, "Odometry")
  .def(py::init<>())
  .def_readwrite("header", &lidarslam_core::Odometry::header)
  .def_readwrite("child_frame_id", &lidarslam_core::Odometry::child_frame_id)
  .def_readwrite("position", &lidarslam_core::Odometry::position)
  .def_readwrite("orientation", &lidarslam_core::Odometry::orientation)
  .def_readwrite("pose_covariance", &lidarslam_core::Odometry::pose_covariance)
  .def_readwrite("linear_velocity", &lidarslam_core::Odometry::linear_velocity)
  .def_readwrite("angular_velocity", &lidarslam_core::Odometry::angular_velocity)
  .def_readwrite("twist_covariance", &lidarslam_core::Odometry::twist_covariance);

  bind_velodyne(m);
#ifdef LIDARSLAM_CORE_HAVE_FRONTEND
  bind_frontend(m);
#endif
#ifdef LIDARSLAM_CORE_HAVE_BACKEND
  bind_backend(m);
#endif
}
