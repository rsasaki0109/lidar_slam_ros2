// Copyright 2026 lidarslam_ros2 contributors. BSD-2-Clause.
//
// lidarslam._core.OdometryDriver: the RKO-LIO offline pipeline (rko_lio::driver::OfflineDriver)
// with its parameters from a Python dict. The ROS 2 node reads the same parameter names, types and
// defaults through rko_lio::driver::declare_driver_config.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "lidarslam_core/sensor_messages.hpp"
#include "rko_lio/driver/driver_config.hpp"
#include "rko_lio/driver/offline_driver.hpp"
#include "rko_lio/driver/parameters.hpp"

namespace py = pybind11;

namespace
{

// rko_lio::driver::Parameters over a Python dict {name: value}. Integers are accepted where the
// node reads a double; other type mismatches raise TypeError naming the parameter.
class DictParameters : public rko_lio::driver::Parameters
{
public:
  explicit DictParameters(py::dict values)
  : values_(std::move(values)) {}

  bool get_bool(const std::string & name, bool default_value) override
  {
    const auto value = lookup(name);
    if (!value) {return default_value;}
    if (!py::isinstance<py::bool_>(*value)) {throw type_error(name, "a bool");}
    return value->cast<bool>();
  }
  int64_t get_int(const std::string & name, int64_t default_value) override
  {
    const auto value = lookup(name);
    if (!value) {return default_value;}
    if (!py::isinstance<py::int_>(*value) || py::isinstance<py::bool_>(*value)) {
      throw type_error(name, "an integer");
    }
    return value->cast<int64_t>();
  }
  double get_double(const std::string & name, double default_value) override
  {
    const auto value = lookup(name);
    if (!value) {return default_value;}
    if (py::isinstance<py::bool_>(*value) ||
      !(py::isinstance<py::float_>(*value) || py::isinstance<py::int_>(*value)))
    {
      throw type_error(name, "a number");
    }
    return value->cast<double>();
  }
  std::string get_string(const std::string & name, const std::string & default_value) override
  {
    const auto value = lookup(name);
    if (!value) {return default_value;}
    if (!py::isinstance<py::str>(*value)) {throw type_error(name, "a string");}
    return value->cast<std::string>();
  }
  std::vector<double> get_double_array(
    const std::string & name, const std::vector<double> & default_value) override
  {
    const auto value = lookup(name);
    if (!value) {return default_value;}
    try {
      return value->cast<std::vector<double>>();
    } catch (const py::cast_error &) {
      throw type_error(name, "a list of numbers");
    }
  }
  std::vector<int64_t> get_int_array(
    const std::string & name, const std::vector<int64_t> & default_value) override
  {
    const auto value = lookup(name);
    if (!value) {return default_value;}
    try {
      return value->cast<std::vector<int64_t>>();
    } catch (const py::cast_error &) {
      throw type_error(name, "a list of integers");
    }
  }
  std::string get_required_string(const std::string & name) override
  {
    const auto value = lookup(name);
    if (!value) {throw py::key_error("required parameter '" + name + "' is not set");}
    if (!py::isinstance<py::str>(*value)) {throw type_error(name, "a string");}
    return value->cast<std::string>();
  }
  void info(const std::string & message) override {log("INFO", message);}
  void warn(const std::string & message) override {log("WARN", message);}

  // Names in the dict that the driver never read (typos, or parameters of other nodes).
  std::vector<std::string> unused() const
  {
    std::vector<std::string> names;
    for (const auto & item : values_) {
      const auto name = item.first.cast<std::string>();
      if (!read_.count(name)) {names.push_back(name);}
    }
    return names;
  }

  static void log(const char * level, const std::string & message)
  {
    std::cerr << "[rko_lio] [" << level << "] " << message << std::endl;
  }

private:
  std::optional<py::object> lookup(const std::string & name)
  {
    read_.insert(name);
    if (!values_.contains(name)) {return std::nullopt;}
    py::object value = values_[py::str(name)];
    if (value.is_none()) {return std::nullopt;}
    return value;
  }
  static py::type_error type_error(const std::string & name, const char * expected)
  {
    return py::type_error("parameter '" + name + "' must be " + expected);
  }

  py::dict values_;
  std::set<std::string> read_;
};

rko_lio::core::Nsec to_nsec(const lidarslam_core::Stamp & stamp)
{
  // rclcpp::Time(stamp).nanoseconds(), as the ROS 2 node converts header stamps.
  return rko_lio::core::Nsec(static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec);
}

rko_lio::driver::PointCloudView view_of(const lidarslam_core::PackedCloud & cloud)
{
  rko_lio::driver::PointCloudView view;
  view.height = cloud.height;
  view.width = cloud.width;
  view.point_step = cloud.point_step;
  view.fields.reserve(cloud.fields.size());
  for (const auto & field : cloud.fields) {
    view.fields.push_back({field.name, field.offset, field.datatype, field.count});
  }
  view.data = cloud.data.data();
  view.data_size = cloud.data.size();
  return view;
}

struct PoseRecord
{
  int64_t time_ns;
  std::array<double, 7> pose;  // x, y, z, qx, qy, qz, qw
};

class OdometryDriver
{
public:
  OdometryDriver(py::dict params, std::size_t max_lidar_buffer_size)
  {
    DictParameters parameters(params);
    auto config = rko_lio::driver::declare_driver_config(parameters);
    unused_ = parameters.unused();
    driver_ = std::make_unique<rko_lio::driver::OfflineDriver>(
      std::move(config),
      [this](const rko_lio::driver::OdometryOutput & output) {on_output(output);},
      [](const std::string & m) {DictParameters::log("INFO", m);},
      [](const std::string & m) {DictParameters::log("WARN", m);},
      max_lidar_buffer_size);
  }

  void add_imu(const lidarslam_core::ImuSample & imu)
  {
    const auto & a = imu.linear_acceleration;
    const auto & w = imu.angular_velocity;
    driver_->add_imu(
      to_nsec(imu.header.stamp), Eigen::Vector3d(a[0], a[1], a[2]), Eigen::Vector3d(w[0], w[1], w[2]));
  }

  void add_lidar(const lidarslam_core::PackedCloud & cloud)
  {
    driver_->add_lidar(view_of(cloud), to_nsec(cloud.header.stamp));
  }

  py::list take_outputs()
  {
    py::list result;
    for (auto & output : outputs_) {
      py::dict item;
      item["time_ns"] = output.time_ns;
      item["pose"] = output.pose;
      item["velocity"] = output.velocity;
      item["angular_velocity"] = output.angular_velocity;
      item["pose_covariance"] = output.pose_covariance;
      if (!output.scan.empty()) {
        py::array_t<double> scan({static_cast<py::ssize_t>(output.scan.size()), py::ssize_t{3}});
        auto view = scan.mutable_unchecked<2>();
        for (std::size_t i = 0; i < output.scan.size(); ++i) {
          for (int k = 0; k < 3; ++k) {view(i, k) = output.scan[i](k);}
        }
        item["deskewed_scan"] = scan;
      }
      result.append(item);
    }
    outputs_.clear();
    return result;
  }

  void write_tum(const std::string & path) const {driver_->write_tum(path);}
  std::size_t pending_lidar_frames() const {return driver_->pending_lidar_frames();}
  std::vector<std::string> unused_parameters() const {return unused_;}
  std::string lidar_topic() const {return driver_->config().lidar_topic;}
  std::string imu_topic() const {return driver_->config().imu_topic;}

private:
  struct Output
  {
    int64_t time_ns;
    std::array<double, 7> pose;
    std::array<double, 3> velocity;
    std::array<double, 3> angular_velocity;
    std::array<double, 36> pose_covariance;
    rko_lio::core::Vector3dVector scan;
  };

  void on_output(const rko_lio::driver::OdometryOutput & output)
  {
    const auto & t = output.pose.translation();
    const auto & q = output.pose.unit_quaternion();
    outputs_.push_back(Output{
        output.time.count(),
        {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()},
        {output.velocity.x(), output.velocity.y(), output.velocity.z()},
        {output.angular_velocity.x(), output.angular_velocity.y(), output.angular_velocity.z()},
        output.pose_covariance,
        output.deskewed_scan});
  }

  std::unique_ptr<rko_lio::driver::OfflineDriver> driver_;
  std::vector<Output> outputs_;
  std::vector<std::string> unused_;
};

}  // namespace

void bind_frontend(py::module_ & m)
{
  py::class_<OdometryDriver>(m, "OdometryDriver",
    "RKO-LIO offline odometry. `params` uses the ROS 2 node's parameter names "
    "(imu_topic, lidar_topic, base_frame, extrinsic_*2base_quat_xyzw_xyz, voxel_size, ...).")
  .def(py::init<py::dict, std::size_t>(), py::arg("params"), py::arg("max_lidar_buffer_size") = 10)
  .def("add_imu", &OdometryDriver::add_imu, py::arg("imu"))
  .def("add_lidar", &OdometryDriver::add_lidar, py::arg("cloud"))
  .def("take_outputs", &OdometryDriver::take_outputs,
    "Odometry produced since the last call: dicts with time_ns, pose [x y z qx qy qz qw], "
    "velocity, angular_velocity, pose_covariance and, with publish_deskewed_scan, deskewed_scan.")
  .def("write_tum", &OdometryDriver::write_tum, py::arg("path"),
    "The trajectory file the ROS 2 node writes with dump_results (6 decimals).")
  .def_property_readonly("pending_lidar_frames", &OdometryDriver::pending_lidar_frames)
  .def_property_readonly("unused_parameters", &OdometryDriver::unused_parameters)
  .def_property_readonly("lidar_topic", &OdometryDriver::lidar_topic)
  .def_property_readonly("imu_topic", &OdometryDriver::imu_topic);
}
