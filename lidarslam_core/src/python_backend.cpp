// Copyright 2026 lidarslam_ros2 contributors. BSD-2-Clause.
//
// lidarslam._core.BackendPipeline: the offline backend of graph_slam_offline_runner
// (graphslam::offline::OfflinePipeline) with its parameters from a Python dict. Values keep the
// types of the ROS 2 parameter they stand for (bool, int, float, str, or a list of one of those),
// and each is read with the type rules of rclcpp, so a parameter file gives the same values here
// as in the ROS 2 runner.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/offline_pipeline.hpp"
#include "graph_based_slam/offline_runtime.hpp"
#include "lidarslam_core/sensor_messages.hpp"

namespace py = pybind11;

namespace
{

using graphslam::offline::ParameterValue;

[[noreturn]] void unsupported(const std::string & name, const py::handle & value)
{
  throw py::type_error(
          "parameter '" + name + "': unsupported value " + py::repr(value).cast<std::string>() +
          " (expected bool, int, float, str or a list of one of these)");
}

// One dict value as the ROS 2 parameter type a parameter file gives it.
ParameterValue toParameterValue(const std::string & name, const py::handle & value)
{
  if (py::isinstance<py::bool_>(value)) {
    return value.cast<bool>();
  }
  if (py::isinstance<py::int_>(value)) {
    return value.cast<std::int64_t>();
  }
  if (py::isinstance<py::float_>(value)) {
    return value.cast<double>();
  }
  if (py::isinstance<py::str>(value)) {
    return value.cast<std::string>();
  }
  if (py::isinstance<py::bytes>(value)) {
    const std::string bytes = value.cast<std::string>();
    return std::vector<std::uint8_t>(bytes.begin(), bytes.end());
  }
  if (py::isinstance<py::list>(value) || py::isinstance<py::tuple>(value)) {
    const py::sequence items = py::reinterpret_borrow<py::sequence>(value);
    if (items.size() == 0) {
      // rcl's parameter file parser rejects an empty list; there is no type to give it.
      throw py::value_error("parameter '" + name + "': an empty list has no type");
    }
    const py::handle first = items[0];
    const auto all = [&items](auto predicate) {
        for (const py::handle item : items) {
          if (!predicate(item)) {
            return false;
          }
        }
        return true;
      };
    if (py::isinstance<py::bool_>(first)) {
      if (!all([](py::handle h) {return py::isinstance<py::bool_>(h);})) {unsupported(name, value);}
      return items.cast<std::vector<bool>>();
    }
    if (py::isinstance<py::int_>(first)) {
      if (!all([](py::handle h) {
          return py::isinstance<py::int_>(h) && !py::isinstance<py::bool_>(h);
        }))
      {
        unsupported(name, value);
      }
      return items.cast<std::vector<std::int64_t>>();
    }
    if (py::isinstance<py::float_>(first)) {
      if (!all([](py::handle h) {return py::isinstance<py::float_>(h);})) {unsupported(name, value);}
      return items.cast<std::vector<double>>();
    }
    if (py::isinstance<py::str>(first)) {
      if (!all([](py::handle h) {return py::isinstance<py::str>(h);})) {unsupported(name, value);}
      return items.cast<std::vector<std::string>>();
    }
  }
  unsupported(name, value);
}

graphslam::offline::Parameters toParameters(const py::dict & values)
{
  graphslam::offline::Parameters parameters;
  for (const auto & item : values) {
    const std::string name = py::str(item.first);
    if (item.second.is_none()) {
      continue;  // an empty YAML value: the parameter is not set
    }
    parameters.set(name, toParameterValue(name, item.second));
  }
  return parameters;
}

// Messages to a Python callable(level, message), or to stderr when none is given.
class PythonLog : public graphslam::offline::Log
{
public:
  explicit PythonLog(py::object callback)
  : callback_(std::move(callback)) {}
  void info(const std::string & message) const override {write("INFO", message);}
  void warn(const std::string & message) const override {write("WARN", message);}
  void error(const std::string & message) const override {write("ERROR", message);}

private:
  void write(const char * level, const std::string & message) const
  {
    if (callback_.is_none()) {
      py::print(
        std::string("[") + level + "] [graph_based_slam]: " + message,
        py::arg("file") = py::module_::import("sys").attr("stderr"));
    } else {
      callback_(level, message);
    }
  }
  py::object callback_;
};

graphslam::msg::Header toHeader(const lidarslam_core::Header & in)
{
  graphslam::msg::Header out;
  out.stamp.sec = in.stamp.sec;
  out.stamp.nanosec = in.stamp.nanosec;
  out.frame_id = in.frame_id;
  return out;
}

graphslam::msg::Odometry toOdometry(const lidarslam_core::Odometry & in)
{
  graphslam::msg::Odometry out;
  out.header = toHeader(in.header);
  out.child_frame_id = in.child_frame_id;
  out.pose.pose.position.x = in.position[0];
  out.pose.pose.position.y = in.position[1];
  out.pose.pose.position.z = in.position[2];
  out.pose.pose.orientation.x = in.orientation[0];
  out.pose.pose.orientation.y = in.orientation[1];
  out.pose.pose.orientation.z = in.orientation[2];
  out.pose.pose.orientation.w = in.orientation[3];
  out.pose.covariance = in.pose_covariance;
  out.twist.twist.linear.x = in.linear_velocity[0];
  out.twist.twist.linear.y = in.linear_velocity[1];
  out.twist.twist.linear.z = in.linear_velocity[2];
  out.twist.twist.angular.x = in.angular_velocity[0];
  out.twist.twist.angular.y = in.angular_velocity[1];
  out.twist.twist.angular.z = in.angular_velocity[2];
  out.twist.covariance = in.twist_covariance;
  return out;
}

graphslam::msg::PointCloud2 toCloud(const lidarslam_core::PackedCloud & in)
{
  graphslam::msg::PointCloud2 out;
  out.header = toHeader(in.header);
  out.height = in.height;
  out.width = in.width;
  out.fields.reserve(in.fields.size());
  for (const auto & field : in.fields) {
    out.fields.push_back({field.name, field.offset, field.datatype, field.count});
  }
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.row_step = in.row_step;
  out.data = in.data;
  out.is_dense = in.is_dense;
  return out;
}

class BackendPipeline
{
public:
  BackendPipeline(const py::dict & params, py::object log)
  : parameters_(toParameters(params)), log_(std::move(log)),
    pipeline_(std::make_unique<graphslam::offline::OfflinePipeline>(parameters_, log_)) {}

  bool configure()
  {
    try {
      return pipeline_->configure();
    } catch (const graphslam::offline::ParameterTypeError & error) {
      throw py::type_error(error.what());
    }
  }

  void begin() {pipeline_->begin();}
  void addOdometry(const lidarslam_core::Odometry & odometry)
  {
    pipeline_->addOdometry(toOdometry(odometry));
  }
  void addCloud(const lidarslam_core::PackedCloud & cloud) {pipeline_->addCloud(toCloud(cloud));}
  bool finish() {return pipeline_->finish();}

  const graphslam::offline::OfflinePipeline & pipeline() const {return *pipeline_;}

private:
  graphslam::offline::Parameters parameters_;
  PythonLog log_;
  std::unique_ptr<graphslam::offline::OfflinePipeline> pipeline_;
};

}  // namespace

void bind_backend(py::module_ & m)
{
  py::class_<BackendPipeline>(
    m, "BackendPipeline",
    "The offline backend of graph_slam_offline_runner: configure(), begin(), the odometry and "
    "clouds in recorded order, finish().")
  .def(py::init<py::dict, py::object>(), py::arg("params"), py::arg("log") = py::none())
  .def("configure", &BackendPipeline::configure,
    "Reads the parameters and activates the loop registration; False (after logging why) when "
    "that fails.")
  .def_property_readonly("input_path", [](const BackendPipeline & b) {
      return b.pipeline().inputPath();
    })
  .def_property_readonly("odometry_topic", [](const BackendPipeline & b) {
      return b.pipeline().odometryTopic();
    })
  .def_property_readonly("cloud_topic", [](const BackendPipeline & b) {
      return b.pipeline().cloudTopic();
    })
  .def_property_readonly("output_dir", [](const BackendPipeline & b) {
      return b.pipeline().outputDir();
    })
  .def("begin", &BackendPipeline::begin)
  .def("add_odometry", &BackendPipeline::addOdometry, py::arg("odometry"))
  .def("add_cloud", &BackendPipeline::addCloud, py::arg("cloud"))
  .def("finish", &BackendPipeline::finish,
    "The pose graph, refinement and all outputs; False (after logging why) when the fixed loop "
    "edges cannot be loaded.");
}
