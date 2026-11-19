// Copyright 2026 lidarslam_ros2 contributors. BSD-2-Clause.
//
// lidarslam._core.BackendPipeline: the offline backend of graph_slam_offline_runner
// (graphslam::offline::OfflinePipeline) with its parameters from a Python dict; and
// lidarslam._core.run_join_tool: graph_slam_multisession_join (multisession_join_tool.hpp) on logs
// read by a Python callable. Values keep the
// types of the ROS 2 parameter they stand for (bool, int, float, str, or a list of one of those),
// and each is read with the type rules of rclcpp, so a parameter file gives the same values here
// as in the ROS 2 runner.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "graph_based_slam/cloud_message.hpp"
#include "graph_based_slam/map_store.hpp"
#include "graph_based_slam/multisession_join_tool.hpp"
#include "graph_based_slam/offline_pipeline.hpp"
#include "graph_based_slam/offline_runtime.hpp"
#include "graph_based_slam/startup_relocalization.hpp"
#include "lidarslam_core/sensor_messages.hpp"
#include "python_map_changes.hpp"

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

std::vector<double> poseList(const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond q(pose.rotation());
  const Eigen::Vector3d t = pose.translation();
  return {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
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
  void addGnssFix(const lidarslam_core::NavSatFix & in)
  {
    graphslam::gnss::Fix fix;
    fix.header_stamp_sec = graphslam::cloud_msg::stampSeconds(in.header.stamp);
    fix.status = in.status;
    fix.latitude = in.latitude;
    fix.longitude = in.longitude;
    fix.altitude = in.altitude;
    fix.position_covariance = in.position_covariance;
    fix.position_covariance_type = in.position_covariance_type;
    pipeline_->addGnssFix(fix);
  }
  bool finish() {return pipeline_->finish();}
  void keepSessionMap(bool keep) {pipeline_->keepSessionMap(keep);}
  std::size_t writeSessionMap(const std::string & directory) const
  {
    const graphslam::map_store::SessionMap & session = pipeline_->sessionMap();
    std::string error;
    if (!graphslam::map_store::writeSession(session, directory, error)) {
      throw std::runtime_error(error);
    }
    return session.submaps.size();
  }

  // listener(dict): stamp_sec, odometry and map ([x, y, z, qx, qy, qz, qw]), new_submap, submaps,
  // corrections_applied; called during add_odometry / add_cloud (causal_trajectory_output).
  void setLivePoseListener(py::object listener)
  {
    if (listener.is_none()) {
      pipeline_->setLivePoseListener(nullptr);
      return;
    }
    pipeline_->setLivePoseListener(
      [listener](const graphslam::offline::LivePose & live) {
        py::dict out;
        out["stamp_sec"] = live.stamp_sec;
        out["odometry"] = poseList(live.odometry);
        out["map"] = poseList(live.map);
        out["new_submap"] = live.new_submap;
        out["submaps"] = live.submaps;
        out["corrections_applied"] = live.corrections_applied;
        listener(out);
      });
  }

  const graphslam::offline::OfflinePipeline & pipeline() const {return *pipeline_;}

private:
  graphslam::offline::Parameters parameters_;
  PythonLog log_;
  std::unique_ptr<graphslam::offline::OfflinePipeline> pipeline_;
};

// The sessions' logs through read(path, topics), which yields (topic, message) of those topics in
// the log's order (core messages: Odometry, PackedCloud, NavSatFix).
class PythonSessionReader : public graphslam::multisession::SessionLogReader
{
public:
  explicit PythonSessionReader(py::object read)
  : read_(std::move(read)) {}

  void readOdometryAndClouds(
    const std::string & path, const std::string & odometry_topic, const std::string & cloud_topic,
    const std::function<void(graphslam::msg::Odometry &&)> & odometry,
    const std::function<void(graphslam::msg::PointCloud2 &&)> & cloud) override
  {
    const std::vector<std::string> topics {odometry_topic, cloud_topic};
    for (const py::handle item : py::iter(read_(path, topics))) {
      const py::tuple record = py::reinterpret_borrow<py::tuple>(item);
      const std::string topic = record[0].cast<std::string>();
      if (topic == odometry_topic) {
        odometry(toOdometry(record[1].cast<const lidarslam_core::Odometry &>()));
      } else if (topic == cloud_topic) {
        cloud(toCloud(record[1].cast<const lidarslam_core::PackedCloud &>()));
      }
    }
  }

  void readNavSatFixes(
    const std::string & path, const std::string & topic,
    const std::function<void(const graphslam::multisession::NavSatFixMessage &)> & fix) override
  {
    for (const py::handle item : py::iter(read_(path, std::vector<std::string>{topic}))) {
      const py::tuple record = py::reinterpret_borrow<py::tuple>(item);
      if (record[0].cast<std::string>() != topic) {
        continue;
      }
      const auto & in = record[1].cast<const lidarslam_core::NavSatFix &>();
      graphslam::multisession::NavSatFixMessage out;
      out.stamp.sec = in.header.stamp.sec;
      out.stamp.nanosec = in.header.stamp.nanosec;
      out.status = in.status;
      out.latitude = in.latitude;
      out.longitude = in.longitude;
      out.altitude = in.altitude;
      out.position_covariance = in.position_covariance;
      fix(out);
    }
  }

private:
  py::object read_;
};

Eigen::Isometry3d rigidTransform(const Eigen::Matrix4d & matrix)
{
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  if (!matrix.allFinite() ||
    !matrix.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1)) ||
    !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
    std::abs(rotation.determinant() - 1) > 1e-6)
  {
    throw std::invalid_argument("expected a finite rigid 4x4 transform");
  }
  return Eigen::Isometry3d(matrix);
}

class StoreRelocalizer
{
public:
  using Scan = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
  using Windows = std::map<std::size_t, Scan>;

  StoreRelocalizer(
    const std::vector<std::string> & directories,
    const std::vector<Eigen::Matrix4d> & transforms, const std::vector<Windows> & windows,
    const std::vector<std::vector<Eigen::Matrix4d>> & optimized_poses)
  : matcher_(load(directories, transforms, windows, optimized_poses, session_poses_)) {}

  const std::vector<std::vector<Eigen::Matrix4d>> & sessionPoses() const {return session_poses_;}

  static Scan thin(const Eigen::Ref<const Scan> & points)
  {
    std::vector<Eigen::Vector3f> finite;
    finite.reserve(points.rows());
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
      if (points.row(i).allFinite()) {finite.emplace_back(points.row(i).transpose());}
    }
    const auto thinned = small_gicp::voxelgrid_sampling<
      std::vector<Eigen::Vector3f>, small_gicp::PointCloud>(finite, 0.2);
    Scan result(thinned->size(), 3);
    for (std::size_t i = 0; i < thinned->size(); ++i) {
      result.row(i) = thinned->point(i).head<3>().cast<float>().transpose();
    }
    return result;
  }

  py::object match(const Eigen::Ref<const Eigen::MatrixX3f> & points,
    const Eigen::Matrix4d & prior)
  {
    const auto result = registration(points, prior);
    if (!result) {return py::none();}
    return py::cast(Eigen::Matrix4d(result->pose.matrix()));
  }

  py::object matchWithReference(const Eigen::Ref<const Eigen::MatrixX3f> & points,
    const Eigen::Matrix4d & prior)
  {
    const auto result = registration(points, prior);
    if (!result) {return py::none();}
    return py::make_tuple(result->reference, Eigen::Matrix4d(result->pose.matrix()));
  }

  py::object matchNearLaunch(const Eigen::Ref<const Eigen::MatrixX3f> & points)
  {
    const auto result = matcher_.matchNearLaunch(keyframe(points));
    if (!result) {return py::none();}
    return py::make_tuple(result->reference, Eigen::Matrix4d(result->pose.matrix()));
  }

  // An absent reference denotes a registration to the joint shared-surface map.
  using Reference = std::optional<std::size_t>;
  using Observation = std::tuple<std::size_t, Reference, Eigen::Matrix4d>;

  std::pair<std::vector<Eigen::Matrix4d>, std::string> optimizeTracking(
    const std::vector<Eigen::Matrix4d> & local_poses,
    const Eigen::Matrix4d & initial_transform, const std::vector<Observation> & observations) const
  {
    namespace graph = graphslam::pose_graph;
    if (local_poses.empty() || observations.empty()) {
      throw std::invalid_argument("tracking needs poses and at least one reference observation");
    }
    const auto initial = rigidTransform(initial_transform);
    std::map<Reference, int> references;
    std::vector<graph::SubmapNode> nodes;
    // Use a single fixed store-frame vertex for all registrations to the joint map.
    if (std::any_of(observations.begin(), observations.end(), [](const auto & observation) {
        return !std::get<1>(observation);
      }))
    {
      graph::SubmapNode origin;
      origin.session_start = true;
      nodes.push_back(origin);
      references.emplace(std::nullopt, 0);
    }
    for (const auto & observation : observations) {
      const auto reference = std::get<1>(observation);
      if (references.emplace(reference, static_cast<int>(nodes.size())).second) {
        graph::SubmapNode node;
        node.session_start = true;
        node.pose = matcher_.referencePose(reference.value());
        nodes.push_back(node);
      }
    }
    const auto fixed = nodes.size();
    for (std::size_t i = 0; i < local_poses.size(); ++i) {
      graph::SubmapNode node;
      node.pose = initial * rigidTransform(local_poses[i]);
      node.session_start = (i == 0);
      nodes.push_back(node);
    }
    std::vector<graph::LoopConstraint> edges;
    for (const auto & [index, reference, pose] : observations) {
      if (index >= local_poses.size()) {
        throw std::invalid_argument("tracking observation refers to a missing pose");
      }
      graph::LoopConstraint edge;
      edge.from = references.at(reference);
      edge.to = static_cast<int>(fixed + index);
      edge.relative_pose = nodes[edge.from].pose.inverse() * rigidTransform(pose);
      edges.push_back(edge);
    }
    graph::AdjacentEdgeConfig adjacent;
    adjacent.num_adjacent_pose_constraints = 3;
    graph::LoopEdgeConfig loop;
    loop.robust_kernel_type = "Huber";
    loop.constant_info_trans = 2000.;
    loop.constant_info_rot = 2000.;
    const auto optimized = graph::optimizePoseGraph(
      nodes, edges, {}, {}, adjacent, loop, {}, graph::Chi2Collection::NONE, true, 10, {}, fixed);
    std::vector<Eigen::Matrix4d> result;
    for (std::size_t i = fixed; i < optimized.poses.size(); ++i) {
      result.push_back(optimized.poses[i].matrix());
    }
    return {std::move(result), optimized.pose_graph_g2o};
  }

private:
  std::optional<graphslam::multisession::StartupRelocalizer::Match> registration(
    const Eigen::Ref<const Eigen::MatrixX3f> & points, const Eigen::Matrix4d & prior)
  {
    return matcher_.match(keyframe(points), rigidTransform(prior));
  }

  static graphslam::multisession::Keyframe keyframe(
    const Eigen::Ref<const Eigen::MatrixX3f> & points)
  {
    graphslam::multisession::Keyframe query;
    query.points.reserve(points.rows());
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
      if (points.row(i).allFinite()) {
        query.points.emplace_back(points.row(i).transpose());
      }
    }
    return query;
  }

  static std::vector<graphslam::multisession::Keyframe> load(
    const std::vector<std::string> & directories,
    const std::vector<Eigen::Matrix4d> & transforms,
    const std::vector<Windows> & windows,
    const std::vector<std::vector<Eigen::Matrix4d>> & optimized_poses,
    std::vector<std::vector<Eigen::Matrix4d>> & session_poses)
  {
    if (directories.size() != transforms.size() || directories.size() != windows.size() ||
      (!optimized_poses.empty() && directories.size() != optimized_poses.size()))
    {
      throw std::invalid_argument("one transform and window index are required per session");
    }
    std::vector<graphslam::multisession::Keyframe> frames;
    session_poses.resize(directories.size());
    for (std::size_t i = 0; i < directories.size(); ++i) {
      const auto store_from_session = rigidTransform(transforms[i]);
      graphslam::map_store::SessionMap session;
      std::string error;
      if (!graphslam::map_store::readSession(directories[i], session, error)) {
        throw std::runtime_error(error);
      }
      const bool tracked = !optimized_poses.empty() && !optimized_poses[i].empty();
      if (tracked && optimized_poses[i].size() != session.submaps.size()) {
        throw std::invalid_argument("tracking poses do not match the stored submaps");
      }
      session_poses[i].reserve(session.submaps.size());
      for (std::size_t j = 0; j < session.submaps.size(); ++j) {
        const auto & submap = session.submaps[j];
        const auto pose = tracked ? rigidTransform(optimized_poses[i][j]) :
          store_from_session * submap.pose;
        session_poses[i].push_back(pose.matrix());
        graphslam::multisession::Keyframe frame;
        frame.frame = graphslam::multisession::gravityAlignedFrame(pose);
        const auto local = frame.frame.inverse() * pose;
        const auto append = [&](const Eigen::Vector3d & point) {
          const Eigen::Vector3d xyz = local * point;
          if (xyz.squaredNorm() < 1600.) {
            frame.points.push_back(xyz.cast<float>());
          }
        };
        if (const auto window = windows[i].find(j); window != windows[i].end()) {
          frame.points.reserve(window->second.rows());
          for (Eigen::Index row = 0; row < window->second.rows(); ++row) {
            append(window->second.row(row).transpose().cast<double>());
          }
        } else {
          const auto cloud = graphslam::map_store::submapCloud(submap);
          frame.points.reserve(cloud->size());
          for (const auto & point : *cloud) {
            append(point.getVector3fMap().cast<double>());
          }
        }
        frames.push_back(std::move(frame));
      }
    }
    return frames;
  }

  std::vector<std::vector<Eigen::Matrix4d>> session_poses_;
  graphslam::multisession::StartupRelocalizer matcher_;
};

// The Autoware map bundle of stored sessions (map_store::exportBundle) with the map-saving
// parameters of params. Returns the number of submaps.
std::size_t exportStoreBundle(
  const std::vector<std::string> & session_dirs, const py::dict & params,
  const std::string & out_dir, py::object log,
  const std::vector<Eigen::Matrix4d> & transforms,
  const std::vector<std::vector<Eigen::Matrix4d>> & optimized_poses,
  const graphslam::map_store::RemovedVoxels & removed)
{
  if (!transforms.empty() && transforms.size() != session_dirs.size()) {
    throw std::invalid_argument("one transform is required per exported session");
  }
  if (!optimized_poses.empty() && optimized_poses.size() != session_dirs.size()) {
    throw std::invalid_argument("one pose list is required per exported session");
  }
  const graphslam::offline::Parameters parameters = toParameters(params);
  const graphslam::map_export::MapExportConfig config =
    graphslam::offline::mapExportConfig(parameters);
  std::vector<graphslam::map_store::SessionMap> sessions(session_dirs.size());
  std::size_t submaps = 0U;
  for (std::size_t i = 0; i < session_dirs.size(); ++i) {
    std::string error;
    if (!graphslam::map_store::readSession(session_dirs[i], sessions[i], error)) {
      throw std::runtime_error(error);
    }
    if (!optimized_poses.empty() && !optimized_poses[i].empty()) {
      if (optimized_poses[i].size() != sessions[i].submaps.size()) {
        throw std::invalid_argument("tracking poses do not match the stored submaps");
      }
      for (std::size_t j = 0; j < sessions[i].submaps.size(); ++j) {
        sessions[i].submaps[j].pose = rigidTransform(optimized_poses[i][j]);
      }
    } else if (!transforms.empty() && !transforms[i].isIdentity()) {
      const auto transform = rigidTransform(transforms[i]);
      for (auto & submap : sessions[i].submaps) {
        submap.pose = transform * submap.pose;
      }
    }
    submaps += sessions[i].submaps.size();
  }
  const PythonLog python_log(std::move(log));
  graphslam::map_export::MapExportLog export_log;
  export_log.line = [&python_log](const std::string & line) {python_log.info(line);};
  export_log.info = export_log.line;
  graphslam::map_store::exportBundle(sessions, config, out_dir, export_log, removed);
  return submaps;
}

int runJoinTool(const py::dict & params, py::object read, py::object log)
{
  const graphslam::offline::Parameters parameters = toParameters(params);
  const PythonLog python_log(std::move(log));
  PythonSessionReader reader(std::move(read));
  return graphslam::multisession::runJoinTool(parameters, python_log, reader);
}

}  // namespace

void bind_backend(py::module_ & m)
{
  using Evidence = lidarslam_core::MapChangeEvidence;
  py::class_<Evidence>(m, "MapChangeEvidence")
  .def(py::init<const std::vector<std::string> &, const Evidence::Poses &>())
  .def("observe", &Evidence::observe)
  .def("snapshot", &Evidence::snapshot);
  using Refined = lidarslam_core::RefinedNewPointEvidence;
  py::class_<Refined>(m, "RefinedNewPointEvidence")
  .def(py::init<>())
  .def("observe", &Refined::observe)
  .def("suppression_mask", &Refined::suppressionMask);
  using NewPoints = lidarslam_core::NewPointEvidence;
  py::class_<NewPoints>(m, "NewPointEvidence")
  .def(py::init<>())
  .def("observe", &NewPoints::observe)
  .def("suppression_mask", &NewPoints::suppressionMask);
  py::class_<StoreRelocalizer>(m, "StoreRelocalizer")
  .def(py::init<const std::vector<std::string> &, const std::vector<Eigen::Matrix4d> &,
    const std::vector<StoreRelocalizer::Windows> &,
    const std::vector<std::vector<Eigen::Matrix4d>> &>(), py::arg("directories"),
    py::arg("transforms"), py::arg("windows"),
    py::arg("optimized_poses") = std::vector<std::vector<Eigen::Matrix4d>>{})
  .def("match", &StoreRelocalizer::match, py::arg("points"), py::arg("prior"))
  .def("match_with_reference", &StoreRelocalizer::matchWithReference)
  .def("match_near_launch", &StoreRelocalizer::matchNearLaunch)
  .def("session_poses", &StoreRelocalizer::sessionPoses)
  .def("optimize_tracking", &StoreRelocalizer::optimizeTracking)
  .def_static("thin", &StoreRelocalizer::thin);
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
  .def_property_readonly("uses_gnss", [](const BackendPipeline & b) {
      return b.pipeline().usesGnss();
    })
  .def_property_readonly("gnss_topic", [](const BackendPipeline & b) {
      return b.pipeline().gnssTopic();
    })
  .def("begin", &BackendPipeline::begin)
  .def("add_odometry", &BackendPipeline::addOdometry, py::arg("odometry"))
  .def("add_cloud", &BackendPipeline::addCloud, py::arg("cloud"))
  .def("add_gnss_fix", &BackendPipeline::addGnssFix, py::arg("fix"),
    "With use_gnss: a NavSatFix of gnss_topic, in recorded order with the odometry and clouds.")
  .def("set_live_pose_listener", &BackendPipeline::setLivePoseListener, py::arg("listener"),
    "Call listener(dict) for every odometry / cloud pair with its live (causal) pose: stamp_sec, "
    "odometry and map as [x, y, z, qx, qy, qz, qw], new_submap, submaps, corrections_applied. "
    "Needs causal_trajectory_output; set it before begin().")
  .def("finish", &BackendPipeline::finish,
    "The pose graph, refinement and all outputs; False (after logging why) when the fixed loop "
    "edges cannot be loaded.")
  .def("keep_session_map", &BackendPipeline::keepSessionMap, py::arg("keep") = true,
    "Keep the session's submaps for write_session_map (a long-term map store session); set it "
    "before begin(). The outputs are the same either way.")
  .def("write_session_map", &BackendPipeline::writeSessionMap, py::arg("directory"),
    "After finish() with keep_session_map: writes the submaps at their optimized poses "
    "(submaps.bin, pose_graph.g2o) to directory; returns the number of submaps.");
  m.def(
    "export_store_bundle", &exportStoreBundle, py::arg("session_dirs"), py::arg("params"),
    py::arg("out_dir"), py::arg("log") = py::none(),
    py::arg("transforms") = std::vector<Eigen::Matrix4d>{},
    py::arg("optimized_poses") = std::vector<std::vector<Eigen::Matrix4d>>{},
    py::arg("removed") = graphslam::map_store::RemovedVoxels{},
    "Writes the Autoware map bundle of the stored sessions (directories written by "
    "write_session_map) with the map-saving parameters of params to out_dir; returns the number "
    "of submaps. For one session it is the bundle the backend wrote.");
  m.def(
    "run_join_tool", &runJoinTool, py::arg("params"), py::arg("read"), py::arg("log") = py::none(),
    "graph_slam_multisession_join with its parameters (node graph_based_slam); read(path, topics) "
    "yields (topic, message) of a session log in its order. Returns the executable's exit code.");
}
