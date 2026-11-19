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
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
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

#include "graph_based_slam/map_store.hpp"
#include "graph_based_slam/map_change_evidence.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "graph_based_slam/pointcloud2_conversion.hpp"
#include "filesystem_io_ports.hpp"

namespace graphslam
{
namespace map_store
{
namespace
{

constexpr char kMagic[8] = {'L', 'S', 'S', 'U', 'B', 'M', 'A', 'P'};
constexpr std::uint32_t kVersion = 1U;

class Writer
{
public:
  template<typename T>
  void put(const T & value)
  {
    const auto * bytes = reinterpret_cast<const char *>(&value);
    out_.append(bytes, sizeof(T));
  }
  void putBytes(const void * data, std::size_t size)
  {
    put(static_cast<std::uint64_t>(size));
    out_.append(static_cast<const char *>(data), size);
  }
  void putPose(const Eigen::Isometry3d & pose)
  {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 4; ++c) {
        put(pose.matrix()(r, c));
      }
    }
  }
  std::string & bytes() {return out_;}

private:
  std::string out_;
};

class Reader
{
public:
  explicit Reader(std::string bytes)
  : in_(std::move(bytes)) {}

  template<typename T>
  T get()
  {
    need(sizeof(T));
    T value;
    std::memcpy(&value, in_.data() + at_, sizeof(T));
    at_ += sizeof(T);
    return value;
  }
  std::string getString()
  {
    const auto size = static_cast<std::size_t>(get<std::uint64_t>());
    need(size);
    std::string value = in_.substr(at_, size);
    at_ += size;
    return value;
  }
  std::vector<std::uint8_t> getBytes()
  {
    const auto size = static_cast<std::size_t>(get<std::uint64_t>());
    need(size);
    const auto * begin = reinterpret_cast<const std::uint8_t *>(in_.data() + at_);
    at_ += size;
    return std::vector<std::uint8_t>(begin, begin + size);
  }
  Eigen::Isometry3d getPose()
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 4; ++c) {
        pose.matrix()(r, c) = get<double>();
      }
    }
    return pose;
  }
  bool done() const {return at_ == in_.size();}

private:
  void need(std::size_t size) const
  {
    if (size > in_.size() - at_) {
      throw std::runtime_error("submaps.bin ends early");
    }
  }
  std::string in_;
  std::size_t at_ {0U};
};

bool writeFile(const std::string & path, const std::string & bytes, std::string & error)
{
  const std::string temporary = path + ".tmp";
  {
    std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (!out) {
      error = "cannot write " + temporary;
      return false;
    }
  }
  std::error_code code;
  std::filesystem::rename(temporary, path, code);
  if (code) {
    error = "cannot rename " + temporary + ": " + code.message();
    return false;
  }
  return true;
}

}  // namespace

bool writeSession(const SessionMap & session, const std::string & directory, std::string & error)
{
  std::error_code code;
  std::filesystem::create_directories(directory, code);
  if (code) {
    error = "cannot create " + directory + ": " + code.message();
    return false;
  }
  Writer w;
  w.bytes().append(kMagic, sizeof(kMagic));
  w.put(kVersion);
  w.put(static_cast<std::uint8_t>(session.origin.set ? 1 : 0));
  w.put(session.origin.latitude);
  w.put(session.origin.longitude);
  w.put(static_cast<std::uint64_t>(session.loop_edge_count));
  w.put(static_cast<std::uint64_t>(session.submaps.size()));
  for (const StoredSubmap & submap : session.submaps) {
    if (!submap.cloud) {
      error = "a submap has no cloud";
      return false;
    }
    const msg::PointCloud2 & m = submap.cloud->message;
    w.put(submap.stamp_sec);
    w.putPose(submap.odometry_pose);
    w.putPose(submap.pose);
    w.put(m.header.stamp.sec);
    w.put(m.header.stamp.nanosec);
    w.putBytes(m.header.frame_id.data(), m.header.frame_id.size());
    w.put(m.height);
    w.put(m.width);
    w.put(static_cast<std::uint32_t>(m.fields.size()));
    for (const msg::PointField & field : m.fields) {
      w.putBytes(field.name.data(), field.name.size());
      w.put(field.offset);
      w.put(field.datatype);
      w.put(field.count);
    }
    w.put(static_cast<std::uint8_t>(m.is_bigendian ? 1 : 0));
    w.put(m.point_step);
    w.put(m.row_step);
    w.put(static_cast<std::uint8_t>(m.is_dense ? 1 : 0));
    w.put(submap.cloud->quantization_step);
    w.putBytes(submap.cloud->compressed.data(), submap.cloud->compressed.size());
    w.putBytes(m.data.data(), m.data.size());
  }
  if (!writeFile(directory + "/submaps.bin", w.bytes(), error)) {
    return false;
  }
  const std::string g2o = directory + "/pose_graph.g2o";
  if (!session.pose_graph_g2o.empty()) {
    return writeFile(g2o, session.pose_graph_g2o, error);
  }
  std::filesystem::remove(g2o, code);
  return true;
}

bool readSession(const std::string & directory, SessionMap & session, std::string & error)
{
  session = SessionMap();
  const std::string path = directory + "/submaps.bin";
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "cannot read " + path;
    return false;
  }
  std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  try {
    if (bytes.size() < sizeof(kMagic) || std::memcmp(bytes.data(), kMagic, sizeof(kMagic)) != 0) {
      throw std::runtime_error("not a submaps.bin");
    }
    Reader r(bytes.substr(sizeof(kMagic)));
    const auto version = r.get<std::uint32_t>();
    if (version != kVersion) {
      throw std::runtime_error("unsupported version " + std::to_string(version));
    }
    session.origin.set = r.get<std::uint8_t>() != 0U;
    session.origin.latitude = r.get<double>();
    session.origin.longitude = r.get<double>();
    session.loop_edge_count = static_cast<std::size_t>(r.get<std::uint64_t>());
    const auto count = static_cast<std::size_t>(r.get<std::uint64_t>());
    session.submaps.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      StoredSubmap submap;
      auto cloud = std::make_shared<StoredCloud>();
      msg::PointCloud2 & m = cloud->message;
      submap.stamp_sec = r.get<double>();
      submap.odometry_pose = r.getPose();
      submap.pose = r.getPose();
      m.header.stamp.sec = r.get<std::int32_t>();
      m.header.stamp.nanosec = r.get<std::uint32_t>();
      m.header.frame_id = r.getString();
      m.height = r.get<std::uint32_t>();
      m.width = r.get<std::uint32_t>();
      m.fields.resize(r.get<std::uint32_t>());
      for (msg::PointField & field : m.fields) {
        field.name = r.getString();
        field.offset = r.get<std::uint32_t>();
        field.datatype = r.get<std::uint8_t>();
        field.count = r.get<std::uint32_t>();
      }
      m.is_bigendian = r.get<std::uint8_t>() != 0U;
      m.point_step = r.get<std::uint32_t>();
      m.row_step = r.get<std::uint32_t>();
      m.is_dense = r.get<std::uint8_t>() != 0U;
      cloud->quantization_step = r.get<double>();
      cloud->compressed = r.getBytes();
      m.data = r.getBytes();
      submap.cloud = std::move(cloud);
      session.submaps.push_back(std::move(submap));
    }
    if (!r.done()) {
      throw std::runtime_error("trailing bytes");
    }
  } catch (const std::exception & e) {
    error = path + ": " + e.what();
    return false;
  }
  std::ifstream g2o(directory + "/pose_graph.g2o", std::ios::binary);
  if (g2o) {
    session.pose_graph_g2o.assign(
      (std::istreambuf_iterator<char>(g2o)), std::istreambuf_iterator<char>());
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr submapCloud(const StoredSubmap & submap)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pointcloud2_conversion::fromRosMsgPointXYZI(cloud_compression::restore(*submap.cloud), *cloud);
  return cloud;
}

void exportBundle(
  const std::vector<SessionMap> & sessions, const map_export::MapExportConfig & config,
  const std::string & save_dir, const map_export::MapExportLog & log)
{
  exportBundle(sessions, config, save_dir, log, {});
}

void exportBundle(
  const std::vector<SessionMap> & sessions, const map_export::MapExportConfig & config,
  const std::string & save_dir, const map_export::MapExportLog & log,
  const RemovedVoxels & removed, const StoreFrameEdits & edits)
{
  if (!removed.empty() && removed.size() != sessions.size()) {
    throw std::invalid_argument("expected one removal mask per exported session");
  }
  std::vector<const StoredSubmap *> submaps;
  std::vector<const std::set<std::array<int, 3>> *> masks;
  std::vector<Eigen::Isometry3d> poses;
  std::size_t map_points = 0U;
  std::size_t loop_edges = 0U;
  for (std::size_t i = 0; i < sessions.size(); ++i) {
    const auto & session = sessions[i];
    loop_edges += session.loop_edge_count;
    for (std::size_t j = 0; j < session.submaps.size(); ++j) {
      const auto & submap = session.submaps[j];
      const std::set<std::array<int, 3>> * mask = nullptr;
      if (!removed.empty()) {
        const auto found = removed[i].find(j);
        if (found != removed[i].end() && !found->second.empty()) {
          mask = &found->second;
        }
      }
      masks.push_back(mask);
      submaps.push_back(&submap);
      poses.push_back(submap.pose);
      map_points += static_cast<std::size_t>(submap.cloud->message.width) *
        submap.cloud->message.height;
    }
  }
  const map_export::SubmapCloudLoader load = [&submaps, &masks, &poses, &edits](std::size_t i) {
      auto cloud = submapCloud(*submaps[i]);
      if (!edits.empty()) {
        const auto * mask = masks[i];
        const Eigen::Isometry3d & pose = poses[i];
        cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
          [mask, &pose, &edits](const auto & point) {
            const PointState state = pointState(point, mask, pose, edits);
            return state == PointState::removed_by_evidence ||
                   state == PointState::removed_by_edit;
            }), cloud->points.end());
        cloud->width = cloud->points.size();
        cloud->height = 1U;
      } else if (const auto * mask = masks[i]) {
        cloud->points.erase(std::remove_if(cloud->points.begin(), cloud->points.end(),
          [mask](const auto & point) {
            const map_change::Point xyz{point.x, point.y, point.z};
            return map_change::usable(xyz) && mask->count(map_change::voxel(xyz)) > 0;
            }), cloud->points.end());
        cloud->width = cloud->points.size();
        cloud->height = 1U;
      }
      return cloud;
    };
  const map_export::ProjectorOrigin origin =
    sessions.empty() ? map_export::ProjectorOrigin() : sessions.front().origin;
  const std::string g2o = sessions.size() == 1U ? sessions.front().pose_graph_g2o : std::string();
  adapters::FilesystemMapOutput output;
  map_export::writeBundle(load, poses, map_points, loop_edges, g2o, save_dir, config, origin,
    output, log);
}

std::array<int, 3> storeFrameVoxel(const pcl::PointXYZI & point, const Eigen::Isometry3d & pose)
{
  const Eigen::Vector3d world = pose * Eigen::Vector3d(point.x, point.y, point.z);
  return map_change::voxel({world.x(), world.y(), world.z()});
}

PointState pointState(
  const pcl::PointXYZI & point, const std::set<std::array<int, 3>> * mask,
  const Eigen::Isometry3d & pose, const StoreFrameEdits & edits)
{
  const map_change::Point xyz{point.x, point.y, point.z};
  if (!map_change::usable(xyz)) {
    return PointState::kept;
  }
  const bool masked = mask != nullptr && mask->count(map_change::voxel(xyz)) > 0;
  if (!edits.empty()) {
    const map_change::Voxel cell = storeFrameVoxel(point, pose);
    if (edits.keep.count(cell) > 0) {
      return masked ? PointState::kept_by_edit : PointState::kept;
    }
    if (edits.remove.count(cell) > 0) {
      return PointState::removed_by_edit;
    }
  }
  return masked ? PointState::removed_by_evidence : PointState::kept;
}

void forEachPoint(
  const std::vector<SessionMap> & sessions, const RemovedVoxels & removed,
  const StoreFrameEdits & edits,
  const std::function<void(
    std::size_t, std::size_t, const Eigen::Vector3f &, const std::array<int, 3> &,
    PointState)> & visit)
{
  if (!removed.empty() && removed.size() != sessions.size()) {
    throw std::invalid_argument("expected one removal mask per session");
  }
  for (std::size_t i = 0; i < sessions.size(); ++i) {
    for (std::size_t j = 0; j < sessions[i].submaps.size(); ++j) {
      const StoredSubmap & submap = sessions[i].submaps[j];
      const std::set<std::array<int, 3>> * mask = nullptr;
      if (!removed.empty()) {
        const auto found = removed[i].find(j);
        if (found != removed[i].end() && !found->second.empty()) {
          mask = &found->second;
        }
      }
      const auto cloud = submapCloud(submap);
      const Eigen::Isometry3f pose = submap.pose.cast<float>();
      for (const auto & point : cloud->points) {
        if (!map_change::usable({point.x, point.y, point.z})) {
          continue;
        }
        // The position as the map export moves it (float transform, as pcl::transformPointCloud).
        const Eigen::Vector3f world = pose * Eigen::Vector3f(point.x, point.y, point.z);
        visit(
          i, j, world, storeFrameVoxel(point, submap.pose),
          pointState(point, mask, submap.pose, edits));
      }
    }
  }
}

}  // namespace map_store
}  // namespace graphslam
