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

#ifndef GRAPH_BASED_SLAM__GRAPH_STATE_STORE_HPP_
#define GRAPH_BASED_SLAM__GRAPH_STATE_STORE_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <lidarslam_msgs/msg/map_array.hpp>
#include <lidarslam_msgs/msg/sub_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace graphslam
{

class GraphStateStore
{
public:
  // A submap's cloud never changes once it is stored, so the store keeps one copy of it and
  // hands out shared pointers: a Snapshot copies the graph's metadata and the pointers only.
  using CloudPtr = std::shared_ptr<const sensor_msgs::msg::PointCloud2>;

  struct Snapshot
  {
    // The graph with every submap's cloud left empty; clouds[i] is the cloud of submap i.
    lidarslam_msgs::msg::MapArray map_array;
    std::vector<CloudPtr> clouds;
  };

  void replace(lidarslam_msgs::msg::MapArray map_array)
  {
    std::vector<CloudPtr> clouds;
    clouds.reserve(map_array.submaps.size());
    for (auto & submap : map_array.submaps) {
      clouds.push_back(takeCloud(submap));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    map_array_ = std::move(map_array);
    clouds_ = std::move(clouds);
    initialized_ = true;
  }

  std::size_t append(
    lidarslam_msgs::msg::SubMap submap,
    const std_msgs::msg::Header & map_header)
  {
    CloudPtr cloud = takeCloud(submap);
    std::lock_guard<std::mutex> lock(mutex_);
    map_array_.header = map_header;
    map_array_.submaps.push_back(std::move(submap));
    clouds_.push_back(std::move(cloud));
    initialized_ = true;
    return map_array_.submaps.size();
  }

  std::size_t submapCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_array_.submaps.size();
  }

  // The whole graph, clouds included (a deep copy).
  bool snapshot(lidarslam_msgs::msg::MapArray & map_array) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return false;
    }
    map_array = map_array_;
    for (std::size_t i = 0; i < map_array.submaps.size(); ++i) {
      map_array.submaps[i].cloud = *clouds_[i];
    }
    return true;
  }

  // The graph's metadata and shared pointers to its clouds (no point data is copied).
  bool snapshot(Snapshot & snapshot) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return false;
    }
    snapshot.map_array = map_array_;
    snapshot.clouds = clouds_;
    return true;
  }

private:
  static CloudPtr takeCloud(lidarslam_msgs::msg::SubMap & submap)
  {
    auto cloud = std::make_shared<const sensor_msgs::msg::PointCloud2>(std::move(submap.cloud));
    submap.cloud = sensor_msgs::msg::PointCloud2();
    return cloud;
  }

  mutable std::mutex mutex_;
  bool initialized_{false};
  lidarslam_msgs::msg::MapArray map_array_;
  std::vector<CloudPtr> clouds_;
};

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__GRAPH_STATE_STORE_HPP_
