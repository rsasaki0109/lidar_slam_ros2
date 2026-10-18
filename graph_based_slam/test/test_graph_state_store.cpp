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

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>
#include <utility>
#include <vector>

#include "graph_based_slam/graph_state_store.hpp"

TEST(GraphStateStore, IsEmptyUntilTheFirstCompleteStateIsCommitted)
{
  graphslam::GraphStateStore store;
  lidarslam_msgs::msg::MapArray map_array;

  EXPECT_FALSE(store.snapshot(map_array));

  store.replace(lidarslam_msgs::msg::MapArray());
  EXPECT_TRUE(store.snapshot(map_array));
  EXPECT_TRUE(map_array.submaps.empty());
}

TEST(GraphStateStore, ReplaceAndSnapshotDoNotAliasCallerOwnedMessages)
{
  graphslam::GraphStateStore store;
  lidarslam_msgs::msg::MapArray input;
  input.header.frame_id = "map";
  input.submaps.resize(1);
  input.submaps[0].distance = 12.5;
  store.replace(std::move(input));

  lidarslam_msgs::msg::MapArray first_snapshot;
  ASSERT_TRUE(store.snapshot(first_snapshot));
  first_snapshot.submaps[0].distance = 99.0;

  lidarslam_msgs::msg::MapArray second_snapshot;
  ASSERT_TRUE(store.snapshot(second_snapshot));
  EXPECT_EQ(second_snapshot.header.frame_id, "map");
  ASSERT_EQ(second_snapshot.submaps.size(), 1U);
  EXPECT_DOUBLE_EQ(second_snapshot.submaps[0].distance, 12.5);
}

TEST(GraphStateStore, AppendCommitsMapHeaderAndSubmapTogether)
{
  graphslam::GraphStateStore store;
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 42;
  lidarslam_msgs::msg::SubMap submap;
  submap.distance = 3.0;

  EXPECT_EQ(store.append(std::move(submap), header), 1U);

  lidarslam_msgs::msg::MapArray snapshot;
  ASSERT_TRUE(store.snapshot(snapshot));
  EXPECT_EQ(snapshot.header.frame_id, "map");
  EXPECT_EQ(snapshot.header.stamp.sec, 42);
  ASSERT_EQ(snapshot.submaps.size(), 1U);
  EXPECT_DOUBLE_EQ(snapshot.submaps[0].distance, 3.0);
}

TEST(GraphStateStore, ConcurrentAppendAndSnapshotRemainConsistent)
{
  graphslam::GraphStateStore store;
  store.replace(lidarslam_msgs::msg::MapArray());
  std::atomic<bool> writer_done{false};
  std::atomic<bool> inconsistent_snapshot{false};

  std::thread writer(
    [&]() {
      for (int i = 1; i <= 100; ++i) {
        std_msgs::msg::Header header;
        header.stamp.sec = i;
        lidarslam_msgs::msg::SubMap submap;
        submap.distance = static_cast<double>(i);
        store.append(std::move(submap), header);
      }
      writer_done = true;
    });

  while (!writer_done.load()) {
    lidarslam_msgs::msg::MapArray snapshot;
    if (!store.snapshot(snapshot)) {
      inconsistent_snapshot = true;
      break;
    }
    if (!snapshot.submaps.empty() &&
      snapshot.header.stamp.sec != static_cast<int32_t>(snapshot.submaps.size()))
    {
      inconsistent_snapshot = true;
      break;
    }
  }
  writer.join();

  EXPECT_FALSE(inconsistent_snapshot.load());
  EXPECT_EQ(store.submapCount(), 100U);
}

TEST(GraphStateStore, SnapshotsShareEachStoredCloudInsteadOfCopyingIt)
{
  graphslam::GraphStateStore store;
  std_msgs::msg::Header header;
  header.frame_id = "map";
  lidarslam_msgs::msg::SubMap submap;
  submap.distance = 1.0;
  submap.cloud.width = 3;
  submap.cloud.height = 1;
  submap.cloud.data = {1, 2, 3, 4, 5, 6};
  store.append(std::move(submap), header);

  graphslam::GraphStateStore::Snapshot first;
  graphslam::GraphStateStore::Snapshot second;
  ASSERT_TRUE(store.snapshot(first));
  ASSERT_TRUE(store.snapshot(second));
  ASSERT_EQ(first.map_array.submaps.size(), 1U);
  ASSERT_EQ(first.clouds.size(), 1U);
  EXPECT_TRUE(first.map_array.submaps[0].cloud.data.empty());
  EXPECT_DOUBLE_EQ(first.map_array.submaps[0].distance, 1.0);
  EXPECT_EQ(first.clouds[0].get(), second.clouds[0].get());
  EXPECT_EQ(first.clouds[0]->data, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(first.clouds[0]->width, 3U);

  // The deep snapshot still hands out the whole graph, clouds included.
  lidarslam_msgs::msg::MapArray deep;
  ASSERT_TRUE(store.snapshot(deep));
  ASSERT_EQ(deep.submaps.size(), 1U);
  EXPECT_EQ(deep.submaps[0].cloud.data, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6}));
}

TEST(GraphStateStore, ReplaceKeepsTheCloudOfEverySubmap)
{
  graphslam::GraphStateStore store;
  lidarslam_msgs::msg::MapArray input;
  input.submaps.resize(2);
  input.submaps[0].cloud.data = {7};
  input.submaps[1].cloud.data = {8, 9};
  store.replace(std::move(input));

  graphslam::GraphStateStore::Snapshot snapshot;
  ASSERT_TRUE(store.snapshot(snapshot));
  ASSERT_EQ(snapshot.clouds.size(), 2U);
  EXPECT_EQ(snapshot.clouds[0]->data, (std::vector<uint8_t>{7}));
  EXPECT_EQ(snapshot.clouds[1]->data, (std::vector<uint8_t>{8, 9}));
  EXPECT_TRUE(snapshot.map_array.submaps[1].cloud.data.empty());
}
