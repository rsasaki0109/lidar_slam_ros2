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
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
// OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "graph_based_slam/target_cloud_cache.hpp"

namespace
{

using graphslam::backend_core::TargetCloud;
using graphslam::backend_core::TargetCloudCache;
using graphslam::backend_core::TargetCloudCacheKey;
using graphslam::backend_core::TargetCloudCacheRevision;
using graphslam::backend_core::TargetCloudCacheValue;
using graphslam::backend_core::TargetCloudCacheVariant;

TargetCloudCacheKey makeKey(
  int candidate_index, std::uint64_t content_revision = 1U, double x = 0.0)
{
  TargetCloudCacheKey key;
  key.candidate_index = candidate_index;
  key.neighbor_radius = 1;
  key.bbs_neighbor_radius = 1;
  key.voxel_leaf_size = 0.2;
  key.bbs_voxel_leaf_size = 1.0;
  TargetCloudCacheRevision revision;
  revision.submap_index = candidate_index;
  revision.content_revision = content_revision;
  revision.pose = Eigen::Affine3d(Eigen::Translation3d(x, 0.0, 0.0)).matrix();
  key.revisions.push_back(revision);
  return key;
}

TargetCloudCacheValue makeValue(std::size_t points)
{
  TargetCloudCacheValue value;
  value.aggregate.reset(new TargetCloud);
  value.bbs_aggregate.reset(new TargetCloud);
  value.filtered.reset(new TargetCloud);
  value.filtered_bbs.reset(new TargetCloud);
  for (std::size_t i = 0; i < points; ++i) {
    pcl::PointXYZI point;
    point.x = static_cast<float>(i);
    point.y = static_cast<float>(i + 1U);
    point.z = static_cast<float>(i + 2U);
    point.intensity = 1.0F;
    value.aggregate->push_back(point);
    value.bbs_aggregate->push_back(point);
    value.filtered->push_back(point);
    value.filtered_bbs->push_back(point);
  }
  return value;
}

TEST(TargetCloudCacheKey, UnknownRevisionIsFailClosed)
{
  auto key = makeKey(0, 0U);
  EXPECT_FALSE(key.cacheable());

  TargetCloudCache cache;
  EXPECT_FALSE(cache.insert(key, makeValue(1U)));
  TargetCloudCacheValue value;
  EXPECT_FALSE(cache.lookup(key, &value));
  EXPECT_EQ(cache.size(), 0U);
}

TEST(TargetCloudCache, ExactKeyHitsAndRevisionOrPoseChangeMisses)
{
  TargetCloudCache cache;
  const auto key = makeKey(4);
  ASSERT_TRUE(cache.insert(key, makeValue(2U)));

  TargetCloudCacheValue value;
  EXPECT_TRUE(cache.lookup(key, &value));
  ASSERT_TRUE(value.completeFor(TargetCloudCacheVariant::kRegular));
  EXPECT_EQ(value.aggregate->size(), 2U);

  EXPECT_FALSE(cache.lookup(makeKey(4, 2U), &value));
  EXPECT_FALSE(cache.lookup(makeKey(4, 1U, 0.01), &value));
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_EQ(cache.hits(), 1U);
  EXPECT_EQ(cache.misses(), 2U);
}

TEST(TargetCloudCache, CandidateVariantStoresOnlyRequiredRepresentation)
{
  TargetCloudCache cache;

  TargetCloudCacheValue regular_value;
  regular_value.filtered.reset(new TargetCloud);
  regular_value.filtered->push_back(pcl::PointXYZI());
  const auto regular_key = makeKey(10U);
  ASSERT_TRUE(cache.insert(regular_key, regular_value));

  TargetCloudCacheValue loaded;
  ASSERT_TRUE(cache.lookup(regular_key, &loaded));
  EXPECT_TRUE(loaded.filtered);
  EXPECT_FALSE(loaded.aggregate);
  EXPECT_FALSE(loaded.bbs_aggregate);
  EXPECT_FALSE(loaded.filtered_bbs);

  TargetCloudCacheValue scan_context_value;
  scan_context_value.filtered_bbs.reset(new TargetCloud);
  scan_context_value.filtered_bbs->push_back(pcl::PointXYZI());
  auto scan_context_key = makeKey(11U);
  scan_context_key.variant = TargetCloudCacheVariant::kScanContext;
  ASSERT_TRUE(cache.insert(scan_context_key, scan_context_value));
  ASSERT_TRUE(cache.lookup(scan_context_key, &loaded));
  EXPECT_FALSE(loaded.filtered);
  EXPECT_FALSE(loaded.aggregate);
  EXPECT_FALSE(loaded.bbs_aggregate);
  EXPECT_TRUE(loaded.filtered_bbs);

  TargetCloudCacheValue bbs_value;
  bbs_value.bbs_aggregate.reset(new TargetCloud);
  bbs_value.bbs_aggregate->push_back(pcl::PointXYZI());
  bbs_value.filtered_bbs.reset(new TargetCloud);
  bbs_value.filtered_bbs->push_back(pcl::PointXYZI());
  auto bbs_key = makeKey(12U);
  bbs_key.variant = TargetCloudCacheVariant::kScanContextWithThreeDBbs;
  ASSERT_TRUE(cache.insert(bbs_key, bbs_value));
  ASSERT_TRUE(cache.lookup(bbs_key, &loaded));
  EXPECT_FALSE(loaded.filtered);
  EXPECT_FALSE(loaded.aggregate);
  EXPECT_TRUE(loaded.bbs_aggregate);
  EXPECT_TRUE(loaded.filtered_bbs);

  // The same revisions/candidate cannot cross the regular and ScanContext
  // representations, even when the cloud pointer shape happens to match.
  auto mismatched_key = regular_key;
  mismatched_key.variant = TargetCloudCacheVariant::kScanContext;
  EXPECT_FALSE(cache.lookup(mismatched_key, &loaded));
}

TEST(TargetCloudCacheKey, MapAppendOutsideNeighborhoodDoesNotChangeKey)
{
  TargetCloudCache cache;
  const auto key_before_append = makeKey(4);
  ASSERT_TRUE(cache.insert(key_before_append, makeValue(1U)));

  // The key builder records only the candidate's target neighborhood.  A
  // submap appended beyond that neighborhood is therefore a safe hit.
  const auto key_after_outside_append = makeKey(4);
  TargetCloudCacheValue value;
  EXPECT_TRUE(cache.lookup(key_after_outside_append, &value));

  // A new revision inside the neighborhood must not reuse the old target.
  auto key_after_inside_change = key_before_append;
  TargetCloudCacheRevision new_neighbor;
  new_neighbor.submap_index = 5;
  new_neighbor.content_revision = 1U;
  key_after_inside_change.revisions.push_back(new_neighbor);
  EXPECT_FALSE(cache.lookup(key_after_inside_change, &value));
}

TEST(TargetCloudCache, EvictsLeastRecentlyUsedDeterministically)
{
  TargetCloudCache::Config config;
  config.capacity = 2U;
  config.max_points = 100U;
  TargetCloudCache cache(config);
  const auto key_a = makeKey(0);
  const auto key_b = makeKey(1);
  const auto key_c = makeKey(2);
  ASSERT_TRUE(cache.insert(key_a, makeValue(1U)));
  ASSERT_TRUE(cache.insert(key_b, makeValue(1U)));

  TargetCloudCacheValue value;
  ASSERT_TRUE(cache.lookup(key_a, &value));
  ASSERT_TRUE(cache.insert(key_c, makeValue(1U)));
  EXPECT_TRUE(cache.lookup(key_a, &value));
  EXPECT_FALSE(cache.lookup(key_b, &value));
  EXPECT_TRUE(cache.lookup(key_c, &value));
  EXPECT_EQ(cache.size(), 2U);
}

TEST(TargetCloudCache, PointBudgetRejectsOversizedEntries)
{
  TargetCloudCache::Config config;
  config.capacity = 8U;
  config.max_points = 3U;
  TargetCloudCache cache(config);
  EXPECT_FALSE(cache.insert(makeKey(0), makeValue(1U)));
  EXPECT_EQ(cache.size(), 0U);
  EXPECT_EQ(cache.totalPoints(), 0U);
}

TEST(TargetCloudCache, ContentRevisionChangesWhenCloudChanges)
{
  TargetCloud cloud;
  pcl::PointXYZI point;
  point.x = 1.0F;
  point.y = 2.0F;
  point.z = 3.0F;
  point.intensity = 4.0F;
  cloud.push_back(point);
  const auto first = graphslam::backend_core::targetCloudContentRevision(cloud);
  cloud.points.front().z = 4.0F;
  const auto second = graphslam::backend_core::targetCloudContentRevision(cloud);
  EXPECT_NE(first, second);
}

}  // namespace
