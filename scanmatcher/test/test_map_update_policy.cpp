#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "scanmatcher/map_update_policy.hpp"

namespace
{
namespace policy = graphslam::map_update_policy;
}  // namespace

TEST(MapUpdatePolicy, ShouldTriggerMapUpdateRequiresAllConditions)
{
  // The historical condition: trans_ >= trans_for_mapupdate_ &&
  // !mapping_flag_ && !suppress_map_update.
  EXPECT_TRUE(policy::shouldTriggerMapUpdate(1.5, 1.5, false, false));  // >= boundary
  EXPECT_TRUE(policy::shouldTriggerMapUpdate(2.0, 1.5, false, false));
  EXPECT_FALSE(policy::shouldTriggerMapUpdate(1.4999, 1.5, false, false));
  EXPECT_FALSE(policy::shouldTriggerMapUpdate(2.0, 1.5, true, false));   // async in flight
  EXPECT_FALSE(policy::shouldTriggerMapUpdate(2.0, 1.5, false, true));   // reject cooldown
}

TEST(MapUpdatePolicy, UseAsyncMapUpdateWarmupGate)
{
  // warmup <= 0 disables the warmup entirely.
  EXPECT_TRUE(policy::useAsyncMapUpdate(true, 0, 0));
  EXPECT_TRUE(policy::useAsyncMapUpdate(true, -1, 0));
  // Otherwise async starts only once the map holds warmup submaps.
  EXPECT_FALSE(policy::useAsyncMapUpdate(true, 10, 9));
  EXPECT_TRUE(policy::useAsyncMapUpdate(true, 10, 10));  // >= boundary
  EXPECT_TRUE(policy::useAsyncMapUpdate(true, 10, 11));
  // Async disabled wins regardless.
  EXPECT_FALSE(policy::useAsyncMapUpdate(false, 0, 100));
}

TEST(MapUpdatePolicy, VoxelHashLocalRadiusCapsAtFifty)
{
  EXPECT_DOUBLE_EQ(policy::voxelHashLocalRadius(30.0), 30.0);
  EXPECT_DOUBLE_EQ(policy::voxelHashLocalRadius(50.0), 50.0);
  EXPECT_DOUBLE_EQ(policy::voxelHashLocalRadius(120.0), 50.0);
}

TEST(MapUpdatePolicy, SelectSpatialSubmapIndicesNewestFirstWithinRadius)
{
  // Submaps along x: 0, 1, 2, ..., 5; the robot sits at x = 5.
  std::vector<Eigen::Vector3d> positions;
  for (int i = 0; i <= 5; ++i) {
    positions.emplace_back(static_cast<double>(i), 0.0, 0.0);
  }
  const Eigen::Vector3d current(5.0, 0.0, 0.0);

  // Radius 2.0 admits x in [3, 5]; the cap (num_targeted_cloud - 1 = 10)
  // does not bind. Historical iteration is newest to oldest.
  const std::vector<int> indices =
    policy::selectSpatialSubmapIndices(positions, current, 2.0, 11);
  ASSERT_EQ(indices.size(), 3u);
  EXPECT_EQ(indices[0], 5);
  EXPECT_EQ(indices[1], 4);
  EXPECT_EQ(indices[2], 3);

  // dist == radius is included (<=).
  const std::vector<int> boundary =
    policy::selectSpatialSubmapIndices(positions, current, 1.0, 11);
  ASSERT_EQ(boundary.size(), 2u);
  EXPECT_EQ(boundary[1], 4);
}

TEST(MapUpdatePolicy, SelectSpatialSubmapIndicesCapsAtTargetedCloud)
{
  std::vector<Eigen::Vector3d> positions(20, Eigen::Vector3d::Zero());
  const Eigen::Vector3d current(0.0, 0.0, 0.0);

  // All 20 submaps are in range but only num_targeted_cloud - 1 = 4 fit.
  const std::vector<int> indices =
    policy::selectSpatialSubmapIndices(positions, current, 10.0, 5);
  ASSERT_EQ(indices.size(), 4u);
  EXPECT_EQ(indices[0], 19);
  EXPECT_EQ(indices[3], 16);

  // num_targeted_cloud = 1 leaves room for the live scan only.
  EXPECT_TRUE(policy::selectSpatialSubmapIndices(positions, current, 10.0, 1).empty());
}

TEST(MapUpdatePolicy, SelectTemporalSubmapIndicesMostRecentFirst)
{
  // 10 submaps, num_targeted_cloud 4 -> the 3 most recent, newest first.
  const std::vector<int> indices = policy::selectTemporalSubmapIndices(10, 4);
  ASSERT_EQ(indices.size(), 3u);
  EXPECT_EQ(indices[0], 9);
  EXPECT_EQ(indices[1], 8);
  EXPECT_EQ(indices[2], 7);

  // Fewer submaps than requested: negative indices are skipped, not clamped.
  const std::vector<int> sparse = policy::selectTemporalSubmapIndices(2, 4);
  ASSERT_EQ(sparse.size(), 2u);
  EXPECT_EQ(sparse[0], 1);
  EXPECT_EQ(sparse[1], 0);

  EXPECT_TRUE(policy::selectTemporalSubmapIndices(10, 1).empty());
  EXPECT_TRUE(policy::selectTemporalSubmapIndices(0, 4).empty());
}

TEST(MapUpdatePolicy, ShouldPublishMapStrictGreaterThan)
{
  EXPECT_FALSE(policy::shouldPublishMap(1.0, 1.0));  // dt == period does not publish
  EXPECT_TRUE(policy::shouldPublishMap(1.0001, 1.0));
  EXPECT_FALSE(policy::shouldPublishMap(0.5, 1.0));
}

TEST(MapUpdatePolicy, UpdateAdaptiveCorrespondenceEmaSeedAndBlend)
{
  // First positive sample seeds the EMA.
  EXPECT_DOUBLE_EQ(policy::updateAdaptiveCorrespondenceEma(0.0, 0.8, 0.1), 0.8);
  EXPECT_DOUBLE_EQ(policy::updateAdaptiveCorrespondenceEma(-1.0, 0.8, 0.1), 0.8);

  // Subsequent samples blend with the historical expression.
  const double ema = 0.5;
  const double mean_corr = 0.9;
  const double alpha = 0.1;
  const double expected = alpha * mean_corr + (1.0 - alpha) * ema;
  EXPECT_DOUBLE_EQ(policy::updateAdaptiveCorrespondenceEma(ema, mean_corr, alpha), expected);

  // A non-positive measurement leaves the EMA untouched.
  EXPECT_DOUBLE_EQ(policy::updateAdaptiveCorrespondenceEma(0.5, 0.0, 0.1), 0.5);
  EXPECT_DOUBLE_EQ(policy::updateAdaptiveCorrespondenceEma(0.5, -0.2, 0.1), 0.5);
}

TEST(MapUpdatePolicy, AdaptiveMaxCorrespondenceDistanceIsPlainProduct)
{
  EXPECT_DOUBLE_EQ(policy::adaptiveMaxCorrespondenceDistance(3.0, 0.4), 3.0 * 0.4);
}

TEST(MapUpdatePolicy, EmaChainBitwiseIdentical)
{
  const auto run_chain = [] {
      double ema = 0.0;
      for (int i = 1; i <= 100; ++i) {
        const double mean_corr = (i % 5 == 0) ? 0.0 : 0.3 + 0.01 * (i % 17);
        ema = policy::updateAdaptiveCorrespondenceEma(ema, mean_corr, 0.1);
      }
      return ema;
    };

  const double first = run_chain();
  const double second = run_chain();
  EXPECT_EQ(std::memcmp(&first, &second, sizeof(double)), 0);
}
