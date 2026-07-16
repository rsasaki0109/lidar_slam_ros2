// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)

#include <gtest/gtest.h>

#include "graph_based_slam/loop_search_schedule.hpp"

TEST(LoopSearchSchedule, StrideOneSearchesEveryNonNegativeQuery)
{
  EXPECT_FALSE(graphslam::loop_search_schedule::shouldSearch(-1, 1));
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(0, 1));
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(17, 1));
}

TEST(LoopSearchSchedule, StrideFiveUsesOneBasedSubmapMultiples)
{
  for (int query = 0; query < 15; ++query) {
    const bool expected = query == 4 || query == 9 || query == 14;
    EXPECT_EQ(graphslam::loop_search_schedule::shouldSearch(query, 5), expected)
      << "query=" << query;
  }
}

TEST(LoopSearchSchedule, InvalidStrideFallsBackToEveryQuery)
{
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(0, 0));
  EXPECT_TRUE(graphslam::loop_search_schedule::shouldSearch(3, -4));
}
