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
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
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

#include "graph_based_slam/scan_context.hpp"

namespace
{

graphslam::ScanContext::Descriptor makeDescriptor(int sector_offset)
{
  graphslam::ScanContext::Descriptor desc =
    graphslam::ScanContext::Descriptor::Zero(
    graphslam::ScanContext::NUM_RINGS,
    graphslam::ScanContext::NUM_SECTORS);
  desc(2, sector_offset % graphslam::ScanContext::NUM_SECTORS) = 1.0;
  desc(5, (sector_offset + 3) % graphslam::ScanContext::NUM_SECTORS) = 2.0;
  return desc;
}

}  // namespace

TEST(ScanContextDatabase, QueryReturnsSubmapIdInsteadOfDescriptorIndex)
{
  graphslam::ScanContext::Database db;
  db.add(10, makeDescriptor(0));
  db.add(42, makeDescriptor(7));

  const auto match = db.query(
    makeDescriptor(0),
    /*num_candidates=*/ 2,
    /*exclude_recent=*/ 1,
    /*threshold=*/ 0.5);

  EXPECT_EQ(match.first, 10);
  EXPECT_NEAR(match.second, 0.0, 1e-9);
}

TEST(ScanContextDatabase, NextSubmapIndexTracksSequentialInsertion)
{
  graphslam::ScanContext::Database db;
  EXPECT_EQ(db.nextSubmapIndex(), 0);

  db.add(0, makeDescriptor(0));
  EXPECT_EQ(db.nextSubmapIndex(), 1);

  db.add(1, makeDescriptor(1));
  EXPECT_EQ(db.nextSubmapIndex(), 2);
}

TEST(ScanContextDatabase, QueryTopMatchesReturnsSortedSubmapIds)
{
  graphslam::ScanContext::Database db;
  db.add(10, makeDescriptor(0));
  db.add(20, makeDescriptor(1));
  db.add(30, makeDescriptor(2));
  db.add(40, makeDescriptor(3));

  const auto matches = db.queryTopMatches(
    makeDescriptor(1),
    /*num_matches=*/ 2,
    /*num_candidates=*/ 4,
    /*exclude_recent=*/ 1,
    /*threshold=*/ 0.5);

  ASSERT_EQ(matches.size(), 2u);
  EXPECT_NEAR(matches[0].second, 0.0, 1e-9);
  EXPECT_NEAR(matches[1].second, 0.0, 1e-9);
  EXPECT_TRUE(matches[0].first == 10 || matches[0].first == 20);
  EXPECT_TRUE(matches[1].first == 10 || matches[1].first == 20);
  EXPECT_NE(matches[0].first, matches[1].first);
}

TEST(ScanContextDatabase, QueryTopMatchesWithYawReturnsShift)
{
  graphslam::ScanContext::Database db;
  db.add(10, makeDescriptor(7));
  db.add(20, makeDescriptor(11));

  const auto matches = db.queryTopMatchesWithYaw(
    makeDescriptor(0),
    /*num_matches=*/ 1,
    /*num_candidates=*/ 2,
    /*exclude_recent=*/ 0,
    /*threshold=*/ 0.5);

  ASSERT_EQ(matches.size(), 1u);
  EXPECT_EQ(matches[0].submap_id, 10);
  EXPECT_NEAR(matches[0].distance, 0.0, 1e-9);
  EXPECT_EQ(matches[0].yaw_shift, 7);
}
