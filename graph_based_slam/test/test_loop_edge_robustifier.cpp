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

#include <string>

#include "graph_based_slam/loop_edge_robustifier.hpp"

namespace graphslam
{
namespace robust
{
namespace
{

TEST(LoopEdgeRobustifier, DefaultUnknownStringFallsBackToHuber)
{
  EXPECT_EQ(LoopEdgeKernelType::Huber, parseLoopEdgeKernelType(""));
  EXPECT_EQ(LoopEdgeKernelType::Huber, parseLoopEdgeKernelType("unknown"));
  EXPECT_EQ(LoopEdgeKernelType::Huber, parseLoopEdgeKernelType("HUbeR"));
}

TEST(LoopEdgeRobustifier, ParsesDcsCaseInsensitively)
{
  EXPECT_EQ(LoopEdgeKernelType::DCS, parseLoopEdgeKernelType("dcs"));
  EXPECT_EQ(LoopEdgeKernelType::DCS, parseLoopEdgeKernelType("DCS"));
  EXPECT_EQ(LoopEdgeKernelType::DCS, parseLoopEdgeKernelType("Dcs"));
}

TEST(LoopEdgeRobustifier, ParsesCauchyCaseInsensitively)
{
  EXPECT_EQ(LoopEdgeKernelType::Cauchy, parseLoopEdgeKernelType("cauchy"));
  EXPECT_EQ(LoopEdgeKernelType::Cauchy, parseLoopEdgeKernelType("CAUCHY"));
}

TEST(LoopEdgeRobustifier, NamesRoundTrip)
{
  for (const auto type :
    {LoopEdgeKernelType::Huber, LoopEdgeKernelType::DCS, LoopEdgeKernelType::Cauchy})
  {
    const std::string name = loopEdgeKernelTypeName(type);
    EXPECT_EQ(type, parseLoopEdgeKernelType(name));
  }
}

TEST(LoopEdgeRobustifier, NamesAreLowerCaseAscii)
{
  EXPECT_STREQ("huber", loopEdgeKernelTypeName(LoopEdgeKernelType::Huber));
  EXPECT_STREQ("dcs", loopEdgeKernelTypeName(LoopEdgeKernelType::DCS));
  EXPECT_STREQ("cauchy", loopEdgeKernelTypeName(LoopEdgeKernelType::Cauchy));
}

TEST(LoopEdgeRobustifier, ToLowerAsciiHandlesEmptyAndMixed)
{
  EXPECT_EQ("", toLowerAscii(""));
  EXPECT_EQ("abc123", toLowerAscii("AbC123"));
  EXPECT_EQ("test_kernel", toLowerAscii("TEST_KERNEL"));
}

}  // namespace
}  // namespace robust
}  // namespace graphslam
