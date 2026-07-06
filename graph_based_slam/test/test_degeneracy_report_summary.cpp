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
#include <vector>

#include "graph_based_slam/degeneracy_report_summary.hpp"
#include "graph_based_slam/localizability_analysis.hpp"
#include "graph_based_slam/odometry_covariance_localizability.hpp"

namespace
{

using graphslam::degeneracy::CovarianceLocalizabilityResult;
using graphslam::degeneracy::DegeneracyReportAccumulator;
using graphslam::degeneracy::DegeneracyReportSummary;
using graphslam::degeneracy::DegeneracyScanSample;
using graphslam::degeneracy::degeneracyReportYamlLines;
using graphslam::degeneracy::LocalizabilityCategory;
using graphslam::degeneracy::LocalizabilityReport;
using graphslam::degeneracy::summarizeDegeneracyScans;
using graphslam::degeneracy::worstCategory;

CovarianceLocalizabilityResult makeResult(int well, int degenerate, int non_observable)
{
  CovarianceLocalizabilityResult result;
  result.diagnostics_available = true;
  result.report.well_conditioned_count = well;
  result.report.degenerate_count = degenerate;
  result.report.non_observable_count = non_observable;
  return result;
}

CovarianceLocalizabilityResult makeUnavailable()
{
  return CovarianceLocalizabilityResult();
}

TEST(DegeneracyReportSummary, WorstCategoryPrecedence)
{
  LocalizabilityReport report;
  report.well_conditioned_count = 6;
  EXPECT_EQ(worstCategory(report), LocalizabilityCategory::WELL_CONDITIONED);

  report.well_conditioned_count = 5;
  report.degenerate_count = 1;
  EXPECT_EQ(worstCategory(report), LocalizabilityCategory::DEGENERATE);

  report.degenerate_count = 1;
  report.non_observable_count = 2;
  EXPECT_EQ(worstCategory(report), LocalizabilityCategory::NON_OBSERVABLE);
}

TEST(DegeneracyReportSummary, EmptyRunProducesZeroedSummary)
{
  const DegeneracyReportSummary summary = summarizeDegeneracyScans({});
  EXPECT_EQ(summary.total_scans, 0u);
  EXPECT_EQ(summary.diagnostics_available_scans, 0u);
  EXPECT_DOUBLE_EQ(summary.diagnosticsAvailableRatio(), 0.0);
  EXPECT_DOUBLE_EQ(summary.wellConditionedRatio(), 0.0);
  EXPECT_FALSE(summary.worst_interval.valid);
}

TEST(DegeneracyReportSummary, CountsAndRatiosAreRelativeToAvailableScans)
{
  std::vector<DegeneracyScanSample> samples;
  samples.push_back({0.0, makeUnavailable()});
  samples.push_back({1.0, makeResult(6, 0, 0)});
  samples.push_back({2.0, makeResult(5, 1, 0)});
  samples.push_back({3.0, makeResult(3, 0, 3)});
  samples.push_back({4.0, makeResult(6, 0, 0)});

  const DegeneracyReportSummary summary = summarizeDegeneracyScans(samples);
  EXPECT_EQ(summary.total_scans, 5u);
  EXPECT_EQ(summary.diagnostics_available_scans, 4u);
  EXPECT_EQ(summary.well_conditioned_scans, 2u);
  EXPECT_EQ(summary.degenerate_scans, 1u);
  EXPECT_EQ(summary.non_observable_scans, 1u);
  EXPECT_DOUBLE_EQ(summary.diagnosticsAvailableRatio(), 4.0 / 5.0);
  EXPECT_DOUBLE_EQ(summary.wellConditionedRatio(), 0.5);
  EXPECT_DOUBLE_EQ(summary.degenerateRatio(), 0.25);
  EXPECT_DOUBLE_EQ(summary.nonObservableRatio(), 0.25);
}

TEST(DegeneracyReportSummary, WorstIntervalIsLongestContiguousDegenerateRun)
{
  std::vector<DegeneracyScanSample> samples;
  // Run 1: 2 degenerate scans at t=0,1.
  samples.push_back({0.0, makeResult(5, 1, 0)});
  samples.push_back({1.0, makeResult(5, 1, 0)});
  // Break.
  samples.push_back({2.0, makeResult(6, 0, 0)});
  // Run 2 (worst): 3 degenerate scans at t=3..5.
  samples.push_back({3.0, makeResult(5, 1, 0)});
  samples.push_back({4.0, makeResult(5, 1, 0)});
  samples.push_back({5.0, makeResult(5, 1, 0)});
  // Break via unavailable scan, then a short run.
  samples.push_back({6.0, makeUnavailable()});
  samples.push_back({7.0, makeResult(5, 1, 0)});

  const DegeneracyReportSummary summary = summarizeDegeneracyScans(samples);
  ASSERT_TRUE(summary.worst_interval.valid);
  EXPECT_EQ(summary.worst_interval.length_scans, 3u);
  EXPECT_DOUBLE_EQ(summary.worst_interval.start_stamp_sec, 3.0);
  EXPECT_DOUBLE_EQ(summary.worst_interval.end_stamp_sec, 5.0);
  EXPECT_EQ(summary.worst_interval.category, LocalizabilityCategory::DEGENERATE);
}

TEST(DegeneracyReportSummary, IntervalCategoryEscalatesToNonObservable)
{
  std::vector<DegeneracyScanSample> samples;
  samples.push_back({0.0, makeResult(5, 1, 0)});
  samples.push_back({1.0, makeResult(3, 0, 3)});
  samples.push_back({2.0, makeResult(5, 1, 0)});

  const DegeneracyReportSummary summary = summarizeDegeneracyScans(samples);
  ASSERT_TRUE(summary.worst_interval.valid);
  EXPECT_EQ(summary.worst_interval.length_scans, 3u);
  EXPECT_EQ(summary.worst_interval.category, LocalizabilityCategory::NON_OBSERVABLE);
}

TEST(DegeneracyReportSummary, SummaryIsReadableMidStreamWithoutClosingTheRun)
{
  DegeneracyReportAccumulator accumulator;
  accumulator.add(0.0, makeResult(5, 1, 0));
  accumulator.add(1.0, makeResult(5, 1, 0));

  // Mid-stream read: the in-progress run must already be visible...
  const DegeneracyReportSummary mid = accumulator.summary();
  ASSERT_TRUE(mid.worst_interval.valid);
  EXPECT_EQ(mid.worst_interval.length_scans, 2u);

  // ...and reading it must not have mutated state: the run keeps growing.
  accumulator.add(2.0, makeResult(5, 1, 0));
  const DegeneracyReportSummary after = accumulator.summary();
  ASSERT_TRUE(after.worst_interval.valid);
  EXPECT_EQ(after.worst_interval.length_scans, 3u);
  EXPECT_DOUBLE_EQ(after.worst_interval.start_stamp_sec, 0.0);
  EXPECT_DOUBLE_EQ(after.worst_interval.end_stamp_sec, 2.0);
}

TEST(DegeneracyReportSummary, TieBreakPrefersFirstIntervalForDeterminism)
{
  std::vector<DegeneracyScanSample> samples;
  samples.push_back({0.0, makeResult(5, 1, 0)});
  samples.push_back({1.0, makeResult(5, 1, 0)});
  samples.push_back({2.0, makeResult(6, 0, 0)});
  samples.push_back({3.0, makeResult(5, 1, 0)});
  samples.push_back({4.0, makeResult(5, 1, 0)});

  const DegeneracyReportSummary summary = summarizeDegeneracyScans(samples);
  ASSERT_TRUE(summary.worst_interval.valid);
  EXPECT_EQ(summary.worst_interval.length_scans, 2u);
  EXPECT_DOUBLE_EQ(summary.worst_interval.start_stamp_sec, 0.0);
}

TEST(DegeneracyReportSummary, TieBreakEscalatesToNonObservableInterval)
{
  std::vector<DegeneracyScanSample> samples;
  samples.push_back({0.0, makeResult(5, 1, 0)});
  samples.push_back({1.0, makeResult(5, 1, 0)});
  samples.push_back({2.0, makeResult(6, 0, 0)});
  samples.push_back({3.0, makeResult(3, 0, 3)});
  samples.push_back({4.0, makeResult(3, 0, 3)});

  const DegeneracyReportSummary summary = summarizeDegeneracyScans(samples);
  ASSERT_TRUE(summary.worst_interval.valid);
  EXPECT_EQ(summary.worst_interval.length_scans, 2u);
  EXPECT_EQ(summary.worst_interval.category, LocalizabilityCategory::NON_OBSERVABLE);
  EXPECT_DOUBLE_EQ(summary.worst_interval.start_stamp_sec, 3.0);
}

TEST(DegeneracyReportSummary, YamlLinesContainAllSummaryFields)
{
  std::vector<DegeneracyScanSample> samples;
  samples.push_back({10.0, makeResult(5, 1, 0)});
  samples.push_back({11.0, makeResult(6, 0, 0)});
  const DegeneracyReportSummary summary = summarizeDegeneracyScans(samples);

  const std::vector<std::string> lines = degeneracyReportYamlLines(summary);
  ASSERT_FALSE(lines.empty());
  EXPECT_EQ(lines.front(), "degeneracy_report:");

  const auto contains = [&lines](const std::string & needle) {
      for (const auto & line : lines) {
        if (line.find(needle) != std::string::npos) {return true;}
      }
      return false;
    };
  EXPECT_TRUE(contains("total_scans: 2"));
  EXPECT_TRUE(contains("diagnostics_available_scans: 2"));
  EXPECT_TRUE(contains("degenerate_scans: 1"));
  EXPECT_TRUE(contains("well_conditioned_ratio: 0.500000"));
  EXPECT_TRUE(contains("worst_interval:"));
  EXPECT_TRUE(contains("valid: true"));
  EXPECT_TRUE(contains("length_scans: 1"));
}

TEST(DegeneracyReportSummary, YamlLinesForEmptyRunOmitIntervalDetails)
{
  const DegeneracyReportSummary summary = summarizeDegeneracyScans({});
  const std::vector<std::string> lines = degeneracyReportYamlLines(summary);
  const auto contains = [&lines](const std::string & needle) {
      for (const auto & line : lines) {
        if (line.find(needle) != std::string::npos) {return true;}
      }
      return false;
    };
  EXPECT_TRUE(contains("valid: false"));
  EXPECT_FALSE(contains("start_stamp_sec"));
}

TEST(DegeneracyReportSummary, SameInputTwiceProducesIdenticalYaml)
{
  std::vector<DegeneracyScanSample> samples;
  samples.push_back({0.5, makeResult(5, 1, 0)});
  samples.push_back({1.5, makeUnavailable()});
  samples.push_back({2.5, makeResult(6, 0, 0)});

  const std::vector<std::string> first =
    degeneracyReportYamlLines(summarizeDegeneracyScans(samples));
  const std::vector<std::string> second =
    degeneracyReportYamlLines(summarizeDegeneracyScans(samples));
  EXPECT_EQ(first, second);
}

}  // namespace
