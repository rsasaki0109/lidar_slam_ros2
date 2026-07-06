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

#include <array>
#include <sstream>
#include <string>
#include <vector>

#include "graph_based_slam/degeneracy_diagnostics_csv.hpp"
#include "graph_based_slam/localizability_analysis.hpp"
#include "graph_based_slam/odometry_covariance_localizability.hpp"
#include "graph_based_slam/synthetic_degeneracy_fixtures.hpp"

namespace
{

using graphslam::degeneracy::analyzeLocalizability;
using graphslam::degeneracy::BoxFixtureConfig;
using graphslam::degeneracy::buildGaussNewtonSystem;
using graphslam::degeneracy::CovarianceLocalizabilityResult;
using graphslam::degeneracy::degeneracyDiagnosticsCsvHeaderLine;
using graphslam::degeneracy::degeneracyDiagnosticsCsvRowLine;
using graphslam::degeneracy::makeBoxFixture;

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  // std::getline drops a trailing empty field after the last comma; restore
  // it so column counts stay exact for the "unavailable" row shape.
  if (!line.empty() && line.back() == ',') {
    fields.push_back("");
  }
  return fields;
}

TEST(DegeneracyDiagnosticsCsv, HeaderHasExpectedColumnCount)
{
  const std::vector<std::string> columns = splitCsv(degeneracyDiagnosticsCsvHeaderLine());
  // stamp_sec, diagnostics_available, 6 eigenvalues, 6 categories,
  // well/degenerate/non_observable counts, condition_number, 6x6
  // eigenvector components = 54.
  EXPECT_EQ(columns.size(), 54u);
  EXPECT_EQ(columns.front(), "stamp_sec");
  EXPECT_EQ(columns[17], "condition_number");
  EXPECT_EQ(columns[18], "eigenvector_0_tx");
  EXPECT_EQ(columns.back(), "eigenvector_5_rz");
}

TEST(DegeneracyDiagnosticsCsv, UnavailableRowHasSameColumnCountAsHeaderAndBlankFields)
{
  CovarianceLocalizabilityResult unavailable;  // diagnostics_available defaults false
  const std::string row = degeneracyDiagnosticsCsvRowLine(12.5, unavailable);
  const std::vector<std::string> header_columns = splitCsv(degeneracyDiagnosticsCsvHeaderLine());
  const std::vector<std::string> row_columns = splitCsv(row);

  ASSERT_EQ(row_columns.size(), header_columns.size());
  EXPECT_EQ(row_columns[0], "12.5");
  EXPECT_EQ(row_columns[1], "0");
  for (size_t i = 2; i < row_columns.size(); ++i) {
    EXPECT_EQ(row_columns[i], "") << i;
  }
}

TEST(DegeneracyDiagnosticsCsv, AvailableRowHasSameColumnCountAsHeaderAndPopulatedFields)
{
  const auto fixture = makeBoxFixture(BoxFixtureConfig());
  const auto system = buildGaussNewtonSystem(fixture.correspondences);
  CovarianceLocalizabilityResult result;
  result.diagnostics_available = true;
  result.report = analyzeLocalizability(system.h, system.b);

  const std::string row = degeneracyDiagnosticsCsvRowLine(3.0, result);
  const std::vector<std::string> header_columns = splitCsv(degeneracyDiagnosticsCsvHeaderLine());
  const std::vector<std::string> row_columns = splitCsv(row);

  ASSERT_EQ(row_columns.size(), header_columns.size());
  EXPECT_EQ(row_columns[0], "3");
  EXPECT_EQ(row_columns[1], "1");
  // Box fixture: all six directions WELL_CONDITIONED (categories at
  // columns 8..13, 0-indexed: stamp, avail, 6 eigenvalues, then categories).
  for (size_t i = 8; i < 14; ++i) {
    EXPECT_EQ(row_columns[i], "WELL_CONDITIONED") << i;
  }
  EXPECT_EQ(row_columns[14], "6");   // well_conditioned_count
  EXPECT_EQ(row_columns[15], "0");   // degenerate_count
  EXPECT_EQ(row_columns[16], "0");   // non_observable_count
  EXPECT_FALSE(row_columns[17].empty());  // condition_number

  // Columns 18..53: each direction's sign-canonicalized eigenvector,
  // matching the report's own values exactly (%.17g round-trips a double).
  for (int i = 0; i < 6; ++i) {
    for (int a = 0; a < 6; ++a) {
      EXPECT_DOUBLE_EQ(
        std::stod(row_columns[static_cast<size_t>(18 + 6 * i + a)]),
        result.report.directions[i].eigenvector(a)) << i << "," << a;
    }
  }
}

TEST(DegeneracyDiagnosticsCsv, RowFormattingIsDeterministic)
{
  const auto fixture = makeBoxFixture(BoxFixtureConfig());
  const auto system = buildGaussNewtonSystem(fixture.correspondences);
  CovarianceLocalizabilityResult result;
  result.diagnostics_available = true;
  result.report = analyzeLocalizability(system.h, system.b);

  const std::string first = degeneracyDiagnosticsCsvRowLine(42.0, result);
  const std::string second = degeneracyDiagnosticsCsvRowLine(42.0, result);
  EXPECT_EQ(first, second);
}

TEST(DegeneracyDiagnosticsCsv, NoRowContainsANewline)
{
  CovarianceLocalizabilityResult unavailable;
  EXPECT_EQ(degeneracyDiagnosticsCsvRowLine(1.0, unavailable).find('\n'), std::string::npos);
  EXPECT_EQ(degeneracyDiagnosticsCsvHeaderLine().find('\n'), std::string::npos);
}

}  // namespace
