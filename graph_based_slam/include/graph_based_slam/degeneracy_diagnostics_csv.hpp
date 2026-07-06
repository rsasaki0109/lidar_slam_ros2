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

#ifndef GRAPH_BASED_SLAM__DEGENERACY_DIAGNOSTICS_CSV_HPP_
#define GRAPH_BASED_SLAM__DEGENERACY_DIAGNOSTICS_CSV_HPP_

// Pure header: per-scan degeneracy diagnostics CSV line formatting
// (docs/roadmap/v0.8.md §5 Phase 1, "a per-scan diagnostics CSV in the
// offline runners"). Kept separate from the ROS component / offline runner
// so the exact text format is unit-testable without rclcpp, a bag, or a
// filesystem -- the same "pure header, thin ROS shell" split every other
// opt-in diagnostic in this package uses (`adjacent_edge_auto_scale.hpp`,
// `map_saver.hpp`).
//
// One row per received odometry+cloud pair (i.e. per scan on the
// `use_odom_input` path), independent of whether that scan becomes a new
// submap -- the degeneracy signal is a property of the frontend's ICP solve,
// not of the backend's submap-distance decision.
//
// Format (`%.17g` for every float, the same round-trip-safe precision
// `graph_slam_offline_runner.cpp`'s `loop_edges.csv` already uses):
//   stamp_sec,diagnostics_available,
//   eigenvalue_0..eigenvalue_5 (ascending),
//   category_0..category_5 (ascending, matching the eigenvalue columns),
//   well_conditioned_count,degenerate_count,non_observable_count,
//   condition_number,
//   eigenvector_0_{tx,ty,tz,rx,ry,rz} .. eigenvector_5_{tx,ty,tz,rx,ry,rz}
//   (each direction's sign-canonicalized eigenvector, ascending-eigenvalue
//   order matching the eigenvalue/category columns, twist order
//   [translation, rotation]). Together with the six eigenvalues these
//   reconstruct the full H eigenstructure per scan, which is what
//   identifies *which* physical direction is ill-constrained -- e.g. the
//   along-corridor translation axis on the HILTI exp07 substrate, the
//   Phase 1 gate's direction-identification evidence in
//   docs/roadmap/v0.8.md §5 -- and lets thresholds be re-swept offline
//   without re-running the substrate.
// When `diagnostics_available` is `0` (fallback / unpopulated covariance,
// see `odometry_covariance_localizability.hpp`), every remaining column is
// empty rather than a fabricated `0`/`WELL_CONDITIONED` value, so a
// downstream consumer cannot silently mistake "no diagnostics this scan"
// for "well-conditioned this scan".

#include <array>
#include <cstdio>
#include <string>

#include "graph_based_slam/localizability_analysis.hpp"
#include "graph_based_slam/odometry_covariance_localizability.hpp"

namespace graphslam
{
namespace degeneracy
{

/// Column header, one line, no trailing newline (caller appends `"\n"`,
/// matching `graph_slam_offline_runner.cpp`'s existing CSV writers).
inline std::string degeneracyDiagnosticsCsvHeaderLine()
{
  std::string header =
    "stamp_sec,diagnostics_available,"
    "eigenvalue_0,eigenvalue_1,eigenvalue_2,eigenvalue_3,eigenvalue_4,eigenvalue_5,"
    "category_0,category_1,category_2,category_3,category_4,category_5,"
    "well_conditioned_count,degenerate_count,non_observable_count,condition_number";
  const char * axes[6] = {"tx", "ty", "tz", "rx", "ry", "rz"};
  for (int i = 0; i < 6; ++i) {
    for (int a = 0; a < 6; ++a) {
      header += ",eigenvector_" + std::to_string(i) + "_" + axes[a];
    }
  }
  return header;
}

namespace detail
{

inline const char * categoryName(LocalizabilityCategory category)
{
  switch (category) {
    case LocalizabilityCategory::WELL_CONDITIONED:
      return "WELL_CONDITIONED";
    case LocalizabilityCategory::DEGENERATE:
      return "DEGENERATE";
    case LocalizabilityCategory::NON_OBSERVABLE:
      return "NON_OBSERVABLE";
  }
  return "UNKNOWN";
}

inline std::string formatDouble(double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  return std::string(buffer);
}

}  // namespace detail

/// One data row, no trailing newline. `stamp_sec` is the odometry message's
/// header stamp in seconds (caller's responsibility to convert).
inline std::string degeneracyDiagnosticsCsvRowLine(
  double stamp_sec,
  const CovarianceLocalizabilityResult & result)
{
  std::string line = detail::formatDouble(stamp_sec) + "," +
    (result.diagnostics_available ? "1" : "0");

  if (!result.diagnostics_available) {
    // 6 eigenvalue columns + 6 category columns + 4 summary columns + 36
    // eigenvector columns = 52 empty fields, keeping every row the same
    // column count for a plain CSV reader (pandas/csv.reader) regardless
    // of availability.
    for (int i = 0; i < 52; ++i) {
      line += ",";
    }
    return line;
  }

  const LocalizabilityReport & report = result.report;
  for (int i = 0; i < 6; ++i) {
    line += "," + detail::formatDouble(report.directions[i].eigenvalue);
  }
  for (int i = 0; i < 6; ++i) {
    line += ",";
    line += detail::categoryName(report.directions[i].category);
  }
  line += "," + std::to_string(report.well_conditioned_count);
  line += "," + std::to_string(report.degenerate_count);
  line += "," + std::to_string(report.non_observable_count);
  line += "," + detail::formatDouble(report.condition_number);
  for (int i = 0; i < 6; ++i) {
    for (int a = 0; a < 6; ++a) {
      line += "," + detail::formatDouble(report.directions[i].eigenvector(a));
    }
  }
  return line;
}

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__DEGENERACY_DIAGNOSTICS_CSV_HPP_
