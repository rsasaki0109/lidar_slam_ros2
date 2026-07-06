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

#ifndef GRAPH_BASED_SLAM__DEGENERACY_REPORT_SUMMARY_HPP_
#define GRAPH_BASED_SLAM__DEGENERACY_REPORT_SUMMARY_HPP_

// Pure header: streaming summary of a run's per-scan degeneracy
// classifications into the "degeneracy report" the map bundle writes at
// `/map_save` time (docs/roadmap/v0.8.md §5 Phase 1, the v0.7 §6 stretch
// item). Consumes exactly the same `CovarianceLocalizabilityResult` the
// per-scan CSV (`degeneracy_diagnostics_csv.hpp`) writes, so the two opt-in
// outputs are always mutually consistent (same classifier, same input,
// computed once per scan).
//
// Streaming (`O(1)` memory per scan, not `O(n)`) rather than
// batch-over-a-stored-vector: a live run can be arbitrarily long (HILTI
// exp01 alone is a 904 m / many-thousand-scan sweep), and the report is only
// needed once, at `/map_save` time, not the full per-scan history (that's
// what the CSV is for).
//
// "Worst interval": the longest contiguous run of scans whose per-scan worst
// direction category is DEGENERATE or NON_OBSERVABLE (i.e. not fully
// WELL_CONDITIONED), reported so a reader can jump straight to e.g. "frames
// 120-340 of the corridor sweep" instead of scanning the whole CSV. A scan
// with no diagnostics available (`odometry_covariance_localizability.hpp`'s
// fallback case) breaks a run -- conservative by construction, since "no
// diagnostics" is not evidence either way.

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "graph_based_slam/localizability_analysis.hpp"
#include "graph_based_slam/odometry_covariance_localizability.hpp"

namespace graphslam
{
namespace degeneracy
{

/// The single worst (most ill-constrained) category present across a
/// report's six per-direction categories: NON_OBSERVABLE > DEGENERATE >
/// WELL_CONDITIONED. Used to reduce a per-scan 6-direction report to one
/// scan-level label for rate/interval bookkeeping.
inline LocalizabilityCategory worstCategory(const LocalizabilityReport & report)
{
  if (report.non_observable_count > 0) {
    return LocalizabilityCategory::NON_OBSERVABLE;
  }
  if (report.degenerate_count > 0) {
    return LocalizabilityCategory::DEGENERATE;
  }
  return LocalizabilityCategory::WELL_CONDITIONED;
}

/// A contiguous run of not-fully-well-conditioned scans.
struct DegeneracyWorstInterval
{
  bool valid {false};
  double start_stamp_sec {0.0};
  double end_stamp_sec {0.0};
  std::size_t length_scans {0};
  /// The worst category observed anywhere inside the interval (escalates to
  /// NON_OBSERVABLE if any scan in the run was NON_OBSERVABLE, else
  /// DEGENERATE -- a run is never reported as WELL_CONDITIONED, that is not
  /// a "worst interval" candidate at all).
  LocalizabilityCategory category {LocalizabilityCategory::DEGENERATE};
};

/// Whole-run degeneracy classification summary.
struct DegeneracyReportSummary
{
  std::size_t total_scans {0};
  std::size_t diagnostics_available_scans {0};
  std::size_t well_conditioned_scans {0};
  std::size_t degenerate_scans {0};
  std::size_t non_observable_scans {0};
  DegeneracyWorstInterval worst_interval;

  /// Fraction of `total_scans` that had a usable covariance-derived
  /// classification at all. 0.0 when `total_scans == 0`.
  double diagnosticsAvailableRatio() const
  {
    if (total_scans == 0) {return 0.0;}
    return static_cast<double>(diagnostics_available_scans) / static_cast<double>(total_scans);
  }

  /// The three category rates below are relative to
  /// `diagnostics_available_scans` (not `total_scans`): a run with mostly
  /// unavailable diagnostics should not silently report near-zero rates for
  /// every category. 0.0 when no scan had diagnostics available.
  double wellConditionedRatio() const {return categoryRatio(well_conditioned_scans);}
  double degenerateRatio() const {return categoryRatio(degenerate_scans);}
  double nonObservableRatio() const {return categoryRatio(non_observable_scans);}

private:
  double categoryRatio(std::size_t count) const
  {
    if (diagnostics_available_scans == 0) {return 0.0;}
    return static_cast<double>(count) / static_cast<double>(diagnostics_available_scans);
  }
};

namespace detail
{

/// Keep the longer interval; on an exact length tie, keep the more severe
/// one (NON_OBSERVABLE beats DEGENERATE); on a full tie, keep `current`
/// (first-encountered wins, for determinism -- same input stream always
/// picks the same interval regardless of where in a tie it occurs).
inline DegeneracyWorstInterval pickWorseInterval(
  const DegeneracyWorstInterval & current,
  const DegeneracyWorstInterval & candidate)
{
  if (!candidate.valid) {return current;}
  if (!current.valid) {return candidate;}
  if (candidate.length_scans > current.length_scans) {return candidate;}
  if (candidate.length_scans == current.length_scans &&
    candidate.category == LocalizabilityCategory::NON_OBSERVABLE &&
    current.category != LocalizabilityCategory::NON_OBSERVABLE)
  {
    return candidate;
  }
  return current;
}

}  // namespace detail

/// Streaming accumulator: call `add()` once per scan, in timestamp order,
/// then read `summary()` at any point (idempotent, safe mid-stream -- an
/// in-progress run is folded in as a hypothetical candidate without
/// mutating accumulator state, so `/map_save` can call `summary()` even if
/// the run is still ongoing).
class DegeneracyReportAccumulator
{
public:
  void add(double stamp_sec, const CovarianceLocalizabilityResult & result)
  {
    ++summary_.total_scans;

    if (!result.diagnostics_available) {
      closeCurrentRun();
      return;
    }

    ++summary_.diagnostics_available_scans;
    const LocalizabilityCategory worst = worstCategory(result.report);
    switch (worst) {
      case LocalizabilityCategory::WELL_CONDITIONED:
        ++summary_.well_conditioned_scans;
        closeCurrentRun();
        return;
      case LocalizabilityCategory::DEGENERATE:
        ++summary_.degenerate_scans;
        break;
      case LocalizabilityCategory::NON_OBSERVABLE:
        ++summary_.non_observable_scans;
        break;
    }

    if (!current_run_active_) {
      current_run_active_ = true;
      current_run_.valid = true;
      current_run_.start_stamp_sec = stamp_sec;
      current_run_.length_scans = 0;
      current_run_.category = worst;
    } else if (worst == LocalizabilityCategory::NON_OBSERVABLE) {
      current_run_.category = LocalizabilityCategory::NON_OBSERVABLE;
    }
    current_run_.end_stamp_sec = stamp_sec;
    ++current_run_.length_scans;
  }

  DegeneracyReportSummary summary() const
  {
    DegeneracyReportSummary result = summary_;
    result.worst_interval = detail::pickWorseInterval(result.worst_interval, current_run_);
    return result;
  }

private:
  void closeCurrentRun()
  {
    if (!current_run_active_) {return;}
    summary_.worst_interval = detail::pickWorseInterval(summary_.worst_interval, current_run_);
    current_run_active_ = false;
    current_run_ = DegeneracyWorstInterval();
  }

  DegeneracyReportSummary summary_;
  bool current_run_active_ {false};
  DegeneracyWorstInterval current_run_;
};

/// Batch convenience wrapper over the streaming accumulator, for tests and
/// any offline caller that already has every scan in memory.
struct DegeneracyScanSample
{
  double stamp_sec {0.0};
  CovarianceLocalizabilityResult result;
};

inline DegeneracyReportSummary summarizeDegeneracyScans(
  const std::vector<DegeneracyScanSample> & samples)
{
  DegeneracyReportAccumulator accumulator;
  for (const auto & sample : samples) {
    accumulator.add(sample.stamp_sec, sample.result);
  }
  return accumulator.summary();
}

namespace detail
{

inline std::string reportDouble(double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.6f", value);
  return std::string(buffer);
}

inline const char * intervalCategoryName(LocalizabilityCategory category)
{
  switch (category) {
    case LocalizabilityCategory::NON_OBSERVABLE:
      return "NON_OBSERVABLE";
    case LocalizabilityCategory::DEGENERATE:
    case LocalizabilityCategory::WELL_CONDITIONED:
    default:
      return "DEGENERATE";
  }
}

}  // namespace detail

/// Render `summary` as the `degeneracy_report.yaml` bundle artifact
/// (docs/roadmap/v0.8.md §5 Phase 1), one line per element, no trailing
/// newline on any line (matching `map_refiner.hpp::refinerReportYamlLines`'s
/// convention -- the caller joins with `"\n"`).
inline std::vector<std::string> degeneracyReportYamlLines(const DegeneracyReportSummary & summary)
{
  std::vector<std::string> lines;
  lines.push_back("degeneracy_report:");
  lines.push_back("  total_scans: " + std::to_string(summary.total_scans));
  lines.push_back(
    "  diagnostics_available_scans: " +
    std::to_string(summary.diagnostics_available_scans));
  lines.push_back(
    "  diagnostics_available_ratio: " +
    detail::reportDouble(summary.diagnosticsAvailableRatio()));
  lines.push_back("  well_conditioned_scans: " + std::to_string(summary.well_conditioned_scans));
  lines.push_back("  degenerate_scans: " + std::to_string(summary.degenerate_scans));
  lines.push_back(
    "  non_observable_scans: " + std::to_string(summary.non_observable_scans));
  lines.push_back(
    "  well_conditioned_ratio: " + detail::reportDouble(summary.wellConditionedRatio()));
  lines.push_back("  degenerate_ratio: " + detail::reportDouble(summary.degenerateRatio()));
  lines.push_back(
    "  non_observable_ratio: " + detail::reportDouble(summary.nonObservableRatio()));
  lines.push_back("  worst_interval:");
  lines.push_back(
    "    valid: " + std::string(summary.worst_interval.valid ? "true" : "false"));
  if (summary.worst_interval.valid) {
    lines.push_back(
      "    category: " +
      std::string(detail::intervalCategoryName(summary.worst_interval.category)));
    lines.push_back(
      "    start_stamp_sec: " + detail::reportDouble(summary.worst_interval.start_stamp_sec));
    lines.push_back(
      "    end_stamp_sec: " + detail::reportDouble(summary.worst_interval.end_stamp_sec));
    lines.push_back(
      "    length_scans: " + std::to_string(summary.worst_interval.length_scans));
  }
  return lines;
}

}  // namespace degeneracy
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__DEGENERACY_REPORT_SUMMARY_HPP_
