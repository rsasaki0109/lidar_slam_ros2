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

#ifndef GRAPH_BASED_SLAM__MAP_THICKNESS_ATTRIBUTION_CSV_HPP_
#define GRAPH_BASED_SLAM__MAP_THICKNESS_ATTRIBUTION_CSV_HPP_

#include <cmath>
#include <cstdint>
#include <istream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "graph_based_slam/map_thickness_attribution.hpp"

namespace graphslam
{
namespace map_thickness
{

inline const char * attributedPointCsvHeader()
{
  return "x,y,z,scan_id,submap_id,revisit_id";
}

namespace detail
{

inline std::vector<std::string> splitCsvFields(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

inline double parseFiniteDouble(const std::string & field, const std::size_t row)
{
  std::size_t consumed = 0U;
  double value = 0.0;
  try {
    value = std::stod(field, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error("row " + std::to_string(row) + ": invalid floating-point field");
  }
  if (consumed != field.size() || !std::isfinite(value)) {
    throw std::runtime_error("row " + std::to_string(row) + ": non-finite or trailing data");
  }
  return value;
}

inline std::int64_t parseInt64(const std::string & field, const std::size_t row)
{
  std::size_t consumed = 0U;
  std::int64_t value = 0;
  try {
    value = std::stoll(field, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error("row " + std::to_string(row) + ": invalid integer field");
  }
  if (consumed != field.size()) {
    throw std::runtime_error("row " + std::to_string(row) + ": trailing integer data");
  }
  return value;
}

}  // namespace detail

inline std::vector<AttributedPoint> readAttributedPointCsv(std::istream & input)
{
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("attributed-point CSV is empty");
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != attributedPointCsvHeader()) {
    throw std::runtime_error(
            "unexpected CSV header; expected: " + std::string(attributedPointCsvHeader()));
  }

  std::vector<AttributedPoint> points;
  std::size_t row = 1U;
  while (std::getline(input, line)) {
    ++row;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> fields = detail::splitCsvFields(line);
    if (fields.size() != 6U) {
      throw std::runtime_error("row " + std::to_string(row) + ": expected 6 CSV fields");
    }

    AttributedPoint point;
    point.position = Eigen::Vector3d(
      detail::parseFiniteDouble(fields[0], row),
      detail::parseFiniteDouble(fields[1], row),
      detail::parseFiniteDouble(fields[2], row));
    point.scan_id = detail::parseInt64(fields[3], row);
    point.submap_id = detail::parseInt64(fields[4], row);
    point.revisit_id = detail::parseInt64(fields[5], row);
    points.push_back(point);
  }
  return points;
}

}  // namespace map_thickness
}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__MAP_THICKNESS_ATTRIBUTION_CSV_HPP_
