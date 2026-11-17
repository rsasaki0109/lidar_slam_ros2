// Copyright 2012, 2019 Austin Robot Technology, Piyush Khandelwal, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include <stdexcept>
#include <limits>
#include <map>
#include <string>
#include <utility>

#include "velodyne_pointcloud/calibration.hpp"

namespace velodyne_pointcloud
{

constexpr char NUM_LASERS[] = "num_lasers";
constexpr char DISTANCE_RESOLUTION[] = "distance_resolution";
constexpr char LASERS[] = "lasers";
constexpr char LASER_ID[] = "laser_id";
constexpr char ROT_CORRECTION[] = "rot_correction";
constexpr char VERT_CORRECTION[] = "vert_correction";
constexpr char DIST_CORRECTION[] = "dist_correction";
constexpr char TWO_PT_CORRECTION_AVAILABLE[] =
  "two_pt_correction_available";
constexpr char DIST_CORRECTION_X[] = "dist_correction_x";
constexpr char DIST_CORRECTION_Y[] = "dist_correction_y";
constexpr char VERT_OFFSET_CORRECTION[] = "vert_offset_correction";
constexpr char HORIZ_OFFSET_CORRECTION[] = "horiz_offset_correction";
constexpr char MAX_INTENSITY[] = "max_intensity";
constexpr char MIN_INTENSITY[] = "min_intensity";
constexpr char FOCAL_DISTANCE[] = "focal_distance";
constexpr char FOCAL_SLOPE[] = "focal_slope";

/** Read calibration for a single laser. */
// lidarslam: yaml-cpp's node[KEY] >> field becomes yaml_scalar::as<T>(require(node, KEY)), and
// `if (node[KEY])` becomes find(node, KEY); the order of the reads and the arithmetic are unchanged.
void readLaser(
  const std::map<std::string, std::string> & node, std::pair<int, LaserCorrection> & correction)
{
  using yaml_scalar::as;
  using yaml_scalar::find;
  using yaml_scalar::require;
  correction.first = as<int>(require(node, LASER_ID), LASER_ID);
  correction.second.rot_correction = as<float>(require(node, ROT_CORRECTION), ROT_CORRECTION);
  correction.second.vert_correction = as<float>(require(node, VERT_CORRECTION), VERT_CORRECTION);
  correction.second.dist_correction = as<float>(require(node, DIST_CORRECTION), DIST_CORRECTION);

  if (const auto value = find(node, TWO_PT_CORRECTION_AVAILABLE)) {
    correction.second.two_pt_correction_available =
      yaml_scalar::as_bool(*value, TWO_PT_CORRECTION_AVAILABLE);
  } else {
    correction.second.two_pt_correction_available = false;
  }

  correction.second.dist_correction_x =
    as<float>(require(node, DIST_CORRECTION_X), DIST_CORRECTION_X);
  correction.second.dist_correction_y =
    as<float>(require(node, DIST_CORRECTION_Y), DIST_CORRECTION_Y);
  correction.second.vert_offset_correction =
    as<float>(require(node, VERT_OFFSET_CORRECTION), VERT_OFFSET_CORRECTION);

  if (const auto value = find(node, HORIZ_OFFSET_CORRECTION)) {
    correction.second.horiz_offset_correction = as<float>(*value, HORIZ_OFFSET_CORRECTION);
  } else {
    correction.second.horiz_offset_correction = 0;
  }

  float max_intensity_float = 255.0;

  if (const auto value = find(node, MAX_INTENSITY)) {
    max_intensity_float = as<float>(*value, MAX_INTENSITY);
  }

  correction.second.max_intensity = ::floorf(max_intensity_float);

  float min_intensity_float = 0.0;

  if (const auto value = find(node, MIN_INTENSITY)) {
    min_intensity_float = as<float>(*value, MIN_INTENSITY);
  }

  correction.second.min_intensity = ::floorf(min_intensity_float);

  correction.second.focal_distance = as<float>(require(node, FOCAL_DISTANCE), FOCAL_DISTANCE);
  correction.second.focal_slope = as<float>(require(node, FOCAL_SLOPE), FOCAL_SLOPE);

  // Calculate cached values
  correction.second.cos_rot_correction =
    ::cosf(correction.second.rot_correction);
  correction.second.sin_rot_correction =
    ::sinf(correction.second.rot_correction);
  correction.second.cos_vert_correction =
    ::cosf(correction.second.vert_correction);
  correction.second.sin_vert_correction =
    ::sinf(correction.second.vert_correction);

  correction.second.laser_ring = 0;   // clear initially (set later)
}

/** Read entire calibration file. */
void readCalibration(const CalibrationDocument & document, Calibration & calibration)
{
  const int num_lasers =
    yaml_scalar::as<int>(yaml_scalar::require(document.scalars, NUM_LASERS), NUM_LASERS);
  const float distance_resolution_m = yaml_scalar::as<float>(
    yaml_scalar::require(document.scalars, DISTANCE_RESOLUTION), DISTANCE_RESOLUTION);
  const auto & lasers = document.lasers;
  calibration.laser_corrections.clear();
  calibration.num_lasers = num_lasers;
  calibration.distance_resolution_m = distance_resolution_m;
  calibration.laser_corrections.resize(num_lasers);

  for (int i = 0; i < num_lasers; i++) {
    std::pair<int, LaserCorrection> correction;
    if (static_cast<size_t>(i) >= lasers.size()) {
      throw std::runtime_error("calibration: fewer lasers than num_lasers");
    }
    readLaser(lasers[i], correction);
    const int index = correction.first;

    if (static_cast<size_t>(index) >= calibration.laser_corrections.size()) {
      calibration.laser_corrections.resize(index + 1);
    }

    calibration.laser_corrections[index] = (correction.second);
  }

  // For each laser ring, find the next-smallest vertical angle.
  //
  // This implementation is simple, but not efficient.  That is OK,
  // since it only runs while starting up.
  double next_angle = -std::numeric_limits<double>::infinity();

  for (int ring = 0; ring < num_lasers; ++ring) {
    // find minimum remaining vertical offset correction
    double min_seen = std::numeric_limits<double>::infinity();
    int next_index = num_lasers;

    for (int j = 0; j < num_lasers; ++j) {
      double angle = calibration.laser_corrections[j].vert_correction;

      if (next_angle < angle && angle < min_seen) {
        min_seen = angle;
        next_index = j;
      }
    }

    if (next_index < num_lasers) {  // anything found in this ring?
      // store this ring number with its corresponding laser number
      calibration.laser_corrections[next_index].laser_ring = ring;
      next_angle = min_seen;
    }
  }
}

Calibration::Calibration(const CalibrationDocument & calibration_document)
: distance_resolution_m(0.002f)
{
  readCalibration(calibration_document, *this);
}

}  // namespace velodyne_pointcloud
