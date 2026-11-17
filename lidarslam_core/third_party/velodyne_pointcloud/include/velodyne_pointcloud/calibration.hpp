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

#ifndef VELODYNE_POINTCLOUD__CALIBRATION_HPP_
#define VELODYNE_POINTCLOUD__CALIBRATION_HPP_

#include <map>
#include <string>
#include <vector>

#include "velodyne_pointcloud/ros_free.hpp"

namespace velodyne_pointcloud
{

/** \brief correction values for a single laser
 *
 * Correction values for a single laser (as provided by db.xml from
 * Velodyne).  Includes parameters for Velodyne HDL-64E S2.1.
 *
 * http://velodynelidar.com/lidar/products/manual/63-HDL64E%20S2%20Manual_Rev%20D_2011_web.pdf
 */

/** \brief Correction information for a single laser. */
struct LaserCorrection
{
  /** parameters in db.xml */
  float rot_correction;
  float vert_correction;
  float dist_correction;
  bool two_pt_correction_available;
  float dist_correction_x;
  float dist_correction_y;
  float vert_offset_correction;
  float horiz_offset_correction;
  int max_intensity;
  int min_intensity;
  float focal_distance;
  float focal_slope;

  /** cached values calculated when the calibration file is read */
  float cos_rot_correction;              ///< cosine of rot_correction
  float sin_rot_correction;              ///< sine of rot_correction
  float cos_vert_correction;             ///< cosine of vert_correction
  float sin_vert_correction;             ///< sine of vert_correction

  int laser_ring;                        ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class Calibration final
{
public:
  explicit Calibration(const CalibrationDocument & calibration_document);

  float distance_resolution_m;
  std::vector<LaserCorrection> laser_corrections;
  int num_lasers{};
};

}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD__CALIBRATION_HPP_
