// Copyright 2007, 2009, 2010, 2012, 2019 Austin Robot Technology, Patrick Beeson, Jack O'Quin, Kaarta Inc, Shawn Hanna, Joshua Whitley  // NOLINT
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
#include <memory>
#include <string>
#include <cassert>

#include "velodyne_pointcloud/datacontainerbase.hpp"
#include "velodyne_pointcloud/rawdata.hpp"

namespace velodyne_rawdata
{

inline float square(float val)
{
  return val * val;
}

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData(
  const velodyne_pointcloud::CalibrationDocument & calibration_document,
  const std::string & model)
{
  calibration_ = std::make_unique<velodyne_pointcloud::Calibration>(calibration_document);
  config_.model = model;
  buildTimings();

  // RCLCPP_INFO(this->get_logger(), "Number of lasers: %d.",  calibration_->num_lasers);

  setupSinCosCache();
  setupAzimuthCache();
}

/** Update parameters: conversions and update */
void RawData::setParameters(
  double min_range, double max_range, double view_direction, double view_width)
{
  config_.min_range = min_range;
  config_.max_range = max_range;

  // converting angle parameters into the velodyne reference (rad)
  double tmp_min_angle = view_direction + view_width / 2;
  double tmp_max_angle = view_direction - view_width / 2;

  // computing positive modulo to keep theses angles into [0;2*M_PI]
  tmp_min_angle = ::fmod(::fmod(tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  tmp_max_angle = ::fmod(::fmod(tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  // converting into the hardware velodyne ref (negative yaml and degrees)
  // adding 0.5 performs a centered double to int conversion
  config_.min_angle = std::lround(100.0 * (2.0 * M_PI - tmp_min_angle) * 180.0 / M_PI);
  config_.max_angle = std::lround(100.0 * (2.0 * M_PI - tmp_max_angle) * 180.0 / M_PI);

  if (config_.min_angle == config_.max_angle) {
    // avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

int RawData::scansPerPacket() const
{
  if (calibration_->num_lasers == 16) {
    return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK *
           VLP16_SCANS_PER_FIRING;
  }
  return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
}

/**
 * Build a timing table for each block/firing. Stores in timing_offsets vector
 */
bool RawData::buildTimings()
{
  // vlp16
  if (config_.model == "VLP16") {
    // timing table calculation, from velodyne user manual
    timing_offsets_.resize(12);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6;   // seconds
    double single_firing = 2.304 * 1e-6;   // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = (x - (x % 2)) + (y / 16);
        } else {
          dataBlockIndex = (x * 2) + (y / 16);
        }
        dataPointIndex = y % 16;
        // timing_offsets[block][firing]
        timing_offsets_[x][y] = (full_firing_cycle * dataBlockIndex) +
          (single_firing * dataPointIndex);
      }
    }
    // vlp32
  } else if (config_.model == "32C") {
    // timing table calculation, from velodyne user manual
    timing_offsets_.resize(12);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(32);
    }
    // constants
    double full_firing_cycle = 55.296 * 1e-6;   // seconds
    double single_firing = 2.304 * 1e-6;   // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = x / 2;
        } else {
          dataBlockIndex = x;
        }
        dataPointIndex = y / 2;
        timing_offsets_[x][y] = (full_firing_cycle * dataBlockIndex) +
          (single_firing * dataPointIndex);
      }
    }
    // hdl32
  } else if (config_.model == "32E") {
    // timing table calculation, from velodyne user manual
    timing_offsets_.resize(12);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(32);
    }
    // constants
    double full_firing_cycle = 46.080 * 1e-6;   // seconds
    double single_firing = 1.152 * 1e-6;   // seconds
    double dataBlockIndex, dataPointIndex;
    bool dual_mode = false;
    // compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        if (dual_mode) {
          dataBlockIndex = x / 2;
        } else {
          dataBlockIndex = x;
        }
        dataPointIndex = y / 2;
        timing_offsets_[x][y] = (full_firing_cycle * dataBlockIndex) +
          (single_firing * dataPointIndex);
      }
    }
  } else if (config_.model == "VLS128") {
    timing_offsets_.resize(3);
    for (size_t i = 0; i < timing_offsets_.size(); ++i) {
      timing_offsets_[i].resize(17);  // 17 (+1 for the maintenance time after firing group 8)
    }

    double full_firing_cycle = VLS128_SEQ_TDURATION * 1e-6;   // seconds
    double single_firing = VLS128_CHANNEL_TDURATION * 1e-6;   // seconds
    double offset_paket_time = VLS128_TOH_ADJUSTMENT * 1e-6;  // seconds
    double sequenceIndex, firingGroupIndex;
    // Compute timing offsets
    for (size_t x = 0; x < timing_offsets_.size(); ++x) {
      for (size_t y = 0; y < timing_offsets_[x].size(); ++y) {
        sequenceIndex = x;
        firingGroupIndex = y;
        timing_offsets_[x][y] =
          (full_firing_cycle * sequenceIndex) + (single_firing * firingGroupIndex) -
          offset_paket_time;
        RCLCPP_DEBUG(
          rclcpp::get_logger("velodyne_pointcloud"),
          "firing_seque %lu firing_group %lu offset %f", x, y, timing_offsets_[x][y]);
      }
    }
  } else {
    timing_offsets_.clear();
    RCLCPP_WARN(
      rclcpp::get_logger("velodyne_pointcloud"),
      "Timings not supported for model %s", config_.model.c_str());
  }

  if (timing_offsets_.size()) {
    // lidarslam: the timing table is not printed to stdout.
    return true;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("velodyne_pointcloud"),
      "NO TIMING OFFSETS CALCULATED. ARE YOU USING A SUPPORTED VELODYNE SENSOR?");
  }
  return false;
}

int RawData::numLasers() const
{
  return calibration_->num_lasers;
}

void RawData::setupSinCosCache()
{
  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = ::cosf(rotation);
    sin_rot_table_[rot_index] = ::sinf(rotation);
  }
}

void RawData::setupAzimuthCache()
{
  if (config_.model == "VLS128") {
    for (uint8_t i = 0; i < 16; i++) {
      vls_128_laser_azimuth_cache_[i] =
        (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
    }
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("velodyne_pointcloud"),
      "No Azimuth Cache configured for model %s", config_.model.c_str());
  }
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(
  const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data,
  const rclcpp::Time & scan_start_time)
{
  // RCLCPP_DEBUG(this->get_logger(), "Received packet, time: %s", pkt.stamp);

  /** special parsing for the VLS128 **/
  if (pkt.data[1205] == VLS128_MODEL_ID) {  // VLS 128
    unpack_vls128(pkt, data, scan_start_time);
    return;
  }

  /** special parsing for the VLP16 **/
  if (calibration_->num_lasers == 16) {
    unpack_vlp16(pkt, data, scan_start_time);
    return;
  }

  float time_diff_start_to_this_packet =
    (rclcpp::Time(pkt.stamp) - scan_start_time).seconds();

  const raw_packet * raw = reinterpret_cast<const raw_packet *>(&pkt.data[0]);

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    // upper bank lasers are numbered [0..31]
    // NOTE: this is a change from the old velodyne_common implementation

    int bank_origin = 0;

    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      float x, y, z;
      float intensity;
      const uint8_t laser_number = j + bank_origin;

      const velodyne_pointcloud::LaserCorrection & corrections =
        calibration_->laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block & block = raw->blocks[i];
      union two_bytes tmp{};
      tmp.bytes[0] = block.data[k];
      tmp.bytes[1] = block.data[k + 1];

      /*condition added to avoid calculating points which are not
        in the interesting defined area (min_angle < area < max_angle)*/
      if ((config_.min_angle < config_.max_angle &&
        block.rotation >= config_.min_angle &&
        block.rotation <= config_.max_angle) ||
        (config_.min_angle > config_.max_angle &&
        (block.rotation <= config_.max_angle ||
        block.rotation >= config_.min_angle)))
      {
        float time = 0;
        if (timing_offsets_.size()) {
          time = timing_offsets_[i][j] + time_diff_start_to_this_packet;
        }

        if (tmp.uint == 0) {  // no valid laser beam return
          // call to addPoint is still required since output could be organized
          data.addPoint(
            nanf(""), nanf(""), nanf(""), corrections.laser_ring,
            nanf(""), nanf(""), time);
          continue;
        }

        float distance = tmp.uint * calibration_->distance_resolution_m;
        distance += corrections.dist_correction;

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
          cos_rot_table_[block.rotation] * cos_rot_correction +
          sin_rot_table_[block.rotation] * sin_rot_correction;
        float sin_rot_angle =
          sin_rot_table_[block.rotation] * cos_rot_correction -
          cos_rot_table_[block.rotation] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        if (xx < 0.0f) {
          xx = -xx;
        }

        if (yy < 0.0f) {
          yy = -yy;
        }

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;

        if (corrections.two_pt_correction_available) {
          distance_corr_x =
            (corrections.dist_correction - corrections.dist_correction_x) *
            (xx - 2.4f) / (25.04f - 2.4f) +
            corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y =
            (corrections.dist_correction - corrections.dist_correction_y) *
            (yy - 1.93f) / (25.04f - 1.93f) +
            corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
        // the expression wiht '-' is proved to be better than the one with '+'
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */

        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[i].data[k + 2];

        float focal_offset = 256.0f *
          (1.0f - corrections.focal_distance / 13100.0f) *
          (1.0f - corrections.focal_distance / 13100.0f);
        float focal_slope = corrections.focal_slope;
        intensity += focal_slope *
          (std::abs(focal_offset - 256.0f * square((1.0f - distance) / 65535.0f)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        data.addPoint(
          x_coord, y_coord, z_coord, corrections.laser_ring,
          distance, intensity, time);
      }
    }

    data.newLine();
  }
}

/** @brief convert raw VLS128 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack_vls128(
  const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data,
  const rclcpp::Time & scan_start_time)
{
  const raw_packet * raw = reinterpret_cast<const raw_packet *>(&pkt.data[0]);

  bool dual_return = (pkt.data[1204] == 57);
  for (int block = 0; block < BLOCKS_PER_PACKET - (4 * dual_return); block++) {
    // cache block for use
    const raw_block & current_block = raw->blocks[block];

    int bank_origin = 0;
    // Used to detect which bank of 32 lasers is in this block
    switch (current_block.header) {
      case VLS128_BANK_1:
        bank_origin = 0;
        break;
      case VLS128_BANK_2:
        bank_origin = 32;
        break;
      case VLS128_BANK_3:
        bank_origin = 64;
        break;
      case VLS128_BANK_4:
        bank_origin = 96;
        break;
      default:
        // ignore packets with mangled or otherwise different contents
        // Do not flood the log with messages, only issue at most one
        // of these warnings per minute.
        auto clock = rclcpp::Clock{};
        RCLCPP_WARN_STREAM_THROTTLE(
          rclcpp::get_logger("velodyne_pointcloud"), clock, 60000,
          "skipping invalid VLS-128 packet: block " <<
            block << " header value is " <<
            raw->blocks[block].header);
        return;  // bad packet: skip the rest
    }

    uint16_t azimuth{};        // Uniform initializion. equivalent to azimuth = 0;
    uint16_t azimuth_next{};
    float azimuth_diff{};
    // Calculate difference between current and next block's azimuth angle.
    if (!dual_return) {
      switch (block) {
        // 1 firing sequence span across 4 data block, causing them to have the same azimuth
        case 0:
        case 1:
        case 2:
        case 3:
          azimuth = raw->blocks[0].rotation;
          azimuth_next = raw->blocks[4].rotation;
          break;
        case 4:
        case 5:
        case 6:
        case 7:
        // We assume that the difference between 3rd firing sequence and the next packet firing
        // sequence is the same as difference between 2nd and 3rd firing sequence.
        case 8:
        case 9:
        case 10:
        case 11:
          azimuth = raw->blocks[4].rotation;
          azimuth_next = raw->blocks[8].rotation;
          break;
        default:
          assert(false && "block should be between 0 and BLOCK_PER_PACKET(12)");
          break;
      }
      azimuth_diff = static_cast<float>((36000 + azimuth_next - azimuth) % 36000);
    } else {
      // We can't calculate for dual return, since it have 1 azimuth per data packet and
      // we don't have access to the next packet.
      azimuth = raw->blocks[0].rotation;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      // Offset the laser in this block by which block it's in
      uint8_t laser_number = j + bank_origin;
      // VLS-128 fires 8 lasers at a time
      uint8_t firing_order = laser_number / 8;

      float time_diff_start_to_this_packet = (rclcpp::Time(pkt.stamp) - scan_start_time).seconds();
      float time {};
      if (timing_offsets_.size()) {
        time = timing_offsets_[block / 4][firing_order + laser_number / 64] +
          time_diff_start_to_this_packet;
      }

      velodyne_pointcloud::LaserCorrection & corrections =
        calibration_->laser_corrections[laser_number];

      // distance extraction
      union two_bytes tmp;
      tmp.bytes[0] = current_block.data[k];
      tmp.bytes[1] = current_block.data[k + 1];
      float distance = tmp.uint * VLS128_DISTANCE_RESOLUTION;

      // correct for the laser rotation as a function of timing during the firings
      float azimuth_corrected_f = azimuth +
        (azimuth_diff * vls_128_laser_azimuth_cache_[firing_order]);
      uint16_t azimuth_corrected = ((uint16_t) round(azimuth_corrected_f)) % 36000;

      // condition added to avoid calculating points
      // which are not in the interesting defined area (min_angle < area < max_angle)
      // ((Confusing ! Are min_angle and max_angle included or not ?))
      if (((config_.min_angle < config_.max_angle &&
        azimuth_corrected >= config_.min_angle &&
        azimuth_corrected <= config_.max_angle) ||
        (config_.min_angle > config_.max_angle)) &&
        pointInRange(distance))
      {
        // convert polar coordinates to Euclidean XYZ
        // Each laser has an vertical offset and
        // an horizontal offset in addition to azimut
        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
          cos_rot_table_[azimuth_corrected] * cos_rot_correction +
          sin_rot_table_[azimuth_corrected] * sin_rot_correction;
        float sin_rot_angle =
          sin_rot_table_[azimuth_corrected] * cos_rot_correction -
          cos_rot_table_[azimuth_corrected] * sin_rot_correction;

        // (TODO) Improve calculation.
        // We can see different calculation in the implementation of other sensors.
        // This implementation is in agreement (if no error was made) with VLS128 manuel,
        // But it might not be the case with physics.

        // The velodyne VLS-128 manual user show this calcultations.
        float x = distance * cos_vert_angle * sin_rot_angle;
        float y = distance * cos_vert_angle * cos_rot_angle;
        float z = distance * sin_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        data.addPoint(
          x_coord,
          y_coord,
          z_coord,
          corrections.laser_ring,
          // azimuth_corrected,
          distance,
          current_block.data[k + 2],  // intensity
          time);
      } else {
        // call to addPoint is still required since output could be organized
        data.addPoint(
          nanf(""), nanf(""), nanf(""), corrections.laser_ring, nanf(""), nanf(""), time);
      }
    }

    if (current_block.header == VLS128_BANK_4) {
      // add a new line only after the last bank (VLS128_BANK_4)
      data.newLine();
    }
  }
}

/** @brief convert raw VLP16 packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack_vlp16(
  const velodyne_msgs::msg::VelodynePacket & pkt, DataContainerBase & data,
  const rclcpp::Time & scan_start_time)
{
  float azimuth;
  float azimuth_diff;
  int raw_azimuth_diff;
  float last_azimuth_diff = 0.0f;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  float time_diff_start_to_this_packet =
    (rclcpp::Time(pkt.stamp) - scan_start_time).seconds();

  const raw_packet * raw = reinterpret_cast<const raw_packet *>(&pkt.data[0]);

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header) {
      return;  // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = static_cast<float>(raw->blocks[block].rotation);

    if (block < (BLOCKS_PER_PACKET - 1)) {
      raw_azimuth_diff = raw->blocks[block + 1].rotation - raw->blocks[block].rotation;
      azimuth_diff = static_cast<float>((36000 + raw_azimuth_diff) % 36000);

      // some packets contain an angle overflow where azimuth_diff < 0
      if (raw_azimuth_diff < 0) {
        // raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
        // RCLCPP_WARN(
        //   get_logger(), "Packet containing angle overflow, first angle: %u second angle: %u",
        //   raw->blocks[block].rotation, raw->blocks[block+1].rotation);
        // if last_azimuth_diff was not zero, we can assume that the velodyne's
        // speed did not change very much and use the same difference
        if (last_azimuth_diff > 0) {
          azimuth_diff = last_azimuth_diff;
        } else {
          // otherwise we are not able to use this data
          // TODO(somebody): we might just not use the second 16 firings
          continue;
        }
      }

      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {
        velodyne_pointcloud::LaserCorrection & corrections = calibration_->laser_corrections[dsr];

        /** Position Calculation */
        union two_bytes tmp{};
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];

        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f =
          azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) +
          (firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
        azimuth_corrected = (static_cast<int>(std::round(azimuth_corrected_f)) % 36000);

        /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
        if ((azimuth_corrected >= config_.min_angle &&
          azimuth_corrected <= config_.max_angle &&
          config_.min_angle < config_.max_angle) ||
          (config_.min_angle > config_.max_angle &&
          (azimuth_corrected <= config_.max_angle ||
          azimuth_corrected >= config_.min_angle)))
        {
          // convert polar coordinates to Euclidean XYZ
          float distance = tmp.uint * calibration_->distance_resolution_m;
          distance += corrections.dist_correction;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle =
            cos_rot_table_[azimuth_corrected] * cos_rot_correction +
            sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle =
            sin_rot_table_[azimuth_corrected] * cos_rot_correction -
            cos_rot_table_[azimuth_corrected] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          if (xx < 0.0f) {
            xx = -xx;
          }

          if (yy < 0.0f) {
            yy = -yy;
          }

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;

          if (corrections.two_pt_correction_available) {
            distance_corr_x =
              (corrections.dist_correction - corrections.dist_correction_x) *
              (xx - 2.4f) / (25.04f - 2.4f) + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y =
              (corrections.dist_correction - corrections.dist_correction_y) *
              (yy - 1.93f) / (25.04f - 1.93f) + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

          /** Intensity Calculation */
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[block].data[k + 2];

          float focal_offset = 256.0f * square(1.0f - corrections.focal_distance / 13100.0f);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope *
            (std::abs(focal_offset - 256.0f * square((1.0f - distance) / 65535.0f)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          float time = 0;
          if (timing_offsets_.size()) {
            time = timing_offsets_[block][firing * 16 + dsr] + time_diff_start_to_this_packet;
          }

          data.addPoint(
            x_coord, y_coord, z_coord, corrections.laser_ring,
            distance, intensity, time);
        }
      }

      data.newLine();
    }
  }
}
}  // namespace velodyne_rawdata
