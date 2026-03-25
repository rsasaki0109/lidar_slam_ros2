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

#ifndef GRAPH_BASED_SLAM__THREE_D_BBS_LOOP_VERIFIER_HPP_
#define GRAPH_BASED_SLAM__THREE_D_BBS_LOOP_VERIFIER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace graphslam
{

struct ThreeDBBSLoopVerifierConfig
{
  double min_level_res {1.0};
  int max_level {3};
  double score_threshold_percentage {0.25};
  int timeout_msec {0};
  int num_threads {0};
  double translation_search_margin_m {15.0};
  double roll_pitch_search_deg {10.0};
  double yaw_search_deg {180.0};
};

struct ThreeDBBSLoopVerification
{
  bool available {false};
  bool localized {false};
  bool timed_out {false};
  double score_percentage {0.0};
  double elapsed_msec {0.0};
  Eigen::Matrix4f correction_guess {Eigen::Matrix4f::Identity()};
};

class ThreeDBBSLoopVerifier
{
public:
  ThreeDBBSLoopVerification localize(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & source_local,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & target_global,
    const Eigen::Isometry3d & source_pose_map,
    const Eigen::Isometry3d & search_center_pose_map,
    const ThreeDBBSLoopVerifierConfig & config) const;
};

}  // namespace graphslam

#endif  // GRAPH_BASED_SLAM__THREE_D_BBS_LOOP_VERIFIER_HPP_
