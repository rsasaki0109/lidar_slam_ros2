#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set -euo pipefail

: "${BAG_PATH:?BAG_PATH is required}"
: "${OUT_DIR:?OUT_DIR is required}"
: "${LIDAR_TOPIC:?LIDAR_TOPIC is required}"
: "${IMU_TOPIC:?IMU_TOPIC is required}"

set +u
source /opt/ros/jazzy/setup.bash
source /opt/ours_ws/install/setup.bash
set -u

exec bash /runner/scripts/run_rko_lio_graph_benchmark.sh \
  --bag "${BAG_PATH}" \
  --lidar-topic "${LIDAR_TOPIC}" \
  --imu-topic "${IMU_TOPIC}" \
  --rko-param /runner/configs/hilti2022/rko_lio_hilti2022_pandar.yaml \
  --lidarslam-param /runner/lidarslam/param/lidarslam.yaml \
  --output-dir "${OUT_DIR}" \
  --run-name "${RUN_NAME:-m6a}" \
  --offline-timeout-secs "${OFFLINE_TIMEOUT_SECS:-7200}" \
  --save-timeout-secs "${SAVE_TIMEOUT_SECS:-600}" \
  --completion-end-margin-secs 0.25 \
  --gt-blind
