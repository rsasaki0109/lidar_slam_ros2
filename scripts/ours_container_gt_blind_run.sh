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

# ros2 launch writes its own log before the benchmark node starts.  The image
# root is read-only, so keep that state inside the attempt output just like
# the other benchmark wrappers.
export ROS_HOME="${ROS_HOME:-${OUT_DIR}/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${OUT_DIR}/ros_log}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
source /runner/scripts/container_memory_evidence.sh
trap 'm6a5_container_exit_trap "$?"' EXIT

set +u
source /opt/ros/jazzy/setup.bash
source /opt/ours_ws/install/setup.bash
set -u

# The image recipe must build the pinned RKO-LIO package explicitly.  Check
# the installed package and executable before touching the input bag so a
# broken image fails with an actionable provenance/runtime error.
RKO_PREFIX="$(ros2 pkg prefix rko_lio 2>/dev/null)" || {
  echo 'error: rko_lio is not discoverable in the pinned runtime image' >&2
  exit 12
}
[[ -f "${RKO_PREFIX}/share/rko_lio/package.xml" ]] || {
  echo "error: rko_lio package manifest is missing under ${RKO_PREFIX}" >&2
  exit 12
}
[[ -x "${RKO_PREFIX}/lib/rko_lio/offline_node" ]] || {
  echo "error: rko_lio offline_node is missing under ${RKO_PREFIX}" >&2
  exit 12
}
[[ -f /runner/configs/hilti2022/rko_lio_hilti2022_pandar.yaml ]] || {
  echo 'error: pinned RKO-LIO benchmark config is missing' >&2
  exit 12
}

if [[ "${M6A3_SYNTHETIC_SMOKE:-0}" == "1" ]]; then
  # Synthetic smoke deliberately exercises the real launch graph, but stops
  # before replay/scoring.  It is not a benchmark and never accepts a GT
  # path.  Keep the temporary ROS parameter wrapper inside the attempt dir.
  mkdir -p "${OUT_DIR}"
  smoke_param="${OUT_DIR}/rko_params.yaml"
  python3 - "${RKO_PARAM:-/runner/configs/hilti2022/rko_lio_hilti2022_pandar.yaml}" "${smoke_param}" <<'PY'
import shutil
import sys
from pathlib import Path

import yaml

source = Path(sys.argv[1])
target = Path(sys.argv[2])
data = yaml.safe_load(source.read_text()) or {}
if isinstance(data, dict) and any(
    isinstance(value, dict) and "ros__parameters" in value
    for value in data.values()
):
    shutil.copyfile(source, target)
else:
    target.write_text(yaml.safe_dump({"/**": {"ros__parameters": data}}, sort_keys=False))
PY
  launch_log="${OUT_DIR}/synthetic_smoke_launch.log"
  setsid ros2 launch lidarslam rko_lio_slam.launch.py \
    "main_param_dir:=/runner/lidarslam/param/lidarslam.yaml" \
    "rko_param_file:=${smoke_param}" \
    "bag_path:=${BAG_PATH}" \
    "lidar_topic:=${LIDAR_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "base_frame:=base_link" "publish_static_tf:=false" \
    "save_dir:=${OUT_DIR}" "results_dir:=${OUT_DIR}" \
    "run_name:=m6a3_synthetic_smoke" "dump_results:=false" \
    "use_rviz:=false" >"${launch_log}" 2>&1 &
  smoke_pid=$!
  started=0
  for _ in $(seq 1 "${SMOKE_STARTUP_TIMEOUT_SECS:-30}"); do
    if grep -Fq 'RKO LIO Node is up!' "${launch_log}" 2>/dev/null && \
       grep -Fq '[graph_based_slam]: initialization end' "${launch_log}" 2>/dev/null; then
      started=1
      break
    fi
    if ! kill -0 "${smoke_pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  kill -INT -- "-${smoke_pid}" >/dev/null 2>&1 || true
  wait "${smoke_pid}" >/dev/null 2>&1 || true
  if [[ "${started}" != "1" ]]; then
    echo 'error: synthetic RKO launch did not reach both startup markers' >&2
    tail -n 80 "${launch_log}" >&2 || true
    exit 13
  fi
  printf '%s\n' '{"status":"pass","startup_verified":true,"input_verified":true,"clean_shutdown":true,"gt_mounted":false,"performance_run":false}' >"${OUT_DIR}/synthetic_smoke_contract.json"
  exit 0
fi

bash /runner/scripts/run_rko_lio_graph_benchmark.sh \
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
status=$?
exit "${status}"
