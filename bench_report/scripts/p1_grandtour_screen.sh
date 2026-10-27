#!/usr/bin/env bash
# p1_grandtour_screen.sh <variant dir> <label prefix> <ws> <hf_folder:short> ... (chapter 33): run every <variant>.yaml
# of the directory as G_CONFIG of p1_grandtour_run.sh (LIDAR, IMU passed through) on each mission, the mission's data
# downloaded and converted once for all variants and deleted afterwards. Each mission holds the disk lock, so no other
# job downloads at the same time. The data go to DATA_ROOT (default /home/user/data/grandtour/screen), apart from
# the other batches' directories. Appends "<prefix>_<variant>,<short>,<ATE line>,<jitter>" to dev_scores.csv.
set -euo pipefail
vdir=$1; prefix=$2; ws=$3; shift 3
export DATA_ROOT=${DATA_ROOT:-/home/user/data/grandtour/screen}
S=/home/user/lidar_slam_ros2/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python; B=/home/user/bench/grandtour
for item in "$@"; do
  folder=${item%%:*}; short=${item##*:}
  (
    flock 9
    for cfg in "${vdir}"/*.yaml; do
      v=$(basename "${cfg}" .yaml); label=${prefix}_${v}
      [[ -s ${B}/${label}/${short}/${short}.tum ]] && continue
      G_CONFIG=${cfg} bash "${S}/p1_grandtour_run.sh" "${folder}" "${short}" "${label}" "${ws}" 9>&- \
        || { echo "${label},${short},run failed" | tee -a "${B}/dev_scores.csv"; continue; }
      line=$("${PY}" -I "${S}/p1_grandtour_ate.py" "${B}/${label}/${short}/${short}.tum" \
        "/home/user/data/grandtour/reference_dev/${short}/prism_position")
      jit=$("${PY}" -I "${S}/p1_grandtour_jitter.py" "${B}/${label}/${short}/${short}.tum" | cut -d" " -f1-6)
      echo "${label},${short},${line},${jit}" | tee -a "${B}/dev_scores.csv"
    done
    rm -rf "${DATA_ROOT:?}/${short}"
  ) 9>"${B}/.disk.lock"
done
