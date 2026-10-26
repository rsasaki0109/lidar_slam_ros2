#!/usr/bin/env bash
# p1_exp04_run.sh (chapter 30) <label> <ws> [ros overrides...]: frontend-only HILTI exp04 with exp04_gyro002_jerk1000.yaml, scored on the
# seven control points (ape_from_tum --interpolate); appends label,ape,md5 to /home/user/bench/p1/exp04_front.csv
set -uo pipefail
label=$1; ws=$2; shift 2
args=(); for kv in "$@"; do args+=(-p "${kv}"); done
O=/home/user/bench/p1/exp04/${label}; mkdir -p "$(dirname "$O")"
RUN_TIMEOUT=1800 bash /home/user/scripts/run_front.sh "exp04_${label}" "${ws}" /home/user/bench/params/exp04_gyro002_jerk1000.yaml \
  /home/user/data/hilti2022/exp04_ros2 /hesai/pandar /alphasense/imu "${O}" /home/user/bench/p1/exp04_runs.csv "${args[@]}" > /dev/null
tum=${O}/front_0/front_tum_0.txt
if [[ -f ${tum} ]]; then
  python3 /home/user/lidar_slam_ros2/scripts/ape_from_tum.py --ref /home/user/data/hilti2022/exp04_construction_upper_level_gt.txt \
    --est "${tum}" --out "${O}/ape.txt" --interpolate > /dev/null 2>&1
  echo "${label},$(awk '/^rmse:/ {print $2}' "${O}/ape.txt"),$(md5sum "${tum}" | cut -c1-8)" | tee -a /home/user/bench/p1/exp04_front.csv
else
  echo "${label},FAILED" | tee -a /home/user/bench/p1/exp04_front.csv
fi
rm -rf "${O}/roslog"
