#!/usr/bin/env bash
# Usage: run_front.sh <label> <ws> <params.yaml> <bag> <lidar_topic> <imu_topic> <out_dir> <results.csv> [extra ros args...]
# Runs the rko_lio offline_node once, then appends
# label,exit_code,node_seconds,wall_seconds,load1,load5,load15,start_utc,tum_md5,tum_lines to results.csv.
set -uo pipefail
label=$1; ws=$2; params=$3; bag=$4; lidar=$5; imu=$6; out=$7; csv=$8; shift 8
set +u; source /opt/ros/jazzy/setup.bash; source "${ws}/install/setup.bash"; set -u
rm -rf "${out}"; mkdir -p "${out}"
export ROS_LOG_DIR="${out}/roslog"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
read -r l1 l5 l15 _ < /proc/loadavg
start_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
t0=$(date +%s.%N)
timeout -s INT "${RUN_TIMEOUT:-1800}" "${ws}/install/rko_lio/lib/rko_lio/offline_node" --ros-args \
  --params-file "${params}" -p bag_path:="${bag}" -p lidar_topic:="${lidar}" -p imu_topic:="${imu}" \
  -p results_dir:="${out}" "$@" > "${out}/stdout.log" 2>&1
rc=$?
t1=$(date +%s.%N)
wall=$(python3 -c "print(f'{${t1}-${t0}:.3f}')")
took=$(grep -oP 'RKO LIO Offline Node took \K[0-9.]+' "${out}/stdout.log" | tail -1)
tum="${out}/front_0/front_tum_0.txt"
if [[ -f "${tum}" ]]; then md5=$(md5sum "${tum}" | cut -d' ' -f1); n=$(wc -l < "${tum}"); else md5=missing; n=0; fi
echo "${label},${rc},${took:-NA},${wall},${l1},${l5},${l15},${start_utc},${md5},${n}" | tee -a "${csv}"
