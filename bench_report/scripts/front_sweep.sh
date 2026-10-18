#!/usr/bin/env bash
# Frontend-only screening on one MapsHD bag: one run per variant (ROS parameter overrides "k:=v k2:=v2"), scored by
# front_score.py. Appends variant,label,ape,rmse_x,rmse_y,rmse_z,z_hf_std,md5 to <csv>.
# Usage: front_sweep.sh <csv> <bag> <gt_tum> <out_root> "<overrides>" ...
set -uo pipefail
csv=$1; bag=$2; gt=$3; root=$4; shift 4
WS=${WS:-/home/user/ws/cand_77072e4}; P=${PARAMS:-/home/user/bench/params/mapshd_gyro002.yaml}
mkdir -p "${root}"
for v in "$@"; do
  name=$(echo "x${v}" | tr ' =:.' '____')
  args=(); for kv in ${v}; do args+=(-p "${kv}"); done
  bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
  ROS_DOMAIN_ID=37 bash /home/user/scripts/run_front.sh "${name}" "${WS}" "${P}" "${bag}" /livox/pointcloud /livox/imu \
    "${root}/${name}" "${root}/runs.csv" "${args[@]}" > /dev/null
  t="${root}/${name}/front_0/front_tum_0.txt"
  if [[ -f ${t} ]]; then
    s=$(python3 /home/user/scripts/front_score.py "${v:-default}" "${t}" "${gt}")
    echo "${s},$(md5sum "${t}" | cut -c1-8)" | tee -a "${csv}"
  else
    echo "${v},FAILED" | tee -a "${csv}"
  fi
  rm -rf "${root}/${name}/roslog"
done
