#!/usr/bin/env bash
# p1_oxford_run.sh (chapter 30) <label> <ws> <params> [ros overrides...]: frontend-only Oxford Keble 05, scored with ape_oxford.py;
# appends label,ape,md5 to /home/user/bench/p1/oxford_front.csv
set -uo pipefail
label=$1; ws=$2; params=$3; shift 3
args=(); for kv in "$@"; do args+=(-p "${kv}"); done
O=/home/user/bench/p1/oxford/${label}; mkdir -p "$(dirname "$O")"
RUN_TIMEOUT=3600 bash /home/user/scripts/run_front.sh "oxford_${label}" "${ws}" "${params}" /home/user/data/oxford_keble05/ros2 \
  /hesai/pandar /alphasense_driver_ros/imu "${O}" /home/user/bench/p1/oxford_runs.csv "${args[@]}" > /dev/null
tum=${O}/front_0/front_tum_0.txt
if [[ -f ${tum} ]]; then
  /home/user/mapshd/venv/bin/python /home/user/scripts/ape_oxford.py "${tum}" /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
    /home/user/data/oxford_keble05/calib/T_imu_base.npy "${O}/ape" > "${O}/ape.log" 2>&1
  ape=$(grep -oiP 'rmse[^0-9]*\K[0-9.]+' "${O}/ape.log" | head -1)
  echo "${label},${ape},$(md5sum "${tum}" | cut -c1-8)" | tee -a /home/user/bench/p1/oxford_front.csv
else
  echo "${label},FAILED" | tee -a /home/user/bench/p1/oxford_front.csv
fi
rm -rf "${O}/roslog"
