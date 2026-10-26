#!/usr/bin/env bash
# p1_mapshd_batch.sh (chapter 30) <label> <ws> <params.yaml> [ros overrides k:=v ...]: frontend-only run on all 8 MapsHD trials, scored
# with front_score.py; appends label,trial,ape,rmse_x,rmse_y,rmse_z,z_hf_std,md5 to /home/user/bench/p1/mapshd_front.csv
set -uo pipefail
label=$1; ws=$2; params=$3; shift 3
args=(); for kv in "$@"; do args+=(-p "${kv}"); done
OUT=/home/user/bench/p1/runs/${label}; mkdir -p "${OUT}"
for t in calibration-room_trial-1 calibration-room_trial-2 flat-room_trial-1 long-corridor_trial-1 outdoor-urk_trial-1 \
         underground-garage_trial-1 underground-garage_trial-2 underground-garage_trial-3; do
  RUN_TIMEOUT=1800 bash /home/user/scripts/run_front.sh "${label}_${t}" "${ws}" "${params}" /home/user/data/mapshd/bags/${t}_c \
    /livox/pointcloud /livox/imu "${OUT}/${t}" "${OUT}/runs.csv" "${args[@]}" > /dev/null
  tum=${OUT}/${t}/front_0/front_tum_0.txt
  if [[ -f ${tum} ]]; then
    s=$(/home/user/mapshd/venv/bin/python /home/user/scripts/front_score.py "${t}" "${tum}" /home/user/data/mapshd/gt/${t}/ground_truth.tum)
    echo "${label},${s},$(md5sum "${tum}" | cut -c1-8)" | tee -a /home/user/bench/p1/mapshd_front.csv
  else
    echo "${label},${t},FAILED" | tee -a /home/user/bench/p1/mapshd_front.csv
  fi
  rm -rf "${OUT}/${t}/roslog"
done
