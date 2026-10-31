#!/usr/bin/env bash
# p1_mapshd_subset.sh (chapter 30) <label> <ws> <params> "<trial list>" [overrides...]: like p1_mapshd_batch.sh on the given trials.
set -uo pipefail
label=$1; ws=$2; params=$3; trials=$4; shift 4
args=(); for kv in "$@"; do args+=(-p "${kv}"); done
OUT=/home/user/bench/p1/runs/${label}; mkdir -p "${OUT}"
for t in ${trials}; do
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
