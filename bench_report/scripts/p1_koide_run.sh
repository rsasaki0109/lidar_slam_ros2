#!/usr/bin/env bash
# p1_koide_run.sh <label> <ws> <params> <sequence> [ros overrides...] (chapter 30.7): frontend-only run on one Koide
# outdoor_hard bag (/home/user/data/koide/<sequence>_c), scored with scripts/ape_from_tum.py (nearest pose within
# 0.05 s, SE(3) Umeyama) against gt/traj_lidar_<sequence without a/b>.txt; appends label,sequence,ape,x,y,z,md5,poses
# to /home/user/bench/p1/koide_front.csv
set -uo pipefail
label=$1; ws=$2; params=$3; seq=$4; shift 4
args=(); for kv in "$@"; do args+=(-p "${kv}"); done
O=/home/user/bench/p1/koide/${label}/${seq}; mkdir -p "$(dirname "$O")"
gt=/home/user/data/koide/gt/traj_lidar_${seq%[ab]}.txt
RUN_TIMEOUT=3600 bash /home/user/scripts/run_front.sh "koide_${label}_${seq}" "${ws}" "${params}" /home/user/data/koide/${seq}_c \
  /livox/points /livox/imu "${O}" /home/user/bench/p1/koide_runs.csv "${args[@]}" > /dev/null
tum=${O}/front_0/front_tum_0.txt
if [[ -f ${tum} ]]; then
  python3 /home/user/lidar_slam_ros2/scripts/ape_from_tum.py --ref "${gt}" --est "${tum}" --out "${O}/ape.txt" > /dev/null 2>&1
  axes=$(python3 - "${tum}" "${gt}" <<'PY'
import sys
import numpy as np
est = np.loadtxt(sys.argv[1]); gt = np.loadtxt(sys.argv[2])
idx = np.searchsorted(est[:, 0], gt[:, 0]); idx = np.clip(idx, 1, len(est) - 1)
near = np.where(np.abs(est[idx - 1, 0] - gt[:, 0]) < np.abs(est[idx, 0] - gt[:, 0]), idx - 1, idx)
ok = np.abs(est[near, 0] - gt[:, 0]) <= 0.05
P, Q = est[near[ok], 1:4], gt[ok, 1:4]
mp, mq = P.mean(0), Q.mean(0)
U, _, Vt = np.linalg.svd((Q - mq).T @ (P - mp))
D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
R = U @ D @ Vt
e = (R @ (P - mp).T).T + mq - Q
print(','.join(f'{v:.4f}' for v in np.sqrt((e ** 2).mean(0))))
PY
)
  echo "${label},${seq},$(awk '/^rmse:/ {print $2}' "${O}/ape.txt"),${axes},$(md5sum "${tum}" | cut -c1-8),$(wc -l < "${tum}")" \
    | tee -a /home/user/bench/p1/koide_front.csv
else
  echo "${label},${seq},FAILED" | tee -a /home/user/bench/p1/koide_front.csv
fi
rm -rf "${O}/roslog"
