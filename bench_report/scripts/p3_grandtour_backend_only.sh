#!/usr/bin/env bash
# p3_grandtour_backend_only.sh <input label> <short_name> <out label> <backend params.yaml> [name:=value ...]
# (Phase 3, robust loops): the graph backend alone on a backend input recorded by p3_grandtour_backend.sh with
# KEEP_BACKEND_INPUT=1 (/home/user/bench/grandtour/<input label>/<short>/backend_input), with a causal output and the
# extra parameters given; then the causal trajectory at the prism (<out>/<short>_causal.tum), its ATE
# (p1_grandtour_ate.py) and every loop edge's error against the reference (p3_grandtour_loop_errors.py, <out>/loops.csv).
# RUNNER: default ws/slam_rl's build. Development missions only (the ATE and loop errors read the reference).
set -euo pipefail
in_label=$1; short=$2; label=$3; bparams=$4; shift 4
S=/home/user/lidar_slam_ros2/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
I=/home/user/bench/grandtour/${in_label}/${short}; O=/home/user/bench/grandtour/${label}/${short}
RUNNER=${RUNNER:-/home/user/ws/slam_rl/build/graph_based_slam/graph_slam_offline_runner}
[[ -d ${I}/backend_input ]] || { echo "no backend input at ${I}"; exit 1; }
rm -rf "${O}"; mkdir -p "${O}"
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/ws/slam_dev/install/setup.bash
[[ -f /home/user/ws/slam_rl/install/setup.bash ]] && source /home/user/ws/slam_rl/install/setup.bash; set -u
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="${O}/roslog"
EXT=$("${PY}" -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['extrinsic_lidar2base_quat_xyzw_xyz'])" \
  "${I}/extrinsics.txt" | tr -d ' ')
args=(); for o in "$@"; do args+=(-p "$o"); done
t0=$(date +%s.%N)
"${RUNNER}" --ros-args --params-file "${bparams}" -p bag_path:="${I}/backend_input" -p output_dir:="${O}/backend" \
  -p causal_trajectory_output:=true -p extrinsic_lidar2base_quat_xyzw_xyz:="${EXT}" "${args[@]}" > "${O}/backend.log" 2>&1
t1=$(date +%s.%N)
echo "backend_wall_seconds $(python3 -c "print(f'{${t1}-${t0}:.1f}')")" > "${O}/backend_timing.txt"
"${PY}" - "${I}/extrinsics.txt" "${O}/backend/trajectory_causal.tum" "${O}/${short}_causal.tum" <<'PY'
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import R_to_quat, matrix  # noqa: E402
q_t = yaml.safe_load(open(sys.argv[1]))['T_imu_prism_quat_xyzw_xyz']
T_imu_prism = matrix(q_t[4:], q_t[:4])
est = np.loadtxt(sys.argv[2])
with open(sys.argv[3], 'w') as f:
    f.write('# timestamp x y z qx qy qz qw  (T_world_prism)\n')
    for row in est:
        T = matrix(row[1:4], row[4:8]) @ T_imu_prism
        q = R_to_quat(T[:3, :3])
        f.write(f'{row[0]:.9f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n')
PY
ate=$("${PY}" -I "${S}/p1_grandtour_ate.py" "${O}/${short}_causal.tum" \
  "/home/user/data/grandtour/reference_dev/${short}/prism_position" | grep -o "without [0-9]* reference spikes [0-9.]*")
"${PY}" -I "${S}/p3_grandtour_loop_errors.py" "${label}" "${short}" "${O}/backend" "${I}/run/front_0/front_tum_0.txt" \
  "${I}/extrinsics.txt" > "${O}/loops.csv"
echo "${label},${short},causal ${ate},loop edges $(grep -c . "${O}/loops.csv"),$(grep -h "Offline run complete" "${O}/backend.log" | grep -o '[0-9]* loop edges')"
