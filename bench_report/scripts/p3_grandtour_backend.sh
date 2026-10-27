#!/usr/bin/env bash
# p3_grandtour_backend.sh <hf_mission_folder> <short_name> <label> <backend params.yaml> [frontend ws] (chapter 33):
# one GrandTour mission through the frontend and the graph backend with a causal output, what a live system could have
# published at each time (the COMFORT benchmark takes no post-processing):
#   1. p1_grandtour_run.sh (G_CONFIG, LIDAR, IMU, DATA_ROOT passed through): data, parameters, frontend trajectory,
#   2. the frontend again, publishing its odometry and deskewed scans, recorded as the backend input
#      (scripts/record_backend_input.sh); its trajectory must be the same as in step 1,
#   3. graph_slam_offline_runner with causal_trajectory_output (RUNNER, default ws/slam_dev's build),
#   4. <out>/<short>_causal.tum as T_world_prism(t) = T_world_imu(t) T_imu_prism, and the backend's non-causal
#      trajectory_optimized_dense.tum, if written, as <out>/<short>_optimized.tum for comparison only.
# Outputs go to /home/user/bench/grandtour/<label>/<short_name>/.
set -euo pipefail
folder=$1; short=$2; label=$3; bparams=$4; ws=${5:-/home/user/ws/rko_g13}
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
O=/home/user/bench/grandtour/${label}/${short}
RUNNER=${RUNNER:-/home/user/ws/slam_dev/build/graph_based_slam/graph_slam_offline_runner}
bash "${S}/p1_grandtour_run.sh" "${folder}" "${short}" "${label}" "${ws}"
read -r BAG LIDAR_TOPIC IMU_TOPIC < "${O}/inputs.txt"
set +u; source /opt/ros/jazzy/setup.bash; source "${ws}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="${O}/roslog_backend"
rm -rf "${O}/backend_input" "${O}/run_backend_input" "${O}/backend"
bash "${REPO}/scripts/record_backend_input.sh" --output-dir "${O}/backend_input" --setup "${ws}/install/setup.bash" -- \
  "${ws}/install/rko_lio/lib/rko_lio/offline_node" --ros-args --params-file "${O}/params.yaml" \
  -p bag_path:="${BAG}" -p lidar_topic:="${LIDAR_TOPIC}" -p imu_topic:="${IMU_TOPIC}" \
  -p results_dir:="${O}/run_backend_input" -p publish_deskewed_scan:=true -p output_subscriber_timeout_sec:=60.0 \
  > "${O}/backend_input_record.log" 2>&1
a=$(md5sum < "${O}/run/front_0/front_tum_0.txt"); b=$(md5sum < "${O}/run_backend_input/front_0/front_tum_0.txt")
[[ ${a} == "${b}" ]] || { echo "frontend trajectory differs when it publishes the backend input"; exit 1; }
# rko_lio publishes the deskewed scans in the LiDAR frame; the runner moves them into the base (IMU) frame with the
# same extrinsic (without it every submap is placed turned by the LiDAR's mounting, 90 deg of yaw for the Hesai)
EXT=$("${PY}" -c "import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))['extrinsic_lidar2base_quat_xyzw_xyz'])" \
  "${O}/extrinsics.txt" | tr -d ' ')
t0=$(date +%s.%N)
"${RUNNER}" --ros-args --params-file "${bparams}" -p bag_path:="${O}/backend_input" -p output_dir:="${O}/backend" \
  -p causal_trajectory_output:=true -p extrinsic_lidar2base_quat_xyzw_xyz:="${EXT}" > "${O}/backend.log" 2>&1
t1=$(date +%s.%N)
echo "backend_wall_seconds $(python3 -c "print(f'{${t1}-${t0}:.1f}')")" > "${O}/backend_timing.txt"
grep -h "Offline run complete\|Causal trajectory" "${O}/backend.log" || true
"${PY}" - "${O}/extrinsics.txt" "${O}/backend/trajectory_causal.tum" "${O}/${short}_causal.tum" \
  "${O}/backend/trajectory_optimized_dense.tum" "${O}/${short}_optimized.tum" <<'PY'
import os
import sys
import numpy as np
import yaml
sys.path.insert(0, '/home/user/lidar_slam_ros2/bench_report/scripts')
from p1_grandtour_to_ros2 import R_to_quat, matrix  # noqa: E402
q_t = yaml.safe_load(open(sys.argv[1]))['T_imu_prism_quat_xyzw_xyz']
T_imu_prism = matrix(q_t[4:], q_t[:4])
for src, dst in ((sys.argv[2], sys.argv[3]), (sys.argv[4], sys.argv[5])):
    if not os.path.exists(src):
        continue
    est = np.loadtxt(src)
    with open(dst, 'w') as f:
        f.write('# timestamp x y z qx qy qz qw  (T_world_prism)\n')
        for row in est:
            T = matrix(row[1:4], row[4:8]) @ T_imu_prism
            q = R_to_quat(T[:3, :3])
            f.write(f'{row[0]:.9f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} '
                    f'{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n')
    print(f'{len(est)} poses -> {dst}')
PY
# the recorded backend input is as large as the mission's LiDAR data; the outputs above are what is kept
[[ ${KEEP_BACKEND_INPUT:-0} == 1 ]] || rm -rf "${O}/backend_input"
