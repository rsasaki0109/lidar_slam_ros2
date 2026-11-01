#!/usr/bin/env bash
# p2_kitti_front_diag.sh <variant> (diagnostics; plan in data/kitti/front_diag_2026-11-01/plan.txt): the KITTI 05
# frontend alone (WS, default ws/rko_g35) with the KITTI rig file, G12 and one change, scored by p2_kitti_score.py,
# and the yaw error of the sharp turns (|yaw rate| > 0.3 rad/s) from p2_kitti_error_breakdown.py's alignment.
# Appends variant,md5,matched,missing,ape_rmse,ape_max,t_rel,r_rel,yaw_total_deg,yaw_sharp_deg,wall_s to
# /home/user/bench/kitti_raw/front_diag.csv. Development data only (reads the ground truth).
set -uo pipefail
v=$1
REPO=/home/user/lidar_slam_ros2; PY=/home/user/mapshd/venv/bin/python
WS=${WS:-/home/user/ws/rko_g35}
D=/home/user/data/kitti_raw_05; BAG=${BAG:-${D}/ros2}; GT=${D}/gt  # BAG: another bag of the same frames
RIG=${REPO}/bench_report/data/kitti/raw05_2026-10-31/rig_kitti_raw.yaml
EXT="-0.007412088,0.001015086,0.000385188,0.999971941,0.810543972,-0.307054372,0.802723995"
G12=(close_scene_max_median_range:=12.0 close_scene_max_correspondence_distance:=0.25
     close_scene_localizability_boost:=10.0 close_scene_localizability_normal_only:=true
     close_scene_min_inlier_fraction:=0.6 min_icp_keypoints:=100 keypoint_shortfall_voxel_multiplier:=1.0)
case ${v} in
  base) over=() ;;
  rotdeskew) over=() ;;  # base on the rotation-deskewed bag (plan_rotation_deskew.txt; it is ros2 since then)
  elev) over=() ;;  # base on the bag with the elevation correction too (plan_elevation.txt)
  noclose) over=(close_scene_max_median_range:=0.0) ;;
  lever0) EXT="-0.007412088,0.001015086,0.000385188,0.999971941,0.0,0.0,0.0"
          over=("extrinsic_lidar2base_quat_xyzw_xyz:=[${EXT}]") ;;
  icp) over=(max_iterations:=100 convergence_criterion:=1.0e-6) ;;
  *) echo "unknown variant ${v}"; exit 1 ;;
esac
O=/home/user/bench/kitti_raw/diag/${v}
rm -rf "${O}"; mkdir -p "${O}"
set +u; source /opt/ros/jazzy/setup.bash; source "${WS}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR="${O}/roslog"
args=(); for kv in "${G12[@]}" "${over[@]}"; do args+=(-p "${kv}"); done
t0=$(date +%s.%N)
"${WS}/install/rko_lio/lib/rko_lio/offline_node" --ros-args --params-file "${RIG}" -p bag_path:="${BAG}" \
  -p lidar_topic:=/kitti/velodyne/points -p imu_topic:=/kitti/oxts/imu -p results_dir:="${O}/front" \
  -p publish_deskewed_scan:=false "${args[@]}" > "${O}/front.log" 2>&1 || { echo "${v},frontend failed"; exit 1; }
t1=$(date +%s.%N)
FRONT=${O}/front/front_0/front_tum_0.txt
md5=$(md5sum < "${FRONT}" | cut -c1-8)
score=$("${PY}" "${REPO}/bench_report/scripts/p2_kitti_score.py" "${FRONT}" "${GT}/05.txt" "${GT}/calib_05.txt" \
  "${BAG}.frames.txt" "${EXT}" "${O}/score.json")
yaw=$(EXT="${EXT}" python3 - "${FRONT}" <<'PY'
import contextlib, io, os, runpy, sys
import numpy as np
from scipy.spatial.transform import Rotation
with contextlib.redirect_stdout(io.StringIO()):
    g = runpy.run_path('/home/user/lidar_slam_ros2/bench_report/scripts/p2_kitti_error_breakdown.py')
gt, es, ts, rel, n = g['gt'], g['es'], g['ts'], g['rel'], g['n']
err, w = [], []
for k in range(n - 1):
    Rg, Re = rel(gt, k, k + 1), rel(es, k, k + 1)
    err.append(Rotation.from_matrix((np.linalg.inv(Rg) @ Re)[:3, :3]).as_euler('zyx', degrees=True)[0])
    w.append(Rotation.from_matrix(Rg[:3, :3]).as_euler('zyx')[0] / (ts[k + 1] - ts[k]))
err, w = np.array(err), np.array(w)
print(f'{err.sum():.3f},{err[np.abs(w) > 0.3].sum():.3f}')
PY
)
echo "${v},${md5},${score},${yaw},$(python3 -c "print(f'{${t1}-${t0}:.1f}')")" | tee -a /home/user/bench/kitti_raw/front_diag.csv
