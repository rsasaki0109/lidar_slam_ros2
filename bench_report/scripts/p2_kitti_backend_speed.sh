#!/usr/bin/env bash
# p2_kitti_backend_speed.sh <variant> (plan: data/kitti/raw05_2026-10-31/plan_speed.txt): graph_slam_offline_runner on
# the KITTI 05 backend input (/home/user/bench/kitti_raw/prof/backend_input) with the walking preset and one variant
# (base, t4, v05, t4v05), timed with /usr/bin/time, scored with p2_kitti_score.py; appends
# variant,wall_s,user_s,sys_s,max_rss_kb,matched,missing,ape_rmse,ape_max,t_rel,r_rel,loop_edges to
# /home/user/bench/kitti_raw/speed.csv.
set -uo pipefail
v=$1
REPO=/home/user/lidar_slam_ros2; PY=/home/user/mapshd/venv/bin/python
RUNNER=${RUNNER:-/home/user/ws/slam_hi/build/graph_slam_offline_runner}
PRESET=${REPO}/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
P=/home/user/bench/kitti_raw/prof; IN=${P}/backend_input; FRONT=${P}/front/front_0/front_tum_0.txt
GT=/home/user/data/kitti_raw_05/gt; FRAMES=/home/user/data/kitti_raw_05/ros2.frames.txt
EXT="-0.007412088,0.001015086,0.000385188,0.999971941,0.810543972,-0.307054372,0.802723995"
case ${v} in
  base) over=() ;;
  t4) over=(ndt_num_threads:=4) ;;
  v05) over=(voxel_leaf_size:=0.5) ;;
  t4v05) over=(ndt_num_threads:=4 voxel_leaf_size:=0.5) ;;
  *) echo "unknown variant ${v}"; exit 1 ;;
esac
O=/home/user/bench/kitti_raw/speed/${v}${RUN_SUFFIX:-}
rm -rf "${O}"; mkdir -p "${O}"
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/ws/slam_dev/install/setup.bash; set -u
bargs=(); for kv in "${over[@]}"; do bargs+=(-p "${kv}"); done
/usr/bin/time -f "%e %U %S %M" -o "${O}/time.txt" "${RUNNER}" --ros-args --params-file "${PRESET}" \
  -p bag_path:="${IN}" -p output_dir:="${O}/run" -p "extrinsic_lidar2base_quat_xyzw_xyz:=[${EXT}]" "${bargs[@]}" \
  > "${O}/run.log" 2>&1
loops=$(grep -h "Offline run complete" "${O}/run.log" | grep -oP '[0-9]+(?= loop edges)')
PYTHONPATH=${REPO} python3 "${REPO}/scripts/densify_corrected_trajectory.py" --raw "${FRONT}" \
  --corrected "${O}/run/trajectory_optimized.tum" --output "${O}/dense.tum" > "${O}/densify.log" 2>&1
score=$("${PY}" "${REPO}/bench_report/scripts/p2_kitti_score.py" "${O}/dense.tum" "${GT}/05.txt" "${GT}/calib_05.txt" \
  "${FRAMES}" "${EXT}" "${O}/score.json")
read -r wall user sys rss < "${O}/time.txt"
echo "${v}${RUN_SUFFIX:-},${wall},${user},${sys},${rss},${score},${loops:-NA}" | tee -a /home/user/bench/kitti_raw/speed.csv
