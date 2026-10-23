#!/usr/bin/env bash
# Usage: pipe_pipeline.sh <csv> <label> <ws> <lidarslam_yaml> <out_dir>
# Whole-pipeline time with GLIM's end point (results saved): the same lidarslam rko_lio_slam.launch.py as
# run_rko_lio_graph_benchmark.sh and pipe_e2e.sh (benchmark no-map marker set), and
# save_when_frontend_done.py calls /map_save the moment the offline node exits; /map_save first finishes
# every pending loop search, then optimizes and writes the trajectory and the map.
# t0 = just before ros2 launch (date +%s.%N, the saver is already up). Columns: label, frontend_s
# (offline node exit - t0), complete_s (map_save response - t0), save_s (map_save call..response),
# offline_node_took_s, submaps, loop_edges, front_md5, traj_md5, g2o_md5, loops_md5
# BACKEND_SCHED_IDLE=1 (experiment): run every graph_based_slam thread under SCHED_IDLE.
# BAG, LIDAR, IMU, RKO_PARAMS override the exp04 defaults (bag, topics, RKO-LIO ROS parameter file).
# BARRIER=1: launch with wait_for_output_subscribers:=true (and whole-second subscriber_wait_timeout_sec, which
# the installed launch passes to the barrier script unchanged), so the offline node reads the bag only after
# graph_based_slam has subscribed to its odometry and scans (no scans lost at startup; chapter 18).
csv=$1; label=$2; w=$3; yaml=$4; out=$5
bag=${BAG:-/home/user/data/hilti2022/exp04_ros2}
rm -rf "${out}"; mkdir -p "${out}"
set +u; source "${w}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=36 ROS_LOG_DIR="${out}/roslog" M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1
cp "${RKO_PARAMS:-/home/user/bench/params/rko_params.ros.yaml}" "${out}/rko_params.ros.yaml"
barrier=(); [[ ${BARRIER:-0} == 1 ]] && barrier=(wait_for_output_subscribers:=true subscriber_wait_timeout_sec:=30 ${BARRIER_EXTRA:-})
cd /home/user/rsasaki0109/lidar_slam_ros2
timeout 3000 python3 /home/user/scripts/save_when_frontend_done.py "${out}/pipeline_times.json" "${out}/saver.ready" \
  > "${out}/saver.log" 2>&1 &
spid=$!
for ((k = 0; k < 600; k++)); do [[ -e ${out}/saver.ready ]] && break; sleep 0.05; done
t0=$(date +%s.%N)
setsid ros2 launch lidarslam rko_lio_slam.launch.py "main_param_dir:=${yaml}" "rko_param_file:=${out}/rko_params.ros.yaml" \
  "bag_path:=${bag}" "lidar_topic:=${LIDAR:-/hesai/pandar}" "imu_topic:=${IMU:-/alphasense/imu}" \
  base_frame:=base_link publish_static_tf:=true "save_dir:=${out}" "results_dir:=${out}" run_name:=front \
  dump_results:=true use_rviz:=false "${barrier[@]}" > "${out}/slam.launch.log" 2>&1 &
lpid=$!
if [[ ${BACKEND_SCHED_IDLE:-0} == 1 ]]; then
  # Experiment only: once graph_based_slam has finished initializing (all its threads exist), move every
  # thread of the backend to SCHED_IDLE so it runs only on CPU time the frontend leaves idle.
  (for ((k = 0; k < 1200; k++)); do
     grep -q "\[graph_based_slam\]: initialization end" "${out}/slam.launch.log" 2>/dev/null && break; sleep 0.05
   done
   gpid=$(pgrep -f "lib/graph_based_slam/graph_based_slam_node( |$)" | head -1)
   chrt -a -i -p 0 "${gpid}" && ps -L -o tid,cls,comm -p "${gpid}") > "${out}/sched_idle.log" 2>&1 &
fi
wait "${spid}"
kill -INT -- -"${lpid}" 2>/dev/null; sleep 5; kill -KILL -- -"${lpid}" 2>/dev/null; wait
read -r front complete save < <(python3 - "${out}/pipeline_times.json" "${t0}" <<'PY'
import json, sys
try:
    t = json.load(open(sys.argv[1]))
    t0 = float(sys.argv[2])
    print(f"{t['frontend_exit'] - t0:.3f} {t['save_done'] - t0:.3f} {t['save_done'] - t['save_called']:.3f}")
except Exception:
    print('NA NA NA')
PY
)
took=$(grep -ohP 'RKO LIO Offline Node took \K[0-9.]+' "${out}/slam.launch.log" | tail -1)
md5of() { [[ -f "$1" ]] && md5sum "$1" | cut -d' ' -f1 || echo missing; }
traj="${out}/trajectory_optimized.tum"
submaps=$( [[ -f "${traj}" ]] && grep -vc '^#' "${traj}" || echo NA)
loops=$( [[ -f "${out}/loop_edges.csv" ]] && echo $(( $(wc -l < "${out}/loop_edges.csv") - 1 )) || echo NA)
echo "${label},${front},${complete},${save},${took:-NA},${submaps},${loops},$(md5of "${out}/front_0/front_tum_0.txt"),$(md5of "${traj}"),$(md5of "${out}/pose_graph.g2o"),$(md5of "${out}/loop_edges.csv")" | tee -a "${csv}"
