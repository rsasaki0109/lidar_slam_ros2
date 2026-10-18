#!/usr/bin/env bash
# Usage: [RKO_PARAMS=<rko ros params>] pipe_e2e.sh <csv> <label> <ws> <lidarslam_yaml> <out_dir> [bag] [lidar_topic] [imu_topic]
# End-to-end loop-closure run: the same lidarslam rko_lio_slam.launch.py as run_rko_lio_graph_benchmark.sh
# (benchmark no-map marker set, so accepted loops do not write map bundles mid-run), backend kept alive
# after the offline node exits. Per second it records the CPU ticks of both processes. When the backend
# has been idle for 5 s (< 0.05 cores) it calls /map_save once (timed), then stops the launch.
# Columns: label, offline_node_took_s, front_end_s, backend_idle_s, drain_s, map_save_s, submaps,
#          candidate_lines, loop_edges, front_md5, traj_md5, g2o_md5, loops_md5, backend_cpu_s, front_cpu_s
csv=$1; label=$2; w=$3; yaml=$4; out=$5
bag=${6:-/home/user/data/hilti2022/exp04_ros2}; lidar=${7:-/hesai/pandar}; imu=${8:-/alphasense/imu}
rm -rf "${out}"; mkdir -p "${out}"
set +u; source "${w}/install/setup.bash"; set -u
export ROS_DOMAIN_ID=35 ROS_LOG_DIR="${out}/roslog" M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1
cp "${RKO_PARAMS:-/home/user/bench/params/rko_params.ros.yaml}" "${out}/rko_params.ros.yaml"
cd /home/user/rsasaki0109/lidar_slam_ros2
setsid ros2 launch lidarslam rko_lio_slam.launch.py "main_param_dir:=${yaml}" "rko_param_file:=${out}/rko_params.ros.yaml" \
  "bag_path:=${bag}" "lidar_topic:=${lidar}" "imu_topic:=${imu}" \
  base_frame:=base_link publish_static_tf:=true "save_dir:=${out}" "results_dir:=${out}" run_name:=front \
  dump_results:=true use_rviz:=false > "${out}/slam.launch.log" 2>&1 &
lpid=$!
echo "t,offline_ticks,graph_ticks" > "${out}/cpu.csv"
seen_front=0; front_end=""; idle_run=0; idle_start=""; prev_graph=""; last_ot=0; last_gt=0
for ((s = 1; s <= 3600; s++)); do
  sleep 1
  opid=$(pgrep -f "lib/rko_lio/offline_node( |$)" | head -1)
  gpid=$(pgrep -f "lib/graph_based_slam/graph_based_slam_node( |$)" | head -1)
  ot=""; gt=""
  if [[ -n "${opid}" && -r /proc/${opid}/stat ]]; then read -r -a f < /proc/${opid}/stat; ot=$(( f[13] + f[14] )); last_ot=${ot}; seen_front=1; fi
  if [[ -n "${gpid}" && -r /proc/${gpid}/stat ]]; then read -r -a g < /proc/${gpid}/stat; gt=$(( g[13] + g[14] )); last_gt=${gt}; fi
  echo "${s},${ot},${gt}" >> "${out}/cpu.csv"
  if [[ ${seen_front} == 1 && -z "${opid}" && -z "${front_end}" ]]; then front_end=${s}; fi
  if [[ -n "${front_end}" && -n "${gt}" && -n "${prev_graph}" ]]; then
    if (( gt - prev_graph < 5 )); then idle_run=$((idle_run + 1)); else idle_run=0; fi
    if (( idle_run >= 5 )); then idle_start=$((s - 5)); break; fi
  fi
  prev_graph=${gt}
done
ms0=$(date +%s.%N)
timeout 600 ros2 service call /map_save std_srvs/srv/Empty > "${out}/map_save_call.log" 2>&1
ms1=$(date +%s.%N)
map_save=$(python3 -c "print(f'{${ms1} - ${ms0}:.2f}')")
gpid=$(pgrep -f "lib/graph_based_slam/graph_based_slam_node( |$)" | head -1)
if [[ -n "${gpid}" && -r /proc/${gpid}/stat ]]; then read -r -a g < /proc/${gpid}/stat; last_gt=$(( g[13] + g[14] )); fi
kill -INT -- -"${lpid}" 2>/dev/null; sleep 5; kill -KILL -- -"${lpid}" 2>/dev/null
took=$(grep -ohP 'RKO LIO Offline Node took \K[0-9.]+' "${out}/slam.launch.log" | tail -1)
cands=$(grep -c "best_loop_candidate" "${out}/slam.launch.log")
md5of() { [[ -f "$1" ]] && md5sum "$1" | cut -d' ' -f1 || echo missing; }
traj="${out}/trajectory_optimized.tum"
submaps=$( [[ -f "${traj}" ]] && grep -vc '^#' "${traj}" || echo NA)
loops=$( [[ -f "${out}/loop_edges.csv" ]] && echo $(( $(wc -l < "${out}/loop_edges.csv") - 1 )) || echo NA)
drain=$( [[ -n "${front_end}" && -n "${idle_start}" ]] && echo $((idle_start - front_end)) || echo NA)
hz=$(getconf CLK_TCK)
echo "${label},${took:-NA},${front_end:-NA},${idle_start:-NA},${drain},${map_save},${submaps},${cands},${loops},$(md5of "${out}/front_0/front_tum_0.txt"),$(md5of "${traj}"),$(md5of "${out}/pose_graph.g2o"),$(md5of "${out}/loop_edges.csv"),$(( last_gt / hz )),$(( last_ot / hz ))" | tee -a "${csv}"
