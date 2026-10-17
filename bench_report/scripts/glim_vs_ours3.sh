#!/usr/bin/env bash
# Usage: glim_vs_ours3.sh <csv> <rounds>
# Same-VM GLIM vs ours with one end point for all: process start -> results saved. One run at a time,
# 20 s apart, order rotated every round.
#   glim      : glim_run.sh; /usr/bin/time wall of glim_rosbag (start .. global mapping done .. dump saved .. exit)
#   ours_off  : pipe_pipeline.sh, README competitive profile (loop closure off)
#   ours_loop : pipe_pipeline.sh, loop closure on + ndt_num_threads 1
#   (ours: ros2 launch start -> /map_save response, called the moment the frontend exits)
# Columns: system,round,exit,frontend_s,complete_s,rtf,max_rss_mb,cpu_s,fs_inputs,cached_kb_before,
#          result_traj,front_md5,traj_md5,start_utc
csv=$1; rounds=$2
DUR=125.814128037
BASE=/home/user/bench/glim_cmp3
mkdir -p "${BASE}"
order=(glim ours_off ours_loop)
field() { awk -F': ' -v k="$2" '$1 ~ k {print $2}' "$1"; }
for ((i = 1; i <= rounds; i++)); do
  shift_by=$(( (i - 1) % 3 ))
  for ((j = 0; j < 3; j++)); do
    sys=${order[$(( (j + shift_by) % 3 ))]}
    /home/user/scripts/kill_ros_procs.sh > /dev/null
    sleep 20
    out=${BASE}/${sys}_${i}
    rm -rf "${out}" "${out}.time" "${out}.log"
    cached=$(awk '/^Cached:/ {print $2}' /proc/meminfo)
    start_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
    if [[ ${sys} == glim ]]; then
      /home/user/scripts/glim_run.sh "${out}" > "${out}.log" 2>&1
      rc=$(cat "${out}/process_exit_status.txt" 2>/dev/null || echo NA)
      tf=${out}/process_time.txt
      complete=$(python3 - "${tf}" <<'PY'
import re, sys
m = re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): (\S+)', open(sys.argv[1]).read()).group(1).split(':')
print(f'{sum(float(v) * 60 ** i for i, v in enumerate(reversed(m))):.3f}')
PY
)
      front=NA; result=${out}/dump/traj_imu.txt; front_md5=NA; traj_md5=$(md5sum "${result}" | cut -c1-32)
    else
      yaml=/home/user/rsasaki0109/lidar_slam_ros2/configs/hilti2022/lidarslam_competitive_v2.yaml
      [[ ${sys} == ours_loop ]] && yaml=/home/user/bench/params/lidarslam_competitive_v2_loop_ndt1.yaml
      tf=${out}.time
      /usr/bin/time -v -o "${tf}" /home/user/scripts/pipe_pipeline.sh "${BASE}/pipe_rows.csv" "${sys}_${i}" \
        /home/user/ws/pipe_dev_734d54f "${yaml}" "${out}" > "${out}.log" 2>&1
      rc=$?
      IFS=, read -r -a r <<< "$(grep "^${sys}_${i}," "${BASE}/pipe_rows.csv" | tail -1)"
      front=${r[1]:-NA}; complete=${r[2]:-NA}; front_md5=${r[7]:-NA}; traj_md5=${r[8]:-NA}
      result=${out}/trajectory_optimized.tum
    fi
    rtf=$(python3 -c "print(f'{${complete}/${DUR}:.4f}')" 2>/dev/null || echo NA)
    rss=$(awk -F': ' '/Maximum resident/ {printf "%.1f", $2/1024}' "${tf}")
    cpu=$(python3 -c "print(f'{$(field "${tf}" "User time") + $(field "${tf}" "System time"):.1f}')" 2>/dev/null || echo NA)
    fsin=$(field "${tf}" "File system inputs")
    echo "${sys},${i},${rc},${front},${complete},${rtf},${rss},${cpu},${fsin},${cached},${result},${front_md5},${traj_md5},${start_utc}" | tee -a "${csv}"
  done
done
