#!/usr/bin/env bash
# Usage: dist40_ab.sh <csv>
# GLIM vs ours (loop closure on, ndt_num_threads 1, distance_loop_closure 40.0) with the common end point of
# glim_vs_ours3.sh, alternating G D G D G D, 20 s apart. Columns as glim_vs_ours3.sh (system ours_loop_d40).
csv=$1
BASE=/home/user/bench/glim_cmp3
for ((i = 1; i <= 3; i++)); do
  for sys in glim ours_loop_d40; do
    /home/user/scripts/kill_ros_procs.sh > /dev/null
    sleep 20
    out=${BASE}/${sys}_d${i}
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
      tf=${out}.time
      /usr/bin/time -v -o "${tf}" /home/user/scripts/pipe_pipeline.sh "${BASE}/pipe_rows_d40.csv" "${sys}_${i}" \
        /home/user/ws/pipe_dev_734d54f /home/user/bench/params/lidarslam_competitive_v2_loop_ndt1_dist40.yaml \
        "${out}" > "${out}.log" 2>&1
      rc=$?
      IFS=, read -r -a r <<< "$(grep "^${sys}_${i}," "${BASE}/pipe_rows_d40.csv" | tail -1)"
      front=${r[1]:-NA}; complete=${r[2]:-NA}; front_md5=${r[7]:-NA}; traj_md5=${r[8]:-NA}
      result=${out}/trajectory_optimized.tum
    fi
    rtf=$(python3 -c "print(f'{${complete}/125.814128037:.4f}')" 2>/dev/null || echo NA)
    rss=$(awk -F': ' '/Maximum resident/ {printf "%.1f", $2/1024}' "${tf}")
    cpu=$(awk -F': ' '/User time|System time/ {s += $2} END {printf "%.1f", s}' "${tf}")
    fsin=$(awk -F': ' '/File system inputs/ {print $2}' "${tf}")
    echo "${sys},${i},${rc},${front},${complete},${rtf},${rss},${cpu},${fsin},${cached},${result},${front_md5},${traj_md5},${start_utc}" | tee -a "${csv}"
  done
done
