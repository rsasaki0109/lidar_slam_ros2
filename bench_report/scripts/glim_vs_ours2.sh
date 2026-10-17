#!/usr/bin/env bash
# Usage: glim_vs_ours2.sh <csv> <rounds>
# Same-VM GLIM vs ours, one run at a time, 20 s apart, order rotated every round:
#   glim        : glim_run.sh (native equivalent of scripts/glim_container_run.sh); time = /usr/bin/time wall
#                 of glim_rosbag (start .. global mapping done .. dump saved .. exit), as run_glim_benchmark.py.
#   ours_readme : README competitive profile (loop closure off), run_rko_lio_graph_benchmark.sh --gt-blind
#                 --skip-map-save; time = its wall_seconds, as run_ours_competitive_benchmark.py.
#   ours_loop   : loop closure on (+ ndt_num_threads 1), pipe_e2e.sh; time = launch .. frontend done and
#                 backend idle (end-to-end), and that plus the /map_save call that writes the optimized
#                 trajectory and map.
# Columns: system,round,exit,time_s,time_with_save_s,rtf,rtf_with_save,max_rss_mb,cpu_s,fs_inputs,
#          cached_kb_before,traj_lines,front_md5,result_traj,start_utc
csv=$1; rounds=$2
SLAM=/home/user/rsasaki0109/lidar_slam_ros2
DUR=125.814128037
BASE=/home/user/bench/glim_cmp
order=(glim ours_readme ours_loop)
tsec() { python3 - "$1" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
m = re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): (\S+)', t).group(1).split(':')
print(f'{sum(float(v) * 60 ** i for i, v in enumerate(reversed(m))):.2f}')
PY
}
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
      t=$(tsec "${tf}"); tsave=${t}
      front_md5=NA; result=${out}/dump/traj_imu.txt
    elif [[ ${sys} == ours_readme ]]; then
      (cd "${SLAM}" && LIDARSLAM_WS_ROOT=/home/user/ws/pipe_dev_734d54f ROS_DOMAIN_ID=31 \
        /usr/bin/time -v -o "${out}.time" timeout 3000 bash scripts/run_rko_lio_graph_benchmark.sh --gt-blind \
        --bag /home/user/data/hilti2022/exp04_ros2 --lidar-topic /hesai/pandar --imu-topic /alphasense/imu \
        --lidarslam-param "${SLAM}/configs/hilti2022/lidarslam_competitive_v2.yaml" \
        --rko-param configs/hilti2022/rko_lio_hilti2022_pandar_competitive_v2.yaml \
        --output-dir "${out}" --run-name front --save-timeout-secs 420 --skip-map-save) > "${out}.log" 2>&1
      rc=$?
      tf=${out}.time
      t=$(python3 -c "import json;print(f\"{json.load(open('${out}/gt_blind_run.json'))['wall_seconds']:.2f}\")" \
        2>/dev/null || echo NA); tsave=${t}
      result=${out}/front_0/front_tum_0.txt
      front_md5=$(md5sum "${result}" 2>/dev/null | cut -c1-32)
    else
      e2e_csv=${BASE}/e2e_rows.csv
      /usr/bin/time -v -o "${out}.time" /home/user/scripts/pipe_e2e.sh "${e2e_csv}" "${sys}_${i}" \
        /home/user/ws/pipe_dev_734d54f /home/user/bench/params/lidarslam_competitive_v2_loop_ndt1.yaml \
        "${out}" > "${out}.log" 2>&1
      rc=$?
      tf=${out}.time
      row=$(grep "^${sys}_${i}," "${e2e_csv}" | tail -1)
      IFS=, read -r -a r <<< "${row}"
      t=$(python3 -c "print(max(float('${r[2]}'), float('${r[3]}')))" 2>/dev/null || echo NA)
      tsave=$(python3 -c "print(f'{max(float(\"${r[2]}\"), float(\"${r[3]}\")) + float(\"${r[5]}\"):.2f}')" \
        2>/dev/null || echo NA)
      front_md5=${r[9]:-NA}; result=${out}/trajectory_optimized.tum
    fi
    rtf=$(python3 -c "print(f'{${t}/${DUR}:.4f}')" 2>/dev/null || echo NA)
    rtfs=$(python3 -c "print(f'{${tsave}/${DUR}:.4f}')" 2>/dev/null || echo NA)
    rss=$(awk -F': ' '/Maximum resident/ {printf "%.1f", $2/1024}' "${tf}")
    cpu=$(python3 -c "print(f'{$(field "${tf}" "User time") + $(field "${tf}" "System time"):.1f}')" 2>/dev/null || echo NA)
    fsin=$(field "${tf}" "File system inputs")
    lines=$(grep -vc '^#' "${result}" 2>/dev/null || echo 0)
    echo "${sys},${i},${rc},${t},${tsave},${rtf},${rtfs},${rss},${cpu},${fsin},${cached},${lines},${front_md5:-missing},${result},${start_utc}" | tee -a "${csv}"
  done
done
