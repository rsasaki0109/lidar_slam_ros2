#!/usr/bin/env bash
# Usage: sched_idle_ab.sh <csv>
# Loop closure on (+ ndt_num_threads 1): backend under SCHED_IDLE vs normal, alternating I N I N I, 20 s apart.
csv=$1
BASE=/home/user/bench/glim_cmp3
seq=(idle norm idle norm idle)
for ((i = 0; i < ${#seq[@]}; i++)); do
  v=${seq[$i]}
  /home/user/scripts/kill_ros_procs.sh > /dev/null
  sleep 20
  label=loop_${v}_$((i + 1)); out=${BASE}/${label}
  if [[ ${v} == idle ]]; then export BACKEND_SCHED_IDLE=1; else export BACKEND_SCHED_IDLE=0; fi
  /usr/bin/time -v -o "${out}.time" /home/user/scripts/pipe_pipeline.sh "${csv}" "${label}" \
    /home/user/ws/pipe_dev_734d54f /home/user/bench/params/lidarslam_competitive_v2_loop_ndt1.yaml "${out}" \
    > "${out}.log" 2>&1
done
