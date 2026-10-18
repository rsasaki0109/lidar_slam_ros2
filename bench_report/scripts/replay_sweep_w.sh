#!/usr/bin/env bash
# Replays the final pose graph of chapter 21's W runs (8 MapsHD trials) and chapter 20's V runs (Oxford, exp04, MID-360,
# whose profiles W did not change) under each
# weight configuration given as an argument (graph_replay.py key=value lists, one quoted string each). Appends
# run,config,rmse rows to <csv> (MID-360: the residual of the loop back to the start instead of an APE).
# Usage: replay_sweep_all.sh <csv> "<cfg>" ...
set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; set -u
csv=$1; shift
S=/tmp/claude-0/-home-user/34acfe2e-4d27-5ba5-a50f-a6019be1a0de/scratchpad/replay_w
G=/home/user/data/mapshd/gt_sealed
declare -A SC=([oxford]=oxford:/home/user/data/oxford_keble05/gt_sealed/gt-tum.txt,/home/user/data/oxford_keble05/calib/T_imu_base.npy
               [exp04]=exp04:/home/user/data/hilti2022/exp04_construction_upper_level_gt.txt [mid360]=none)
for t in calibration-room_trial-1 calibration-room_trial-2 flat-room_trial-1 long-corridor_trial-1 outdoor-urk_trial-1 \
         underground-garage_trial-1 underground-garage_trial-2 underground-garage_trial-3; do
  SC[$t]=mapshd:${G}/${t%_trial-*}/${t#*_}/ground_truth.tum
done
for t in calibration-room_trial-1 calibration-room_trial-2 flat-room_trial-1 long-corridor_trial-1 outdoor-urk_trial-1 \
         underground-garage_trial-1 underground-garage_trial-2 underground-garage_trial-3 oxford exp04 mid360; do
  for cfg in "$@"; do
    d=${S}/${t}/$(echo "x${cfg}" | tr ' =,' '___')
    # shellcheck disable=SC2086
    case ${t} in oxford|exp04|mid360) run=/home/user/bench/vert/pipe/${t}_V ;; *) run=/home/user/bench/scale/pipe/${t}_W ;; esac
    line=$(python3 /home/user/scripts/graph_replay.py ${run} "${d}" "${SC[$t]}" ${cfg}) || { echo "fail ${t} ${cfg}" >&2; continue; }
    if [[ ${t} == mid360 ]]; then
      res=$(python3 /home/user/scripts/loop_residuals.py ${run} "${d}/trajectory_optimized.tum" | awk -v n="$(grep -vc '^#' ${run}/trajectory_optimized.tum)" '$1 < 30 && $2 >= n - 60 {print $4; exit}')
      line="mid360_V,${cfg:-as_run},${res}"
    fi
    echo "${line}" >> "${csv}"
    rm -f "${d:?}"/*.g2o "${d:?}"/pipeline_*.tum
  done
done
