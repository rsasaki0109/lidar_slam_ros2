#!/usr/bin/env bash
# Replays the final pose graph of every MapsHD B run (NDT 2 m) under each weight configuration given as an
# argument (graph_replay.py key=value lists, one quoted string each) and appends run,config,rmse rows to <csv>.
# Usage: replay_sweep.sh <csv> "<cfg>" ...
set -uo pipefail
set +u; source /opt/ros/jazzy/setup.bash; set -u
csv=$1; shift
S=/tmp/claude-0/-home-user/34acfe2e-4d27-5ba5-a50f-a6019be1a0de/scratchpad/replay; G=/home/user/data/mapshd/gt_sealed
for t in calibration-room_trial-1 calibration-room_trial-2 flat-room_trial-1 long-corridor_trial-1 outdoor-urk_trial-1 \
         underground-garage_trial-1 underground-garage_trial-2 underground-garage_trial-3; do
  sc=${t%_trial-*}; tr=${t#*_}
  for cfg in "$@"; do
    d=${S}/${t}/$(echo "${cfg}" | tr ' =,' '___')
    # shellcheck disable=SC2086
    python3 /home/user/scripts/graph_replay.py /home/user/bench/mapshd/${t}_B "${d}" mapshd:${G}/${sc}/${tr}/ground_truth.tum ${cfg} >> "${csv}" \
      || echo "fail ${t} ${cfg}" >&2
    rm -f "${d:?}"/*.g2o "${d:?}"/pipeline_imu.tum
  done
done
