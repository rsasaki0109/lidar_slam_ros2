#!/bin/bash
# Stage 2 runs (plan_stage2.txt). usage: run_stage2.sh RUN...   RUN = PKG:DS:ERR  (ERR "none" = parameter absent)
set -uo pipefail
B=/home/user/bench/sz3; REPO=/home/user/lidar_slam_ros2
PY=/tmp/claude-0/-home-user/79d76acb-f8a2-53a2-854b-d6db5cb3acb2/scratchpad/venv-ls/bin/python
input_of() { case $1 in
  d1) echo /home/user/bench/ms_heldout/calibration-room_trial-2/backend_input ;;
  d2) echo /home/user/bench/ms_heldout/underground-garage_trial-1/backend_input ;;
  d3) echo /home/user/bench/dev_loops/rb/exp04/backend_input ;;
  d4) echo /home/user/bench/loops_preset/stadtgarten_seq2/backend_input ;;
  d5) echo /home/user/bench/ms_cons/construction_seq1/backend_input ;;
  d6) echo /home/user/bench/ms_outdoor/con-1/backend_input ;;
  d7) echo /home/user/bench/dev_loops/rb/calibration-room_trial-1/backend_input ;;
  d8) echo /home/user/bench/ms_heldout/calibration-room_trial-2/backend_input ;;
esac; }
params_of() { case $1 in
  d4|d5|d7|d8) echo $REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml ;;
  *) echo $REPO/lidarslam/param/lidarslam.yaml ;;
esac; }
for run in "$@"; do
  IFS=: read -r pkg ds err <<< "$run"
  O=$B/s2/$pkg/$ds/$err
  [[ -s $O/done ]] && { echo "$run: done"; continue; }
  mkdir -p $O/out
  extra=(); [[ $err != none ]] && extra=(-p "submap_cloud_max_error_m:=$err")
  /usr/bin/time -v env PYTHONPATH=$B/pkg_$pkg $PY -m lidarslam backend "$(input_of $ds)" \
    --params "$(params_of $ds)" --out $O/out "${extra[@]}" > $O/run.log 2> $O/stderr.log
  echo "exit=$?" > $O/done
  echo "$run: $(cat $O/done)"
done
