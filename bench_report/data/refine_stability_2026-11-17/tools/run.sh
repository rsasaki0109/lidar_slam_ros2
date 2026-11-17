#!/bin/bash
# Refinement stability screen (plan.txt). usage: run.sh RUN...  RUN = DS:VARIANT:PERTURB (PERTURB: 0 or 1um)
set -uo pipefail
B=/home/user/bench/refine; REPO=/home/user/lidar_slam_ros2
PY=/tmp/claude-0/-home-user/79d76acb-f8a2-53a2-854b-d6db5cb3acb2/scratchpad/venv-ls/bin/python
input_of() { case $1 in
  c1) echo /home/user/bench/ms_outdoor/con-1/backend_input ;;
  c2) echo /home/user/bench/ms_outdoor/con-2/backend_input ;;
  m1) echo /home/user/bench/dev_loops/rb/calibration-room_trial-1/backend_input ;;
  h4) echo /home/user/bench/dev_loops/rb/exp04/backend_input ;;
  s2) echo /home/user/bench/loops_preset/stadtgarten_seq2/backend_input ;;
esac; }
params_of() { case $1 in
  m1|s2) echo $REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml ;;
  *) echo $REPO/lidarslam/param/lidarslam.yaml ;;
esac; }
for run in "$@"; do
  IFS=: read -r ds var pert <<< "$run"
  O=$B/runs/$ds/$var/$pert
  [[ -s $O/done ]] && { echo "$run: done"; continue; }
  mkdir -p $O/out
  extra=()
  case $var in
    RB) extra+=(-p refine_window_max_adjacent_change_m:=0.10 -p refine_window_max_adjacent_change_deg:=1.0) ;;
    RA) extra+=(-p refine_relative_translation_sigma:=0.05 -p refine_relative_rotation_sigma_rad:=0.0087) ;;
    OFF) extra+=(-p refine_window_max_adjacent_change_m:=0.0 -p refine_window_max_adjacent_change_deg:=0.0) ;;
  esac
  [[ $pert == 1um ]] && extra+=(-p submap_cloud_max_error_m:=0.000001)
  /usr/bin/time -v env PYTHONPATH=$B/pkg $PY -m lidarslam backend "$(input_of $ds)" \
    --params "$(params_of $ds)" --out $O/out "${extra[@]}" > $O/run.log 2> $O/stderr.log
  echo "exit=$?" > $O/done
  echo "$run: $(cat $O/done)"
done
