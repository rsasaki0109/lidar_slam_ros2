#!/bin/bash
# run.sh STEP...: the runs of plan_construction.txt after the backend inputs are recorded.
#   replay:SEQ  dense:SEQ  gnss:SEQ  truth  joins  checks  gate
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; B=/home/user/bench/ms_cons; S=$REPO/bench_report/scripts
PRESET=$REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
JOIN=$REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_multisession_join
GT=/home/user/data/rtk_slam/rtk_slam_eval/ground_truth; ID=0.0,0.0,0.0,1.0,0.0,0.0,0.0
IDROW=1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$B/roslog
in_() { echo $B/construction_seq$1/backend_input; }
rep() { echo $B/replay/construction_seq$1; }
front() { echo $B/construction_seq$1/front/front_0/front_tum_0.txt; }
join() {  # join NAME args...
  local name=$1; shift; local O=$B/runs/$name
  [[ -s $O/join_report.yaml || -s $O/debug_checks.yaml ]] && { echo "$name present"; return; }
  rm -rf $O; mkdir -p $B/runs $B/cache; local t0=$(date +%s)
  export ROS_DOMAIN_ID=$((40 + RANDOM % 50))
  $JOIN --ros-args --params-file $PRESET -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$ID]" -p multisession_num_threads:=4 \
    "$@" -p output_dir:=$O > $O.log 2>&1
  echo "$name exit=$? $(( $(date +%s) - t0 )) s"
}
for step in "$@"; do
  IFS=: read -r kind s <<< "$step"
  case $kind in
  replay)
    O=$(rep $s); [[ -s $O/trajectory_optimized.tum ]] && { echo "replay $s present"; continue; }
    rm -rf $O; mkdir -p $B/replay; export ROS_DOMAIN_ID=$((40 + RANDOM % 50)); t0=$(date +%s)
    $REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_offline_runner --ros-args --params-file $PRESET \
      -p bag_path:=$(in_ $s) -p output_dir:=$O -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$ID]" > $O.log 2>&1
    echo "replay $s exit=$? $(( $(date +%s) - t0 )) s; $(grep -h 'Offline run complete' $O.log | tail -1 | cut -c1-160)" ;;
  dense)
    python3 -P $REPO/scripts/densify_corrected_trajectory.py --raw $(front $s) --corrected $(rep $s)/trajectory_optimized.tum \
      --output $(rep $s)/dense.tum | tail -1 ;;
  gnss)
    python3 -P $S/p3_ms_gnss_gate.py export --bag $(in_ $s) --out $B/construction_seq${s}_gnss_fix.csv
    echo "gnss $s: $(($(wc -l < $B/construction_seq${s}_gnss_fix.csv) - 1)) fixes" ;;
  truth)
    for o in "1 2" "2 1"; do set -- $o
      python3 -I -c "
import sys, numpy as np
r = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1, ndmin=2)
k = r[(r[:, 4] >= 0) & (r[:, 5] <= 0.09) & (r[:, 6] <= 0.09)][0]
print('[%r, %r, %r]' % (k[1], k[2], k[3]))" $B/construction_seq$1_gnss_fix.csv > $B/origin_seq$1.json
      python3 -I $S/p3_rtk_multisession_eval.py --gt-a $GT/construction_seq$1.csv --gt-b $GT/construction_seq$2.csv \
        --origin $B/origin_seq$1.json --alone-a $(rep $1)/dense.tum --alone-b $(rep $2)/dense.tum > $B/alone_A$1_B$2.json
      python3 -I -c "import json,sys; d=json.load(open(sys.argv[1])); print(sys.argv[1], d.get('shared_checkpoints'), d.get('drift_only_reference'))" $B/alone_A$1_B$2.json
    done; set -- ;;
  joins)
    join j12 -p "session_bag_paths:=['$(in_ 1)','$(in_ 2)']" -p "session_backend_dirs:=['$(rep 1)','$(rep 2)']" -p multisession_cache_dir:=$B/cache
    join j21 -p "session_bag_paths:=['$(in_ 2)','$(in_ 1)']" -p "session_backend_dirs:=['$(rep 2)','$(rep 1)']" -p multisession_cache_dir:=$B/cache
    for s in 1 2; do
      join split${s}_asis -p "session_bag_paths:=['$(in_ $s)']" -p "session_backend_dirs:=['$(rep $s)']" -p debug_split_gap_s:=30.0
      join split${s}_stress -p "session_bag_paths:=['$(in_ $s)']" -p "session_backend_dirs:=['$(rep $s)']" -p debug_split_gap_s:=30.0 \
        -p "debug_exclude_transform:=[$IDROW]" -p debug_exclude_radius_m:=2.0
    done ;;
  checks)
    for o in "1 2" "2 1"; do set -- $o; mkdir -p $B/checks
      python3 -I $S/p3_ms_perturb.py $B/alone_A$1_B$2.json $(rep $2)/trajectory_raw.tum $B/checks/perturb$1$2
      join check$1$2 -p "session_bag_paths:=['$(in_ $1)','$(in_ $2)']" -p "session_backend_dirs:=['$(rep $1)','$(rep $2)']" \
        -p "debug_check_transforms:=[$(cat $B/checks/perturb$1$2.values)]"
    done; set -- ;;
  gate)
    mkdir -p $B/eval
    for o in "1 2" "2 1"; do set -- $o
      python3 -P $S/p3_ms_gnss_local.py --no-floor pair --bag-a $B/construction_seq$1_gnss_fix.csv --bag-b $B/construction_seq$2_gnss_fix.csv \
        --front-a $(front $1) --front-b $(front $2) --keyframes-a $(rep $1)/trajectory_raw.tum --keyframes-b $(rep $2)/trajectory_raw.tum \
        --report $B/runs/j$1$2/join_report.yaml --truth $B/alone_A$1_B$2.json \
        --checks $B/runs/check$1$2/debug_checks.yaml $B/checks/perturb$1$2.csv > $B/eval/local_cons$1$2.json; echo "gate $1$2 exit=$?"
    done; set --
    for s in 1 2; do for m in asis stress; do
      python3 -P $S/p3_ms_gnss_local.py --no-floor split --bag $B/construction_seq${s}_gnss_fix.csv --front $(front $s) \
        --keyframes $(rep $s)/trajectory_raw.tum --report $B/runs/split${s}_$m/join_report.yaml > $B/eval/local_split${s}_$m.json
      echo "gate split${s}_$m exit=$?"
    done; done ;;
  esac
done
