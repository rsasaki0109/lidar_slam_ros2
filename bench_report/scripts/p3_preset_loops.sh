#!/bin/bash
# Resumable runs of bench_report/data/loops_preset_2026-11-07/plan.txt (REPORT chapter 50).
# Needs: the RTK-SLAM Stadtgarten bags (ros2/<seq>_lio), the rtk-slam-eval checkout (RTK_SLAM_EVAL), the
# chapter 49 KITTI backend inputs, and $B/rko_stadtgarten_seq2.ros.yaml: the frontend preset
# configs/mid360_robot/rko_lio_rtk_slam_mid360_stadtgarten_seq2.yaml under "/**: ros__parameters:".
# usage: p3_preset_loops.sh STEP...   steps: rec:SEQ  be:LABEL:ARM  score:SEQ   (SEQ = stadtgarten_seq1 | stadtgarten_seq2)
set -uo pipefail
REPO=${REPO:-$(cd "$(dirname "$0")/../.." && pwd)}; B=${OUT:-/home/user/bench/loops_preset}
RUNNER=$REPO/install/graph_based_slam/lib/graph_based_slam/graph_slam_offline_runner
PRESET=$REPO/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
EVAL=$REPO/bench_report/scripts/rtk_slam_checkpoint_eval.py
GTD=${RTK_SLAM_EVAL:-/home/user/data/rtk_slam/rtk_slam_eval}/ground_truth
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
input_of() { case $1 in
  stadtgarten_seq1|stadtgarten_seq2) echo $B/$1/backend_input ;;
  kitti05) echo /home/user/bench/kitti_raw/v2_05/backend_input ;;
  kitti07) echo /home/user/bench/kitti_raw/k07_A2/backend_input ;;
esac; }
ext_of() { case $1 in
  kitti05|kitti07) /home/user/mapshd/venv/bin/python -c "import sys, yaml; p = yaml.safe_load(open(sys.argv[1])); p = next(iter(p.values()))['ros__parameters']
print(','.join(str(v) for v in p['extrinsic_lidar2base_quat_xyzw_xyz']))" $REPO/bench_report/data/kitti/raw05_2026-10-31/rig_kitti_raw.yaml ;;
  *) echo 0.0,0.0,0.0,1.0,0.0,0.0,0.0 ;;
esac; }
for step in "$@"; do
  IFS=: read -r kind a b <<< "$step"
  export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_LOG_DIR=$B/roslog
  case $kind in
  rec)
    O=$B/$a; [[ -s $O/backend_input/metadata.yaml ]] && { echo "rec $a: present"; continue; }
    rm -rf $O/backend_input $O/front; mkdir -p $O
    t0=$(date +%s.%N)
    BACKEND_CAPTURE_STORAGE_PRESET=zstd_fast bash $REPO/scripts/record_backend_input.sh --output-dir $O/backend_input \
      --setup $REPO/install/setup.bash -- \
      $REPO/install/rko_lio/lib/rko_lio/offline_node --ros-args \
      -p bag_path:=/home/user/data/rtk_slam/ros2/${a}_lio -p lidar_topic:=/livox/points -p imu_topic:=/livox/imu \
      -p base_frame:=os_sensor -p odom_frame:=odom -p publish_deskewed_scan:=true \
      -p odom_topic:=/rko_lio/odometry -p deskewed_scan_topic:=/rko_lio/frame -p publisher_queue_depth:=128 \
      -p output_subscriber_timeout_sec:=60.0 -p dump_results:=true -p results_dir:=$O/front -p run_name:=front \
      --params-file $B/rko_stadtgarten_seq2.ros.yaml > $O/record.log 2>&1
    echo "rec $a exit=$? $(python3 -c "print(f'{$(date +%s.%N)-${t0}:.1f}')") s; $(du -sh $O/backend_input | cut -f1)" ;;
  be)
    I=$(input_of $a); O=$B/runs/$a/$b; mkdir -p $(dirname $O)
    [[ -s $O/trajectory_optimized.tum ]] && { echo "be $a $b: present"; continue; }
    case $b in
      ndt2) over=() ;;
      sg05|sg05b) over=(-p registration_method:=SMALL_GICP -p gicp_max_correspondence_distance:=0.5 -p gicp_transformation_epsilon:=1e-4) ;;
    esac
    rm -rf $O
    $RUNNER --ros-args --params-file $PRESET -p bag_path:=$I -p output_dir:=$O \
      -p "extrinsic_lidar2base_quat_xyzw_xyz:=[$(ext_of $a)]" "${over[@]}" > $O.log 2>&1
    rc=$?
    t_first=$(grep -m1 -oP '^\[\w+\] \[\K[0-9.]+' $O.log); t_done=$(grep -m1 'Offline run complete' $O.log | grep -oP '^\[\w+\] \[\K[0-9.]+')
    loops=$(grep -h "Offline run complete" $O.log | grep -oP '[0-9]+(?= loop edges)')
    echo "be,$a,$b,exit=$rc,bagpass=$(python3 -c "print(f'{${t_done:-0}-${t_first:-0}:.1f}')"),loops=${loops:-NA}" | tee -a $B/times.csv ;;
  score)
    O=$B/$a; F=$(ls $O/front/*/*.txt 2>/dev/null | grep -i tum | head -1); [[ -n $F ]] || F=$(find $O/front -name '*tum*' | head -1)
    case $a in stadtgarten_seq1) ORG=/home/user/bench/s1/s1_seq2preset.origin.json ;; stadtgarten_seq2) ORG=/home/user/bench/gnss/prod_v3.origin.json ;; esac
    ev() { python3 -I $EVAL $GTD/$a.csv --traj "$1" --origin $ORG --lever-arm -0.062 0.0 -0.216 --label "$2" | python3 -c "
import json,sys; d=json.load(sys.stdin); print('%s,se3_rmse_3d=%.4f,n=%d/%d' % (d['label'], d['se3_rmse_3d'], d['n'], d['n_gt']))"; }
    echo "front md5 $(md5sum < $F | cut -c1-8) $F"
    ev "$F" "$a,front" | tee -a $B/scores.csv
    for v in ndt2 sg05; do R=$B/runs/$a/$v; [[ -s $R/trajectory_optimized.tum ]] || continue
      PYTHONPATH=$REPO python3 $REPO/scripts/densify_corrected_trajectory.py --raw "$F" --corrected $R/trajectory_optimized.tum --output $R/dense.tum > $R/densify.log 2>&1
      ev $R/dense.tum "$a,$v" | tee -a $B/scores.csv; done ;;
  esac
done
