#!/usr/bin/env bash
# p3_split_join_eval.sh <name> <backend input bag> <runner output dir> <extrinsic [qx,qy,qz,qw,x,y,z]> <out dir>
# (multi-session step 3, data/grandtour/multisession_2026-10-29/plan.txt): one session split in two
# (graph_slam_multisession_join debug_split_gap_s 30), joined as is (positive) and with the identity's matches excluded
# (stress: every match left is wrong). Prints one line per run: name,mode,keyframes,candidates,accepted,joined,
# error_to_identity_m,sets (size/places/share_below/passed;...),refusal (ambiguous / self-similar a|b, if any).
set -u
name=$1; bag=$2; run=$3; ext=$4; out=$5
J=${JOIN:-/home/user/ws/slam_hi/build/graph_slam_multisession_join}
P=/home/user/lidar_slam_ros2/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/ws/slam_dev/install/setup.bash; set -u
mkdir -p "$out"
for mode in positive stress; do
  ex=()
  [[ $mode == stress ]] && ex=(-p "debug_exclude_transform:=[1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0]")
  o=$out/${name}_$mode
  ROS_LOG_DIR=$out/roslog "$J" --ros-args --params-file "$P" -p "session_bag_paths:=['$bag']" \
    -p "session_backend_dirs:=['$run']" -p output_dir:="$o" -p extrinsic_lidar2base_quat_xyzw_xyz:="$ext" \
    -p debug_split_gap_s:=30.0 "${ex[@]}" > "$o.log" 2>&1
  /home/user/mapshd/venv/bin/python -I - "$o/join_report.yaml" "$name" "$mode" <<'PY'
import sys
import yaml
r = yaml.safe_load(open(sys.argv[1]))['split']
sets = ';'.join(f"{s['size']}/{s['places']}/{s['share_below']:.3f}/{'pass' if s['passed'] else 'fail'}"
                for s in (r.get('sets_tried') or []))
err = r.get('error_to_identity_rms_m', float('nan'))
refusal = 'ambiguous' if r.get('ambiguous') else (f"self-similar {r['self_similar']}" if r.get('self_similar') else '')
print(f"{sys.argv[2]},{sys.argv[3]},{r['keyframes_a']}+{r['keyframes_b']},{r['candidates']},{r['accepted']},"
      f"{'joined' if r['joined'] else 'not joined'},{err:.4f},{sets},{refusal}")
PY
done
rm -rf "$out/roslog"
