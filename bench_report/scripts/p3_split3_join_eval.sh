#!/usr/bin/env bash
# p3_split3_join_eval.sh <name> <backend input bag> <runner output dir> <extrinsic [qx,qy,qz,qw,x,y,z]> <out dir>
# (multi-session step 7, data/grandtour/multisession_2026-10-29/plan.txt): one session cut into 3 parts with 30 s gaps
# (graph_slam_multisession_join debug_split_parts 3), each part a session; four runs: positive and stress (the matches
# near the identity left out: any join is wrong), each joining to the map and to the first part only. The parts'
# self-similarity is stored in <out dir>/<name>_cache and shared by the four runs (RUNS to choose, e.g.
# "positive:map"). Prints one line per run:
# name,mode,join_to,keyframes per part,part 1,part 2,seconds; a part is "joined p<pass> map<sessions> <error to the
# identity, m>", "not joined", "ambiguous" or "self-similar a|b".
set -u
name=$1; bag=$2; run=$3; ext=$4; out=$5
J=${JOIN:-/home/user/ws/slam_hi/build/graph_slam_multisession_join}
P=/home/user/lidar_slam_ros2/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/ws/slam_dev/install/setup.bash; set -u
mkdir -p "$out"
for run_kind in ${RUNS:-positive:map positive:first stress:map stress:first}; do
  mode=${run_kind%%:*}; to=${run_kind##*:}
  {
    ex=()
    [[ $mode == stress ]] && ex=(-p "debug_exclude_transform:=[1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0]")
    o=$out/${name}_${mode}_$to
    t0=$(date +%s)
    ROS_LOG_DIR=$out/roslog "$J" --ros-args --params-file "$P" -p "session_bag_paths:=['$bag']" \
      -p "session_backend_dirs:=['$run']" -p output_dir:="$o" -p extrinsic_lidar2base_quat_xyzw_xyz:="$ext" \
      -p debug_split_gap_s:=30.0 -p debug_split_parts:=3 -p multisession_join_to:=$to \
      -p multisession_cache_dir:="$out/${name}_cache" "${ex[@]}" ${EXTRA_PARAMS:-} > "$o.log" 2>&1
    seconds=$(( $(date +%s) - t0 ))
    /home/user/mapshd/venv/bin/python -I - "$o" "$name" "$mode" "$to" "$seconds" <<'PY'
import re
import sys
import yaml
o, name, mode, to, seconds = sys.argv[1:6]
log = open(o + '.log').read()
parts = '+'.join(re.findall(r'part \d+: (\d+) submaps', log))
try:
    r = yaml.safe_load(open(o + '/join_report.yaml'))
except FileNotFoundError:
    print(f'{name},{mode},{to},{parts},FAILED,FAILED,{seconds}')
    sys.exit(0)
joins = {m['session']: m for m in r.get('map_joins') or []}
cells = []
for s in r['sessions'][1:]:
    m = joins[s['session']]
    if s['joined']:
        cells.append(f"joined p{m['joined_in_pass']} map{m['map_sessions']} {m['error_to_identity_rms_m']:.4f}".replace(',', ''))
    elif s.get('ambiguous'):
        cells.append('ambiguous')
    elif s.get('self_similar'):
        cells.append(f"self-similar {s['self_similar']}")
    else:
        cells.append('not joined')
print(f'{name},{mode},{to},{parts},{cells[0]},{cells[1]},{seconds}')
PY
  }
done
rm -rf "$out/roslog"
