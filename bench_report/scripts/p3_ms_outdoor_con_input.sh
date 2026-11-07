#!/usr/bin/env bash
# con_input.sh <hf_mission_folder> <short>: one GrandTour mission's HS-G15g backend input, as chapter 38 made it
# (bench_report/scripts/p1_grandtour_run.sh + p3_grandtour_backend.sh step 2: Hesai XT-32 + STIM320, deskewed scans
# thinned to 0.1 m), with the repository's installed rko_lio. The converted bag is deleted afterwards.
set -euo pipefail
folder=$1; short=$2
REPO=/home/user/lidar_slam_ros2; S=$REPO/bench_report/scripts; PY=/home/user/mapshd/venv/bin/python
D=/home/user/data/grandtour/$short; O=/home/user/bench/ms_outdoor/$short; BAG=$D/bag_stim320_imu_hesai_points
PRISM="0.3088378015503127 -0.038312997211241984 -0.2349441159711169 -0.005724388155828644 -0.0013879431347362404 -0.004742721620119043 0.9999714053840392"
U=https://huggingface.co/datasets/leggedrobotics/grand_tour_dataset/resolve/main/${folder}/data
fetch() {
  local f=$1 k
  [[ -f $D/z/$f/.complete ]] && return 0
  for k in 1 2 3 4 5 6; do
    rm -rf "$D/z.part" "$D/z/$f"; mkdir -p "$D/z.part" "$D/z"
    if curl -sS -L --fail "$U/$f.tar" | tar xf - -C "$D/z.part"; then
      mv "$D/z.part/$f" "$D/z/$f"; rm -rf "$D/z.part"; touch "$D/z/$f/.complete"; return 0
    fi
    echo "download of $f failed (attempt $k)" >&2; sleep $(( k < 5 ? 60 * 2 ** (k - 1) : 900 ))
  done
  return 1
}
mkdir -p "$D/z" "$O"
set +u; source /opt/ros/jazzy/setup.bash; source $REPO/install/setup.bash; set -u
if [[ ! -d $BAG ]]; then
  fetch hesai_points; fetch stim320_imu
  rm -rf "$BAG.part"
  "$PY" "$S/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --consume "$D/z" "$BAG.part" > "$D/convert.log"
  mv "$BAG.part" "$BAG"
  find "$D/z" -type f ! -name '.z*' -delete 2>/dev/null || true
fi
# shellcheck disable=SC2086
"$PY" "$S/p1_grandtour_to_ros2.py" --imu stim320_imu --lidar hesai_points --extrinsics "$D/z" $PRISM > "$O/extrinsics.txt"
"$PY" - "$REPO/configs/sota_v7/rko_lio_grandtour_hs_G15g_2026-10-28.yaml" "$O/extrinsics.txt" "$O/params.yaml" <<'PY'
import sys
import yaml
g = yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
ext = {k: v for k, v in (yaml.safe_load(open(sys.argv[2])) or {}).items() if k.startswith('extrinsic_')}
rig = {'base_frame': 'base_link', 'odom_frame': 'odom', 'lidar_frame': '', 'imu_frame': '', 'odom_topic': '/rko_lio/odometry',
       'publish_deskewed_scan': False, 'dump_results': True, 'run_name': 'front', 'imu_acceleration_unit': 'auto'}
yaml.safe_dump({'/**': {'ros__parameters': {**rig, **ext, **g}}}, open(sys.argv[3], 'w'), sort_keys=False)
PY
export ROS_DOMAIN_ID=$((40 + RANDOM % 50)) ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$O/roslog
rm -rf "$O/backend_input" "$O/front"
BACKEND_CAPTURE_STORAGE_PRESET=zstd_fast bash "$REPO/scripts/record_backend_input.sh" --output-dir "$O/backend_input" -- \
  "$REPO/install/rko_lio/lib/rko_lio/offline_node" --ros-args --params-file "$O/params.yaml" \
  -p bag_path:="$BAG" -p lidar_topic:=/hesai_points -p imu_topic:=/stim320_imu \
  -p results_dir:="$O/front" -p publish_deskewed_scan:=true -p output_subscriber_timeout_sec:=60.0 \
  -p deskewed_scan_voxel_size:=0.1 > "$O/record.log" 2>&1
"$PY" -c "import sys, yaml; print(','.join(str(v) for v in yaml.safe_load(open(sys.argv[1]))['extrinsic_lidar2base_quat_xyzw_xyz']))" \
  "$O/extrinsics.txt" > "$O/extrinsic.txt"
rm -rf "$BAG"
echo "$short done: $(du -sh $O/backend_input | cut -f1), extrinsic $(cat $O/extrinsic.txt)"
