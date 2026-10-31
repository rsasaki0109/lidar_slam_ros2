#!/usr/bin/env bash
# cleanup_runner_check.sh <runner binary> <tag> (architecture cleanup 2, report chapter 42): graph_slam_offline_runner
# on recorded backend inputs with several option sets; every output file's md5 goes to
# /home/user/bench/cleanup2/<tag>/md5.txt, so two runner builds can be compared file by file.
#   exp04_*: HILTI 2022 exp04 (dev_loops/rb/exp04/backend_input), the walking preset plus
#     default | causal (causal_trajectory_output) | sg05 (SMALL_GICP 0.5 m) | t4v05 (4 NDT threads, 0.5 m loop voxel)
#   cal1_*: MapsHD calibration-room trial 1 (15 loop edges with the preset), default | causal | sg05
#   kitti_t4v05: KITTI 05 (kitti_raw/prof/backend_input) with t4v05 and the causal output (about 12 minutes)
set -u
runner=$1; tag=$2
REPO=/home/user/lidar_slam_ros2; PRESET=${REPO}/lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml
O=/home/user/bench/cleanup2/${tag}; rm -rf "$O"; mkdir -p "$O"
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/ws/slam_dev/install/setup.bash; set -u
EXT_E=$(python3 -c "import sys, yaml; p = yaml.safe_load(open(sys.argv[1])); p = next(iter(p.values()))['ros__parameters']
print('[' + ','.join(str(v) for v in p['extrinsic_lidar2base_quat_xyzw_xyz']) + ']')" ${REPO}/bench_report/params/exp04_gyro002_jerk1000.yaml)
EXT_M=$(python3 -c "import sys, yaml; p = yaml.safe_load(open(sys.argv[1])); p = next(iter(p.values()))['ros__parameters']
print('[' + ','.join(str(v) for v in p['extrinsic_lidar2base_quat_xyzw_xyz']) + ']')" ${REPO}/bench_report/params/mapshd_gyro002_jerk1000.yaml)
EXT_K="[-0.007412088,0.001015086,0.000385188,0.999971941,0.810543972,-0.307054372,0.802723995]"
run() {  # run <name> <input> <extrinsic> [overrides...]
  local name=$1 in=$2 ext=$3; shift 3
  local a=(); for kv in "$@"; do a+=(-p "$kv"); done
  local t0=$(date +%s)
  "${runner}" --ros-args --params-file "${PRESET}" -p bag_path:="${in}" -p output_dir:="$O/${name}" \
    -p "extrinsic_lidar2base_quat_xyzw_xyz:=${ext}" "${a[@]}" > "$O/${name}.log" 2>&1
  echo "${name} exit $? $(( $(date +%s) - t0 )) s"
  (cd "$O/${name}" && find . -type f | sort | xargs md5sum) | sed "s|  \./|  ${name}/|" >> "$O/md5.txt"
}
E=/home/user/bench/dev_loops/rb/exp04/backend_input; M=/home/user/bench/dev_loops/rb/calibration-room_trial-1/backend_input
K=/home/user/bench/kitti_raw/prof/backend_input
run exp04_default "$E" "$EXT_E"
run exp04_causal "$E" "$EXT_E" causal_trajectory_output:=true
run exp04_sg05 "$E" "$EXT_E" registration_method:=SMALL_GICP gicp_max_correspondence_distance:=0.5 gicp_transformation_epsilon:=1e-4
run exp04_t4v05 "$E" "$EXT_E" ndt_num_threads:=4 voxel_leaf_size:=0.5
run cal1_default "$M" "$EXT_M"
run cal1_causal "$M" "$EXT_M" causal_trajectory_output:=true
run cal1_sg05 "$M" "$EXT_M" registration_method:=SMALL_GICP gicp_max_correspondence_distance:=0.5 gicp_transformation_epsilon:=1e-4
[[ ${SKIP_KITTI:-0} == 1 ]] || run kitti_t4v05 "$K" "$EXT_K" ndt_num_threads:=4 voxel_leaf_size:=0.5 causal_trajectory_output:=true
echo CHECK_DONE
