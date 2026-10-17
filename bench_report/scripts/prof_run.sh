#!/usr/bin/env bash
# Usage: [RKO_PARAMS=...] prof_run.sh <label> <yaml> [bag lidar imu]  - one pipe_e2e.sh run with the PROF build
label=$1; yaml=$2; shift 2
out=/home/user/bench/prof/${label}
mkdir -p /home/user/bench/prof
/home/user/scripts/kill_ros_procs.sh > /dev/null
/home/user/scripts/pipe_e2e.sh /home/user/bench/prof/prof.csv "${label}" /home/user/ws/pipe_dev_734d54f "${yaml}" "${out}" "$@" > /dev/null 2>&1
grep -o "PROF .*" "${out}/slam.launch.log" > "${out}/prof_lines.txt"
rm -rf "${out}/map.pcd" "${out}/pointcloud_map" "${out}/roslog"
tail -1 /home/user/bench/prof/prof.csv; wc -l < "${out}/prof_lines.txt"
