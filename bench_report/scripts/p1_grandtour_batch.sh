#!/usr/bin/env bash
# p1_grandtour_batch.sh <label> <zip_name> <hf_folder:short> ... (chapter 31): run p1_grandtour_run.sh on each mission
# in turn, delete that mission's downloaded and converted data once its trajectory is written (the VM's disk holds
# about one mission at a time), then pack the <short>.tum files at the root of
# /home/user/bench/grandtour/<label>/<zip_name>.zip, the COMFORT benchmark's submission format.
set -euo pipefail
label=$1; zip_name=$2; shift 2
S=/home/user/lidar_slam_ros2/bench_report/scripts; B=/home/user/bench/grandtour/${label}
files=()
for item in "$@"; do
  folder=${item%%:*}; short=${item##*:}
  if [[ ! -s ${B}/${short}/${short}.tum ]]; then
    bash "${S}/p1_grandtour_run.sh" "${folder}" "${short}" "${label}"
  fi
  [[ ${KEEP_INPUTS:-0} == 1 ]] || rm -rf "/home/user/data/grandtour/${short}/z" "/home/user/data/grandtour/${short}/bag"
  files+=("${short}/${short}.tum")
done
rm -f "${B}/${zip_name}.zip"
(cd "${B}" && zip -j -X "${zip_name}.zip" "${files[@]}")
(cd "${B}" && md5sum "${files[@]}" "${zip_name}.zip")
