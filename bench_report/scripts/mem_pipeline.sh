#!/usr/bin/env bash
# One pipeline run (pipe_pipeline.sh) with mem_sampler.py running alongside.
# Usage: [BAG=.. LIDAR=.. IMU=.. RKO_PARAMS=..] mem_pipeline.sh <csv> <label> <ws> <lidarslam_yaml> <out_dir>
set -uo pipefail
csv=$1; label=$2; out=$5
bash /home/user/scripts/kill_ros_procs.sh > /dev/null 2>&1
mkdir -p "$(dirname "${out}")"
stop="${out}.mem_stop"; rm -f "${stop}"
python3 /home/user/scripts/mem_sampler.py "${out}.mem" "${stop}" &
spid=$!
bash /home/user/scripts/pipe_pipeline.sh "$@" | tail -1
touch "${stop}"; wait "${spid}"; rm -f "${stop}"
grep -E "sum_of|peak_of|offline_node|graph_based" "${out}.mem_summary.csv" | cut -d, -f1-3
