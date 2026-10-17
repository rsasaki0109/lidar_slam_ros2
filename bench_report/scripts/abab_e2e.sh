#!/usr/bin/env bash
# Usage: [RKO_PARAMS=...] abab_e2e.sh <csv> <labelA> <wsA> <yamlA> <labelB> <wsB> <yamlB> <pairs> [bag lidar imu]
# ABAB pairs of pipe_e2e.sh runs, one at a time. After each run it records the saved map's point count and
# md5 (<csv stem>_maps.csv), then deletes map.pcd and pointcloud_map/ to save disk.
csv=$1; la=$2; wa=$3; ya=$4; lb=$5; wb=$6; yb=$7; pairs=$8; shift 8
dir=$(dirname "${csv}"); stem=$(basename "${csv}" .csv)
for ((i = 1; i <= pairs; i++)); do
  for side in A B; do
    if [[ ${side} == A ]]; then l=${la}; w=${wa}; y=${ya}; else l=${lb}; w=${wb}; y=${yb}; fi
    out="${dir}/${stem}_${l}_${i}"
    /home/user/scripts/kill_ros_procs.sh > /dev/null
    /home/user/scripts/pipe_e2e.sh "${csv}" "${l}" "${w}" "${y}" "${out}" "$@" > /dev/null 2>&1
    pts=$(head -c 400 "${out}/map.pcd" 2>/dev/null | strings | awk '/^POINTS/ {print $2}')
    echo "${l},${i},${pts:-NA},$(md5sum "${out}/map.pcd" 2>/dev/null | cut -c1-32)" >> "${dir}/${stem}_maps.csv"
    rm -rf "${out}/map.pcd" "${out}/pointcloud_map" "${out}/roslog"
    tail -1 "${csv}"
  done
done
