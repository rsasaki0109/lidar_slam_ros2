#!/usr/bin/env bash
# p4_malloc_pairs.sh <seq> <pairs> (bench_report/data/memory/p4_malloc_2026-11-08/plan.txt): alternating pipeline runs
# with the launch default (malloc_mmap_threshold 131072, label "fixed") and malloc_mmap_threshold:=off ("dyn"), the first
# of each pair alternating. Each run: mem_sampler.py beside pipe_pipeline.sh (the per-process VmHWM), under the shared
# VM lock of the blind run (data/blind_v7_2026-11-08), so it never shares the CPU with a G or FAST-LIO2 blind run.
# <seq>: exp04 (the stream-converted development bag) or heap1 (GrandTour HEAP-1, Hesai XT-32 + STIM320, G12 HS rig).
# Rows: /home/user/bench/memB/rows.csv (pipe_pipeline.sh's columns) and peaks.csv (label, offline_node, graph_based_slam,
# sum of peaks, KiB).
set -uo pipefail
seq=$1; pairs=$2
S=/home/user/lidar_slam_ros2/bench_report/scripts; P=/home/user/bench/params; O=/home/user/bench/memB
YAML=${P}/lidarslam_competitive_v2_loop_ndt1_res2.yaml
case ${seq} in
  exp04) env=(BAG=/home/user/data/hilti2022/exp04_streamcheck/bag RKO_PARAMS=${P}/rko_params_gyro002.ros.yaml) ;;
  heap1) env=(BAG=/home/user/data/grandtour/heap-1/bag_time LIDAR=/hesai_points IMU=/stim320_imu
              RKO_PARAMS=${O}/heap1_rko_params.ros.yaml) ;;
  *) echo "unknown sequence ${seq}"; exit 2 ;;
esac
mkdir -p "${O}"
for ((k = 1; k <= pairs; k++)); do
  if ((k % 2)); then order=(fixed dyn); else order=(dyn fixed); fi
  for v in "${order[@]}"; do
    label=${seq}_${v}_p${k}; out=${O}/${label}
    extra=; [[ ${v} == dyn ]] && extra=malloc_mmap_threshold:=off
    exec 8>/home/user/bench/blind_v7/quiet.lock; flock -s 8
    stop="${out}.mem_stop"; rm -f "${stop}"
    python3 "${S}/mem_sampler.py" "${out}.mem" "${stop}" 8>&- &
    spid=$!
    env "${env[@]}" LAUNCH_EXTRA="${extra}" bash "${S}/pipe_pipeline.sh" "${O}/rows.csv" "${label}" /home/user/lidar_slam_ros2 \
      "${YAML}" "${out}" 8>&- | tail -1
    touch "${stop}"; wait "${spid}"; rm -f "${stop}"
    flock -u 8; exec 8>&-
    f=$(grep -E ',offline_node,' "${out}.mem_summary.csv" | cut -d, -f3 | head -1)
    b=$(grep -E ',graph_based_sla,' "${out}.mem_summary.csv" | cut -d, -f3 | head -1)
    t=$(grep -E '^sum_of_process_peaks' "${out}.mem_summary.csv" | cut -d, -f3)
    echo "${label},${f},${b},${t}" | tee -a "${O}/peaks.csv"
  done
done
