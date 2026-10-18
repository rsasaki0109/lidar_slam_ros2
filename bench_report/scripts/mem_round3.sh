#!/usr/bin/env bash
# Memory changes against the baseline, warm caches, one run at a time:
#   mem2 = rko_lio 77072e4 + graph_based_slam working tree (slam_mem), same parameters as the baseline;
#   mem3 = mem2 + frontend async.max_lidar_buffer_size 10 + backend use_pcd_cache (submap clouds on disk).
set -uo pipefail
P=/home/user/bench/params
cat /home/user/ws/slam_mem/install/*/lib/*.so* /home/user/ws/slam_mem/install/graph_based_slam/lib/graph_based_slam/* \
  /home/user/ws/slam_dev/install/*/lib/*.so* /home/user/ws/slam_dev/install/graph_based_slam/lib/graph_based_slam/* > /dev/null 2>&1
M3=(RKO_SUFFIX=_buf10 YAML=${P}/lidarslam_competitive_v2_loop_ndt1_res2_pcdcache.yaml)
for s in mid360 oxford; do
  env WS=/home/user/ws/pipe_mem TAG=mem2 SEQS=${s} bash /home/user/scripts/mem_baseline.sh
  env "${M3[@]}" WS=/home/user/ws/pipe_mem TAG=mem3 SEQS=${s} WARM=0 bash /home/user/scripts/mem_baseline.sh
done
dd if=/home/user/data/hilti2022/exp04_ros2/exp04_ros2.db3 of=/dev/null bs=16M 2>&1 | tail -1
env WS=/home/user/ws/pipe_dev_40b3a06 TAG=basew SEQS=exp04 WARM=0 bash /home/user/scripts/mem_baseline.sh
env WS=/home/user/ws/pipe_mem TAG=mem2 SEQS=exp04 WARM=0 bash /home/user/scripts/mem_baseline.sh
env "${M3[@]}" WS=/home/user/ws/pipe_mem TAG=mem3 SEQS=exp04 WARM=0 bash /home/user/scripts/mem_baseline.sh
cd /home/user/bench/mem
bash /home/user/scripts/out_hashes.sh mid360_base mid360_mem2 mid360_mem3 oxford_base oxford_mem2 oxford_mem3 exp04_basew exp04_mem2 exp04_mem3
for f in mid360_base mid360_mem2 mid360_mem3 oxford_base oxford_mem2 oxford_mem3 exp04_basew exp04_mem2 exp04_mem3; do
  echo "$f sum=$(grep -E 'sum_of' $f.mem_summary.csv | cut -d, -f3) $(grep -E ',offline_node,|,graph_based_sla,' $f.mem_summary.csv | cut -d, -f2,3 | tr '\n' ' ')"
done
echo "round3 done"
