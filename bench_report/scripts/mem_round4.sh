#!/usr/bin/env bash
# MapPoint map assembly and save (graph_based_slam working tree in slam_mem), warm caches, one run at a time:
#   mem4 = rko_lio 77072e4 + slam_mem, same parameters as the baseline;
#   mem5 = mem4 + frontend async.max_lidar_buffer_size 10 + backend use_pcd_cache.
set -uo pipefail
P=/home/user/bench/params
cat /home/user/ws/slam_mem/install/*/lib/*.so* /home/user/ws/slam_mem/install/graph_based_slam/lib/graph_based_slam/* > /dev/null 2>&1
M5=(RKO_SUFFIX=_buf10 YAML=${P}/lidarslam_competitive_v2_loop_ndt1_res2_pcdcache.yaml)
for s in mid360 oxford; do
  env WS=/home/user/ws/pipe_mem TAG=mem4 SEQS=${s} bash /home/user/scripts/mem_baseline.sh
  env "${M5[@]}" WS=/home/user/ws/pipe_mem TAG=mem5 SEQS=${s} WARM=0 bash /home/user/scripts/mem_baseline.sh
done
dd if=/home/user/data/hilti2022/exp04_ros2/exp04_ros2.db3 of=/dev/null bs=16M 2>&1 | tail -1
env WS=/home/user/ws/pipe_mem TAG=mem4 SEQS=exp04 WARM=0 bash /home/user/scripts/mem_baseline.sh
env "${M5[@]}" WS=/home/user/ws/pipe_mem TAG=mem5 SEQS=exp04 WARM=0 bash /home/user/scripts/mem_baseline.sh
cd /home/user/bench/mem
R="mid360_base mid360_mem4 mid360_mem5 oxford_base oxford_mem4 oxford_mem5 exp04_basew exp04_mem4 exp04_mem5"
bash /home/user/scripts/out_hashes.sh ${R}
for f in ${R}; do
  echo "$f sum=$(grep -E 'sum_of' $f.mem_summary.csv | cut -d, -f3) $(grep -E ',offline_node,|,graph_based_sla,' $f.mem_summary.csv | cut -d, -f2,3 | tr '\n' ' ')"
done
echo "round4 done"
