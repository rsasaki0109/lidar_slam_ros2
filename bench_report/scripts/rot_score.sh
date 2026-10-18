#!/usr/bin/env bash
# Scores chapter 17's runs: MapsHD (mapshd_score.py, LiDAR frame, evo -a), Oxford (ape_oxford.py), exp04 (ape_runs.py,
# 7 and 6 checkpoints), MID-360 (residual of the loop back to the start, and the map metrics of R and of B =
# chapter 16's mid360_base, which is the chapter 14 configuration). Usage: rot_score.sh <out_dir>
set -uo pipefail
O=$1; R=/home/user/bench/rot; G=/home/user/data/mapshd/gt_sealed; S=/home/user/lidar_slam_ros2/bench_report/scripts
mkdir -p "${O}"
: > "${O}/mapshd_scores.csv"
for d in ${R}/*_trial-*_[BR]; do
  n=$(basename "$d"); t=${n%_[BR]}; sc=${t%_trial-*}; tr=${t#*_}
  python3 /home/user/scripts/mapshd_score.py "$d" "${G}/${sc}/${tr}/ground_truth.tum" "${O}/score_${n}" >> "${O}/mapshd_scores.csv"
done
PYTHONPATH=/home/user/lidar_slam_ros2 python3 /home/user/lidar_slam_ros2/scripts/densify_corrected_trajectory.py \
  --raw ${R}/oxford_R/front_0/front_tum_0.txt --corrected ${R}/oxford_R/trajectory_optimized.tum --output "${O}/oxford_R_dense.tum"
python3 ${S}/ape_oxford.py "${O}/oxford_R_dense.tum" /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
  /home/user/data/oxford_keble05/calib/T_imu_base.npy "${O}/ape_oxford_R" > "${O}/ape_oxford_R.txt"
python3 ${S}/ape_runs.py "${O}/ape_exp04.csv" ${R}/exp04_R > "${O}/ape_exp04.txt" 2>&1
python3 /home/user/scripts/loop_residuals.py ${R}/mid360_R ${R}/mid360_R/trajectory_optimized.tum > "${O}/mid360_R_loop_residuals.txt"
python3 /home/user/scripts/loop_residuals.py /home/user/bench/mem/mid360_base /home/user/bench/mem/mid360_base/trajectory_optimized.tum \
  > "${O}/mid360_B_loop_residuals.txt"
bash ${S}/map_quality_cmp.sh "${O}/mid360_map_quality.csv" /home/user/bench/mem/mid360_base ${R}/mid360_R
echo "score done"
