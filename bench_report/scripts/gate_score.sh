#!/usr/bin/env bash
# Scores chapter 18's runs (R and G of every sequence) as chapter 17 did: MapsHD (mapshd_score.py, LiDAR frame,
# evo -a), Oxford (ape_oxford.py on the densified trajectory), exp04 (ape_runs.py, 7 and 6 checkpoints), MID-360
# (residual of every loop, map metrics). Also the GT-free loop features of every run (loop_features.py).
# Usage: gate_score.sh <out_dir>
set -uo pipefail
O=$1; R=/home/user/bench/gate; G=/home/user/data/mapshd/gt_sealed; S=/home/user/lidar_slam_ros2/bench_report/scripts
mkdir -p "${O}"
: > "${O}/mapshd_scores.csv"; : > "${O}/loop_features.csv"
for d in ${R}/*_[RG]; do
  n=$(basename "$d"); python3 /home/user/scripts/loop_features.py "$n" "$d" >> "${O}/loop_features.csv"
done
for d in ${R}/*_trial-*_[RG]; do
  n=$(basename "$d"); t=${n%_[RG]}; sc=${t%_trial-*}; tr=${t#*_}
  python3 /home/user/scripts/mapshd_score.py "$d" "${G}/${sc}/${tr}/ground_truth.tum" "${O}/score_${n}" >> "${O}/mapshd_scores.csv"
done
for c in R G; do
  PYTHONPATH=/home/user/lidar_slam_ros2 python3 /home/user/lidar_slam_ros2/scripts/densify_corrected_trajectory.py \
    --raw ${R}/oxford_${c}/front_0/front_tum_0.txt --corrected ${R}/oxford_${c}/trajectory_optimized.tum \
    --output "${O}/oxford_${c}_dense.tum"
  python3 ${S}/ape_oxford.py "${O}/oxford_${c}_dense.tum" /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
    /home/user/data/oxford_keble05/calib/T_imu_base.npy "${O}/ape_oxford_${c}" > "${O}/ape_oxford_${c}.txt"
  python3 /home/user/scripts/loop_residuals.py ${R}/mid360_${c} ${R}/mid360_${c}/trajectory_optimized.tum \
    > "${O}/mid360_${c}_loop_residuals.txt"
done
python3 ${S}/ape_runs.py "${O}/ape_exp04.csv" ${R}/exp04_R ${R}/exp04_G > "${O}/ape_exp04.txt" 2>&1
: > "${O}/mid360_map_quality.csv"
bash ${S}/map_quality_cmp.sh "${O}/mid360_map_quality.csv" ${R}/mid360_R ${R}/mid360_G
echo "score done"
