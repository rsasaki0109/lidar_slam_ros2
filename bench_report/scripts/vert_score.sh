#!/usr/bin/env bash
# Scores chapter 20's V runs like gate_score.sh scored chapter 18's (MapsHD mapshd_score.py, Oxford ape_oxford.py,
# exp04 ape_runs.py, MID-360 loop residuals and map metrics). Usage: vert_score.sh <out_dir>
set -uo pipefail
O=$1; R=/home/user/bench/vert/pipe; G=/home/user/data/mapshd/gt_sealed; S=/home/user/lidar_slam_ros2/bench_report/scripts
mkdir -p "${O}"
: > "${O}/mapshd_scores.csv"; : > "${O}/loop_features.csv"
for d in ${R}/*_V; do python3 /home/user/scripts/loop_features.py "$(basename "$d")" "$d" >> "${O}/loop_features.csv"; done
for d in ${R}/*_trial-*_V; do
  n=$(basename "$d"); t=${n%_V}
  python3 /home/user/scripts/mapshd_score.py "$d" "${G}/${t%_trial-*}/${t#*_}/ground_truth.tum" "${O}/score_${n}" >> "${O}/mapshd_scores.csv"
done
PYTHONPATH=/home/user/lidar_slam_ros2 python3 /home/user/lidar_slam_ros2/scripts/densify_corrected_trajectory.py \
  --raw ${R}/oxford_V/front_0/front_tum_0.txt --corrected ${R}/oxford_V/trajectory_optimized.tum --output "${O}/oxford_V_dense.tum"
python3 ${S}/ape_oxford.py "${O}/oxford_V_dense.tum" /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
  /home/user/data/oxford_keble05/calib/T_imu_base.npy "${O}/ape_oxford_V" > "${O}/ape_oxford_V.txt"
python3 ${S}/ape_oxford.py ${R}/oxford_V/front_0/front_tum_0.txt /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
  /home/user/data/oxford_keble05/calib/T_imu_base.npy "${O}/ape_oxford_V_front" > "${O}/ape_oxford_V_front.txt"
python3 ${S}/ape_runs.py "${O}/ape_exp04.csv" ${R}/exp04_V > "${O}/ape_exp04.txt" 2>&1
python3 /home/user/scripts/loop_residuals.py ${R}/mid360_V ${R}/mid360_V/trajectory_optimized.tum > "${O}/mid360_V_loop_residuals.txt"
: > "${O}/mid360_map_quality.csv"
bash ${S}/map_quality_cmp.sh "${O}/mid360_map_quality.csv" ${R}/mid360_V
echo "score done"
