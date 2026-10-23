#!/usr/bin/env bash
# Scores chapter 27's D runs like c_score.sh scored chapter 25's C (MapsHD mapshd_score.py, Oxford ape_oxford.py,
# exp04 ape_runs.py, MID-360 loop residuals and map metrics). Usage: d_score.sh <out_dir>
set -uo pipefail
O=$1; R=/home/user/bench/loopd/pipe; G=/home/user/data/mapshd/gt_sealed; S=/home/user/lidar_slam_ros2/bench_report/scripts
mkdir -p "${O}"
: > "${O}/mapshd_scores.csv"; : > "${O}/loop_features.csv"
for d in ${R}/*_D; do python3 /home/user/scripts/loop_features.py "$(basename "$d")" "$d" >> "${O}/loop_features.csv"; done
for d in ${R}/*_trial-*_D; do
  n=$(basename "$d"); t=${n%_D}
  python3 /home/user/scripts/mapshd_score.py "$d" "${G}/${t%_trial-*}/${t#*_}/ground_truth.tum" "${O}/score_${n}" >> "${O}/mapshd_scores.csv"
done
PYTHONPATH=/home/user/lidar_slam_ros2 python3 /home/user/lidar_slam_ros2/scripts/densify_corrected_trajectory.py \
  --raw ${R}/oxford_D/front_0/front_tum_0.txt --corrected ${R}/oxford_D/trajectory_optimized.tum --output "${O}/oxford_D_dense.tum"
python3 ${S}/ape_oxford.py "${O}/oxford_D_dense.tum" /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
  /home/user/data/oxford_keble05/calib/T_imu_base.npy "${O}/ape_oxford_D" > "${O}/ape_oxford_D.txt"
python3 ${S}/ape_oxford.py ${R}/oxford_D/front_0/front_tum_0.txt /home/user/data/oxford_keble05/gt_sealed/gt-tum.txt \
  /home/user/data/oxford_keble05/calib/T_imu_base.npy "${O}/ape_oxford_D_front" > "${O}/ape_oxford_D_front.txt"
python3 ${S}/ape_runs.py "${O}/ape_exp04.csv" ${R}/exp04_D > "${O}/ape_exp04.txt" 2>&1
python3 /home/user/scripts/loop_residuals.py ${R}/mid360_D ${R}/mid360_D/trajectory_optimized.tum > "${O}/mid360_D_loop_residuals.txt"
: > "${O}/mid360_map_quality.csv"
bash ${S}/map_quality_cmp.sh "${O}/mid360_map_quality.csv" ${R}/mid360_D
echo "score done"
