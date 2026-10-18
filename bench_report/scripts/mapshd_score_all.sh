#!/usr/bin/env bash
# Score every MapsHD run (mapshd_score.py) and GLIM's dump on calibration-room trial-1 (evo_ape -a, as the benchmark).
set -uo pipefail
OUT=/home/user/bench/mapshd; G=/home/user/data/mapshd/gt_sealed
echo "run,trajectory,frame,poses,evo_pairs,evo_rmse,evo_mean,evo_median,evo_max,interp_rmse" > ${OUT}/scores.csv
for d in ${OUT}/*_trial-*_[AB]; do
  name=$(basename ${d}); trial=${name%_[AB]}; sc=${trial%_trial-*}; t=trial-${trial##*_trial-}
  python3 /home/user/scripts/mapshd_score.py ${d} ${G}/${sc}/${t}/ground_truth.tum ${OUT}/score/${name} >> ${OUT}/scores.csv
done
echo "glim_file,evo_rmse,evo_mean,evo_median,evo_max" > ${OUT}/glim_scores.csv
for f in traj_lidar traj_imu odom_lidar odom_imu; do
  r=$(MPLBACKEND=Agg /home/user/mapshd/venv/bin/evo_ape tum ${G}/calibration-room/trial-1/ground_truth.tum \
      ${OUT}/glim_calibration-room_trial-1/dump/${f}.txt -a | tee ${OUT}/glim_calibration-room_trial-1/${f}.evo.txt)
  echo "${f},$(echo "${r}" | awk '$1=="rmse"{a=$2} $1=="mean"{b=$2} $1=="median"{c=$2} $1=="max"{d=$2} END{print a","b","c","d}')" >> ${OUT}/glim_scores.csv
done
echo "scoring done"
