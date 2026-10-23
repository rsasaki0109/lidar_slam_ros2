#!/usr/bin/env bash
# Usage: official_extrinsic_run.sh fork|official <csv> <label> <out_dir>
# Official-binary extrinsic check on HILTI exp04, whose PandarXT-32 is mounted upside down (LiDAR-to-base
# extrinsic: a half turn). pipe_pipeline.sh (BARRIER=1) with chapter 25's C backend parameters and the exp04
# RKO-LIO parameters; the frontend is dev_c's source rko_lio (fork, which stamps its deskewed scans with the
# LiDAR frame) or the official /opt/ros/jazzy rko_lio 0.4.2 deb (which moves them into the base frame first).
# For "official", dev_c's rko_lio is hidden for the run so that the dev_c setup resolves rko_lio to /opt/ros.
mode=$1; csv=$2; label=$3; out=$4
ws=/home/user/ws/dev_c
if [[ ${mode} == official ]]; then
  mv "${ws}/install/rko_lio" "${ws}/install/.rko_lio.hidden" || exit 1
  trap 'mv "${ws}/install/.rko_lio.hidden" "${ws}/install/rko_lio"' EXIT
fi
BARRIER=1 RKO_PARAMS=/home/user/bench/params/rko_params.ros.yaml bash /home/user/scripts/pipe_pipeline.sh "${csv}" "${label}" \
  "${ws}" /home/user/bench/params/lidarslam_competitive_v2_loop_ndt1_res2_rot1e6_drift_adjt1e4_dist60_loopc.yaml "${out}"
grep -m1 -o 'Publishing [a-z_ ]*\(deskewed_cloud\|deskewed scan\) to [^.]*' "${out}/slam.launch.log"
