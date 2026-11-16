#!/usr/bin/env bash
# p9_odyssey_bisect_step.sh SEQ THRESHOLD_M (bench_report/data/odyssey_dev_2026-11-13/plan.txt, addendum 3): one step
# of `git bisect run` in the fork's worktree /home/user/ws/rko_bisect/src/rko_lio. Builds the checked-out commit,
# runs its offline node with the "up" parameter file (upstream's defaults) on the kept bag
# /dev/shm/odyssey_dev/<seq>/bag and scores it as in H9. Exit 0 (good) if ATE < THRESHOLD_M, 1 (bad) otherwise,
# 125 (skip) if the commit does not build or the run fails.
set -uo pipefail
REPO=/home/user/lidar_slam_ros2; S=${REPO}/bench_report/scripts; B=/home/user/bench/odyssey_dev; WS=/home/user/ws/rko_bisect
seq=$1; thr=$2
sha=$(git -C "${WS}/src/rko_lio" rev-parse --short=8 HEAD)
out=${B}/${seq}/bis_${sha}
if [[ ! -f ${out}/exit.txt ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  (cd "${WS}" && colcon build --packages-select rko_lio --cmake-args -DCMAKE_BUILD_TYPE=Release -DRKO_LIO_BUILD_ROS=ON \
     > "${WS}/build_${sha}.log" 2>&1) || { echo "${sha} build failed" >> "${B}/bisect.log"; exit 125; }
  rm -rf "${out:?}"
  RKO_WS=${WS} bash "${S}/p8_blind_method.sh" G "${B}/variants/rig_up.yaml" "/dev/shm/odyssey_dev/${seq}/bag" "${out}" \
    > /dev/null 2>> "${B}/${seq}/errors.log"
fi
line=$(cd /tmp && /home/user/mapshd/venv/bin/python "${S}/p9_odyssey_dev_score.py" "${B}" /home/user/data/blind2/odyssey/gt 2>/dev/null |
  grep -E "^${seq} bis_${sha} ")
ate=$(awk '{print $4}' <<< "${line}")
echo "$(date -u +%FT%TZ) ${sha} $(git -C "${WS}/src/rko_lio" log -1 --format=%s | cut -c1-70) | ${line}" >> "${B}/bisect.log"
[[ ${ate} =~ ^[0-9.]+$ ]] || exit 125
awk -v a="${ate}" -v t="${thr}" 'BEGIN { exit !(a < t) }'
