#!/usr/bin/env bash
set -uo pipefail

# Run one official FAST-LIVO2 replay inside the pinned ROS1 container.
BAG_PATH="${BAG_PATH:?BAG_PATH is required}"
OUT_DIR="${OUT_DIR:-/out}"
RATE="${RATE:-1.0}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-5}"
SAVE_MAP="${SAVE_MAP:-0}"
MAPPING_LAUNCH_PATH="${MAPPING_LAUNCH:-}"
MAPPING_MAP_LAUNCH_PATH="${MAPPING_MAP_LAUNCH:-}"

mkdir -p "${OUT_DIR}"
set +u
source /opt/ros/noetic/setup.bash
source /bench/catkin_ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOME="${OUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
if [[ "${SAVE_MAP}" == "1" ]]; then
  mkdir -p /bench/FAST-LIVO2/Log/pcd
fi

write_status() { printf '%s\n' "$2" >"${OUT_DIR}/$1"; }
cleanup() {
  set +e
  for process_group in "${ODOM_PID:-}" "${MAPPER_PID:-}" "${ROSCORE_PID:-}"; do
    if [[ -n "${process_group}" ]]; then
      kill -TERM -- "-${process_group}" >/dev/null 2>&1 || true
      kill -TERM "${process_group}" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT

rosbag info --yaml "${BAG_PATH}" >"${OUT_DIR}/rosbag_info.yaml" 2>"${OUT_DIR}/rosbag_info.err"
write_status rosbag_info_exit_status.txt "$?"

setsid roscore >"${OUT_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!
master_ready=0
for _ in $(seq 1 50); do
  if rosparam list >/dev/null 2>&1; then master_ready=1; break; fi
  sleep 0.1
done
write_status master_ready.txt "${master_ready}"
if [[ "${master_ready}" != 1 ]]; then exit 20; fi
rosparam set use_sim_time true

MAPPING_LAUNCH=(fast_livo mapping_hesaixt32_hilti22.launch rviz:=false)
if [[ -n "${MAPPING_LAUNCH_PATH}" ]]; then
  MAPPING_LAUNCH=("${MAPPING_LAUNCH_PATH}")
fi
if [[ "${SAVE_MAP}" == "1" && -n "${MAPPING_MAP_LAUNCH_PATH}" ]]; then
  MAPPING_LAUNCH=("${MAPPING_MAP_LAUNCH_PATH}")
elif [[ "${SAVE_MAP}" == "1" ]]; then
  MAPPING_LAUNCH=(/runner/configs/fast_livo2/mapping_hesaixt32_hilti22_benchmark_map.launch)
fi
setsid /usr/bin/time -v -o "${OUT_DIR}/mapper_time.txt" \
  roslaunch "${MAPPING_LAUNCH[@]}" \
  >"${OUT_DIR}/mapper.log" 2>&1 &
MAPPER_PID=$!
mapper_ready=0
for _ in $(seq 1 300); do
  if ! kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then break; fi
  if rostopic info /aft_mapped_to_init 2>/dev/null | grep -q 'Type: nav_msgs/Odometry'; then
    mapper_ready=1
    break
  fi
  sleep 0.1
done
write_status mapper_ready.txt "${mapper_ready}"
if [[ "${mapper_ready}" != 1 ]]; then
  wait "${MAPPER_PID}"
  write_status mapper_shutdown_exit_status.txt "$?"
  exit 21
fi

setsid rostopic echo -p /aft_mapped_to_init \
  >"${OUT_DIR}/odometry.csv" 2>"${OUT_DIR}/odometry.err" &
ODOM_PID=$!

/usr/bin/time -v -o "${OUT_DIR}/bag_time.txt" \
  rosbag play --clock --rate "${RATE}" "${BAG_PATH}" \
  >"${OUT_DIR}/rosbag_play.log" 2>&1
BAG_EXIT=$?
write_status bag_exit_status.txt "${BAG_EXIT}"

# The mapper is a continuous node. Record input-completion health separately
# from its response to the benchmark's external shutdown request.
mapper_alive=0
if kill -0 "${MAPPER_PID}" >/dev/null 2>&1 && rosnode ping -c 1 /laserMapping >/dev/null 2>&1; then
  mapper_alive=1
fi
write_status mapper_alive_after_bag.txt "${mapper_alive}"
sleep "${SHUTDOWN_GRACE_SECONDS}"

kill -TERM -- "-${ODOM_PID}" >/dev/null 2>&1 || true
wait "${ODOM_PID}" >/dev/null 2>&1 || true
ODOM_PID=""
if kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then
  kill -INT -- "-${MAPPER_PID}" >/dev/null 2>&1 || true
fi
wait "${MAPPER_PID}"
MAPPER_EXIT=$?
write_status mapper_shutdown_exit_status.txt "${MAPPER_EXIT}"
MAPPER_PID=""

if [[ "${BAG_EXIT}" -ne 0 || "${mapper_alive}" -ne 1 ]]; then exit 22; fi
exit 0
