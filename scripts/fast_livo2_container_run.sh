#!/usr/bin/env bash
set -uo pipefail

# Run one official FAST-LIVO2 replay inside the pinned ROS1 container.
BAG_PATH="${BAG_PATH:?BAG_PATH is required}"
OUT_DIR="${OUT_DIR:-/out}"
RATE="${RATE:-1.0}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-5}"
SAVE_MAP="${SAVE_MAP:-0}"

mkdir -p "${OUT_DIR}"
source /runner/scripts/container_memory_evidence.sh
write_status() { printf '%s\n' "$2" >"${OUT_DIR}/$1"; }
cleanup() {
  local exit_status=$?
  set +e
  for process_group in "${ODOM_PID:-}" "${MAPPER_PID:-}" "${ROSCORE_PID:-}"; do
    if [[ -n "${process_group}" ]]; then
      kill -TERM -- "-${process_group}" >/dev/null 2>&1 || true
      kill -TERM "${process_group}" >/dev/null 2>&1 || true
    fi
  done
  # The shared EXIT finalizer stops the sampler and publishes evidence before
  # waiting for remaining children. This keeps early exits signal-safe while
  # retaining the existing process-group cleanup.
  m6a5_write_container_memory_evidence "${exit_status}" || true
  wait >/dev/null 2>&1 || true
  return "${exit_status}"
}
trap cleanup EXIT
m6a5_install_container_signal_traps
if ! m6a7_start_process_rss_sampler; then
  echo 'FAST process RSS sampler failed to start' >&2
  exit 11
fi

set +u
source /opt/ros/noetic/setup.bash
if [[ -f /opt/fast_livo_ws/devel/setup.bash ]]; then
  # Repo-owned pinned image workspace. The /bench mount is reserved for the
  # input asset tree and must not shadow the built image workspace.
  source /opt/fast_livo_ws/devel/setup.bash
elif [[ -f /bench/catkin_ws/devel/setup.bash ]]; then
  # Compatibility with the historical externally-built image.
  source /bench/catkin_ws/devel/setup.bash
else
  echo 'FAST-LIVO2 catkin workspace is missing' >&2
  exit 10
fi
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME="${OUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
if [[ "${SAVE_MAP}" == "1" ]]; then
  mkdir -p /bench/FAST-LIVO2/Log/pcd
fi

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
if [[ "${SAVE_MAP}" == "1" ]]; then
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

if [[ "${M6A3_SYNTHETIC_SMOKE:-0}" == "1" ]]; then
  # Consume the synthetic ROS1 bag briefly so this path checks input parsing,
  # loopback master connectivity, mapper startup, and clean shutdown without
  # producing a performance result.
  rosbag play --clock --rate "${RATE}" "${BAG_PATH}" \
    >"${OUT_DIR}/synthetic_smoke_rosbag_play.log" 2>&1 &
  smoke_bag_pid=$!
  sleep "${SMOKE_INPUT_SECONDS:-2}"
  kill -INT "${smoke_bag_pid}" >/dev/null 2>&1 || true
  wait "${smoke_bag_pid}" >/dev/null 2>&1 || true
  printf '%s\n' '{"status":"pass","startup_verified":true,"input_verified":true,"clean_shutdown":true,"gt_mounted":false,"performance_run":false,"loopback_only":true}' >"${OUT_DIR}/synthetic_smoke_contract.json"
  exit 0
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
