#!/usr/bin/env bash
set -uo pipefail

BAG_PATH="${BAG_PATH:?BAG_PATH is required}"
OUT_DIR="${OUT_DIR:-/out}"
OVERRIDE_CONFIG="${OVERRIDE_CONFIG:-/runner/configs/glim/hilti2022_cpu}"

# The container root is intentionally read-only.  Keep all ROS state inside
# the attempt output mount; no host home or GT path is used.
export ROS_HOME="${ROS_HOME:-${OUT_DIR}/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${OUT_DIR}/ros_log}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
source /runner/scripts/container_memory_evidence.sh
trap 'm6a5_container_exit_trap "$?"' EXIT

set +u
source /opt/ros/jazzy/setup.bash
source /opt/glim_ws/install/setup.bash
set -u

mkdir -p "${OUT_DIR}"
rm -rf /tmp/glim_benchmark_config
cp -a /opt/glim_ws/src/glim/config /tmp/glim_benchmark_config
for override in config.json config_sensors.json config_ros.json config_logging.json; do
  cp "${OVERRIDE_CONFIG}/${override}" "/tmp/glim_benchmark_config/${override}"
done

if [[ "${M6A3_SYNTHETIC_SMOKE:-0}" == "1" ]]; then
  smoke_log="${OUT_DIR}/synthetic_smoke_glim.log"
  setsid ros2 run glim_ros glim_rosbag "${BAG_PATH}" --ros-args \
    -p config_path:=/tmp/glim_benchmark_config \
    -p auto_quit:=false \
    -p dump_path:="${OUT_DIR}/smoke_dump" >"${smoke_log}" 2>&1 &
  smoke_pid=$!
  started=0
  for _ in $(seq 1 "${SMOKE_STARTUP_TIMEOUT_SECS:-20}"); do
    if ! kill -0 "${smoke_pid}" >/dev/null 2>&1; then
      break
    fi
    # ros2 node list can wait indefinitely when DDS discovery is isolated by
    # network=none.  GLIM's own config+bag-open records are the stronger
    # startup contract for this process-level smoke.
    if grep -Fq "config_path: /tmp/glim_benchmark_config" "${smoke_log}" && \
       grep -Fq "opening ${BAG_PATH}" "${smoke_log}" && \
       grep -Fq "Opened database" "${smoke_log}"; then
      started=1
      break
    fi
    sleep 1
  done
  kill -INT -- "-${smoke_pid}" >/dev/null 2>&1 || true
  clean_shutdown=0
  for _ in $(seq 1 "${SMOKE_SHUTDOWN_TIMEOUT_SECS:-10}"); do
    if ! kill -0 "${smoke_pid}" >/dev/null 2>&1; then
      clean_shutdown=1
      break
    fi
    sleep 1
  done
  if [[ "${clean_shutdown}" != "1" ]]; then
    kill -TERM -- "-${smoke_pid}" >/dev/null 2>&1 || true
    sleep 1
  fi
  wait "${smoke_pid}" >/dev/null 2>&1 || true
  if [[ "${started}" != "1" || ! -s "${smoke_log}" ]]; then
    echo 'error: synthetic GLIM process did not reach an observable node' >&2
    tail -n 80 "${smoke_log}" >&2 || true
    exit 13
  fi
  if [[ "${clean_shutdown}" != "1" ]]; then
    echo 'error: synthetic GLIM process did not cleanly stop within timeout' >&2
    exit 13
  fi
  if grep -Fq "/root/.ros/log" "${smoke_log}"; then
    echo 'error: GLIM attempted to write ROS state outside the attempt output' >&2
    exit 13
  fi
  printf '%s\n' '{"status":"pass","startup_verified":true,"input_verified":true,"clean_shutdown":true,"gt_mounted":false,"performance_run":false}' >"${OUT_DIR}/synthetic_smoke_contract.json"
  exit 0
fi

/usr/bin/time -v -o "${OUT_DIR}/process_time.txt" \
  ros2 run glim_ros glim_rosbag "${BAG_PATH}" --ros-args \
    -p config_path:=/tmp/glim_benchmark_config \
    -p auto_quit:=true \
    -p dump_path:="${OUT_DIR}/dump" \
  >"${OUT_DIR}/glim.log" 2>&1
status=$?
printf '%s\n' "${status}" >"${OUT_DIR}/process_exit_status.txt"
exit "${status}"
