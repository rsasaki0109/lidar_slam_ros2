#!/usr/bin/env bash
set -uo pipefail

BAG_PATH="${BAG_PATH:?BAG_PATH is required}"
OUT_DIR="${OUT_DIR:-/out}"
OVERRIDE_CONFIG="${OVERRIDE_CONFIG:-/runner/configs/glim/hilti2022_cpu}"

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

/usr/bin/time -v -o "${OUT_DIR}/process_time.txt" \
  ros2 run glim_ros glim_rosbag "${BAG_PATH}" --ros-args \
    -p config_path:=/tmp/glim_benchmark_config \
    -p auto_quit:=true \
    -p dump_path:="${OUT_DIR}/dump" \
  >"${OUT_DIR}/glim.log" 2>&1
status=$?
printf '%s\n' "${status}" >"${OUT_DIR}/process_exit_status.txt"
exit "${status}"
