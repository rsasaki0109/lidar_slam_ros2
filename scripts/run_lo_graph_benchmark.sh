#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_lo_graph_benchmark.sh [options]

Options:
  --bag <dir>                    rosbag2 directory (Point Cloud2)
  --reference-tum <file>         Reference trajectory (TUM)
  --reference-meta <file>        Reference JSON (lidar_to_prism_translation_m for ape_from_tum)
  --input-cloud <topic>          Remapped scanmatcher input (default: /points_raw)
  --lidarslam-param <file>       LO YAML (default: lidarslam/param/lidarslam_lo.yaml)
  --robot-frame <frame>          scanmatcher robot_frame_id (default: base_link)
  --lidar-frame <frame>          static_tf child when publish_static_tf (default: lidar)
  --publish-static-tf BOOL       (default: true)
  --play-rate <float>            ros2 bag play --rate (default: 5.0)
  --play-delay-secs <sec>        Delay before bag play start (default: 1.0)
  --drain-secs <sec>             Wait after bag ends before map_save (default: 20)
  --output-dir <dir>             Output root
  --startup-timeout-secs <sec>   (default: 60)
  --save-timeout-secs <sec>      (default: 60)
  --skip-map-save
  --reference-source LABEL       metrics.json reference label
  --raw-path-topic TOPIC         scanmatcher path (default: /path)
  --corrected-path-topic TOPIC   graph path (default: /modified_path)
  --help

Pipeline:
  lo_slam (scanmatcher + graph) -> path loggers -> ros2 bag play -> APE -> metrics.json

  PointCloud2 playback uses --qos-profile-overrides-path so publishers match
  scanmatcher's SensorDataQoS (best_effort); avoids empty TUMs when the bag
  was recorded with reliable QoS.
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
WS_ROOT="${REPO_ROOT}"
if [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
fi

DEFAULT_LIDARSLAM_PARAM="${REPO_ROOT}/lidarslam/param/lidarslam_lo.yaml"

BAG_PATH=""
REFERENCE_TUM=""
REFERENCE_META=""
INPUT_CLOUD="/points_raw"
LIDARSLAM_PARAM="$DEFAULT_LIDARSLAM_PARAM"
ROBOT_FRAME="base_link"
LIDAR_FRAME="lidar"
PUBLISH_STATIC_TF=true
PLAY_RATE=5.0
PLAY_DELAY_SECS=1.0
DRAIN_SECS=20
OUTPUT_DIR=""
STARTUP_TIMEOUT_SECS=60
SAVE_TIMEOUT_SECS=60
SKIP_MAP_SAVE=false
REFERENCE_SOURCE="lidar_odometry_scanmatcher"
RAW_PATH_TOPIC="/path"
CORRECTED_PATH_TOPIC="/modified_path"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || usage
      BAG_PATH=$(realpath "$2")
      shift 2
      ;;
    --reference-tum)
      [[ $# -ge 2 ]] || usage
      REFERENCE_TUM=$(realpath -m "$2")
      shift 2
      ;;
    --reference-meta)
      [[ $# -ge 2 ]] || usage
      REFERENCE_META=$(realpath -m "$2")
      shift 2
      ;;
    --input-cloud)
      [[ $# -ge 2 ]] || usage
      INPUT_CLOUD="$2"
      shift 2
      ;;
    --lidarslam-param)
      [[ $# -ge 2 ]] || usage
      LIDARSLAM_PARAM=$(realpath "$2")
      shift 2
      ;;
    --robot-frame)
      [[ $# -ge 2 ]] || usage
      ROBOT_FRAME="$2"
      shift 2
      ;;
    --lidar-frame)
      [[ $# -ge 2 ]] || usage
      LIDAR_FRAME="$2"
      shift 2
      ;;
    --publish-static-tf)
      [[ $# -ge 2 ]] || usage
      PUBLISH_STATIC_TF="$2"
      shift 2
      ;;
    --play-rate)
      [[ $# -ge 2 ]] || usage
      PLAY_RATE="$2"
      shift 2
      ;;
    --play-delay-secs)
      [[ $# -ge 2 ]] || usage
      PLAY_DELAY_SECS="$2"
      shift 2
      ;;
    --drain-secs)
      [[ $# -ge 2 ]] || usage
      DRAIN_SECS="$2"
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || usage
      OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --startup-timeout-secs)
      [[ $# -ge 2 ]] || usage
      STARTUP_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --save-timeout-secs)
      [[ $# -ge 2 ]] || usage
      SAVE_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --skip-map-save)
      SKIP_MAP_SAVE=true
      shift
      ;;
    --reference-source)
      [[ $# -ge 2 ]] || usage
      REFERENCE_SOURCE="$2"
      shift 2
      ;;
    --raw-path-topic)
      [[ $# -ge 2 ]] || usage
      RAW_PATH_TOPIC="$2"
      shift 2
      ;;
    --corrected-path-topic)
      [[ $# -ge 2 ]] || usage
      CORRECTED_PATH_TOPIC="$2"
      shift 2
      ;;
    --help|-h)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
done

[[ -n "$BAG_PATH" ]] || usage
[[ -f "$REFERENCE_TUM" ]] || { echo "reference TUM not found: $REFERENCE_TUM" >&2; exit 1; }
[[ -f "$REFERENCE_META" ]] || { echo "reference meta JSON not found: $REFERENCE_META" >&2; exit 1; }
[[ -f "$LIDARSLAM_PARAM" ]] || { echo "lidarslam param not found: $LIDARSLAM_PARAM" >&2; exit 1; }
[[ -d "$BAG_PATH" ]] || { echo "rosbag2 dir not found: $BAG_PATH" >&2; exit 1; }
[[ -f "$BAG_PATH/metadata.yaml" ]] || { echo "metadata.yaml missing under $BAG_PATH" >&2; exit 1; }

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/bench_lo_graph_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR"

set +u
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "python3 not found" >&2; exit 1; }

if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${OUTPUT_DIR}/.ros_log"
fi
mkdir -p "$ROS_LOG_DIR"

STARTED_AT="$(date -Iseconds)"
STARTED_AT_UNIX="$(date +%s)"
BENCH_T0="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"

LAUNCH_LOG="${OUTPUT_DIR}/lo_slam.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
BAG_PLAY_LOG="${OUTPUT_DIR}/bag_play.log"
RAW_TUM="${OUTPUT_DIR}/traj_raw.tum"
CORRECTED_TUM="${OUTPUT_DIR}/traj_corrected.tum"
RAW_TUM_PRISM="${OUTPUT_DIR}/traj_raw_prism.tum"
CORRECTED_TUM_PRISM="${OUTPUT_DIR}/traj_corrected_prism.tum"
RAW_APE="${OUTPUT_DIR}/ape_raw_vs_gt.txt"
CORRECTED_APE="${OUTPUT_DIR}/ape_corrected_vs_gt.txt"
RAW_LOG="${OUTPUT_DIR}/path_raw_logger.log"
CORRECTED_LOG="${OUTPUT_DIR}/path_corrected_logger.log"
QOS_OVERRIDE_YAML="${OUTPUT_DIR}/rosbag2_play_qos.yaml"

BAG_PID=""
LAUNCH_PID=""
LAUNCH_PGID=""
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""

terminate_pid() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill "$pid" >/dev/null 2>&1 || true
  for _ in {1..10}; do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.2
  done
  kill -9 "$pid" >/dev/null 2>&1 || true
  wait "$pid" 2>/dev/null || true
}

terminate_process_group() {
  local pgid="${1:-}"
  local leader_pid="${2:-}"
  [[ -n "$pgid" ]] || return 0
  kill -- "-${pgid}" >/dev/null 2>&1 || true
  for _ in {1..10}; do
    if [[ -n "$leader_pid" ]] && ! kill -0 "$leader_pid" >/dev/null 2>&1; then
      wait "$leader_pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.2
  done
  kill -9 -- "-${pgid}" >/dev/null 2>&1 || true
  if [[ -n "$leader_pid" ]]; then
    wait "$leader_pid" 2>/dev/null || true
  fi
}

cleanup() {
  terminate_pid "$RAW_LOGGER_PID"
  terminate_pid "$CORRECTED_LOGGER_PID"
  terminate_pid "$BAG_PID"
  if [[ -n "$LAUNCH_PGID" ]]; then
    terminate_process_group "$LAUNCH_PGID" "$LAUNCH_PID"
  else
    terminate_pid "$LAUNCH_PID"
  fi
}
trap cleanup EXIT INT TERM

wait_for_log_pattern() {
  local pattern="$1"
  local timeout_secs="$2"
  local deadline=$((SECONDS + timeout_secs))
  while (( SECONDS < deadline )); do
    if grep -Fq "$pattern" "$LAUNCH_LOG" 2>/dev/null; then
      return 0
    fi
    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 1
    fi
    sleep 1
  done
  return 1
}

wait_for_map_outputs() {
  local timeout_secs="$1"
  local deadline=$((SECONDS + timeout_secs))
  while (( SECONDS < deadline )); do
    if [[ -f "$OUTPUT_DIR/map_projector_info.yaml" && -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

call_map_save_with_retry() {
  local deadline=$((SECONDS + SAVE_TIMEOUT_SECS))
  while (( SECONDS < deadline )); do
    if timeout 15 ros2 service call /map_save std_srvs/srv/Empty "{}" >"${MAP_SAVE_LOG}" 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch lidarslam lo_slam.launch.py \
    use_sim_time:=true \
    "main_param_dir:=${LIDARSLAM_PARAM}" \
    "input_cloud:=${INPUT_CLOUD}" \
    "robot_frame_id:=${ROBOT_FRAME}" \
    "lidar_frame:=${LIDAR_FRAME}" \
    "base_frame:=${ROBOT_FRAME}" \
    "publish_static_tf:=${PUBLISH_STATIC_TF}" \
    "save_dir:=${OUTPUT_DIR}" \
    "use_rviz:=false" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
  LAUNCH_PGID="$LAUNCH_PID"
else
  ros2 launch lidarslam lo_slam.launch.py \
    use_sim_time:=true \
    "main_param_dir:=${LIDARSLAM_PARAM}" \
    "input_cloud:=${INPUT_CLOUD}" \
    "robot_frame_id:=${ROBOT_FRAME}" \
    "lidar_frame:=${LIDAR_FRAME}" \
    "base_frame:=${ROBOT_FRAME}" \
    "publish_static_tf:=${PUBLISH_STATIC_TF}" \
    "save_dir:=${OUTPUT_DIR}" \
    "use_rviz:=false" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
fi

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic "$RAW_PATH_TOPIC" \
  --output "$RAW_TUM" \
  --use-sim-time true \
  >"${RAW_LOG}" 2>&1 &
RAW_LOGGER_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic "$CORRECTED_PATH_TOPIC" \
  --output "$CORRECTED_TUM" \
  --use-sim-time true \
  >"${CORRECTED_LOG}" 2>&1 &
CORRECTED_LOGGER_PID="$!"

if ! wait_for_log_pattern "[graph_based_slam]: initialization end" "$STARTUP_TIMEOUT_SECS"; then
  echo "Timed out waiting for graph_based_slam. Log tail:" >&2
  tail -n 80 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

sleep "$PLAY_DELAY_SECS"

cat >"$QOS_OVERRIDE_YAML" <<EOF
${INPUT_CLOUD}:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 5
EOF

ros2 bag play "$BAG_PATH" --clock --rate "${PLAY_RATE}" \
  --qos-profile-overrides-path "$QOS_OVERRIDE_YAML" \
  >"${BAG_PLAY_LOG}" 2>&1 &
BAG_PID="$!"

echo "LO stack is up; waiting for bag play to finish (pid ${BAG_PID})"
wait "$BAG_PID" || true
BAG_PID=""
echo "Bag finished; draining ${DRAIN_SECS}s for backend"
sleep "$DRAIN_SECS"

if [[ "$SKIP_MAP_SAVE" == "false" ]]; then
  echo "Calling /map_save ..."
  if ! call_map_save_with_retry; then
    echo "map_save failed. Log tail:" >&2
    tail -n 80 "$LAUNCH_LOG" >&2 || true
    cat "$MAP_SAVE_LOG" >&2 || true
    exit 1
  fi
  if ! wait_for_map_outputs "$SAVE_TIMEOUT_SECS"; then
    echo "Timed out waiting for map under $OUTPUT_DIR" >&2
    exit 1
  fi
fi

terminate_pid "$RAW_LOGGER_PID"
terminate_pid "$CORRECTED_LOGGER_PID"
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""

if [[ -n "$LAUNCH_PGID" ]]; then
  terminate_process_group "$LAUNCH_PGID" "$LAUNCH_PID"
elif [[ -n "$LAUNCH_PID" ]]; then
  terminate_pid "$LAUNCH_PID"
fi
LAUNCH_PID=""
LAUNCH_PGID=""

trap - EXIT INT TERM

if [[ -f "$RAW_TUM" ]] && [[ ! -s "$CORRECTED_TUM" ]]; then
  echo "warn: corrected trajectory missing or empty; copying raw -> corrected" >&2
  cp -f "$RAW_TUM" "$CORRECTED_TUM"
fi

readarray -t PRISM_OFFSET < <(python3 - "$REFERENCE_META" <<'PY'
import json
import sys
from pathlib import Path

meta = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
offset = meta.get('lidar_to_prism_translation_m') or {}
print(offset.get('x', 0.0))
print(offset.get('y', 0.0))
print(offset.get('z', 0.0))
PY
)

python3 "${SCRIPT_DIR}/apply_tum_frame_offset.py" \
  --in "$RAW_TUM" \
  --out "$RAW_TUM_PRISM" \
  --tx "${PRISM_OFFSET[0]}" \
  --ty "${PRISM_OFFSET[1]}" \
  --tz "${PRISM_OFFSET[2]}"

python3 "${SCRIPT_DIR}/apply_tum_frame_offset.py" \
  --in "$CORRECTED_TUM" \
  --out "$CORRECTED_TUM_PRISM" \
  --tx "${PRISM_OFFSET[0]}" \
  --ty "${PRISM_OFFSET[1]}" \
  --tz "${PRISM_OFFSET[2]}"

python3 "${SCRIPT_DIR}/ape_from_tum.py" \
  --ref "$REFERENCE_TUM" \
  --est "$RAW_TUM_PRISM" \
  --out "$RAW_APE"

python3 "${SCRIPT_DIR}/ape_from_tum.py" \
  --ref "$REFERENCE_TUM" \
  --est "$CORRECTED_TUM_PRISM" \
  --out "$CORRECTED_APE"

BENCH_T1="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"
BENCH_WALL_SEC="$(python3 - <<PY
t0 = float("${BENCH_T0}")
t1 = float("${BENCH_T1}")
print(t1 - t0)
PY
)"

python3 "${SCRIPT_DIR}/write_rko_lio_benchmark_metrics.py" \
  --pipeline lo \
  --out-dir "$OUTPUT_DIR" \
  --bag "$BAG_PATH" \
  --reference-tum "$REFERENCE_TUM" \
  --reference-meta "$REFERENCE_META" \
  --points-topic "$INPUT_CLOUD" \
  --imu-topic "" \
  --lidarslam-param "$LIDARSLAM_PARAM" \
  --rko-param "$LIDARSLAM_PARAM" \
  --run-name "$(basename "$OUTPUT_DIR")" \
  --raw-tum "$RAW_TUM_PRISM" \
  --corrected-tum "$CORRECTED_TUM_PRISM" \
  --raw-ape "$RAW_APE" \
  --corrected-ape "$CORRECTED_APE" \
  --launch-log "$LAUNCH_LOG" \
  --started-at "$STARTED_AT" \
  --started-at-unix "$STARTED_AT_UNIX" \
  --wall-sec "$BENCH_WALL_SEC" \
  --robot-frame-id "$ROBOT_FRAME" \
  --raw-path-topic "$RAW_PATH_TOPIC" \
  --corrected-path-topic "$CORRECTED_PATH_TOPIC" \
  --reference-source "$REFERENCE_SOURCE"

echo "LO benchmark completed"
echo "  output_dir: $OUTPUT_DIR"
echo "  metrics:    $OUTPUT_DIR/metrics.json"
