#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_rko_lio_graph_benchmark.sh [options]

Options:
  --bag <dir>                    Restamped rosbag2 used by RKO-LIO
  --reference-bag <dir>          rosbag2 directory containing /leica/pose/relative
  --reference-tum <file>         Reference TUM trajectory path
  --reference-meta <file>        Reference metadata JSON path
  --lidar-topic <topic>          LiDAR topic
  --imu-topic <topic>            IMU topic
  --base-frame <frame>           RKO-LIO base frame (default: base_link)
  --lidarslam-param <file>       graph_based_slam parameter YAML
  --rko-param <file>             RKO-LIO parameter YAML
  --output-dir <dir>             Output directory for logs and artifacts
  --run-name <name>              RKO-LIO run name
  --startup-timeout-secs <sec>   Timeout waiting for startup (default: 30)
  --save-timeout-secs <sec>      Timeout waiting for map outputs (default: 60)
  --quiescence-secs <sec>        Treat the run as done after this many stable seconds (default: 20)
  --skip-map-save                Do not call /map_save or verify the map bundle
  --skip-reference-gen           Reuse an existing reference TUM/meta without regenerating it
  --publish-static-tf BOOL       static_transform_publisher (default: true)
  --reference-source LABEL       Label stored in metrics.json (default: leica_prism_gt)
  --help                         Show this help

This runs the recommended benchmark path:
  rosbag2 -> RKO-LIO + graph_based_slam -> raw/corrected trajectories -> APE -> metrics.json
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
WS_ROOT="${REPO_ROOT}"
if [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
fi

DEFAULT_BAG="${REPO_ROOT}/demo_data/ntu_viral/tnp_01_points_restamped_vn100_rosbag2"
DEFAULT_REFERENCE_BAG="${REPO_ROOT}/demo_data/ntu_viral/tnp_01_rosbag2"
DEFAULT_REFERENCE_TUM="${REPO_ROOT}/output/ntu_viral_tnp01_gt_leica.tum"
DEFAULT_REFERENCE_META="${REPO_ROOT}/output/ntu_viral_tnp01_reference.json"
DEFAULT_LIDAR_TOPIC="/os1_cloud_node1/points"
DEFAULT_IMU_TOPIC="/imu/imu"
DEFAULT_BASE_FRAME="base_link"
DEFAULT_LIDARSLAM_PARAM="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
DEFAULT_RKO_PARAM="${REPO_ROOT}/lidarslam/param/rko_lio_ntu_viral.yaml"

BAG_PATH="$DEFAULT_BAG"
REFERENCE_BAG="$DEFAULT_REFERENCE_BAG"
REFERENCE_TUM="$DEFAULT_REFERENCE_TUM"
REFERENCE_META="$DEFAULT_REFERENCE_META"
LIDAR_TOPIC="$DEFAULT_LIDAR_TOPIC"
IMU_TOPIC="$DEFAULT_IMU_TOPIC"
BASE_FRAME="$DEFAULT_BASE_FRAME"
LIDARSLAM_PARAM="$DEFAULT_LIDARSLAM_PARAM"
RKO_PARAM="$DEFAULT_RKO_PARAM"
OUTPUT_DIR=""
RUN_NAME=""
STARTUP_TIMEOUT_SECS=30
SAVE_TIMEOUT_SECS=60
QUIESCENCE_SECS=20
SKIP_MAP_SAVE=false
SKIP_REFERENCE_GEN=false
PUBLISH_STATIC_TF=true
REFERENCE_SOURCE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || usage
      BAG_PATH=$(realpath "$2")
      shift 2
      ;;
    --reference-bag)
      [[ $# -ge 2 ]] || usage
      REFERENCE_BAG=$(realpath "$2")
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
    --lidar-topic)
      [[ $# -ge 2 ]] || usage
      LIDAR_TOPIC="$2"
      shift 2
      ;;
    --imu-topic)
      [[ $# -ge 2 ]] || usage
      IMU_TOPIC="$2"
      shift 2
      ;;
    --base-frame)
      [[ $# -ge 2 ]] || usage
      BASE_FRAME="$2"
      shift 2
      ;;
    --lidarslam-param)
      [[ $# -ge 2 ]] || usage
      LIDARSLAM_PARAM=$(realpath "$2")
      shift 2
      ;;
    --rko-param)
      [[ $# -ge 2 ]] || usage
      RKO_PARAM=$(realpath "$2")
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || usage
      OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --run-name)
      [[ $# -ge 2 ]] || usage
      RUN_NAME="$2"
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
    --quiescence-secs)
      [[ $# -ge 2 ]] || usage
      QUIESCENCE_SECS="$2"
      shift 2
      ;;
    --skip-map-save)
      SKIP_MAP_SAVE=true
      shift
      ;;
    --skip-reference-gen)
      SKIP_REFERENCE_GEN=true
      shift
      ;;
    --publish-static-tf)
      [[ $# -ge 2 ]] || usage
      PUBLISH_STATIC_TF="$2"
      shift 2
      ;;
    --reference-source)
      [[ $# -ge 2 ]] || usage
      REFERENCE_SOURCE="$2"
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

default_benchmark_bag_missing_hint() {
  cat >&2 <<EOF
Default NTU VIRAL benchmark bags not found under demo_data/.

Prepare them with:
  bash scripts/download_ntu_viral_tnp01.sh
EOF
}

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/bench_rko_lio_ntu_viral_$(date +%Y%m%d_%H%M%S)"
fi
if [[ -z "$RUN_NAME" ]]; then
  RUN_NAME="$(basename "$OUTPUT_DIR")"
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "rosbag2 directory not found: $BAG_PATH" >&2
  if [[ "$BAG_PATH" == "$DEFAULT_BAG" ]]; then
    default_benchmark_bag_missing_hint
  fi
  exit 1
fi
[[ -f "$BAG_PATH/metadata.yaml" ]] || { echo "metadata.yaml not found under $BAG_PATH" >&2; exit 1; }
[[ -f "$LIDARSLAM_PARAM" ]] || { echo "lidarslam param file not found: $LIDARSLAM_PARAM" >&2; exit 1; }
[[ -f "$RKO_PARAM" ]] || { echo "RKO-LIO param file not found: $RKO_PARAM" >&2; exit 1; }
if [[ "$SKIP_REFERENCE_GEN" == "false" ]]; then
  if [[ ! -d "$REFERENCE_BAG" ]]; then
    echo "reference rosbag2 directory not found: $REFERENCE_BAG" >&2
    if [[ "$REFERENCE_BAG" == "$DEFAULT_REFERENCE_BAG" ]]; then
      default_benchmark_bag_missing_hint
    fi
    exit 1
  fi
  [[ -f "$REFERENCE_BAG/metadata.yaml" ]] || { echo "metadata.yaml not found under $REFERENCE_BAG" >&2; exit 1; }
fi

set +u
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found in PATH" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "python3 not found in PATH" >&2; exit 1; }

mkdir -p "$OUTPUT_DIR"
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

LAUNCH_LOG="${OUTPUT_DIR}/slam.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
RAW_TUM="${OUTPUT_DIR}/traj_raw.tum"
CORRECTED_TUM="${OUTPUT_DIR}/traj_corrected.tum"
RAW_TUM_PRISM="${OUTPUT_DIR}/traj_raw_prism.tum"
CORRECTED_TUM_PRISM="${OUTPUT_DIR}/traj_corrected_prism.tum"
RAW_APE="${OUTPUT_DIR}/ape_raw_vs_gt.txt"
CORRECTED_APE="${OUTPUT_DIR}/ape_corrected_vs_gt.txt"
RAW_LOG="${OUTPUT_DIR}/odom_logger.log"
CORRECTED_LOG="${OUTPUT_DIR}/path_logger.log"
RKO_ROS_PARAM_FILE="${OUTPUT_DIR}/rko_params.ros.yaml"
RKO_RESULT_TUM="${OUTPUT_DIR}/${RUN_NAME}_0/${RUN_NAME}_tum_0.txt"

LAUNCH_PID=""
LAUNCH_PGID=""
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""

cleanup() {
  terminate_pid "$RAW_LOGGER_PID"
  terminate_pid "$CORRECTED_LOGGER_PID"
  if [[ -n "$LAUNCH_PGID" ]]; then
    terminate_process_group "$LAUNCH_PGID" "$LAUNCH_PID"
  else
    terminate_pid "$LAUNCH_PID"
  fi
}
trap cleanup EXIT INT TERM

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

run_state_signature() {
  python3 - "$RAW_TUM" "$CORRECTED_TUM" "$LAUNCH_LOG" <<'PY'
import os
import sys

parts = []
for path in sys.argv[1:]:
    if not path or not os.path.exists(path):
        parts.append("missing")
        continue
    st = os.stat(path)
    parts.append(f"{int(st.st_mtime)}:{st.st_size}")
print("|".join(parts))
PY
}

offline_completion_recorded() {
  if grep -Fq "RKO LIO Offline Node took" "$LAUNCH_LOG" 2>/dev/null; then
    return 0
  fi
  if [[ -f "$OUTPUT_DIR/rko_lio.log" ]] && grep -Fq "RKO LIO Offline Node took" "$OUTPUT_DIR/rko_lio.log" 2>/dev/null; then
    return 0
  fi
  return 1
}

wait_for_offline_completion() {
  local timeout_secs="$1"
  local last_signature=""
  local stable_since=0
  local deadline=$((SECONDS + timeout_secs))

  while (( SECONDS < deadline )); do
    if [[ -s "$RKO_RESULT_TUM" ]]; then
      return 0
    fi
    if offline_completion_recorded; then
      return 0
    fi

    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 0
    fi

    local current_signature
    current_signature="$(run_state_signature)"
    if [[ "$current_signature" == "$last_signature" ]] && [[ -f "$RAW_TUM" ]]; then
      if (( stable_since == 0 )); then
        stable_since=$SECONDS
      fi
      if (( SECONDS - stable_since >= QUIESCENCE_SECS )); then
        echo "No new trajectory/log updates for ${QUIESCENCE_SECS}s; treating benchmark run as complete"
        return 0
      fi
    else
      stable_since=0
      last_signature="$current_signature"
    fi

    sleep 2
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

python3 - "$RKO_PARAM" "$RKO_ROS_PARAM_FILE" <<'PY'
import shutil
import sys
from pathlib import Path

import yaml

src_path = Path(sys.argv[1])
dst_path = Path(sys.argv[2])
data = yaml.safe_load(src_path.read_text()) or {}

if isinstance(data, dict) and any(
    isinstance(v, dict) and "ros__parameters" in v for v in data.values()
):
    shutil.copyfile(src_path, dst_path)
    sys.exit(0)

wrapped = {"/**": {"ros__parameters": data}}
dst_path.write_text(yaml.safe_dump(wrapped, sort_keys=False))
PY

if [[ "$SKIP_REFERENCE_GEN" == "false" || ! -f "$REFERENCE_TUM" || ! -f "$REFERENCE_META" ]]; then
  python3 "${SCRIPT_DIR}/generate_ntu_viral_tnp01_reference.py" \
    --source-bag "$REFERENCE_BAG" \
    --out "$REFERENCE_TUM" \
    --rko-param "$RKO_PARAM" \
    --write-meta "$REFERENCE_META"
fi

echo "Running RKO-LIO benchmark"
echo "  bag:            $BAG_PATH"
echo "  reference_tum:  $REFERENCE_TUM"
echo "  reference_meta: $REFERENCE_META"
echo "  lidar_topic:    $LIDAR_TOPIC"
echo "  imu_topic:      $IMU_TOPIC"
echo "  base_frame:     $BASE_FRAME"
echo "  lidarslam_yaml: $LIDARSLAM_PARAM"
echo "  rko_yaml:       $RKO_PARAM"
echo "  output_dir:     $OUTPUT_DIR"
echo "  run_name:       $RUN_NAME"

if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch lidarslam rko_lio_slam.launch.py \
    "main_param_dir:=${LIDARSLAM_PARAM}" \
    "rko_param_file:=${RKO_ROS_PARAM_FILE}" \
    "bag_path:=${BAG_PATH}" \
    "lidar_topic:=${LIDAR_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "base_frame:=${BASE_FRAME}" \
    "publish_static_tf:=${PUBLISH_STATIC_TF}" \
    "save_dir:=${OUTPUT_DIR}" \
    "results_dir:=${OUTPUT_DIR}" \
    "run_name:=${RUN_NAME}" \
    "dump_results:=true" \
    "use_rviz:=false" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
  LAUNCH_PGID="$LAUNCH_PID"
else
  ros2 launch lidarslam rko_lio_slam.launch.py \
    "main_param_dir:=${LIDARSLAM_PARAM}" \
    "rko_param_file:=${RKO_ROS_PARAM_FILE}" \
    "bag_path:=${BAG_PATH}" \
    "lidar_topic:=${LIDAR_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "base_frame:=${BASE_FRAME}" \
    "publish_static_tf:=${PUBLISH_STATIC_TF}" \
    "save_dir:=${OUTPUT_DIR}" \
    "results_dir:=${OUTPUT_DIR}" \
    "run_name:=${RUN_NAME}" \
    "dump_results:=true" \
    "use_rviz:=false" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
fi

python3 "${SCRIPT_DIR}/odom_to_tum.py" \
  --topic /rko_lio/odometry \
  --output "$RAW_TUM" \
  --use-sim-time false \
  >"${RAW_LOG}" 2>&1 &
RAW_LOGGER_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic /modified_path \
  --output "$CORRECTED_TUM" \
  --use-sim-time false \
  >"${CORRECTED_LOG}" 2>&1 &
CORRECTED_LOGGER_PID="$!"

if ! wait_for_log_pattern "RKO LIO Node is up!" "$STARTUP_TIMEOUT_SECS"; then
  echo "Timed out waiting for RKO-LIO startup. Recent launch log:" >&2
  tail -n 80 "$LAUNCH_LOG" >&2 || true
  exit 1
fi
if ! wait_for_log_pattern "[graph_based_slam]: initialization end" "$STARTUP_TIMEOUT_SECS"; then
  echo "Timed out waiting for graph_based_slam startup. Recent launch log:" >&2
  tail -n 80 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

echo "SLAM launch is up"
if ! wait_for_offline_completion 1800; then
  echo "Launch terminated before offline node completed. Recent launch log:" >&2
  tail -n 120 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

if [[ "$SKIP_MAP_SAVE" == "false" ]]; then
  echo "Calling /map_save ..."
  if ! call_map_save_with_retry; then
    echo "map_save service call failed. Recent launch log:" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    cat "$MAP_SAVE_LOG" >&2 || true
    exit 1
  fi
  if ! wait_for_map_outputs "$SAVE_TIMEOUT_SECS"; then
    echo "Timed out waiting for saved map outputs under $OUTPUT_DIR" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi
fi

for pid in "$RAW_LOGGER_PID" "$CORRECTED_LOGGER_PID"; do
  terminate_pid "$pid"
done
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""

if [[ -n "$LAUNCH_PGID" ]]; then
  terminate_process_group "$LAUNCH_PGID" "$LAUNCH_PID"
elif [[ -n "$LAUNCH_PID" ]]; then
  terminate_pid "$LAUNCH_PID"
fi
LAUNCH_PID=""
LAUNCH_PGID=""

readarray -t PRISM_OFFSET < <(python3 - "$REFERENCE_META" <<'PY'
import json
import sys
from pathlib import Path

meta = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
offset = meta.get("lidar_to_prism_translation_m") or {}
print(offset.get("x", 0.0))
print(offset.get("y", 0.0))
print(offset.get("z", 0.0))
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

METRICS_ARGS=(
  --out-dir "$OUTPUT_DIR"
  --bag "$BAG_PATH"
  --reference-tum "$REFERENCE_TUM"
  --reference-meta "$REFERENCE_META"
  --points-topic "$LIDAR_TOPIC"
  --imu-topic "$IMU_TOPIC"
  --lidarslam-param "$LIDARSLAM_PARAM"
  --rko-param "$RKO_PARAM"
  --run-name "$RUN_NAME"
  --raw-tum "$RAW_TUM_PRISM"
  --corrected-tum "$CORRECTED_TUM_PRISM"
  --raw-ape "$RAW_APE"
  --corrected-ape "$CORRECTED_APE"
  --launch-log "$LAUNCH_LOG"
  --started-at "$STARTED_AT"
  --started-at-unix "$STARTED_AT_UNIX"
  --wall-sec "$BENCH_WALL_SEC"
)
if [[ -n "$REFERENCE_SOURCE" ]]; then
  METRICS_ARGS+=(--reference-source "$REFERENCE_SOURCE")
fi
python3 "${SCRIPT_DIR}/write_rko_lio_benchmark_metrics.py" "${METRICS_ARGS[@]}"

echo "Benchmark completed"
echo "  output_dir:     $OUTPUT_DIR"
echo "  metrics_json:   $OUTPUT_DIR/metrics.json"
echo "  raw_ape:        $RAW_APE"
echo "  corrected_ape:  $CORRECTED_APE"
