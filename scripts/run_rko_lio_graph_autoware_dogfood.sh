#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
WS_ROOT="${REPO_ROOT}"
if [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
fi

usage() {
  cat <<'EOF' >&2
Usage:
  run_rko_lio_graph_autoware_dogfood.sh [options]

Options:
  --bag <dir>                    rosbag2 directory
  --lidar-topic <topic>          LiDAR topic
  --imu-topic <topic>            IMU topic
  --lidarslam-param <file>       graph_based_slam parameter YAML
  --rko-param <file>             RKO-LIO parameter YAML
  --output-dir <dir>             Directory for SLAM outputs and logs
  --run-name <name>              RKO-LIO run_name
  --save-timeout-secs <sec>      Timeout waiting for saved map files (default: 60)
  --startup-timeout-secs <sec>   Timeout waiting for SLAM node startup (default: 30)
  --wait-for-offline-completion  Wait for the full offline bag run to finish before saving
  --viewer-run-dir <dir>         Reuse an existing built Autoware map-loader run directory
  --viewer-rebuild               Rebuild the minimal Autoware runtime workspace before viewing
  --auto-exit-secs <sec>         Auto-close RViz after N seconds
  --autoware-core-dir <dir>      autoware_core checkout for the viewer
  --work-dir <dir>               Runtime workspace directory for the viewer
  --keep-launch                  Keep the SLAM launch alive after map save
  --skip-viewer                  Stop after verified map output without opening a viewer
  --help                         Show this help

Defaults target the NTU VIRAL tnp_01 restamped VN100 rosbag2 currently stored in this repository.
The script runs RKO-LIO + graph_based_slam, waits for offline odometry to finish, calls /map_save,
then stages the resulting map for Autoware and opens it in the host's rviz2.
EOF
  exit 1
}

DEFAULT_BAG="${REPO_ROOT}/demo_data/ntu_viral/tnp_01_points_restamped_vn100_rosbag2"
DEFAULT_LIDAR_TOPIC="/os1_cloud_node1/points"
DEFAULT_IMU_TOPIC="/imu/imu"
DEFAULT_LIDARSLAM_PARAM="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
DEFAULT_RKO_PARAM="${REPO_ROOT}/lidarslam/param/rko_lio_ntu_viral.yaml"
DEFAULT_AUTOWARE_CORE="/tmp/autoware_core"
DEFAULT_WORK_DIR="/tmp/autoware_map_runtime_ws"

BAG_PATH="$DEFAULT_BAG"
LIDAR_TOPIC="$DEFAULT_LIDAR_TOPIC"
IMU_TOPIC="$DEFAULT_IMU_TOPIC"
LIDARSLAM_PARAM="$DEFAULT_LIDARSLAM_PARAM"
RKO_PARAM="$DEFAULT_RKO_PARAM"
OUTPUT_DIR=""
RUN_NAME=""
SAVE_TIMEOUT_SECS=60
STARTUP_TIMEOUT_SECS=30
VIEWER_RUN_DIR=""
VIEWER_REBUILD=false
AUTO_EXIT_SECS=""
AUTOWARE_CORE_DIR="$DEFAULT_AUTOWARE_CORE"
WORK_DIR="$DEFAULT_WORK_DIR"
KEEP_LAUNCH=false
WAIT_FOR_OFFLINE_COMPLETION=false
SKIP_VIEWER=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || usage
      BAG_PATH=$(realpath "$2")
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
    --save-timeout-secs)
      [[ $# -ge 2 ]] || usage
      SAVE_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --startup-timeout-secs)
      [[ $# -ge 2 ]] || usage
      STARTUP_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --viewer-run-dir)
      [[ $# -ge 2 ]] || usage
      VIEWER_RUN_DIR=$(realpath "$2")
      shift 2
      ;;
    --wait-for-offline-completion)
      WAIT_FOR_OFFLINE_COMPLETION=true
      shift
      ;;
    --viewer-rebuild)
      VIEWER_REBUILD=true
      shift
      ;;
    --auto-exit-secs)
      [[ $# -ge 2 ]] || usage
      AUTO_EXIT_SECS="$2"
      shift 2
      ;;
    --autoware-core-dir)
      [[ $# -ge 2 ]] || usage
      AUTOWARE_CORE_DIR=$(realpath "$2")
      shift 2
      ;;
    --work-dir)
      [[ $# -ge 2 ]] || usage
      WORK_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --keep-launch)
      KEEP_LAUNCH=true
      shift
      ;;
    --skip-viewer)
      SKIP_VIEWER=true
      shift
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

default_bag_missing_hint() {
  cat >&2 <<EOF
Default NTU VIRAL dogfood bag not found: ${DEFAULT_BAG}

Prepare it with:
  bash scripts/download_ntu_viral_tnp01.sh
EOF
}

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/dogfood_rko_lio_autoware_$(date +%Y%m%d_%H%M%S)"
fi

if [[ -z "$RUN_NAME" ]]; then
  RUN_NAME="$(basename "$OUTPUT_DIR")"
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "rosbag2 directory not found: $BAG_PATH" >&2
  if [[ "$BAG_PATH" == "$DEFAULT_BAG" ]]; then
    default_bag_missing_hint
  fi
  exit 1
fi
[[ -f "$BAG_PATH/metadata.yaml" ]] || { echo "metadata.yaml not found under $BAG_PATH" >&2; exit 1; }
[[ -f "$LIDARSLAM_PARAM" ]] || { echo "lidarslam param file not found: $LIDARSLAM_PARAM" >&2; exit 1; }
[[ -f "$RKO_PARAM" ]] || { echo "RKO-LIO param file not found: $RKO_PARAM" >&2; exit 1; }
if [[ "$SKIP_VIEWER" == "false" ]]; then
  [[ -d "$AUTOWARE_CORE_DIR" ]] || { echo "autoware_core directory not found: $AUTOWARE_CORE_DIR" >&2; exit 1; }
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

mkdir -p "$OUTPUT_DIR"
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${OUTPUT_DIR}/.ros_log"
fi
mkdir -p "$ROS_LOG_DIR"

LAUNCH_LOG="${OUTPUT_DIR}/slam.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
RKO_ROS_PARAM_FILE="${OUTPUT_DIR}/rko_params.ros.yaml"
LAUNCH_PID=""
LAUNCH_PGID=""
KEEP_RUNNING=0

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

cleanup() {
  if [[ "$KEEP_RUNNING" -eq 1 ]]; then
    return
  fi
  if [[ -n "$LAUNCH_PGID" ]]; then
    kill -- "-${LAUNCH_PGID}" >/dev/null 2>&1 || true
    if [[ -n "$LAUNCH_PID" ]]; then
      wait "$LAUNCH_PID" 2>/dev/null || true
    fi
  elif [[ -n "$LAUNCH_PID" ]]; then
    kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
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

map_output_snapshot() {
  if [[ ! -f "$OUTPUT_DIR/map_projector_info.yaml" || ! -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
    return 1
  fi

  {
    stat -c 'projector %Y %s' "$OUTPUT_DIR/map_projector_info.yaml"
    stat -c 'metadata %Y %s' "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml"
    find "$OUTPUT_DIR/pointcloud_map" -maxdepth 1 -type f -name '*.pcd' -printf 'pcd %f %T@ %s\n' | sort
  }
}

wait_for_offline_completion() {
  local timeout_secs="$1"
  local quiet_secs="$2"
  local deadline=$((SECONDS + timeout_secs))
  local last_snapshot=""
  local last_change_secs=$SECONDS

  while (( SECONDS < deadline )); do
    if grep -Fq "RKO LIO Offline Node took" "$LAUNCH_LOG" 2>/dev/null; then
      return 0
    fi

    if snapshot=$(map_output_snapshot 2>/dev/null); then
      if [[ "$snapshot" != "$last_snapshot" ]]; then
        last_snapshot="$snapshot"
        last_change_secs=$SECONDS
      elif (( SECONDS - last_change_secs >= quiet_secs )); then
        return 0
      fi
    fi

    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 1
    fi
    sleep 2
  done
  return 1
}

echo "Running end-to-end dogfood pipeline"
echo "  bag:            $BAG_PATH"
echo "  lidar_topic:    $LIDAR_TOPIC"
echo "  imu_topic:      $IMU_TOPIC"
echo "  lidarslam_yaml: $LIDARSLAM_PARAM"
echo "  rko_yaml:       $RKO_PARAM"
echo "  output_dir:     $OUTPUT_DIR"
echo "  run_name:       $RUN_NAME"
echo "  rko_ros_param:  $RKO_ROS_PARAM_FILE"

if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch lidarslam rko_lio_slam.launch.py \
    "main_param_dir:=${LIDARSLAM_PARAM}" \
    "rko_param_file:=${RKO_ROS_PARAM_FILE}" \
    "bag_path:=${BAG_PATH}" \
    "lidar_topic:=${LIDAR_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
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
    "save_dir:=${OUTPUT_DIR}" \
    "results_dir:=${OUTPUT_DIR}" \
    "run_name:=${RUN_NAME}" \
    "dump_results:=true" \
    "use_rviz:=false" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
fi

echo "launch log: $LAUNCH_LOG"

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

if [[ "$WAIT_FOR_OFFLINE_COMPLETION" == "true" ]]; then
  echo "Waiting for offline bag playback to finish ..."
  if ! wait_for_offline_completion 900 15; then
    echo "Timed out waiting for offline completion or quiescent map outputs. Recent launch log:" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi
else
  echo "Waiting for the first saved Autoware map bundle ..."
  if ! wait_for_map_outputs "$SAVE_TIMEOUT_SECS"; then
    echo "Timed out waiting for the first saved map outputs under $OUTPUT_DIR" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi
fi

sleep 3
echo "Calling /map_save ..."
if ! call_map_save_with_retry; then
  if [[ -f "$OUTPUT_DIR/map_projector_info.yaml" && -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
    echo "Warning: /map_save call failed, but usable map outputs already exist. Proceeding with current bundle." >&2
  else
    echo "map_save service call failed. Recent launch log:" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    cat "$MAP_SAVE_LOG" >&2 || true
    exit 1
  fi
fi

if ! wait_for_map_outputs "$SAVE_TIMEOUT_SECS"; then
  echo "Timed out waiting for saved map outputs under $OUTPUT_DIR" >&2
  tail -n 120 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

echo "Map outputs saved under $OUTPUT_DIR"

if [[ "$KEEP_LAUNCH" == "false" ]]; then
  if [[ -n "$LAUNCH_PGID" ]]; then
    kill -- "-${LAUNCH_PGID}" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  elif [[ -n "$LAUNCH_PID" ]]; then
    kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  LAUNCH_PID=""
  LAUNCH_PGID=""
else
  KEEP_RUNNING=1
fi

if [[ "$SKIP_VIEWER" == "true" ]]; then
  exit 0
fi

VIEWER_CMD=(
  bash "$SCRIPT_DIR/run_graph_slam_pointcloud_map_in_autoware.sh"
  "$OUTPUT_DIR"
  --autoware-core-dir "$AUTOWARE_CORE_DIR"
  --work-dir "$WORK_DIR"
)

if [[ -n "$VIEWER_RUN_DIR" ]]; then
  VIEWER_CMD+=(--run-dir "$VIEWER_RUN_DIR")
fi
if [[ "$VIEWER_REBUILD" == "true" ]]; then
  VIEWER_CMD+=(--rebuild)
fi
if [[ -n "$AUTO_EXIT_SECS" ]]; then
  VIEWER_CMD+=(--auto-exit-secs "$AUTO_EXIT_SECS")
fi

"${VIEWER_CMD[@]}"
