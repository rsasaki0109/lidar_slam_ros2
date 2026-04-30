#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_small_gicp_graph_benchmark.sh [options]

Required:
  --bag <dir>                    rosbag2 directory (PointCloud2)
  --reference-tum <file>         Reference trajectory (TUM)
  --reference-meta <file>        Reference JSON (lidar_to_prism_translation_m for ape_from_tum)

Options:
  --input-cloud <topic>          PointCloud2 topic to feed frontend (default: /points_raw)
  --lidarslam-param <file>       graph_based_slam YAML (default: lidarslam/param/lidarslam_lo.yaml)
  --small-gicp-param <file>      YAML for small_gicp_odom_node (default: lidarslam/param/small_gicp_kitti_velodyne.yaml if exists)
  --robot-frame <frame>          Robot/LiDAR frame id label (default: base_link)
  --publish-tf BOOL              Frontend publishes odom->robot TF (default: true)
  --play-rate <float>            ros2 bag play --rate (default: 5.0)
  --play-delay-secs <sec>        Delay before bag play start (default: 1.0)
  --drain-secs <sec>             Wait after bag ends before map_save (default: 20)
  --output-dir <dir>             Output root
  --startup-timeout-secs <sec>   (default: 60)
  --save-timeout-secs <sec>      (default: 60)
  --skip-map-save
  --reference-source LABEL       metrics.json reference label (default: kitti_odometry_gt_velo)
  --ds <m>                       Override downsampling_resolution
  --voxel <m>                    Override voxel_resolution
  --corr <m>                     Override max_correspondence_distance
  --threads <n>                  Override num_threads
  --range <min> <max>            Override min_range/max_range
  --use-gicp BOOL                Override use_gicp (false=ICP, true=GICP)
  --help

Pipeline:
  small_gicp_odom_node -> graph_based_slam(use_odom_input) -> odom/path loggers -> ros2 bag play
    -> odom/path TUMs -> APE -> metrics.json

  Playback writes a QoS override for --input-cloud so ros2 bag play publishes
  PointCloud2 with best_effort (matches small_gicp subscriber); without this,
  reliable bags may not deliver any scans.
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
DEFAULT_SMALL_GICP_PARAM="${REPO_ROOT}/lidarslam/param/small_gicp_kitti_velodyne.yaml"

BAG_PATH=""
REFERENCE_TUM=""
REFERENCE_META=""
INPUT_CLOUD="/points_raw"
LIDARSLAM_PARAM="$DEFAULT_LIDARSLAM_PARAM"
SMALL_GICP_PARAM=""
ROBOT_FRAME="base_link"
PUBLISH_TF=true
PLAY_RATE=5.0
PLAY_DELAY_SECS=1.0
DRAIN_SECS=20
OUTPUT_DIR=""
STARTUP_TIMEOUT_SECS=60
SAVE_TIMEOUT_SECS=60
SKIP_MAP_SAVE=false
REFERENCE_SOURCE="kitti_odometry_gt_velo"
OVERRIDE_DS=""
OVERRIDE_VOXEL=""
OVERRIDE_CORR=""
OVERRIDE_THREADS=""
OVERRIDE_MIN_RANGE=""
OVERRIDE_MAX_RANGE=""
OVERRIDE_USE_GICP=""

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
    --small-gicp-param)
      [[ $# -ge 2 ]] || usage
      SMALL_GICP_PARAM=$(realpath "$2")
      shift 2
      ;;
    --robot-frame)
      [[ $# -ge 2 ]] || usage
      ROBOT_FRAME="$2"
      shift 2
      ;;
    --publish-tf)
      [[ $# -ge 2 ]] || usage
      PUBLISH_TF="$2"
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
    --ds)
      [[ $# -ge 2 ]] || usage
      OVERRIDE_DS="$2"
      shift 2
      ;;
    --voxel)
      [[ $# -ge 2 ]] || usage
      OVERRIDE_VOXEL="$2"
      shift 2
      ;;
    --corr)
      [[ $# -ge 2 ]] || usage
      OVERRIDE_CORR="$2"
      shift 2
      ;;
    --threads)
      [[ $# -ge 2 ]] || usage
      OVERRIDE_THREADS="$2"
      shift 2
      ;;
    --range)
      [[ $# -ge 3 ]] || usage
      OVERRIDE_MIN_RANGE="$2"
      OVERRIDE_MAX_RANGE="$3"
      shift 3
      ;;
    --use-gicp)
      [[ $# -ge 2 ]] || usage
      OVERRIDE_USE_GICP="$2"
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

if [[ -z "$SMALL_GICP_PARAM" && -f "$DEFAULT_SMALL_GICP_PARAM" ]]; then
  SMALL_GICP_PARAM="$DEFAULT_SMALL_GICP_PARAM"
fi
if [[ -n "$SMALL_GICP_PARAM" && ! -f "$SMALL_GICP_PARAM" ]]; then
  echo "small_gicp param not found: $SMALL_GICP_PARAM" >&2
  exit 1
fi

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/bench_small_gicp_graph_$(date +%Y%m%d_%H%M%S)"
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

LAUNCH_LOG="${OUTPUT_DIR}/small_gicp.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
BAG_PLAY_LOG="${OUTPUT_DIR}/bag_play.log"
RAW_TUM="${OUTPUT_DIR}/traj_raw.tum"
CORRECTED_TUM="${OUTPUT_DIR}/traj_corrected.tum"
RAW_TUM_PRISM="${OUTPUT_DIR}/traj_raw_prism.tum"
CORRECTED_TUM_PRISM="${OUTPUT_DIR}/traj_corrected_prism.tum"
RAW_APE="${OUTPUT_DIR}/ape_raw_vs_gt.txt"
CORRECTED_APE="${OUTPUT_DIR}/ape_corrected_vs_gt.txt"
RAW_LOG="${OUTPUT_DIR}/odom_logger.log"
CORRECTED_LOG="${OUTPUT_DIR}/path_logger.log"
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

LAUNCH_ARGS=(
  use_sim_time:=true
  "main_param_dir:=${LIDARSLAM_PARAM}"
  "input_cloud:=${INPUT_CLOUD}"
  "robot_frame_id:=${ROBOT_FRAME}"
  "odom_frame_id:=odom"
  "publish_tf:=${PUBLISH_TF}"
  "save_dir:=${OUTPUT_DIR}"
  "use_rviz:=false"
)
if [[ -n "$SMALL_GICP_PARAM" ]]; then
  LAUNCH_ARGS+=("small_gicp_param_file:=${SMALL_GICP_PARAM}")
fi
if [[ -n "$OVERRIDE_DS" ]]; then
  LAUNCH_ARGS+=("downsampling_resolution:=${OVERRIDE_DS}")
fi
if [[ -n "$OVERRIDE_VOXEL" ]]; then
  LAUNCH_ARGS+=("voxel_resolution:=${OVERRIDE_VOXEL}")
fi
if [[ -n "$OVERRIDE_CORR" ]]; then
  LAUNCH_ARGS+=("max_correspondence_distance:=${OVERRIDE_CORR}")
fi
if [[ -n "$OVERRIDE_THREADS" ]]; then
  LAUNCH_ARGS+=("num_threads:=${OVERRIDE_THREADS}")
fi
if [[ -n "$OVERRIDE_MIN_RANGE" ]]; then
  LAUNCH_ARGS+=("min_range:=${OVERRIDE_MIN_RANGE}")
fi
if [[ -n "$OVERRIDE_MAX_RANGE" ]]; then
  LAUNCH_ARGS+=("max_range:=${OVERRIDE_MAX_RANGE}")
fi
if [[ -n "$OVERRIDE_USE_GICP" ]]; then
  LAUNCH_ARGS+=("use_gicp:=${OVERRIDE_USE_GICP}")
fi

if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch lidarslam small_gicp_lo_slam.launch.py "${LAUNCH_ARGS[@]}" >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
  LAUNCH_PGID="$LAUNCH_PID"
else
  ros2 launch lidarslam small_gicp_lo_slam.launch.py "${LAUNCH_ARGS[@]}" >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
fi

python3 "${SCRIPT_DIR}/odom_to_tum.py" \
  --topic /small_gicp/odom \
  --output "$RAW_TUM" \
  --use-sim-time true \
  >"${RAW_LOG}" 2>&1 &
RAW_LOGGER_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic /modified_path \
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

wait "$BAG_PID" || true
BAG_PID=""
sleep "$DRAIN_SECS"

if [[ "$SKIP_MAP_SAVE" == "false" ]]; then
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

FRONTEND_PARAM="${SMALL_GICP_PARAM:-}"
if [[ -z "$FRONTEND_PARAM" ]]; then
  FRONTEND_PARAM="$LIDARSLAM_PARAM"
fi

python3 "${SCRIPT_DIR}/write_rko_lio_benchmark_metrics.py" \
  --pipeline small_gicp \
  --out-dir "$OUTPUT_DIR" \
  --bag "$BAG_PATH" \
  --reference-tum "$REFERENCE_TUM" \
  --reference-meta "$REFERENCE_META" \
  --points-topic "$INPUT_CLOUD" \
  --imu-topic "" \
  --lidarslam-param "$LIDARSLAM_PARAM" \
  --rko-param "$FRONTEND_PARAM" \
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
  --reference-source "$REFERENCE_SOURCE"

echo "small_gicp benchmark completed"
echo "  output_dir: $OUTPUT_DIR"
echo "  metrics:    $OUTPUT_DIR/metrics.json"
