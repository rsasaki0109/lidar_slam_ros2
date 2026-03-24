#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${REPO_ROOT}"
if [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
fi

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh --bag /path/to/rosbag2 [options]

Required:
  --bag PATH                  Main rosbag2 directory containing VelodyneScan and Applanix GSOF.

Options:
  --packet-topic TOPIC        VelodyneScan topic in the main bag (auto-detect if omitted).
  --reference-tum FILE        Optional reference TUM trajectory. If omitted, one is extracted from GSOF49.
  --gnss-bag PATH             Optional NavSatFix sidecar rosbag2. If omitted and --use-gnss=true, one is generated.
  --gnss-topic TOPIC          NavSatFix topic (default: /gnss/fix).
  --gsof49-topic TOPIC        Applanix GSOF49 topic.
  --gsof50-topic TOPIC        Applanix GSOF50 topic.
  --applanix-msg-dir PATH     Path to applanix_msgs/msg (default: /tmp/applanix/applanix_msgs/msg).
  --velodyne-overlay DIR      Overlay workspace with velodyne_pointcloud (default: /tmp/velodyne_ws).
  --velodyne-model MODEL      Velodyne model for packet conversion (default: VLP16).
  --velodyne-calibration FILE Explicit calibration YAML. If omitted, derived from the model.
  --param FILE                Base lidarslam parameter YAML.
  --output-dir DIR            Output directory (default: output/open_data_applanix_velodyne_gnss_benchmark_<timestamp>).
  --rate FLOAT                ros2 bag play rate (default: 5.0).
  --play-wall-sec SEC         Playback timeout. If omitted, derived from bag duration and rate.
  --drain-sec SEC             Extra wait before /map_save (default: 8).
  --use-gnss BOOL             Enable backend GNSS constraints (default: true).
  --verify-map                Run verify_autoware_map.py after /map_save.
  --ros-distro DISTRO         ROS 2 distro used for sourcing and overlay build (default: $ROS_DISTRO or jazzy).
  --skip-prepare-overlay      Do not auto-build the velodyne overlay when missing.

This wrapper runs:
  VelodyneScan + Applanix GSOF49/50 -> PointCloud2 + NavSatFix -> lidarslam.launch.py
  -> raw/corrected TUM -> aligned metrics.json
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

detect_topic_by_type() {
  local bag_path="$1"
  local msg_type="$2"
  local extra_msg_dir="${3:-}"
  python3 - "${bag_path}" "${msg_type}" "${extra_msg_dir}" <<'PY'
from pathlib import Path
import sys

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

bag_path = Path(sys.argv[1])
msg_type = sys.argv[2]
extra_msg_dir = Path(sys.argv[3]) if sys.argv[3] else None
best_topic = ''
best_count = -1
typestore = get_typestore(Stores.LATEST)

if extra_msg_dir is not None:
    package_name = extra_msg_dir.parent.name
    for path in sorted(extra_msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        typestore.register(get_types_from_msg(text, f'{package_name}/msg/{path.stem}'))

with AnyReader([bag_path], default_typestore=typestore) as reader:
    for connection in reader.connections:
        if connection.msgtype != msg_type:
            continue
        message_count = getattr(connection, 'msgcount', 0)
        if message_count > best_count:
            best_count = message_count
            best_topic = connection.topic

if best_topic:
    print(best_topic)
PY
}

detect_first_header_frame() {
  local bag_path="$1"
  local topic="$2"
  local extra_msg_dir="${3:-}"
  python3 - "${bag_path}" "${topic}" "${extra_msg_dir}" <<'PY'
from pathlib import Path
import sys

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

bag_path = Path(sys.argv[1])
topic = sys.argv[2]
extra_msg_dir = Path(sys.argv[3]) if sys.argv[3] else None
typestore = get_typestore(Stores.LATEST)

if extra_msg_dir is not None:
    package_name = extra_msg_dir.parent.name
    for path in sorted(extra_msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        typestore.register(get_types_from_msg(text, f'{package_name}/msg/{path.stem}'))

with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [conn for conn in reader.connections if conn.topic == topic]
    if not connections:
        raise SystemExit(1)
    for conn, _, raw in reader.messages(connections=connections):
        msg = reader.deserialize(raw, conn.msgtype)
        header = getattr(msg, 'header', None)
        frame_id = getattr(header, 'frame_id', '')
        if frame_id:
            print(frame_id)
            break
PY
}

bag_duration_seconds() {
  local bag_path="$1"
  python3 - "${bag_path}" <<'PY'
from pathlib import Path
import sys

metadata = Path(sys.argv[1]) / 'metadata.yaml'
if not metadata.is_file():
    raise SystemExit(1)

lines = metadata.read_text(encoding='utf-8', errors='replace').splitlines()
in_duration = False
for line in lines:
    stripped = line.strip()
    if stripped.startswith('duration:'):
        in_duration = True
        continue
    if in_duration and stripped.startswith('nanoseconds:'):
        nanoseconds = int(stripped.split(':', 1)[1].strip())
        print(nanoseconds / 1e9)
        raise SystemExit(0)
    if in_duration and stripped and not line.startswith(' '):
        break
raise SystemExit(1)
PY
}

compute_play_wall_sec() {
  local bag_path="$1"
  local rate="$2"
  python3 - "${bag_path}" "${rate}" <<'PY'
from pathlib import Path
import math
import sys

metadata = Path(sys.argv[1]) / 'metadata.yaml'
rate = float(sys.argv[2])
if rate <= 0.0:
    raise SystemExit('rate must be > 0')
lines = metadata.read_text(encoding='utf-8', errors='replace').splitlines()
in_duration = False
duration_ns = None
for line in lines:
    stripped = line.strip()
    if stripped.startswith('duration:'):
        in_duration = True
        continue
    if in_duration and stripped.startswith('nanoseconds:'):
        duration_ns = int(stripped.split(':', 1)[1].strip())
        break
if duration_ns is None:
    raise SystemExit('failed to parse bag duration')
duration_sec = duration_ns / 1e9
print(int(math.ceil(duration_sec / rate + 60.0)))
PY
}

create_main_param() {
  local base_param="$1"
  local out_param="$2"
  local use_gnss="$3"
  cp "${base_param}" "${out_param}"
  python3 - "${out_param}" "${use_gnss}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
use_gnss = sys.argv[2].strip().lower() in {'1', 'true', 'yes', 'on'}
text = path.read_text(encoding='utf-8')
if '      use_gnss: true' in text or '      use_gnss: false' in text:
    text = text.replace(
        '      use_gnss: true',
        f'      use_gnss: {"true" if use_gnss else "false"}',
        1,
    )
    text = text.replace(
        '      use_gnss: false',
        f'      use_gnss: {"true" if use_gnss else "false"}',
        1,
    )
else:
    raise SystemExit('could not find graph_based_slam use_gnss parameter in base YAML')
path.write_text(text, encoding='utf-8')
PY
}

call_map_save_with_retry() {
  local log_file="$1"
  for _ in $(seq 1 5); do
    if timeout 20 ros2 service call /map_save std_srvs/srv/Empty "{}" >"${log_file}" 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

wait_for_nonempty_file() {
  local path="$1"
  local timeout_secs="$2"
  local deadline=$((SECONDS + timeout_secs))
  while (( SECONDS < deadline )); do
    if [[ -s "${path}" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

terminate_pid() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0

  if kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return 0
      fi
      sleep 0.5
    done
    kill -9 "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

ensure_velodyne_overlay() {
  local overlay_dir="$1"
  local ros_distro_name="$2"

  if [[ -f "${overlay_dir}/install/setup.bash" ]]; then
    return 0
  fi
  bash "${SCRIPT_DIR}/prepare_velodyne_pointcloud_overlay.sh" \
    --overlay-dir "${overlay_dir}" \
    --ros-distro "${ros_distro_name}"
}

resolve_velodyne_msg_dir() {
  local overlay_dir="$1"
  local candidate=""
  for candidate in \
    "${overlay_dir}/src/velodyne/velodyne_msgs/msg" \
    "${overlay_dir}/install/velodyne_msgs/share/velodyne_msgs/msg"
  do
    if [[ -d "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

default_calibration_for_model() {
  local overlay_dir="$1"
  local model="$2"
  case "${model}" in
    VLP16)
      printf '%s\n' "${overlay_dir}/install/velodyne_pointcloud/share/velodyne_pointcloud/params/VLP16db.yaml"
      ;;
    32C|VLP32C)
      printf '%s\n' "${overlay_dir}/install/velodyne_pointcloud/share/velodyne_pointcloud/params/VeloView-VLP-32C.yaml"
      ;;
    VLS128)
      printf '%s\n' "${overlay_dir}/install/velodyne_pointcloud/share/velodyne_pointcloud/params/VLS128.yaml"
      ;;
    *)
      die "unsupported velodyne model: ${model}"
      ;;
  esac
}

BAG_PATH=""
PACKET_TOPIC=""
REFERENCE_TUM=""
GNSS_BAG=""
GNSS_TOPIC="/gnss/fix"
GSOF49_TOPIC="/lvx_client/gsof/ins_solution_49"
GSOF50_TOPIC="/lvx_client/gsof/ins_solution_rms_50"
APPLANIX_MSG_DIR="/tmp/applanix/applanix_msgs/msg"
VELODYNE_OVERLAY="/tmp/velodyne_ws"
VELODYNE_MODEL="VLP16"
VELODYNE_CALIBRATION=""
PARAM_FILE="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
OUTPUT_DIR=""
RATE="5.0"
PLAY_WALL_SEC=""
DRAIN_SEC="8"
USE_GNSS="true"
VERIFY_MAP="false"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
PREPARE_OVERLAY="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag)
      BAG_PATH="$(realpath "${2:-}")"; shift 2 ;;
    --packet-topic)
      PACKET_TOPIC="${2:-}"; shift 2 ;;
    --reference-tum)
      REFERENCE_TUM="$(realpath -m "${2:-}")"; shift 2 ;;
    --gnss-bag)
      GNSS_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --gnss-topic)
      GNSS_TOPIC="${2:-}"; shift 2 ;;
    --gsof49-topic)
      GSOF49_TOPIC="${2:-}"; shift 2 ;;
    --gsof50-topic)
      GSOF50_TOPIC="${2:-}"; shift 2 ;;
    --applanix-msg-dir)
      APPLANIX_MSG_DIR="$(realpath "${2:-}")"; shift 2 ;;
    --velodyne-overlay)
      VELODYNE_OVERLAY="$(realpath -m "${2:-}")"; shift 2 ;;
    --velodyne-model)
      VELODYNE_MODEL="${2:-}"; shift 2 ;;
    --velodyne-calibration)
      VELODYNE_CALIBRATION="$(realpath "${2:-}")"; shift 2 ;;
    --param)
      PARAM_FILE="$(realpath "${2:-}")"; shift 2 ;;
    --output-dir)
      OUTPUT_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --rate)
      RATE="${2:-}"; shift 2 ;;
    --play-wall-sec)
      PLAY_WALL_SEC="${2:-}"; shift 2 ;;
    --drain-sec)
      DRAIN_SEC="${2:-}"; shift 2 ;;
    --use-gnss)
      USE_GNSS="${2:-}"; shift 2 ;;
    --verify-map)
      VERIFY_MAP="true"; shift ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"; shift 2 ;;
    --skip-prepare-overlay)
      PREPARE_OVERLAY="false"; shift ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

[[ -n "${BAG_PATH}" ]] || { usage; die "--bag is required"; }
[[ -d "${BAG_PATH}" ]] || die "bag not found: ${BAG_PATH}"
[[ -f "${BAG_PATH}/metadata.yaml" ]] || die "metadata.yaml not found under ${BAG_PATH}"
[[ -f "${PARAM_FILE}" ]] || die "param file not found: ${PARAM_FILE}"

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/open_data_applanix_velodyne_gnss_benchmark_$(timestamp)"
fi
mkdir -p "${OUTPUT_DIR}"

[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || {
  die "ROS setup not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
}

if [[ "${PREPARE_OVERLAY}" == "true" ]]; then
  ensure_velodyne_overlay "${VELODYNE_OVERLAY}" "${ROS_DISTRO_NAME}"
fi
[[ -f "${VELODYNE_OVERLAY}/install/setup.bash" ]] || {
  die "velodyne overlay not found: ${VELODYNE_OVERLAY}/install/setup.bash"
}

VELODYNE_MSG_DIR="$(resolve_velodyne_msg_dir "${VELODYNE_OVERLAY}")" || {
  die "velodyne_msgs definitions not found under ${VELODYNE_OVERLAY}"
}

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
fi
# shellcheck source=/dev/null
source "${VELODYNE_OVERLAY}/install/setup.bash"
set -u

command -v ros2 >/dev/null 2>&1 || die "ros2 not found"

if [[ -z "${PACKET_TOPIC}" ]]; then
  PACKET_TOPIC="$(detect_topic_by_type \
    "${BAG_PATH}" \
    "velodyne_msgs/msg/VelodyneScan" \
    "${VELODYNE_MSG_DIR}")"
fi
[[ -n "${PACKET_TOPIC}" ]] || die "failed to detect VelodyneScan topic"

ROBOT_FRAME_ID="$(detect_first_header_frame \
  "${BAG_PATH}" \
  "${PACKET_TOPIC}" \
  "${VELODYNE_MSG_DIR}")"
[[ -n "${ROBOT_FRAME_ID}" ]] || die "failed to detect frame_id for ${PACKET_TOPIC}"

if [[ -z "${VELODYNE_CALIBRATION}" ]]; then
  VELODYNE_CALIBRATION="$(default_calibration_for_model "${VELODYNE_OVERLAY}" "${VELODYNE_MODEL}")"
fi
[[ -f "${VELODYNE_CALIBRATION}" ]] || die "velodyne calibration not found: ${VELODYNE_CALIBRATION}"

REFERENCE_TUM_AUTO="${OUTPUT_DIR}/reference_applanix.tum"
REFERENCE_META="${OUTPUT_DIR}/reference_applanix.json"
if [[ -z "${REFERENCE_TUM}" ]]; then
  REFERENCE_TUM="${REFERENCE_TUM_AUTO}"
  python3 "${SCRIPT_DIR}/extract_applanix_gsof49_reference.py" \
    --input "${BAG_PATH}" \
    --output "${REFERENCE_TUM}" \
    --topic "${GSOF49_TOPIC}" \
    --applanix-msg-dir "${APPLANIX_MSG_DIR}" \
    --meta-out "${REFERENCE_META}" \
    >"${OUTPUT_DIR}/reference_extract.log" 2>&1
fi
[[ -f "${REFERENCE_TUM}" ]] || die "reference TUM not found: ${REFERENCE_TUM}"

CONVERT_LOG="${OUTPUT_DIR}/convert_applanix.log"
if [[ "${USE_GNSS,,}" == "true" ]]; then
  if [[ -z "${GNSS_BAG}" ]]; then
    [[ -d "${APPLANIX_MSG_DIR}" ]] || {
      die "applanix_msgs dir not found: ${APPLANIX_MSG_DIR}"
    }
    GNSS_BAG="${OUTPUT_DIR}/applanix_navsatfix_sidecar"
    python3 "${SCRIPT_DIR}/convert_applanix_gsof_to_navsatfix_bag.py" \
      --input "${BAG_PATH}" \
      --output "${GNSS_BAG}" \
      --gsof49-topic "${GSOF49_TOPIC}" \
      --gsof50-topic "${GSOF50_TOPIC}" \
      --output-topic "${GNSS_TOPIC}" \
      --applanix-msg-dir "${APPLANIX_MSG_DIR}" \
      --force \
      >"${CONVERT_LOG}" 2>&1
  fi
  [[ -d "${GNSS_BAG}" ]] || die "gnss bag not found: ${GNSS_BAG}"
  [[ -f "${GNSS_BAG}/metadata.yaml" ]] || die "metadata.yaml not found under ${GNSS_BAG}"
fi

if [[ -z "${PLAY_WALL_SEC}" ]]; then
  PLAY_WALL_SEC="$(compute_play_wall_sec "${BAG_PATH}" "${RATE}")"
fi

TMP_PARAM="$(mktemp --suffix=.yaml)"
VELODYNE_PARAM="$(mktemp --suffix=.yaml)"
QOS_FILE="$(mktemp --suffix=.yaml)"
create_main_param "${PARAM_FILE}" "${TMP_PARAM}" "${USE_GNSS}"

cat >"${VELODYNE_PARAM}" <<EOF
velodyne_transform_node:
  ros__parameters:
    calibration: ${VELODYNE_CALIBRATION}
    model: ${VELODYNE_MODEL}
    min_range: 0.9
    max_range: 200.0
    view_direction: 0.0
    fixed_frame: ""
    target_frame: ""
    organize_cloud: false
EOF

cat >"${QOS_FILE}" <<EOF
${PACKET_TOPIC}:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 10
EOF

LAUNCH_LOG="${OUTPUT_DIR}/lidarslam.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
MAIN_PLAY_LOG="${OUTPUT_DIR}/main_bag_play.log"
GNSS_PLAY_LOG="${OUTPUT_DIR}/gnss_bag_play.log"
VELODYNE_LOG="${OUTPUT_DIR}/velodyne_transform.log"
VERIFY_LOG="${OUTPUT_DIR}/verify_autoware_map.log"
RAW_TUM="${OUTPUT_DIR}/traj_raw.tum"
CORRECTED_TUM="${OUTPUT_DIR}/traj_corrected.tum"
RAW_LOG="${OUTPUT_DIR}/path_logger_raw.log"
CORRECTED_LOG="${OUTPUT_DIR}/path_logger_corrected.log"
POINTS_TOPIC="/open_data/velodyne_points"

LAUNCH_PID=""
MAIN_PLAY_PID=""
GNSS_PLAY_PID=""
VELODYNE_PID=""
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""
cleanup() {
  for pid in \
    "${GNSS_PLAY_PID}" \
    "${MAIN_PLAY_PID}" \
    "${RAW_LOGGER_PID}" \
    "${CORRECTED_LOGGER_PID}" \
    "${VELODYNE_PID}" \
    "${LAUNCH_PID}"
  do
    terminate_pid "${pid}"
  done
  rm -f "${TMP_PARAM}" "${VELODYNE_PARAM}" "${QOS_FILE}"
}
trap cleanup EXIT INT TERM

BENCH_T0="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"
STARTED_AT="$(date -Iseconds)"
STARTED_AT_UNIX="$(date +%s)"

echo "Running Applanix + Velodyne GNSS benchmark:"
echo "  bag:                 ${BAG_PATH}"
echo "  packet_topic:        ${PACKET_TOPIC}"
echo "  reference_tum:       ${REFERENCE_TUM}"
echo "  use_gnss:            ${USE_GNSS}"
if [[ "${USE_GNSS,,}" == "true" ]]; then
  echo "  gnss_bag:            ${GNSS_BAG}"
  echo "  gnss_topic:          ${GNSS_TOPIC}"
fi
echo "  rate:                ${RATE}"
echo "  play_wall_sec:       ${PLAY_WALL_SEC}"
echo "  velodyne_model:      ${VELODYNE_MODEL}"
echo "  velodyne_calibration:${VELODYNE_CALIBRATION}"
echo "  robot_frame:         ${ROBOT_FRAME_ID}"
echo "  output_dir:          ${OUTPUT_DIR}"

ros2 run velodyne_pointcloud velodyne_transform_node \
  --ros-args \
  --params-file "${VELODYNE_PARAM}" \
  -r "velodyne_packets:=${PACKET_TOPIC}" \
  -r "velodyne_points:=${POINTS_TOPIC}" \
  >"${VELODYNE_LOG}" 2>&1 &
VELODYNE_PID="$!"

ros2 launch lidarslam lidarslam.launch.py \
  "main_param_dir:=${TMP_PARAM}" \
  "input_cloud:=${POINTS_TOPIC}" \
  "gnss_topic:=${GNSS_TOPIC}" \
  "robot_frame_id:=${ROBOT_FRAME_ID}" \
  "base_frame:=${ROBOT_FRAME_ID}" \
  "lidar_frame:=${ROBOT_FRAME_ID}" \
  "global_frame_id:=map" \
  "use_graph_based_slam:=true" \
  "use_sim_time:=true" \
  "publish_static_tf:=false" \
  "save_dir:=${OUTPUT_DIR}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic /path \
  --output "${RAW_TUM}" \
  --use-sim-time true \
  >"${RAW_LOG}" 2>&1 &
RAW_LOGGER_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic /modified_path \
  --output "${CORRECTED_TUM}" \
  --use-sim-time true \
  >"${CORRECTED_LOG}" 2>&1 &
CORRECTED_LOGGER_PID="$!"

sleep 5

timeout "${PLAY_WALL_SEC}" ros2 bag play "${BAG_PATH}" \
  --clock \
  --rate "${RATE}" \
  --topics "${PACKET_TOPIC}" \
  --qos-profile-overrides-path "${QOS_FILE}" \
  >"${MAIN_PLAY_LOG}" 2>&1 &
MAIN_PLAY_PID="$!"

if [[ "${USE_GNSS,,}" == "true" ]]; then
  timeout "${PLAY_WALL_SEC}" ros2 bag play "${GNSS_BAG}" \
    --rate "${RATE}" \
    >"${GNSS_PLAY_LOG}" 2>&1 &
  GNSS_PLAY_PID="$!"
fi

wait "${MAIN_PLAY_PID}" || true
MAIN_PLAY_PID=""
if [[ -n "${GNSS_PLAY_PID}" ]]; then
  wait "${GNSS_PLAY_PID}" || true
  GNSS_PLAY_PID=""
fi

sleep "${DRAIN_SEC}"

if ! call_map_save_with_retry "${MAP_SAVE_LOG}"; then
  echo "map_save service call failed. Recent launch log:" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

if ! wait_for_nonempty_file "${RAW_TUM}" 60; then
  echo "raw trajectory not found or empty: ${RAW_TUM}" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi
if ! wait_for_nonempty_file "${CORRECTED_TUM}" 60; then
  echo "corrected trajectory not found or empty: ${CORRECTED_TUM}" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

if [[ "${VERIFY_MAP}" == "true" ]]; then
  python3 "${REPO_ROOT}/scripts/verify_autoware_map.py" "${OUTPUT_DIR}" >"${VERIFY_LOG}" 2>&1
fi

terminate_pid "${RAW_LOGGER_PID}"
RAW_LOGGER_PID=""
terminate_pid "${CORRECTED_LOGGER_PID}"
CORRECTED_LOGGER_PID=""
terminate_pid "${VELODYNE_PID}"
VELODYNE_PID=""
terminate_pid "${LAUNCH_PID}"
LAUNCH_PID=""

BENCH_T1="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"
WALL_SEC="$(python3 - "${BENCH_T0}" "${BENCH_T1}" <<'PY'
import sys
print(float(sys.argv[2]) - float(sys.argv[1]))
PY
)"

python3 "${SCRIPT_DIR}/write_aligned_trajectory_metrics.py" \
  --out-dir "${OUTPUT_DIR}" \
  --bag "${BAG_PATH}" \
  --reference-tum "${REFERENCE_TUM}" \
  --corrected-tum "${CORRECTED_TUM}" \
  --raw-tum "${RAW_TUM}" \
  --graph-log "${LAUNCH_LOG}" \
  --lidarslam-param "${TMP_PARAM}" \
  --points-topic "${POINTS_TOPIC}" \
  --points-frame "${ROBOT_FRAME_ID}" \
  --robot-frame "${ROBOT_FRAME_ID}" \
  --reference-source "applanix_gsof49_reference" \
  --reference-kind "cross_validation" \
  --reference-label "Applanix_GSOF49" \
  --wall-sec "${WALL_SEC}" \
  --started-at "${STARTED_AT}" \
  --started-at-unix "${STARTED_AT_UNIX}" \
  >"${OUTPUT_DIR}/metrics_path.txt"

if [[ -f "${OUTPUT_DIR}/map_projector_info.yaml" ]]; then
  echo "map_projector_info.yaml:"
  cat "${OUTPUT_DIR}/map_projector_info.yaml"
fi
echo "metrics_json: ${OUTPUT_DIR}/metrics.json"
echo "done: ${OUTPUT_DIR}"
