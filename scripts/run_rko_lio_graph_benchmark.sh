#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_rko_lio_graph_benchmark.sh [options]

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
  --offline-timeout-secs <sec>   Max wall-clock for the offline node to finish the bag (default: 1800)
  --quiescence-secs <sec>        Treat the run as done after this many stable seconds (default: 20)
  --completion-end-margin-secs <sec>
                                 Required trajectory proximity to bag end (default: 120)
  --skip-map-save                Do not call /map_save or verify the map bundle
  --gt-blind                     Run trajectory/map generation without any GT/APE access
  --skip-reference-gen           Reuse an existing reference TUM/meta without regenerating it
  --publish-static-tf BOOL       static_transform_publisher (default: true)
  --reference-source LABEL       Label stored in metrics.json (default: leica_prism_gt)
  --help                         Show this help

This runs the recommended benchmark path:
  rosbag2 -> RKO-LIO + graph_based_slam -> raw/corrected trajectories -> APE -> metrics.json
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run 'bash scripts/run_rko_lio_graph_benchmark.sh --help' for valid options." >&2
  exit 2
}

die() {
  echo "error: $*" >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == -* ]]; then
    fail "option requires a value: ${option}"
  fi
  OPTION_VALUE="${value}"
}

parse_bool() {
  local option="$1"
  local value="$2"
  case "${value}" in
    true|false)
      printf '%s\n' "${value}"
      ;;
    *)
      fail "${option} expects true or false."
      ;;
  esac
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "${SCRIPT_DIR}/container_phase_evidence.sh"
source "${SCRIPT_DIR}/m6a10_completion_contract.sh"
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
WS_ROOT="${REPO_ROOT}"
if [[ -n "${LIDARSLAM_WS_ROOT:-}" ]]; then
  WS_ROOT="$(realpath -m "${LIDARSLAM_WS_ROOT}")"
elif [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
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
COMPLETION_END_MARGIN_SECS=120
# Max wall-clock to wait for the offline node to finish replaying the bag.
# Long / compute-heavy sequences (e.g. dense indoor MID-360) can process well
# below real time; raise this so the run is not cut off mid-bag.
OFFLINE_TIMEOUT_SECS=1800
SKIP_MAP_SAVE=false
SKIP_REFERENCE_GEN=false
GT_BLIND=false
PUBLISH_STATIC_TF=true
REFERENCE_SOURCE=""
OPTION_VALUE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      require_value "$1" "${2:-}"
      BAG_PATH=$(realpath -m "${OPTION_VALUE}")
      shift 2
      ;;
    --reference-bag)
      require_value "$1" "${2:-}"
      REFERENCE_BAG=$(realpath -m "${OPTION_VALUE}")
      shift 2
      ;;
    --reference-tum)
      require_value "$1" "${2:-}"
      REFERENCE_TUM=$(realpath -m "${OPTION_VALUE}")
      shift 2
      ;;
    --reference-meta)
      require_value "$1" "${2:-}"
      REFERENCE_META=$(realpath -m "${OPTION_VALUE}")
      shift 2
      ;;
    --lidar-topic)
      require_value "$1" "${2:-}"
      LIDAR_TOPIC="${OPTION_VALUE}"
      shift 2
      ;;
    --imu-topic)
      require_value "$1" "${2:-}"
      IMU_TOPIC="${OPTION_VALUE}"
      shift 2
      ;;
    --base-frame)
      require_value "$1" "${2:-}"
      BASE_FRAME="${OPTION_VALUE}"
      shift 2
      ;;
    --lidarslam-param)
      require_value "$1" "${2:-}"
      LIDARSLAM_PARAM=$(realpath -m "${OPTION_VALUE}")
      shift 2
      ;;
    --rko-param)
      require_value "$1" "${2:-}"
      RKO_PARAM=$(realpath -m "${OPTION_VALUE}")
      shift 2
      ;;
    --output-dir)
      require_value "$1" "${2:-}"
      OUTPUT_DIR=$(realpath -m "${OPTION_VALUE}")
      shift 2
      ;;
    --run-name)
      require_value "$1" "${2:-}"
      RUN_NAME="${OPTION_VALUE}"
      shift 2
      ;;
    --startup-timeout-secs)
      require_value "$1" "${2:-}"
      STARTUP_TIMEOUT_SECS="${OPTION_VALUE}"
      shift 2
      ;;
    --save-timeout-secs)
      require_value "$1" "${2:-}"
      SAVE_TIMEOUT_SECS="${OPTION_VALUE}"
      shift 2
      ;;
    --offline-timeout-secs)
      require_value "$1" "${2:-}"
      OFFLINE_TIMEOUT_SECS="${OPTION_VALUE}"
      shift 2
      ;;
    --quiescence-secs)
      require_value "$1" "${2:-}"
      QUIESCENCE_SECS="${OPTION_VALUE}"
      shift 2
      ;;
    --completion-end-margin-secs)
      require_value "$1" "${2:-}"
      COMPLETION_END_MARGIN_SECS="${OPTION_VALUE}"
      shift 2
      ;;
    --skip-map-save)
      SKIP_MAP_SAVE=true
      shift
      ;;
    --gt-blind)
      GT_BLIND=true
      SKIP_REFERENCE_GEN=true
      REFERENCE_BAG=""
      REFERENCE_TUM=""
      REFERENCE_META=""
      shift
      ;;
    --skip-reference-gen)
      SKIP_REFERENCE_GEN=true
      shift
      ;;
    --publish-static-tf)
      require_value "$1" "${2:-}"
      PUBLISH_STATIC_TF=$(parse_bool "$1" "${OPTION_VALUE}")
      shift 2
      ;;
    --reference-source)
      require_value "$1" "${2:-}"
      REFERENCE_SOURCE="${OPTION_VALUE}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      fail "unknown option: $1"
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

if [[ -e "$OUTPUT_DIR" && ! -d "$OUTPUT_DIR" ]]; then
  fail "output directory path is a file: $OUTPUT_DIR"
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "error: rosbag2 directory not found: $BAG_PATH" >&2
  echo "hint: --bag must point to a rosbag2 directory containing metadata.yaml." >&2
  if [[ "$BAG_PATH" == "$DEFAULT_BAG" ]]; then
    default_benchmark_bag_missing_hint
  fi
  exit 2
fi
[[ -f "$BAG_PATH/metadata.yaml" ]] || die "metadata.yaml not found under $BAG_PATH"
[[ -f "$LIDARSLAM_PARAM" ]] || die "lidarslam param file not found: $LIDARSLAM_PARAM"
[[ -f "$RKO_PARAM" ]] || die "RKO-LIO param file not found: $RKO_PARAM"
if [[ "$GT_BLIND" == "true" ]]; then
  : # No reference path is accepted, opened, or passed in GT-blind mode.
elif [[ "$SKIP_REFERENCE_GEN" == "false" ]]; then
  if [[ ! -d "$REFERENCE_BAG" ]]; then
    echo "error: reference rosbag2 directory not found: $REFERENCE_BAG" >&2
    echo "hint: --reference-bag must point to a rosbag2 directory containing metadata.yaml." >&2
    if [[ "$REFERENCE_BAG" == "$DEFAULT_REFERENCE_BAG" ]]; then
      default_benchmark_bag_missing_hint
    fi
    exit 2
  fi
  [[ -f "$REFERENCE_BAG/metadata.yaml" ]] || die "metadata.yaml not found under $REFERENCE_BAG"
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

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: required command not found: ros2" >&2
  echo "hint: source a ROS 2 setup file before running this benchmark." >&2
  exit 2
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "error: required command not found: python3" >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${OUTPUT_DIR}/.ros_log"
fi
mkdir -p "$ROS_LOG_DIR"

# A v2 run crosses two shell/process boundaries before it reaches the
# application-owned OfflineNode.  Assert and export the contract at this
# boundary, and leave a small machine-readable receipt in the attempt output.
# This prevents the historical trajectory-quiescence fallback from being
# mistaken for v2 consumer acknowledgement evidence when a child loses the
# environment bridge.
M6A10_CHILD_STARTUP_ENV="${OUTPUT_DIR}/m6a10_child_startup_env.json"
if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
  [[ "${M6A10_PHASE_MODE:-}" == "paced_1x" ||
     "${M6A10_PHASE_MODE:-}" == "unpaced_ack" ]] ||
    die "v2 phase mode is missing or unsupported"
  [[ -n "${M6A10_CONSUMER_EVIDENCE:-}" ]] ||
    die "v2 consumer evidence path is missing"
  case "${M6A10_CONSUMER_EVIDENCE}" in
    "${OUTPUT_DIR}"/*) ;;
    *) die "v2 consumer evidence path must stay inside output directory" ;;
  esac
  export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE M6A10_CONSUMER_EVIDENCE
  python3 - "$M6A10_CHILD_STARTUP_ENV" \
    "$M6A10_PHASE_CONTRACT_VERSION" "$M6A10_PHASE_MODE" \
    "$M6A10_CONSUMER_EVIDENCE" <<'PY'
import json
import os
import sys
from pathlib import Path

target = Path(sys.argv[1])
if target.exists() or target.with_name(target.name + '.part').exists():
    raise SystemExit(f'startup receipt already exists: {target}')
document = {
    'schema_version': 1,
    'kind': 'm6a10_child_startup_environment',
    'status': 'pass',
    'contract_version': sys.argv[2],
    'phase_mode': sys.argv[3],
    'consumer_evidence_path': sys.argv[4],
    'environment_exported': True,
    'v2_barrier_required': True,
    'gt_mounted': False,
    'scorer_invoked': False,
}
part = target.with_name(target.name + '.part')
part.write_text(json.dumps(document, sort_keys=True) + '\n', encoding='utf-8')
os.replace(part, target)
PY
else
  M6A10_CHILD_STARTUP_ENV=""
fi

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
CORRECTED_SPARSE_TUM="${OUTPUT_DIR}/traj_corrected_sparse.tum"
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

bag_end_stamp() {
  # Unix-epoch end time of the bag (start + duration), from metadata.yaml.
  python3 - "$BAG_PATH" <<'PY'
import sys
from pathlib import Path

import yaml

meta = yaml.safe_load((Path(sys.argv[1]) / 'metadata.yaml').read_text())
info = meta['rosbag2_bagfile_information']
start_ns = info['starting_time']['nanoseconds_since_epoch']
dur_ns = info['duration']['nanoseconds']
print((start_ns + dur_ns) / 1e9)
PY
}

raw_tum_reached() {
  # True once the last raw-trajectory stamp is within margin of the bag end.
  local end_stamp="$1"
  local margin="$2"
  python3 - "$RAW_TUM" "$end_stamp" "$margin" <<'PY'
import sys

try:
    with open(sys.argv[1], 'rb') as fh:
        fh.seek(-256, 2)
        last = fh.read().decode(errors='replace').strip().splitlines()[-1]
    reached = float(last.split()[0]) >= float(sys.argv[2]) - float(sys.argv[3])
except (OSError, IndexError, ValueError):
    reached = False
sys.exit(0 if reached else 1)
PY
}

raw_tum_reached_fraction() {
  # Guard the deliberately generous end margin below on short bags. Without
  # this check, a 120 s margin makes every pose after 5.8 s look "near the
  # end" on a 125.8 s bag and a mid-run writer pause can terminate the run.
  local fraction="$1"
  python3 - "$RAW_TUM" "$BAG_PATH" "$fraction" <<'PY'
import sys
from pathlib import Path

import yaml

try:
    with open(sys.argv[1], 'rb') as fh:
        fh.seek(-256, 2)
        last = fh.read().decode(errors='replace').strip().splitlines()[-1]
    last_stamp = float(last.split()[0])
    meta = yaml.safe_load((Path(sys.argv[2]) / 'metadata.yaml').read_text())
    info = meta['rosbag2_bagfile_information']
    start_ns = info['starting_time']['nanoseconds_since_epoch']
    duration_ns = info['duration']['nanoseconds']
    required_stamp = (start_ns + duration_ns * float(sys.argv[3])) / 1e9
    reached = last_stamp >= required_stamp
except (OSError, KeyError, IndexError, TypeError, ValueError, yaml.YAMLError):
    reached = False
sys.exit(0 if reached else 1)
PY
}

run_state_signature() {
  # Trajectory files only: the launch log keeps growing with TF warnings on
  # some bags, which would starve a log-based quiescence check forever and
  # burn the whole --offline-timeout-secs budget after the bag is processed.
  python3 - "$RAW_TUM" "$CORRECTED_TUM" <<'PY'
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

v2_consumer_evidence_state() {
  # Return 0=pass, 1=invalid/malformed, 2=pending, 124=terminal drain
  # timeout.  The validator owns the JSON schema; this shell only preserves
  # its fail-closed state across the launch/map-save boundary.
  if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" != "m6a10-online-compute-v2" ]]; then
    return 0
  fi
  [[ -n "${M6A10_CONSUMER_EVIDENCE:-}" ]] || return 1
  [[ -s "${M6A10_CHILD_STARTUP_ENV:-}" ]] || return 1
  [[ ! -e "${M6A10_CONSUMER_EVIDENCE}.part" ]] || return 1
  local diagnostic_base="${M6A10_DRAIN_DIAGNOSTIC:-${OUTPUT_DIR}/m6a10_drain_diagnostic.json}"
  local state
  # Keep errexit state untouched.  A `set -e` toggled inside this helper
  # would leak back to callers and abort before they can preserve 1/2/124.
  if python3 "${M6A10_PHASE_SCRIPT}" consumer-state \
      --input "${M6A10_CONSUMER_EVIDENCE}" \
      --diagnostic-base "${diagnostic_base}" >/dev/null; then
    state=0
  else
    state=$?
  fi
  return "${state}"
}

v2_consumer_evidence_ready() {
  # The v2 application evidence is the authoritative EOF/ack boundary.  A
  # trajectory reaching the bag end is not sufficient: the graph wrapper can
  # observe a long writer stall while OfflineNode is still draining.  Keep
  # ordinary (non-v2) runs byte-for-byte on their historical completion path.
  # Preserve the validator's complete state space.  In particular, 1 is an
  # invalid/malformed terminal document, 2 is still pending, and 124 is a
  # valid drain-timeout diagnostic.  Collapsing these into boolean failure
  # would make the wait loop either spin on a terminal error or turn a timeout
  # into a successful map-save path.
  v2_consumer_evidence_state
}

offline_completion_recorded() {
  if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
    local evidence_state
    v2_consumer_evidence_ready
    evidence_state=$?
    [[ "${evidence_state}" -eq 0 ]] || return "${evidence_state}"
  fi
  if grep -Fq "RKO LIO Offline Node took" "$LAUNCH_LOG" 2>/dev/null; then
    return 0
  fi
  if [[ -f "$OUTPUT_DIR/rko_lio.log" ]] && grep -Fq "RKO LIO Offline Node took" "$OUTPUT_DIR/rko_lio.log" 2>/dev/null; then
    return 0
  fi
  if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
    return 2
  fi
  return 1
}

wait_for_offline_completion() {
  local timeout_secs="$1"
  local last_signature=""
  local stable_since=0
  local deadline=$((SECONDS + timeout_secs))
  # Primary completion signal: the raw trajectory has reached the bag end and
  # gone quiet. Quiescence alone is NOT enough — buffered TUM writes can stall
  # for tens of seconds mid-run and fake completion. A long stall (10x the
  # quiescence window) is still accepted as a crash/abort fallback.
  local end_stamp=""
  end_stamp="$(bag_end_stamp 2>/dev/null)" || end_stamp=""
  local stall_secs=$((QUIESCENCE_SECS * 10))

  while (( SECONDS < deadline )); do
    if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
      # v2 requires both application EOF/drain evidence and the launch-side
      # completion log.  The first proves that all input callbacks returned;
      # the second keeps graph fixed-lag post-processing out of the map-save
      # phase.  Do not use the historical TUM shortcut for v2.
      if offline_completion_recorded; then
        return 0
      else
        local completion_state=$?
        if [[ "${completion_state}" -eq 124 ]]; then
          echo "v2 consumer evidence recorded a terminal drain timeout" >&2
          return 124
        fi
        if [[ "${completion_state}" -eq 1 ]]; then
          echo "v2 consumer evidence is invalid or malformed" >&2
          return 125
        fi
      fi
    else
      if [[ -s "$RKO_RESULT_TUM" ]]; then
        return 0
      fi
      if offline_completion_recorded; then
        return 0
      fi
    fi

    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" != "m6a10-online-compute-v2" ]]; then
        return 0
      fi
      if offline_completion_recorded; then
        return 0
      else
        local exited_state=$?
        if [[ "${exited_state}" -eq 124 ]]; then
          return 124
        fi
        if [[ "${exited_state}" -eq 1 ]]; then
          return 125
        fi
      fi
      echo "OfflineNode exited before v2 consumer evidence/completion was finalized" >&2
      return 125
    fi

    local current_signature
    current_signature="$(run_state_signature)"
    if [[ "$current_signature" == "$last_signature" ]] && [[ -f "$RAW_TUM" ]]; then
      if (( stable_since == 0 )); then
        stable_since=$SECONDS
      fi
      if (( SECONDS - stable_since >= QUIESCENCE_SECS )); then
        # Some generic bags retain non-LiDAR topics after the sensor stream,
        # so the historical default is 120 s. Competitive profiles set a
        # dataset-specific tight margin and record it in their provenance.
        if [[ -n "$end_stamp" ]] && \
          raw_tum_reached "$end_stamp" "$COMPLETION_END_MARGIN_SECS" && \
          raw_tum_reached_fraction 0.8; then
          if m6a10_completion_barrier_allows; then
            :
          else
            # Do not let the historical trajectory-stall fallback trigger
            # map-save/cleanup before OfflineNode has written its atomic
            # consumer counters/EOF proof and completed graph post-processing.
            stable_since=$SECONDS
            sleep 2
            continue
          fi
          echo "Trajectory reached the bag end and stayed quiet for ${QUIESCENCE_SECS}s; treating benchmark run as complete"
          return 0
        fi
        if (( SECONDS - stable_since >= stall_secs )); then
          if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
            echo "Trajectory stalled before v2 consumer evidence/completion was finalized" >&2
            return 1
          fi
          echo "Warning: trajectory stalled for ${stall_secs}s before the bag end; treating benchmark run as aborted-but-scoreable" >&2
          return 0
        fi
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
    local remaining_secs=$((deadline - SECONDS))
    if timeout "${remaining_secs}" ros2 service call /map_save std_srvs/srv/Empty "{}" >"${MAP_SAVE_LOG}" 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

assert_no_map_artifacts() {
  # ``--skip-map-save`` is a fail-closed contract, not merely a skipped
  # service call.  Accepted loop closures can otherwise invoke
  # doPoseAdjustment(..., true) and write the same map bundle asynchronously.
  # Keep this list explicit and narrow: trajectory dumps, phase evidence, and
  # consumer proofs remain valid outputs, while every map/bundle path is
  # forbidden in the benchmark-only no-map mode.
  local forbidden_path
  while IFS= read -r -d '' forbidden_path; do
    echo "error: forbidden map artifact present in --skip-map-save output: ${forbidden_path}" >&2
    return 125
  done < <(
    find "$OUTPUT_DIR" -mindepth 1 \( \
      -name 'pointcloud_map' -o \
      -name 'map.pcd' -o \
      -name 'map_bundle.yaml' -o \
      -name 'map_projector_info.yaml' -o \
      -name 'degeneracy_report.yaml' -o \
      -name 'pose_graph.g2o' -o \
      -name 'trajectory_optimized.tum' -o \
      -name 'loop_edges.csv' -o \
      -name 'map_save.log' \
    \) -print0
  )
}

python3 - "$RKO_PARAM" "$RKO_ROS_PARAM_FILE" <<'PY'
import os
import shutil
import sys
from pathlib import Path

import yaml

src_path = Path(sys.argv[1])
dst_path = Path(sys.argv[2])
data = yaml.safe_load(src_path.read_text()) or {}

phase_contract = os.environ.get("M6A10_PHASE_CONTRACT_VERSION", "")
phase_mode = os.environ.get("M6A10_PHASE_MODE", "")
consumer_evidence = os.environ.get("M6A10_CONSUMER_EVIDENCE", "")

def add_benchmark_parameters(parameters):
    if phase_contract == "m6a10-online-compute-v2":
        parameters["m6a10_phase_contract_version"] = phase_contract
        parameters["m6a10_phase_mode"] = phase_mode
        parameters["m6a10_consumer_evidence_path"] = consumer_evidence
        parameters["m6a10_drain_timeout_seconds"] = float(
            os.environ.get("M6A10_DRAIN_TIMEOUT_SECONDS", "30"))
        parameters["m6a10_drain_diagnostic_path"] = os.environ.get(
            "M6A10_DRAIN_DIAGNOSTIC",
            str(dst_path.parent / "m6a10_drain_diagnostic.json"))

if isinstance(data, dict):
    parameter_blocks = [
        value["ros__parameters"] for value in data.values()
        if isinstance(value, dict) and isinstance(value.get("ros__parameters"), dict)
    ]
    if parameter_blocks:
        add_benchmark_parameters(parameter_blocks[0])
        dst_path.write_text(yaml.safe_dump(data, sort_keys=False))
        sys.exit(0)

add_benchmark_parameters(data)
wrapped = {"/**": {"ros__parameters": data}}
dst_path.write_text(yaml.safe_dump(wrapped, sort_keys=False))
PY

if [[ "$GT_BLIND" == "true" ]]; then
  : # Keep the run entirely independent of ground truth.
elif [[ "$SKIP_REFERENCE_GEN" == "false" ]]; then
  python3 "${SCRIPT_DIR}/generate_ntu_viral_tnp01_reference.py" \
    --source-bag "$REFERENCE_BAG" \
    --out "$REFERENCE_TUM" \
    --rko-param "$RKO_PARAM" \
    --write-meta "$REFERENCE_META"
else
  [[ -f "$REFERENCE_TUM" ]] || die "--skip-reference-gen set but reference TUM not found: $REFERENCE_TUM (pass --reference-tum)"
  [[ -f "$REFERENCE_META" ]] || die "--skip-reference-gen set but reference meta not found: $REFERENCE_META (pass --reference-meta, e.g. an empty '{}' JSON if no prism-offset metadata applies)"
fi

echo "Running RKO-LIO benchmark"
echo "  bag:            $BAG_PATH"
if [[ "$GT_BLIND" == "false" ]]; then
  echo "  reference_tum:  $REFERENCE_TUM"
  echo "  reference_meta: $REFERENCE_META"
fi
echo "  lidar_topic:    $LIDAR_TOPIC"
echo "  imu_topic:      $IMU_TOPIC"
echo "  base_frame:     $BASE_FRAME"
echo "  lidarslam_yaml: $LIDARSLAM_PARAM"
echo "  rko_yaml:       $RKO_PARAM"
echo "  output_dir:     $OUTPUT_DIR"
echo "  run_name:       $RUN_NAME"

# The additive v2 phase contract must use the harness launch source mounted by
# the wrapper, because the pinned image's installed launch file predates the
# explicit environment-to-parameter bridge.  Ordinary runs retain the
# installed package launch path and therefore remain unchanged.
RKO_LAUNCH_TARGET=(lidarslam rko_lio_slam.launch.py)
if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" &&
      -f "${REPO_ROOT}/lidarslam/launch/rko_lio_slam.launch.py" ]]; then
  RKO_LAUNCH_TARGET=("${REPO_ROOT}/lidarslam/launch/rko_lio_slam.launch.py")
fi

if command -v setsid >/dev/null 2>&1; then
    setsid bash "${SCRIPT_DIR}/run_with_resource_report.sh" \
    "${OUTPUT_DIR}/compute_time.txt" \
    ros2 launch "${RKO_LAUNCH_TARGET[@]}" \
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
  bash "${SCRIPT_DIR}/run_with_resource_report.sh" \
  "${OUTPUT_DIR}/compute_time.txt" \
  ros2 launch "${RKO_LAUNCH_TARGET[@]}" \
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
M6A10_PHASE_STARTUP_MARKED=0
if [[ -n "${M6A10_PHASE_EVENTS:-}" ]]; then
  m6a10_phase_mark startup_end || true
  m6a10_phase_mark input_start || true
  M6A10_PHASE_STARTUP_MARKED=1
fi
if wait_for_offline_completion "$OFFLINE_TIMEOUT_SECS"; then
  :
else
  completion_status=$?
  echo "Launch terminated before offline node completed. Recent launch log:" >&2
  tail -n 120 "$LAUNCH_LOG" >&2 || true
  exit "${completion_status}"
fi

if [[ -n "${M6A10_PHASE_EVENTS:-}" ]]; then
  m6a10_phase_mark input_end || true
  m6a10_phase_mark drain_start || true
  M6A10_PHASE_END_STAMP="${M6A10_SENSOR_END_TIMESTAMP_SECONDS:-}"
  if [[ -z "${M6A10_PHASE_END_STAMP}" ]]; then
    M6A10_PHASE_END_STAMP="$(bag_end_stamp 2>/dev/null || true)"
  fi
  if [[ -n "${M6A10_PHASE_END_STAMP}" ]]; then
    m6a10_phase_record_trajectory_coverage \
      "${RAW_TUM}" "${M6A10_PHASE_END_STAMP}" || true
  fi
  m6a10_phase_mark drain_end || true
  m6a10_phase_mark postprocess_start || true
  m6a10_phase_mark postprocess_end || true
fi

if [[ "$SKIP_MAP_SAVE" == "false" ]]; then
  if [[ -n "${M6A10_PHASE_EVENTS:-}" ]]; then
    m6a10_phase_mark save_start || true
  fi
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
  if [[ -n "${M6A10_PHASE_EVENTS:-}" ]]; then
    m6a10_phase_mark save_end || true
  fi
else
  if [[ -n "${M6A10_PHASE_EVENTS:-}" ]]; then
    m6a10_phase_mark save_start || true
    m6a10_phase_mark save_end || true
  fi
fi

for pid in "$RAW_LOGGER_PID" "$CORRECTED_LOGGER_PID"; do
  terminate_pid "$pid"
done
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""

# Logger termination flushes the final trajectory sample. Re-record coverage
# after that flush so a slow writer cannot leave a stale end-gap proof behind.
if [[ -n "${M6A10_PHASE_EVENTS:-}" ]]; then
  M6A10_PHASE_END_STAMP="${M6A10_SENSOR_END_TIMESTAMP_SECONDS:-}"
  if [[ -z "${M6A10_PHASE_END_STAMP}" ]]; then
    M6A10_PHASE_END_STAMP="$(bag_end_stamp 2>/dev/null || true)"
  fi
  if [[ -n "${M6A10_PHASE_END_STAMP}" ]]; then
    m6a10_phase_record_trajectory_coverage \
      "${RAW_TUM}" "${M6A10_PHASE_END_STAMP}" || true
  fi
fi

if [[ -n "$LAUNCH_PGID" ]]; then
  terminate_process_group "$LAUNCH_PGID" "$LAUNCH_PID"
elif [[ -n "$LAUNCH_PID" ]]; then
  terminate_pid "$LAUNCH_PID"
fi
LAUNCH_PID=""
LAUNCH_PGID=""

if [[ "$SKIP_MAP_SAVE" == "true" ]]; then
  assert_no_map_artifacts
fi

# The ROS topic logger is best-effort observation and can miss startup samples
# under host load.  The offline node's atomic full-rate dump is authoritative
# for every benchmark mode; require exactly one and use it for scoring.
readarray -t BACKEND_TUMS < <(
  find "$OUTPUT_DIR" -mindepth 2 -maxdepth 2 -type f -name '*_tum_*.txt' -print
)
if [[ "${#BACKEND_TUMS[@]}" -ne 1 || ! -s "${BACKEND_TUMS[0]}" ]]; then
  die "benchmark expected exactly one authoritative full-rate TUM dump"
fi
cp "${BACKEND_TUMS[0]}" "$RAW_TUM"
echo "Authoritative full-rate trajectory: ${BACKEND_TUMS[0]}"

# /modified_path is emitted by map_save. A --skip-map-save run cannot measure
# graph correction, so use the same authoritative dump for both paths and
# explicitly treat the result as a frontend/passthrough benchmark.
if [[ "$SKIP_MAP_SAVE" == "true" ]]; then
    cp "${BACKEND_TUMS[0]}" "$CORRECTED_TUM"
    echo "Trajectory-only passthrough from full-rate dump: ${BACKEND_TUMS[0]}"
elif [[ ! -s "$CORRECTED_TUM" ]]; then
  die "corrected trajectory missing after map_save"
fi

# /modified_path contains graph anchors, not a full-rate trajectory.  Scoring
# those sparse poses directly changes timestamp association and can report a
# large error even when every optimized anchor is identical to the raw pose.
# Preserve the anchors for audit, then propagate their world-side corrections
# onto every raw pose so traj_corrected.tum has the same temporal support as
# traj_raw.tum.
cp "$CORRECTED_TUM" "$CORRECTED_SPARSE_TUM"
python3 "${SCRIPT_DIR}/densify_corrected_trajectory.py" \
  --raw "$RAW_TUM" \
  --corrected "$CORRECTED_SPARSE_TUM" \
  --output "$CORRECTED_TUM"
if [[ "$(wc -l < "$CORRECTED_TUM")" -ne "$(wc -l < "$RAW_TUM")" ]]; then
  die "dense corrected trajectory pose count differs from raw trajectory"
fi

if [[ "$GT_BLIND" == "true" ]]; then
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
  python3 - "$OUTPUT_DIR/gt_blind_run.json" "$BAG_PATH" \
    "$RAW_TUM" "$CORRECTED_TUM" "$BENCH_WALL_SEC" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

output, bag, raw, corrected, wall = sys.argv[1:]
document = {
    'schema_version': 1,
    'ground_truth_accessed': False,
    'scorer_invoked': False,
    'bag_path': bag,
    'trajectory': {
        'raw_path': raw,
        'raw_sha256': digest(raw),
        'corrected_path': corrected,
        'corrected_sha256': digest(corrected),
    },
    'wall_seconds': float(wall),
}
Path(output).write_text(json.dumps(document, indent=2) + '\n', encoding='utf-8')
PY
  echo "GT-blind benchmark completed"
  echo "  output_dir: $OUTPUT_DIR"
  echo "  gt_blind_manifest: $OUTPUT_DIR/gt_blind_run.json"
  exit 0
fi

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
  --sparse-match \
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
