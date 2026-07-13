#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${REPO_ROOT}"
if [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
MODE=""
OUTPUT_DIR="${REPO_ROOT}/output/backend_replay_${TIMESTAMP}"
OUTPUT_DIR_GIVEN=false
INPUT_BAG=""
RUNS=3
PLAY_RATE="1.0"
DRAIN_SECS="10"
ADJACENT_WINDOW=5
LIDARSLAM_PARAM="lidarslam/param/lidarslam_mid360_rko_graph.yaml"
LIDARSLAM_PARAM_GIVEN=false
REFERENCE_TUM="output/glim_mid360_reference.tum"
SOURCE_BAG=""

RECORDER_PID=""
LOGGER_PID=""
BACKEND_PID=""
BACKEND_PGID=""

usage() {
  cat <<'EOF'
Usage: run_backend_replay_variance.sh --mode record|replay [options]

Measures backend + ROS transport replay variance from byte-identical RKO-LIO output topics.

Options:
  --mode record|replay
      record: run the MID-360 cross-validation benchmark once and record backend inputs
      replay: replay a recorded backend input bag into graph_based_slam only

  --output-dir DIR
      Output directory.
      Default in record mode: output/backend_replay_<timestamp>
      Required explicitly in replay mode so the recorded bag can be reused.

  --input-bag DIR
      Replay input bag directory.
      Replay default: $OUTPUT_DIR/backend_input

  --runs N
      Replay run count. Default: 3

  --rate FLOAT
      ros2 bag play rate for replay mode. Default: 1.0

  --drain-secs SECS
      Seconds to wait after bag playback before /map_save. Default: 10

  --adjacent-window N
      g2o edges with |i-j| <= N are treated as odometry adjacency constraints,
      not loop closures (match num_adjacent_pose_cnstraints). Default: 5

  --lidarslam-param FILE
      graph_based_slam parameter file.
      Default: lidarslam/param/lidarslam_mid360_rko_graph.yaml
      In record mode this is forwarded to the source benchmark only when given.

  --reference-tum FILE
      Reference trajectory for APE post-processing.
      Default: output/glim_mid360_reference.tum

  --bag FILE
      Source bag forwarded to run_rko_lio_mid360_crossval_benchmark.sh in record mode.

  --help
      Show this help.

Replay mode keeps the current ROS_DOMAIN_ID environment. No other ROS nodes should be
running in the same domain while this harness is recording or replaying.
EOF
}

resolve_repo_path() {
  local path="$1"
  if [[ "${path}" = /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s\n' "${REPO_ROOT}/${path}"
  fi
}

source_ros() {
  set +u
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${WS_ROOT}/install/setup.bash"
  elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  fi
  set -u
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_secs="$2"
  local deadline=$((SECONDS + timeout_secs))

  while kill -0 "${pid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 1
  done
  return 0
}

wait_for_pgid_exit() {
  local pgid="$1"
  local timeout_secs="$2"
  local deadline=$((SECONDS + timeout_secs))

  while kill -0 -- "-${pgid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 1
  done
  return 0
}

stop_pid_with_guard() {
  local pid="$1"
  local label="$2"
  local grace_secs="${3:-10}"

  if [[ -z "${pid}" ]]; then
    return 0
  fi

  if kill -0 "${pid}" 2>/dev/null; then
    echo "Stopping ${label} pid=${pid}"
    kill -INT "${pid}" 2>/dev/null || true
    if ! wait_for_pid_exit "${pid}" "${grace_secs}"; then
      echo "WARN: ${label} did not exit after SIGINT; sending SIGKILL"
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi

  wait "${pid}" 2>/dev/null || true
}

stop_backend_group() {
  if [[ -z "${BACKEND_PGID}" ]]; then
    return 0
  fi

  if kill -0 -- "-${BACKEND_PGID}" 2>/dev/null; then
    echo "Stopping backend process group pgid=${BACKEND_PGID}"
    kill -INT -- "-${BACKEND_PGID}" 2>/dev/null || true
    if ! wait_for_pgid_exit "${BACKEND_PGID}" 10; then
      echo "WARN: backend process group did not exit after SIGINT; sending SIGKILL"
      kill -KILL -- "-${BACKEND_PGID}" 2>/dev/null || true
    fi
  fi

  if [[ -n "${BACKEND_PID}" ]]; then
    wait "${BACKEND_PID}" 2>/dev/null || true
  fi

  BACKEND_PID=""
  BACKEND_PGID=""
}

cleanup() {
  set +e
  stop_pid_with_guard "${RECORDER_PID}" "rosbag recorder"
  RECORDER_PID=""

  stop_pid_with_guard "${LOGGER_PID}" "path logger"
  LOGGER_PID=""

  stop_backend_group
}
trap cleanup EXIT INT TERM

topic_count_from_info() {
  local topic="$1"
  local info_file="$2"

  awk -v topic="${topic}" '
    function emit_count(line) {
      if (match(line, /(Count|message_count):[[:space:]]*[0-9]+/)) {
        value = substr(line, RSTART, RLENGTH)
        gsub(/[^0-9]/, "", value)
        print value
        exit
      }
    }

    {
      if (index($0, "Topic: " topic) > 0) {
        in_topic = 1
        emit_count($0)
        next
      }

      if ($0 ~ /^[[:space:]]*Topic:/) {
        in_topic = 0
      }

      if ($0 ~ /^[[:space:]]*name:[[:space:]]*/) {
        name = $0
        sub(/^[[:space:]]*name:[[:space:]]*/, "", name)
        in_topic = (name == topic)
      }

      if (in_topic) {
        emit_count($0)
      }
    }
  ' "${info_file}"
}

record_mode() {
  mkdir -p "${OUTPUT_DIR}"
  export ROS_LOG_DIR="${OUTPUT_DIR}/ros_logs_record"
  mkdir -p "${ROS_LOG_DIR}"

  local record_log="${OUTPUT_DIR}/backend_input_record.log"
  local bag_dir="${OUTPUT_DIR}/backend_input"
  local benchmark_status=0

  echo "Recording backend inputs to ${bag_dir}"
  ros2 bag record -o "${bag_dir}" /rko_lio/odometry /rko_lio/frame \
    > "${record_log}" 2>&1 &
  RECORDER_PID=$!

  sleep 2

  local benchmark_cmd=(
    bash "${SCRIPT_DIR}/run_rko_lio_mid360_crossval_benchmark.sh"
    --output-dir "${OUTPUT_DIR}/source_run"
    --reference-tum "${REFERENCE_TUM}"
  )

  if [[ -n "${SOURCE_BAG}" ]]; then
    benchmark_cmd+=(--bag "${SOURCE_BAG}")
  fi

  if [[ "${LIDARSLAM_PARAM_GIVEN}" == true ]]; then
    benchmark_cmd+=(--lidarslam-param "${LIDARSLAM_PARAM}")
  fi

  echo "Running source benchmark"
  set +e
  "${benchmark_cmd[@]}"
  benchmark_status=$?
  set -e

  # A pointcloud-heavy mcap can take well over 10 s to flush on SIGINT;
  # killing the recorder early loses metadata.yaml.
  stop_pid_with_guard "${RECORDER_PID}" "rosbag recorder" 60
  RECORDER_PID=""

  if [[ ! -f "${bag_dir}/metadata.yaml" ]]; then
    echo "WARN: recorder left no metadata.yaml; reindexing ${bag_dir}" >&2
    ros2 bag reindex "${bag_dir}" -s mcap
  fi

  local info_file="${OUTPUT_DIR}/backend_input_info.txt"
  echo "Inspecting recorded bag"
  ros2 bag info "${bag_dir}" | tee "${info_file}"

  local odom_count
  local frame_count
  odom_count="$(topic_count_from_info /rko_lio/odometry "${info_file}")"
  frame_count="$(topic_count_from_info /rko_lio/frame "${info_file}")"
  odom_count="${odom_count:-0}"
  frame_count="${frame_count:-0}"

  echo "Recorded /rko_lio/odometry messages: ${odom_count}"
  echo "Recorded /rko_lio/frame messages: ${frame_count}"

  if (( odom_count <= 0 || frame_count <= 0 )); then
    echo "ERROR: recorded bag is missing required backend input messages" >&2
    exit 1
  fi

  if (( benchmark_status != 0 )); then
    echo "ERROR: source benchmark failed with status ${benchmark_status}" >&2
    exit "${benchmark_status}"
  fi

  echo "Record complete: ${OUTPUT_DIR}"
}

extract_loop_edges() {
  local g2o_file="$1"
  local out_file="$2"

  if [[ ! -s "${g2o_file}" ]]; then
    : > "${out_file}"
    echo "WARN: pose graph missing or empty: ${g2o_file}" >&2
    return 1
  fi

  # Edges within num_adjacent_pose_cnstraints (default 5) of each other are
  # odometry adjacency constraints, not loop closures.
  awk -v window="${ADJACENT_WINDOW}" '
    $1 == "EDGE_SE3:QUAT" {
      i = $2
      j = $3
      d = i - j
      if (d < 0) {
        d = -d
      }
      if (d > window) {
        if (i <= j) {
          print i, j
        } else {
          print j, i
        }
      }
    }
  ' "${g2o_file}" | sort -n -k1,1 -k2,2 -u > "${out_file}"
}

call_map_save() {
  local run_dir="$1"
  local log_file="${run_dir}/map_save.log"

  if timeout 15 ros2 service call /map_save std_srvs/srv/Empty "{}" \
    > "${log_file}" 2>&1; then
    return 0
  fi

  echo "WARN: /map_save failed once; retrying" | tee -a "${log_file}" >&2
  sleep 2

  timeout 15 ros2 service call /map_save std_srvs/srv/Empty "{}" \
    >> "${log_file}" 2>&1
}

run_one_replay() {
  local idx="$1"
  local run_dir="${OUTPUT_DIR}/replay_run${idx}"
  local failed=0

  mkdir -p "${run_dir}"
  export ROS_LOG_DIR="${run_dir}/ros_logs"
  mkdir -p "${ROS_LOG_DIR}"

  echo "Replay run ${idx}: ${run_dir}"

  # The backend writes pose_graph.g2o into its CWD (hardcoded relative path
  # in doPoseAdjustment), so give every run its own working directory.
  (
    cd "${run_dir}" && exec setsid ros2 run graph_based_slam graph_based_slam_node --ros-args \
      --params-file "${LIDARSLAM_PARAM}" \
      -r odom_input:=/rko_lio/odometry \
      -r cloud_input:=/rko_lio/frame \
      -p use_odom_input:=true \
      -p use_sim_time:=true \
      -p map_save_dir:="${run_dir}" \
      -p save_map_path:="${run_dir}/map.pcd" \
      -p save_pose_graph_path:="${run_dir}/pose_graph.g2o"
  ) > "${run_dir}/backend.log" 2>&1 &
  BACKEND_PID=$!
  BACKEND_PGID="${BACKEND_PID}"

  python3 "${SCRIPT_DIR}/path_to_tum.py" \
    --topic /modified_path \
    --output "${run_dir}/traj_corrected.tum" \
    --use-sim-time true \
    > "${run_dir}/path_logger.log" 2>&1 &
  LOGGER_PID=$!

  sleep 5

  if ! kill -0 "${BACKEND_PID}" 2>/dev/null; then
    echo "WARN: backend exited before playback in run ${idx}" >&2
    failed=1
  fi

  if ! ros2 bag play "${INPUT_BAG}" --clock 100 --rate "${PLAY_RATE}" \
    > "${run_dir}/bag_play.log" 2>&1; then
    echo "WARN: ros2 bag play failed in run ${idx}" >&2
    failed=1
  fi

  sleep "${DRAIN_SECS}"

  if ! call_map_save "${run_dir}"; then
    echo "WARN: /map_save failed twice in run ${idx}; continuing" >&2
  fi

  sleep 3

  stop_pid_with_guard "${LOGGER_PID}" "path logger"
  LOGGER_PID=""

  stop_backend_group

  if ! python3 "${SCRIPT_DIR}/ape_from_tum.py" \
    --ref "${REFERENCE_TUM}" \
    --est "${run_dir}/traj_corrected.tum" \
    --out "${run_dir}/ape.txt" \
    > "${run_dir}/ape_postprocess.log" 2>&1; then
    echo "WARN: APE post-processing failed in run ${idx}; continuing" >&2
  fi

  if ! extract_loop_edges "${run_dir}/pose_graph.g2o" "${run_dir}/loop_edges.txt"; then
    failed=1
  fi

  if (( failed == 0 )); then
    echo "PASS" > "${run_dir}/status.txt"
  else
    echo "FAIL" > "${run_dir}/status.txt"
  fi

  return "${failed}"
}

replay_mode() {
  if [[ "${OUTPUT_DIR_GIVEN}" != true ]]; then
    echo "ERROR: --output-dir is required in replay mode" >&2
    exit 2
  fi

  mkdir -p "${OUTPUT_DIR}"

  if [[ -z "${INPUT_BAG}" ]]; then
    INPUT_BAG="${OUTPUT_DIR}/backend_input"
  fi

  if [[ ! -d "${INPUT_BAG}" ]]; then
    echo "ERROR: input bag directory does not exist: ${INPUT_BAG}" >&2
    exit 2
  fi

  local failures=0
  local idx
  for idx in $(seq 1 "${RUNS}"); do
    set +e
    run_one_replay "${idx}"
    local status=$?
    set -e

    if (( status != 0 )); then
      failures=$((failures + 1))
      echo "WARN: replay run ${idx} recorded FAIL"
    fi
  done

  local summary_file="${OUTPUT_DIR}/backend_replay_summary.md"
  python3 "${SCRIPT_DIR}/aggregate_backend_replay.py" \
    --bench-dir "${OUTPUT_DIR}" | tee "${summary_file}"

  echo "Summary written to ${summary_file}"

  if (( failures > 0 )); then
    echo "WARN: ${failures} replay run(s) recorded FAIL; see per-run status.txt files" >&2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$(resolve_repo_path "$2")"
      OUTPUT_DIR_GIVEN=true
      shift 2
      ;;
    --input-bag)
      INPUT_BAG="$(resolve_repo_path "$2")"
      shift 2
      ;;
    --runs)
      RUNS="$2"
      shift 2
      ;;
    --rate)
      PLAY_RATE="$2"
      shift 2
      ;;
    --drain-secs)
      DRAIN_SECS="$2"
      shift 2
      ;;
    --adjacent-window)
      ADJACENT_WINDOW="$2"
      shift 2
      ;;
    --lidarslam-param)
      LIDARSLAM_PARAM="$(resolve_repo_path "$2")"
      LIDARSLAM_PARAM_GIVEN=true
      shift 2
      ;;
    --reference-tum)
      REFERENCE_TUM="$(resolve_repo_path "$2")"
      shift 2
      ;;
    --bag)
      SOURCE_BAG="$(resolve_repo_path "$2")"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${MODE}" ]]; then
  echo "ERROR: --mode is required" >&2
  usage >&2
  exit 2
fi

LIDARSLAM_PARAM="$(resolve_repo_path "${LIDARSLAM_PARAM}")"
REFERENCE_TUM="$(resolve_repo_path "${REFERENCE_TUM}")"

source_ros

case "${MODE}" in
  record)
    record_mode
    ;;
  replay)
    replay_mode
    ;;
  *)
    echo "ERROR: --mode must be record or replay" >&2
    exit 2
    ;;
esac
