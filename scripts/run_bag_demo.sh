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
  bash scripts/run_bag_demo.sh --bag /path/to/bag [options]

Required:
  --bag PATH                  rosbag2 directory (or file, depending on your storage plugin)

Options:
  --points-topic TOPIC        PointCloud2 topic (auto-detects if omitted)
  --imu-topic TOPIC           Imu topic (auto-detects if omitted)
  --no-imu                    Disable IMU remap and pick non-IMU defaults
  --param FILE                Parameter YAML (default: auto-select; IMU->lidarslam_solid_state_imu.yaml else lidarslam.yaml)
  --save-dir DIR              Output directory (default: ./output under the repo)
  --rviz                       Start RViz (default: off)
  --no-graph-based-slam        Do not launch the graph_based_slam backend node
  --use-sim-time true|false   Use /clock time and play bag with --clock (default: true)
  --rate FLOAT                Playback rate (default: 1.0)
  --loop                       Loop playback

Frames (optional):
  --global-frame-id FRAME      Global frame id passed to lidarslam launch (default: map)
  --odom-frame-id FRAME        Odom frame id passed to lidarslam launch (default: odom)
  --robot-frame-id FRAME       Robot/base frame id passed to lidarslam launch (default: base_link)

Trajectory logging (optional):
  --tum-out FILE               Write TF trajectory in TUM format (t x y z qx qy qz qw)
  --tum-parent-frame FRAME     Parent frame to log (default: --global-frame-id)
  --tum-child-frame FRAME      Child frame to log (default: --robot-frame-id)
  --tum-rate HZ                Logging rate (default: 50.0)

TF helpers (optional):
  --publish-static-tf          Publish a static TF from base_frame -> lidar_frame (default: off)
  --auto-static-tf             If bag has no /tf(/tf_static), enable identity static TF (default: off)
  --auto-static-tf-timeout SEC Timeout for frame_id detection used by --auto-static-tf (default: 5)
  --base-frame FRAME           Base frame for the static TF (default: base_link)
  --lidar-frame FRAME          LiDAR frame for the static TF (default: lidar)
  --points-frame-id FRAME      Force PointCloud2 frame_id (also used as fallback for auto-static TF)
  --static-tf "x y z qx qy qz qw"
                               Static transform values (default: identity)

Notes:
  - If your bag already contains /tf(/tf_static), prefer leaving --publish-static-tf off.
  - If PointCloud2 has no intensity field, the frontend will set intensity=0 internally.
  - --auto-static-tf is intended for demos; for production, record /tf and /tf_static in your bag.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

is_ntu_viral_tnp01_bag() {
  local bag_path="${1:-}"
  [[ -n "${bag_path}" ]] || return 1
  [[ "${bag_path}" == *"/tnp_01"* || "${bag_path}" == *"tnp_01_rosbag2"* || "${bag_path}" == *"tnp_01_points_restamped_vn100_rosbag2"* ]]
}

BAG_PATH=""
POINTS_TOPIC=""
IMU_TOPIC=""
IMU_TOPIC_USER_SPECIFIED="false"
NO_IMU="false"
PARAM_FILE=""
SAVE_DIR=""
USE_RVIZ="false"
USE_GRAPH_BASED_SLAM="true"
USE_SIM_TIME="true"
RATE="1.0"
LOOP="false"

GLOBAL_FRAME_ID="map"
ODOM_FRAME_ID="odom"
ROBOT_FRAME_ID="base_link"
ROBOT_FRAME_USER_SPECIFIED="false"

AUTO_STATIC_TF="false"
AUTO_STATIC_TF_TIMEOUT="5"

TUM_OUT=""
TUM_PARENT_FRAME=""
TUM_CHILD_FRAME=""
TUM_RATE="50.0"

PUBLISH_STATIC_TF="false"
BASE_FRAME=""
BASE_FRAME_USER_SPECIFIED="false"
LIDAR_FRAME="lidar"
LIDAR_FRAME_USER_SPECIFIED="false"
POINTS_FRAME_ID=""
POINTS_FRAME_ID_USER_SPECIFIED="false"
STATIC_TF="0 0 0 0 0 0 1"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag)
      BAG_PATH="${2:-}"; shift 2 ;;
    --points-topic)
      POINTS_TOPIC="${2:-}"; shift 2 ;;
    --imu-topic)
      IMU_TOPIC="${2:-}"; IMU_TOPIC_USER_SPECIFIED="true"; shift 2 ;;
    --no-imu)
      NO_IMU="true"; shift ;;
    --param)
      PARAM_FILE="${2:-}"; shift 2 ;;
    --save-dir)
      SAVE_DIR="${2:-}"; shift 2 ;;
    --rviz)
      USE_RVIZ="true"; shift ;;
    --no-graph-based-slam)
      USE_GRAPH_BASED_SLAM="false"; shift ;;
    --use-sim-time)
      USE_SIM_TIME="${2:-}"; shift 2 ;;
    --rate)
      RATE="${2:-}"; shift 2 ;;
    --loop)
      LOOP="true"; shift ;;
    --global-frame-id)
      GLOBAL_FRAME_ID="${2:-}"; shift 2 ;;
    --odom-frame-id)
      ODOM_FRAME_ID="${2:-}"; shift 2 ;;
    --robot-frame-id)
      ROBOT_FRAME_ID="${2:-}"
      ROBOT_FRAME_USER_SPECIFIED="true"
      shift 2 ;;
    --auto-static-tf)
      AUTO_STATIC_TF="true"; shift ;;
    --auto-static-tf-timeout)
      AUTO_STATIC_TF_TIMEOUT="${2:-}"; shift 2 ;;
    --tum-out)
      TUM_OUT="${2:-}"; shift 2 ;;
    --tum-parent-frame)
      TUM_PARENT_FRAME="${2:-}"; shift 2 ;;
    --tum-child-frame)
      TUM_CHILD_FRAME="${2:-}"; shift 2 ;;
    --tum-rate)
      TUM_RATE="${2:-}"; shift 2 ;;
    --publish-static-tf)
      PUBLISH_STATIC_TF="true"; shift ;;
    --base-frame)
      BASE_FRAME="${2:-}"
      BASE_FRAME_USER_SPECIFIED="true"
      shift 2 ;;
    --lidar-frame)
      LIDAR_FRAME="${2:-}"; LIDAR_FRAME_USER_SPECIFIED="true"; shift 2 ;;
    --points-frame-id)
      POINTS_FRAME_ID="${2:-}"
      POINTS_FRAME_ID_USER_SPECIFIED="true"
      shift 2 ;;
    --static-tf)
      STATIC_TF="${2:-}"; shift 2 ;;
    --)
      shift
      break
      ;;
    *)
      if [[ -z "${BAG_PATH}" && ! "$1" =~ ^- ]]; then
        BAG_PATH="$1"; shift
      else
        die "unknown arg: $1 (use --help)"
      fi
      ;;
  esac
done

[[ -n "${BAG_PATH}" ]] || { usage; die "--bag is required"; }

if [[ -z "${SAVE_DIR}" ]]; then
  SAVE_DIR="${REPO_ROOT}/output"
fi
mkdir -p "${SAVE_DIR}"
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${SAVE_DIR}/.ros_log"
fi
mkdir -p "${ROS_LOG_DIR}"

detect_topic_frame_id() {
  local topic="${1:-}"
  local timeout_sec="${2:-}"
  local frame_raw=""
  local line=""
  local candidate=""

  [[ -z "${topic}" ]] && { echo ""; return; }

  if command -v timeout >/dev/null 2>&1; then
    frame_raw="$(timeout "${timeout_sec}" ros2 topic echo --once --qos-profile sensor_data --field header.frame_id "${topic}" 2>/dev/null || true)"
  else
    frame_raw="$(ros2 topic echo --once --qos-profile sensor_data --field header.frame_id "${topic}" 2>/dev/null || true)"
  fi
  if [[ -z "${frame_raw}" ]]; then
    if command -v timeout >/dev/null 2>&1; then
      frame_raw="$(timeout "${timeout_sec}" ros2 topic echo --once --qos-profile sensor_data "${topic}" 2>/dev/null | awk '/frame_id:/ {print $2; exit}' || true)"
    else
      frame_raw="$(ros2 topic echo --once --qos-profile sensor_data "${topic}" 2>/dev/null | awk '/frame_id:/ {print $2; exit}' || true)"
    fi
  fi

  while IFS= read -r line; do
    line="$(echo "${line}" | tr -d '\r' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
    [[ -z "${line}" ]] && continue

    if [[ "${line}" == *":"* ]]; then
      line="${line#*:}"
      line="$(echo "${line}" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
    fi

    line="${line//\"/}"
    line="${line//\'/}"
    if [[ "${line}" == *"message was lost"* || "${line}" == *"does not appear to be published yet"* ]]; then
      continue
    fi
    if [[ "${line}" != *" "* ]] && [[ "${line}" =~ ^[A-Za-z0-9_./:-]+$ ]]; then
      candidate="${line}"
      break
    fi
  done <<< "${frame_raw}"

  echo "${candidate}"
}

sanitize_frame_id() {
  local value="${1:-}"
  value="$(echo "${value}" | tr -d '\r' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
  value="${value#\"}"
  value="${value%\"}"
  value="${value#\'}"
  value="${value%\'}"
  if [[ "${value}" == *" "* ]]; then
    echo ""
    return
  fi
  if [[ ! "${value}" =~ ^[A-Za-z_/.][A-Za-z0-9_./:-]*$ ]]; then
    echo ""
    return
  fi
  echo "${value}"
}

# Best-effort environment setup (won't override an already-sourced environment).
set +u
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || die "ros2 not found in PATH (source your ROS 2 environment first)"

bag_info="$(ros2 bag info "${BAG_PATH}" 2>/dev/null || true)"

if [[ -z "${POINTS_TOPIC}" ]]; then
  POINTS_TOPIC="$(
    echo "${bag_info}" | awk -v type="sensor_msgs/msg/PointCloud2" -F'[|]' '
      $0 ~ ("Type: " type) && $0 ~ /Topic:/ && $0 ~ /Count:/ {
        topic=""; count=0;
        for (i=1; i<=NF; i++) {
          seg=$i;
          gsub(/^[ \t]+|[ \t]+$/, "", seg);
          if (seg ~ /^Topic:/) { sub(/^Topic:[ \t]*/, "", seg); topic=seg; }
          if (seg ~ /^Count:/) { sub(/^Count:[ \t]*/, "", seg); count=seg+0; }
        }
        if (topic != "" && count > best_count) { best_count=count; best_topic=topic; }
      }
      END { if (best_topic != "") print best_topic; }
    '
  )"
fi
HAS_IMU="false"
if [[ "${NO_IMU}" != "true" ]]; then
  if [[ -n "${IMU_TOPIC}" ]]; then
    HAS_IMU="true"
  else
    IMU_TOPIC="$(
      echo "${bag_info}" | awk -v type="sensor_msgs/msg/Imu" -F'[|]' '
        $0 ~ ("Type: " type) && $0 ~ /Topic:/ && $0 ~ /Count:/ {
          topic=""; count=0;
          for (i=1; i<=NF; i++) {
            seg=$i;
            gsub(/^[ \t]+|[ \t]+$/, "", seg);
            if (seg ~ /^Topic:/) { sub(/^Topic:[ \t]*/, "", seg); topic=seg; }
            if (seg ~ /^Count:/) { sub(/^Count:[ \t]*/, "", seg); count=seg+0; }
          }
          if (topic != "" && count > best_count) { best_count=count; best_topic=topic; }
        }
        END { if (best_topic != "") print best_topic; }
      '
    )"
    if [[ -n "${IMU_TOPIC}" ]]; then
      HAS_IMU="true"
    fi
  fi
fi

if [[ -z "${POINTS_TOPIC}" ]]; then
  POINTS_TOPIC="/points_raw"
  echo "warn: failed to auto-detect PointCloud2 topic; using default: ${POINTS_TOPIC}" >&2
fi
if [[ -z "${IMU_TOPIC}" ]]; then
  IMU_TOPIC="/imu"
fi

if [[ -z "${PARAM_FILE}" ]]; then
  if is_ntu_viral_tnp01_bag "${BAG_PATH}"; then
    PARAM_FILE="${REPO_ROOT}/lidarslam/param/lidarslam_ouster_aggressive_noimu.yaml"
    USE_GRAPH_BASED_SLAM="false"
    if [[ "${IMU_TOPIC_USER_SPECIFIED}" != "true" ]]; then
      NO_IMU="true"
    fi
  elif [[ "${HAS_IMU}" == "true" ]]; then
    if [[ "${BAG_PATH}" == *"/glim_mid360/"* || "${BAG_PATH}" == *"rosbag2_2024_04_16-14_17_01"* ]]; then
      PARAM_FILE="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_noimu.yaml"
      USE_GRAPH_BASED_SLAM="false"
    else
      PARAM_FILE="${REPO_ROOT}/lidarslam/param/lidarslam_solid_state_imu.yaml"
    fi
  else
    PARAM_FILE="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
  fi
fi
[[ -f "${PARAM_FILE}" ]] || die "param file not found: ${PARAM_FILE}"

bag_has_tf_topics="false"
if echo "${bag_info}" | grep -qE 'Topic:[[:space:]]*/tf([[:space:]]|[|])'; then
  bag_has_tf_topics="true"
fi
if echo "${bag_info}" | grep -qE 'Topic:[[:space:]]*/tf_static([[:space:]]|[|])'; then
  bag_has_tf_topics="true"
fi

if [[ "${AUTO_STATIC_TF}" == "true" && "${PUBLISH_STATIC_TF}" != "true" && "${bag_has_tf_topics}" != "true" ]]; then
  echo "warn: bag has no /tf or /tf_static; enabling identity static TF for demo" >&2
  PUBLISH_STATIC_TF="true"
  if [[ "${LIDAR_FRAME_USER_SPECIFIED}" != "true" || "${BASE_FRAME_USER_SPECIFIED}" != "true" || "${ROBOT_FRAME_USER_SPECIFIED}" != "true" ]]; then
    if command -v timeout >/dev/null 2>&1; then
      echo "detecting frame_id for auto static TF (timeout=${AUTO_STATIC_TF_TIMEOUT}s)..." >&2
    else
      echo "warn: timeout command not found; frame_id detection may hang if the bag doesn't publish ${POINTS_TOPIC}" >&2
    fi

    play_pid=""
    probe_topics=("${POINTS_TOPIC}")
    if [[ "${NO_IMU}" != "true" && -n "${IMU_TOPIC}" ]]; then
      probe_topics+=("${IMU_TOPIC}")
    fi
    ros2 bag play "${BAG_PATH}" --topics "${probe_topics[@]}" --rate 10.0 >/dev/null 2>&1 &
    play_pid="$!"

    if [[ "${POINTS_FRAME_ID_USER_SPECIFIED}" == "true" ]]; then
      detected_frame_id="$(sanitize_frame_id "${POINTS_FRAME_ID}")"
    else
      detected_frame_id="$(sanitize_frame_id "$(detect_topic_frame_id "${POINTS_TOPIC}" "${AUTO_STATIC_TF_TIMEOUT}")")"
    fi
	    detected_imu_frame_id="$(detect_topic_frame_id "${IMU_TOPIC}" "${AUTO_STATIC_TF_TIMEOUT}")"
	    detected_imu_frame_id="$(sanitize_frame_id "${detected_imu_frame_id}")"

	    if [[ -z "${detected_frame_id}" ]] && is_ntu_viral_tnp01_bag "${BAG_PATH}"; then
	      detected_frame_id="sensor1/os_sensor"
	    fi

	    kill "${play_pid}" 2>/dev/null || true
	    wait "${play_pid}" 2>/dev/null || true

    if [[ -n "${detected_frame_id}" && "${LIDAR_FRAME_USER_SPECIFIED}" != "true" ]]; then
      LIDAR_FRAME="${detected_frame_id}"
    else
      echo "warn: failed to detect PointCloud2 frame_id; using lidar_frame=${LIDAR_FRAME}" >&2
    fi

    if [[ "${ROBOT_FRAME_USER_SPECIFIED}" != "true" ]]; then
      if [[ -n "${detected_frame_id}" ]]; then
        ROBOT_FRAME_ID="${detected_frame_id}"
      fi
    fi

    if [[ -z "${BASE_FRAME}" ]]; then
      BASE_FRAME="${ROBOT_FRAME_ID}"
    fi
  fi
fi

if [[ -z "${BASE_FRAME}" ]]; then
  BASE_FRAME="${ROBOT_FRAME_ID}"
fi
if [[ -n "${POINTS_FRAME_ID}" ]]; then
  POINTS_FRAME_ID="$(sanitize_frame_id "${POINTS_FRAME_ID}")"
fi
if [[ -z "${POINTS_FRAME_ID}" ]]; then
  POINTS_FRAME_ID="$(sanitize_frame_id "${LIDAR_FRAME}")"
  if [[ -z "${POINTS_FRAME_ID}" ]] && is_ntu_viral_tnp01_bag "${BAG_PATH}"; then
    POINTS_FRAME_ID="sensor1/os_sensor"
  elif [[ -z "${POINTS_FRAME_ID}" && "${POINTS_TOPIC}" == *"livox"* ]]; then
    POINTS_FRAME_ID="livox_frame"
  fi
fi

if [[ "${PUBLISH_STATIC_TF}" == "true" && -z "${POINTS_FRAME_ID}" ]]; then
  echo "warn: points frame id could not be resolved; static TF may still be incorrect" >&2
fi
if [[ "${PUBLISH_STATIC_TF}" == "true" && "${BASE_FRAME}" == "${LIDAR_FRAME}" ]]; then
  echo "warn: base frame and lidar frame are identical (${BASE_FRAME}); disabling static TF to avoid invalid publisher configuration" >&2
  PUBLISH_STATIC_TF="false"
fi

echo "bag:          ${BAG_PATH}"
echo "points topic: ${POINTS_TOPIC}"
echo "imu topic:    ${IMU_TOPIC} (detected=${HAS_IMU})"
echo "param:        ${PARAM_FILE}"
echo "save_dir:     ${SAVE_DIR}"
echo "rviz:         ${USE_RVIZ}"
echo "graph_slam:   ${USE_GRAPH_BASED_SLAM}"
echo "sim time:     ${USE_SIM_TIME}"
echo "rate:         ${RATE}"
echo "loop:         ${LOOP}"
echo "global frame: ${GLOBAL_FRAME_ID}"
echo "odom frame:   ${ODOM_FRAME_ID}"
echo "robot frame:  ${ROBOT_FRAME_ID}"
echo "points frame: ${POINTS_FRAME_ID}"
echo "static tf:    ${PUBLISH_STATIC_TF} (${BASE_FRAME} -> ${LIDAR_FRAME})"
if [[ -n "${TUM_OUT}" ]]; then
  echo "tum out:      ${TUM_OUT}"
  echo "tum tf:       ${TUM_PARENT_FRAME:-${GLOBAL_FRAME_ID}} -> ${TUM_CHILD_FRAME:-${ROBOT_FRAME_ID}} @ ${TUM_RATE} Hz"
fi

static_tf_args=()
if [[ "${PUBLISH_STATIC_TF}" == "true" ]]; then
  IFS=' ' read -r -a st <<<"${STATIC_TF}"
  [[ "${#st[@]}" -eq 7 ]] || die "--static-tf expects 7 values: \"x y z qx qy qz qw\""
  static_tf_args+=(
    "publish_static_tf:=true"
    "base_frame:=${BASE_FRAME}"
    "lidar_frame:=${LIDAR_FRAME}"
    "static_tf_x:=${st[0]}"
    "static_tf_y:=${st[1]}"
    "static_tf_z:=${st[2]}"
    "static_tf_qx:=${st[3]}"
    "static_tf_qy:=${st[4]}"
    "static_tf_qz:=${st[5]}"
    "static_tf_qw:=${st[6]}"
  )
else
  static_tf_args+=("publish_static_tf:=false")
fi

LAUNCH_PID=""
LAUNCH_PGID=""
LOGGER_PID=""
cleanup() {
  if [[ -n "${LOGGER_PID}" ]]; then
    kill "${LOGGER_PID}" 2>/dev/null || true
    wait "${LOGGER_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PGID}" ]]; then
    kill -- "-${LAUNCH_PGID}" 2>/dev/null || true
    if [[ -n "${LAUNCH_PID}" ]]; then
      wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
  elif [[ -n "${LAUNCH_PID}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "starting lidarslam..."
if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch lidarslam lidarslam.launch.py \
    "main_param_dir:=${PARAM_FILE}" \
    "input_cloud:=${POINTS_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "global_frame_id:=${GLOBAL_FRAME_ID}" \
    "odom_frame_id:=${ODOM_FRAME_ID}" \
    "robot_frame_id:=${ROBOT_FRAME_ID}" \
    "use_rviz:=${USE_RVIZ}" \
    "use_graph_based_slam:=${USE_GRAPH_BASED_SLAM}" \
    "use_sim_time:=${USE_SIM_TIME}" \
    "save_dir:=${SAVE_DIR}" \
    "${static_tf_args[@]}" \
	    &
  LAUNCH_PID="$!"
  LAUNCH_PGID="${LAUNCH_PID}"
else
  ros2 launch lidarslam lidarslam.launch.py \
    "main_param_dir:=${PARAM_FILE}" \
    "input_cloud:=${POINTS_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "global_frame_id:=${GLOBAL_FRAME_ID}" \
    "odom_frame_id:=${ODOM_FRAME_ID}" \
    "robot_frame_id:=${ROBOT_FRAME_ID}" \
    "use_rviz:=${USE_RVIZ}" \
    "use_graph_based_slam:=${USE_GRAPH_BASED_SLAM}" \
    "use_sim_time:=${USE_SIM_TIME}" \
    "save_dir:=${SAVE_DIR}" \
	    "${static_tf_args[@]}" &
  LAUNCH_PID="$!"
fi

if [[ -n "${TUM_OUT}" ]]; then
  command -v python3 >/dev/null 2>&1 || die "python3 not found in PATH (required for --tum-out)"
  [[ -f "${REPO_ROOT}/scripts/tf_to_tum.py" ]] || die "missing script: ${REPO_ROOT}/scripts/tf_to_tum.py"

  tum_parent="${TUM_PARENT_FRAME:-${GLOBAL_FRAME_ID}}"
  tum_child="${TUM_CHILD_FRAME:-${ROBOT_FRAME_ID}}"
  echo "logging TF trajectory to TUM: ${tum_parent} -> ${tum_child}"
  tum_log="${SAVE_DIR}/tf_to_tum.log"
  echo "tf_to_tum log: ${tum_log}"
  python3 "${REPO_ROOT}/scripts/tf_to_tum.py" \
    --parent-frame "${tum_parent}" \
    --child-frame "${tum_child}" \
    --output "${TUM_OUT}" \
    --rate "${TUM_RATE}" \
    --use-sim-time "${USE_SIM_TIME}" \
    >"${tum_log}" 2>&1 &
  LOGGER_PID="$!"
fi

sleep 1

play_args=(--rate "${RATE}")
if [[ "${LOOP}" == "true" ]]; then
  play_args+=(--loop)
fi
if [[ "${USE_SIM_TIME}" == "true" ]]; then
  play_args+=(--clock)
fi

echo "playing bag..."
ros2 bag play "${BAG_PATH}" "${play_args[@]}"

# Wait for scanmatcher to drain its queue after bag playback finishes
echo "bag playback finished, waiting for scan processing to drain..."
sleep 30
