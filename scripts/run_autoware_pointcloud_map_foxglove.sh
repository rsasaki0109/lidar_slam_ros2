#!/usr/bin/env bash
set -euo pipefail

usage() {
  local exit_code="${1:-1}"
  cat <<'EOF' >&2
Usage:
  run_autoware_pointcloud_map_foxglove.sh <autoware_map_dir> [autoware_core_dir] [work_dir] [options]

Options:
  --run-dir <dir>              Use an existing built Docker workspace run directory
  --rebuild                    Rebuild the minimal Autoware workspace before launching
  --foxglove-prefix <dir>      User-writable prefix prepared by prepare_foxglove_bridge_prefix.sh
  --port <port>                Foxglove Bridge port (default: 8765)
  --address <addr>             Foxglove Bridge bind address (default: 127.0.0.1)
  --topic-whitelist <expr>     Bridge topic whitelist (default: ['.*'])
  --auto-exit-secs <sec>       Auto-stop the bridge after N seconds
  --help                       Show this help

This launches Autoware's map loaders in Docker, waits for `/map/pointcloud_map`,
and then starts `foxglove_bridge` on the host so the map can be viewed from a
browser or Foxglove Desktop.
EOF
  exit "$exit_code"
}

fail() {
  echo "error: $1" >&2
  if [[ $# -gt 1 ]]; then
    echo "hint: $2" >&2
  fi
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "$value" || "$value" == --* ]]; then
    fail "option requires a value: ${option}" \
      "run this script with --help for valid options."
  fi
}

if [[ $# -eq 1 && ( "$1" == "--help" || "$1" == "-h" ) ]]; then
  usage 0
fi

if [[ $# -lt 1 ]]; then
  fail "autoware_map_dir is required." \
    "pass the directory that contains pointcloud_map/ and map_projector_info.yaml."
fi

if [[ "$1" == --* ]]; then
  fail "autoware_map_dir is required before options." \
    "put <autoware_map_dir> before options; run --help for details."
fi

if [[ ! -d "$1" ]]; then
  fail "Autoware map directory not found: $1" \
    "pass the bundle directory, not pointcloud_map/ or a PCD tile."
fi

MAP_DIR=$(realpath "$1")
shift

AUTOWARE_CORE_DIR=/tmp/autoware_core
WORK_DIR=/tmp/autoware_map_runtime_ws
if [[ $# -gt 0 && "$1" != --* ]]; then
  AUTOWARE_CORE_DIR=$(realpath -m "$1")
  shift
fi
if [[ $# -gt 0 && "$1" != --* ]]; then
  WORK_DIR=$(realpath -m "$1")
  shift
fi

RUN_DIR=""
REBUILD=false
FOXGLOVE_PREFIX="${FOXGLOVE_BRIDGE_PREFIX:-}"
PORT=8765
ADDRESS=127.0.0.1
TOPIC_WHITELIST="['.*']"
AUTO_EXIT_SECS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-dir)
      require_value "$1" "${2:-}"
      RUN_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    --foxglove-prefix)
      require_value "$1" "${2:-}"
      FOXGLOVE_PREFIX=$(realpath -m "$2")
      shift 2
      ;;
    --port)
      require_value "$1" "${2:-}"
      PORT="$2"
      shift 2
      ;;
    --address)
      require_value "$1" "${2:-}"
      ADDRESS="$2"
      shift 2
      ;;
    --topic-whitelist)
      require_value "$1" "${2:-}"
      TOPIC_WHITELIST="$2"
      shift 2
      ;;
    --auto-exit-secs)
      require_value "$1" "${2:-}"
      AUTO_EXIT_SECS="$2"
      shift 2
      ;;
    --help|-h)
      usage 0
      ;;
    *)
      fail "unknown option: $1" \
        "run this script with --help for valid options."
      ;;
  esac
done

if [[ ! -d "$MAP_DIR/pointcloud_map" || ! -f "$MAP_DIR/map_projector_info.yaml" ]]; then
  fail "Autoware map bundle is incomplete under $MAP_DIR" \
    "expected pointcloud_map/ and map_projector_info.yaml."
fi
if [[ ! -d "$AUTOWARE_CORE_DIR" ]]; then
  fail "autoware_core directory not found: $AUTOWARE_CORE_DIR" \
    "pass the autoware_core checkout as the second positional argument."
fi
if [[ -n "$RUN_DIR" && ! -d "$RUN_DIR" ]]; then
  fail "Autoware runtime run directory not found: $RUN_DIR" \
    "omit --run-dir to let the script build or discover one under the work directory."
fi

command -v docker >/dev/null 2>&1 || { echo "docker not found" >&2; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found on host" >&2; exit 1; }

find_latest_run_dir() {
  find "$WORK_DIR" -maxdepth 1 -mindepth 1 -type d -name 'run.*' | sort -r | while read -r dir; do
    if [[ -x "$dir/install/lib/autoware_map_loader/autoware_pointcloud_map_loader" ]]; then
      echo "$dir"
      return 0
    fi
  done
  return 1
}

if [[ -z "$RUN_DIR" || "$REBUILD" == "true" ]]; then
  bash "$(dirname "$0")/run_autoware_map_loader_smoke_docker.sh" "$MAP_DIR" "$AUTOWARE_CORE_DIR" "$WORK_DIR" >/tmp/autoware_foxglove_smoke.log
  RUN_DIR=$(find_latest_run_dir || true)
fi

if [[ -z "$RUN_DIR" ]]; then
  RUN_DIR=$(find_latest_run_dir || true)
fi
if [[ -z "$RUN_DIR" ]]; then
  echo "Could not find a built Autoware runtime run directory under $WORK_DIR" >&2
  exit 1
fi

if [[ -n "$FOXGLOVE_PREFIX" ]]; then
  if [[ ! -f "$FOXGLOVE_PREFIX/share/foxglove_bridge/local_setup.bash" ]]; then
    echo "foxglove_bridge prefix is missing local_setup.bash: $FOXGLOVE_PREFIX" >&2
    exit 1
  fi
  FOXGLOVE_SETUP="$FOXGLOVE_PREFIX/share/foxglove_bridge/local_setup.bash"
elif source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  FOXGLOVE_SETUP=""
else
  cat <<'EOF' >&2
foxglove_bridge is not available on the host.

Either install it system-wide, or prepare a local prefix:

  bash scripts/prepare_foxglove_bridge_prefix.sh
  bash scripts/run_autoware_pointcloud_map_foxglove.sh <map_dir> --foxglove-prefix /tmp/foxglove_bridge_jazzy
EOF
  exit 1
fi

DEFAULT_IMAGE=lidarslam_autoware_map_runtime:jazzy
IMAGE=${AUTOWARE_DOCKER_IMAGE:-$DEFAULT_IMAGE}
CONTAINER_NAME="autoware_pointcloud_map_foxglove_$$"
export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  if [[ -n "${AUTOWARE_DOCKER_IMAGE:-}" ]]; then
    echo "Configured AUTOWARE_DOCKER_IMAGE is not available locally: $IMAGE" >&2
    exit 1
  fi
  bash "$(dirname "$0")/build_autoware_map_runtime_image.sh" "$IMAGE"
fi

dump_container_logs() {
  echo "Docker logs from $CONTAINER_NAME:" >&2
  docker logs "$CONTAINER_NAME" >&2 || true
  docker exec "$CONTAINER_NAME" bash -lc '
set -euo pipefail
for path in /tmp/map_projection_loader.log /tmp/pointcloud_map_loader.log; do
  if [[ -f "$path" ]]; then
    echo "==> $path" >&2
    tail -n 80 "$path" >&2 || true
  fi
done
' || true
}

wait_for_container_topics() {
  docker exec "$CONTAINER_NAME" bash -lc '
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /autoware_ws/install/setup.bash
set -u
python3 - <<'"'"'PY'"'"'
import sys
import time

import rclpy
from autoware_map_msgs.msg import MapProjectorInfo
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Waiter(Node):
    def __init__(self):
        super().__init__("autoware_pointcloud_map_foxglove_waiter")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_msg = None
        self.proj_msg = None
        self.create_subscription(PointCloud2, "/map/pointcloud_map", self.on_map, qos)
        self.create_subscription(MapProjectorInfo, "/map/map_projector_info", self.on_proj, qos)

    def on_map(self, msg):
        self.map_msg = msg

    def on_proj(self, msg):
        self.proj_msg = msg


rclpy.init()
node = Waiter()
deadline = time.time() + 20.0
while time.time() < deadline and (node.map_msg is None or node.proj_msg is None):
    rclpy.spin_once(node, timeout_sec=0.5)

if node.proj_msg is None:
    print("ERROR: did not receive /map/map_projector_info", file=sys.stderr)
    sys.exit(2)
if node.map_msg is None:
    print("ERROR: did not receive /map/pointcloud_map", file=sys.stderr)
    sys.exit(3)

print(
    f"MAP_PROJECTOR_INFO projector_type={node.proj_msg.projector_type} "
    f"scale_factor={node.proj_msg.scale_factor}"
)
print(
    f"POINTCLOUD_MAP width={node.map_msg.width} height={node.map_msg.height} "
    f"frame_id={node.map_msg.header.frame_id} data_bytes={len(node.map_msg.data)}"
)

node.destroy_node()
rclpy.shutdown()
PY
'
}

wait_for_host_pointcloud() {
  timeout 20 bash -lc '
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
python3 - <<'"'"'PY'"'"'
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Waiter(Node):
    def __init__(self):
        super().__init__("autoware_pointcloud_map_host_waiter")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_msg = None
        self.create_subscription(PointCloud2, "/map/pointcloud_map", self.on_map, qos)

    def on_map(self, msg):
        self.map_msg = msg


rclpy.init()
node = Waiter()
deadline = time.time() + 15.0
while time.time() < deadline and node.map_msg is None:
    rclpy.spin_once(node, timeout_sec=0.5)

if node.map_msg is None:
    print("ERROR: host did not receive /map/pointcloud_map", file=sys.stderr)
    sys.exit(4)

print(
    f"HOST_POINTCLOUD_MAP width={node.map_msg.width} height={node.map_msg.height} "
    f"frame_id={node.map_msg.header.frame_id} data_bytes={len(node.map_msg.data)}"
)

node.destroy_node()
rclpy.shutdown()
PY
'
}

wait_for_tcp_port() {
  local host="$1"
  local port="$2"
  python3 - "$host" "$port" <<'PY'
import socket
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
deadline = time.time() + 15.0
while time.time() < deadline:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.0)
    try:
        sock.connect((host, port))
    except OSError:
        time.sleep(0.25)
    else:
        sock.close()
        print(f"FOXGLOVE_BRIDGE listening on {host}:{port}")
        sys.exit(0)
    finally:
        sock.close()
print(f"ERROR: bridge did not open {host}:{port}", file=sys.stderr)
sys.exit(5)
PY
}

cleanup() {
  local bridge_pid="${FOXGLOVE_PID:-}"
  if [[ -n "$bridge_pid" ]]; then
    kill "$bridge_pid" >/dev/null 2>&1 || true
    wait "$bridge_pid" >/dev/null 2>&1 || true
  fi
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker run -d \
  --name "$CONTAINER_NAME" \
  --net host \
  ${ROS_DOMAIN_ID:+-e ROS_DOMAIN_ID="$ROS_DOMAIN_ID"} \
  ${RMW_IMPLEMENTATION:+-e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION"} \
  -e FASTDDS_BUILTIN_TRANSPORTS="$FASTDDS_BUILTIN_TRANSPORTS" \
  -v "$MAP_DIR:/maps:ro" \
  -v "$RUN_DIR:/autoware_ws:ro" \
  "$IMAGE" bash -lc '
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /autoware_ws/install/setup.bash
set -u

cat >/tmp/map_projection_loader_params.yaml <<PARAMS
/**:
  ros__parameters:
    map_projector_info_path: "/maps/map_projector_info.yaml"
    lanelet2_map_path: ""
PARAMS

cat >/tmp/pointcloud_map_loader_params.yaml <<PARAMS
/**:
  ros__parameters:
    enable_whole_load: true
    enable_downsampled_whole_load: false
    enable_partial_load: true
    enable_selected_load: false
    leaf_size: 3.0
    pcd_paths_or_directory: ["/maps/pointcloud_map"]
    pcd_metadata_path: "/maps/pointcloud_map/pointcloud_map_metadata.yaml"
PARAMS

ros2 run autoware_map_projection_loader autoware_map_projection_loader_node \
  --ros-args \
  --params-file /tmp/map_projection_loader_params.yaml >/tmp/map_projection_loader.log 2>&1 &
P1=$!

ros2 run autoware_map_loader autoware_pointcloud_map_loader \
  --ros-args \
  --params-file /tmp/pointcloud_map_loader_params.yaml \
  --remap output/pointcloud_map:=/map/pointcloud_map \
  --remap service/get_partial_pcd_map:=/map/get_partial_pointcloud_map \
  --remap service/get_selected_pcd_map:=/map/get_selected_pointcloud_map \
  --remap service/get_differential_pcd_map:=/map/get_differential_pointcloud_map \
  >/tmp/pointcloud_map_loader.log 2>&1 &
P2=$!

trap "kill \$P1 \$P2 2>/dev/null || true; wait \$P1 \$P2 2>/dev/null || true" EXIT INT TERM
wait
'

sleep 1
if ! docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  dump_container_logs
  echo "Autoware map loader container exited before topics became ready." >&2
  exit 1
fi

echo "Waiting for Autoware map topics inside Docker ..."
if ! wait_for_container_topics; then
  dump_container_logs
  exit 1
fi

echo "Waiting for host receipt of /map/pointcloud_map ..."
if ! wait_for_host_pointcloud; then
  dump_container_logs
  echo "Host could not receive /map/pointcloud_map." >&2
  exit 1
fi

LAUNCH_CMD=(ros2 launch foxglove_bridge foxglove_bridge_launch.xml "port:=${PORT}" "address:=${ADDRESS}" "topic_whitelist:=${TOPIC_WHITELIST}")
if [[ -n "$FOXGLOVE_SETUP" ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$FOXGLOVE_SETUP"
  set -u
else
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

echo "Launching Foxglove Bridge"
echo "  websocket: ws://${ADDRESS}:${PORT}"
echo "  topic_whitelist: ${TOPIC_WHITELIST}"
"${LAUNCH_CMD[@]}" >/tmp/autoware_foxglove_bridge.log 2>&1 &
FOXGLOVE_PID=$!

if ! wait_for_tcp_port "${ADDRESS}" "${PORT}"; then
  tail -n 80 /tmp/autoware_foxglove_bridge.log >&2 || true
  exit 1
fi

echo "Open Foxglove and connect to:"
echo "  ws://${ADDRESS}:${PORT}"

if [[ -n "$AUTO_EXIT_SECS" ]]; then
  sleep "$AUTO_EXIT_SECS"
else
  wait "$FOXGLOVE_PID"
fi
