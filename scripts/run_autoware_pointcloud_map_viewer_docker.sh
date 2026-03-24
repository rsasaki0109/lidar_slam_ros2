#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_autoware_pointcloud_map_viewer_docker.sh <autoware_map_dir> [autoware_core_dir] [work_dir] [options]

Options:
  --run-dir <dir>           Use an existing built Docker workspace run directory
  --rebuild                 Rebuild the minimal Autoware workspace before launching RViz
  --auto-exit-secs <sec>    Auto-close RViz after N seconds (useful for tests)
  --help                    Show this help

This launches Autoware's map projection loader and pointcloud map loader inside
Docker, then opens the pointcloud map in the host's rviz2. The map directory must contain:
  pointcloud_map/
  map_projector_info.yaml
EOF
  exit 1
}

if [[ $# -lt 1 ]]; then
  usage
fi

MAP_DIR=$(realpath "$1")
shift

AUTOWARE_CORE_DIR=/tmp/autoware_core
WORK_DIR=/tmp/autoware_map_runtime_ws
if [[ $# -gt 0 && "$1" != --* ]]; then
  AUTOWARE_CORE_DIR=$(realpath "$1")
  shift
fi
if [[ $# -gt 0 && "$1" != --* ]]; then
  WORK_DIR=$(realpath -m "$1")
  shift
fi

RUN_DIR=""
REBUILD=false
AUTO_EXIT_SECS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-dir)
      [[ $# -ge 2 ]] || usage
      RUN_DIR=$(realpath "$2")
      shift 2
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    --auto-exit-secs)
      [[ $# -ge 2 ]] || usage
      AUTO_EXIT_SECS="$2"
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

if [[ ! -d "$MAP_DIR/pointcloud_map" || ! -f "$MAP_DIR/map_projector_info.yaml" ]]; then
  echo "Autoware map bundle is incomplete under $MAP_DIR" >&2
  exit 1
fi
if [[ ! -d "$AUTOWARE_CORE_DIR" ]]; then
  echo "autoware_core directory not found: $AUTOWARE_CORE_DIR" >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set; cannot launch RViz." >&2
  exit 1
fi
command -v docker >/dev/null 2>&1 || { echo "docker not found" >&2; exit 1; }
command -v rviz2 >/dev/null 2>&1 || { echo "rviz2 not found on host" >&2; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found on host" >&2; exit 1; }

DISPLAY_NUM=${DISPLAY#:}
DISPLAY_NUM=${DISPLAY_NUM%%.*}
if [[ "$DISPLAY" == :* && ! -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]]; then
  echo "X11 socket for DISPLAY=$DISPLAY was not found." >&2
  exit 1
fi

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
  bash "$(dirname "$0")/run_autoware_map_loader_smoke_docker.sh" "$MAP_DIR" "$AUTOWARE_CORE_DIR" "$WORK_DIR" >/tmp/autoware_viewer_smoke.log
  RUN_DIR=$(find_latest_run_dir || true)
fi

if [[ -z "$RUN_DIR" ]]; then
  RUN_DIR=$(find_latest_run_dir || true)
fi
if [[ -z "$RUN_DIR" ]]; then
  echo "Could not find a built Autoware runtime run directory under $WORK_DIR" >&2
  exit 1
fi

RVIZ_CONFIG=$(realpath "$(dirname "$0")/autoware_pointcloud_map.rviz")
DEFAULT_IMAGE=lidarslam_autoware_map_runtime:jazzy
IMAGE=${AUTOWARE_DOCKER_IMAGE:-$DEFAULT_IMAGE}
CONTAINER_NAME="autoware_pointcloud_map_viewer_$$"
export FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  if [[ -n "${AUTOWARE_DOCKER_IMAGE:-}" ]]; then
    echo "Configured AUTOWARE_DOCKER_IMAGE is not available locally: $IMAGE" >&2
    exit 1
  fi
  bash "$(dirname "$0")/build_autoware_map_runtime_image.sh" "$IMAGE"
fi

echo "Launching Autoware pointcloud map viewer"
echo "  map_dir:  $MAP_DIR"
echo "  run_dir:  $RUN_DIR"
echo "  display:  $DISPLAY"

cleanup() {
  docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

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
        super().__init__("autoware_pointcloud_map_viewer_waiter")
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

if [[ -n "$AUTO_EXIT_SECS" ]]; then
  rviz_rc=0
  timeout "${AUTO_EXIT_SECS}" rviz2 -d "$RVIZ_CONFIG" || rviz_rc=$?
  if [[ $rviz_rc -ne 0 && $rviz_rc -ne 124 ]]; then
    exit "$rviz_rc"
  fi
else
  rviz2 -d "$RVIZ_CONFIG"
fi
