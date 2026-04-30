#!/usr/bin/env bash
set -uo pipefail

# ============================================================
# AWSIM Self-Made Map Demo
# ============================================================
# lidarslam で生成した自作マップで Autoware 自動運転デモ
#
# 前提:
#   - Docker + NVIDIA Container Toolkit
#   - ghcr.io/autowarefoundation/autoware:universe-cuda
#   - ~/autoware_data (ML artifacts)
#   - ~/cyclonedds.xml
#
# 使い方:
#   bash scripts/run_awsim_selfmade_map_demo.sh
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

AWSIM_DIR="${AWSIM_DIR:-/workspace/ai_coding_ws/awsim}"
AWSIM_BIN="${AWSIM_BIN:-${AWSIM_DIR}/awsim_labs_v1.6.1/awsim_labs.x86_64}"
AWSIM_CONFIG="${AWSIM_CONFIG:-${AWSIM_DIR}/awsim_labs_v1.6.1/awsim_labs_Data/StreamingAssets/config.json}"
MY_MAP="${MY_MAP:-${REPO_ROOT}/output/awsim_shinjuku_slam/autoware_map}"
NDT_OVERRIDE="${NDT_OVERRIDE:-/tmp/autoware_config_override/ndt_scan_matcher.param.yaml}"
AUTOWARE_IMAGE="${AUTOWARE_IMAGE:-ghcr.io/autowarefoundation/autoware:universe-cuda}"

RECORD_VIDEO="${1:-false}"
VIDEO_OUT="${VIDEO_OUT:-${REPO_ROOT}/output/awsim_shinjuku_slam/demo.mp4}"
RECORD_DURATION="${RECORD_DURATION:-180}"

die() { echo "ERROR: $*" >&2; exit 1; }

# --- Pre-flight checks ---
[ -x "${AWSIM_BIN}" ] || die "AWSIM not found: ${AWSIM_BIN}"
[ -f "${AWSIM_CONFIG}" ] || die "AWSIM config not found: ${AWSIM_CONFIG}"
[ -d "${MY_MAP}" ] || die "Map not found: ${MY_MAP}"
[ -f "${MY_MAP}/pointcloud_map.pcd" ] || die "PCD not found"
[ -f "${MY_MAP}/lanelet2_map.osm" ] || die "Lanelet2 not found"
[ -f "${HOME}/cyclonedds.xml" ] || die "CycloneDDS config not found"
[ -f "${NDT_OVERRIDE}" ] || die "NDT override not found: ${NDT_OVERRIDE}"
docker image inspect "${AUTOWARE_IMAGE}" >/dev/null 2>&1 || die "Autoware image not pulled"

echo "=== Self-Made Map Autonomous Driving Demo ==="
echo "AWSIM: ${AWSIM_DIR}"
echo "Map: ${MY_MAP}"
echo "NDT override: ${NDT_OVERRIDE}"
echo ""

# --- Prevent screen lock ---
keep_awake() {
    while true; do
        xdotool mousemove_relative --sync 1 0 2>/dev/null || true
        xdotool mousemove_relative --sync -- -1 0 2>/dev/null || true
        sleep 30
    done
}
keep_awake &
KEEPAWAKE_PID=$!

# --- Cleanup ---
cleanup() {
    echo "Cleaning up..."
    kill ${KEEPAWAKE_PID} 2>/dev/null || true
    docker rm -f awsim_demo autoware_demo 2>/dev/null || true
    [ "${RECORD_VIDEO}" = "true" ] && kill %ffmpeg 2>/dev/null || true
}
trap cleanup EXIT

# --- Start screen recording ---
if [ "${RECORD_VIDEO}" = "true" ]; then
    echo "[1/6] Starting screen recording..."
    mkdir -p "$(dirname "${VIDEO_OUT}")"
    DISPLAY=:1 ffmpeg -y -f x11grab -framerate 10 -video_size 1920x1080 \
        -i :1+1920,0 -t "${RECORD_DURATION}" \
        -c:v libx264 -preset ultrafast -crf 25 -pix_fmt yuv420p \
        "${VIDEO_OUT}" </dev/null >/dev/null 2>&1 &
    FFMPEG_PID=$!
    sleep 2
    echo "  Recording right display to ${VIDEO_OUT} (PID: ${FFMPEG_PID})"
else
    echo "[1/6] Screen recording: skipped (pass 'true' to enable)"
fi

# --- Start AWSIM ---
echo "[2/6] Starting AWSIM..."
xhost +local:docker 2>/dev/null || true
docker rm -f awsim_demo 2>/dev/null || true

docker run -d --name awsim_demo \
    --net=host --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY="${DISPLAY}" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=/cyclonedds.xml \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${HOME}/cyclonedds.xml:/cyclonedds.xml:ro" \
    -v "${AWSIM_DIR}/awsim_labs_v1.6.1:/awsim:rw" \
    "${AUTOWARE_IMAGE}" \
    bash -c "export LD_LIBRARY_PATH=/awsim/awsim_labs_Data/Plugins:\${LD_LIBRARY_PATH} && \
        /awsim/awsim_labs.x86_64 -force-vulkan --config /awsim/awsim_labs_Data/StreamingAssets/config.json"

echo "  Waiting for AWSIM to initialize (30s)..."
sleep 30
docker ps --filter name=awsim_demo --format "  AWSIM: {{.Status}}"

# Move AWSIM window to right display
echo "  Moving AWSIM window to right display..."
sleep 5
for wid in $(xdotool search --name "AWSIM Labs" 2>/dev/null); do
    xdotool windowmove "${wid}" 1920 0 || true
    xdotool windowsize "${wid}" 1920 1080 || true
    xdotool windowactivate "${wid}" || true
done

# --- Start Autoware ---
echo "[3/6] Starting Autoware with self-made map..."
docker rm -f autoware_demo 2>/dev/null || true

docker run -d --name autoware_demo \
    --net=host --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY="${DISPLAY}" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=/cyclonedds.xml \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${HOME}/cyclonedds.xml:/cyclonedds.xml:ro" \
    -v "${MY_MAP}:/autoware_map:ro" \
    -v "${HOME}/autoware_data:/root/autoware_data:rw" \
    -v "${NDT_OVERRIDE}:/opt/autoware/share/autoware_launch/config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml:ro" \
    "${AUTOWARE_IMAGE}" \
    bash -c "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && \
        ros2 launch autoware_launch e2e_simulator.launch.xml \
            vehicle_model:=awsim_labs_vehicle \
            sensor_model:=awsim_labs_sensor_kit \
            map_path:=/autoware_map"

echo "  Waiting for Autoware to initialize (90s)..."
sleep 90
docker ps --filter name=autoware_demo --format "  Autoware: {{.Status}}"

# Move RViz window to right display (overlay on AWSIM or side-by-side)
echo "  Moving RViz to right display..."
sleep 5
for wid in $(xdotool search --name "rviz2" 2>/dev/null); do
    xdotool windowmove "${wid}" 1920 0 || true
    xdotool windowsize "${wid}" 1920 1080 || true
    xdotool windowactivate "${wid}" || true
done

# --- Set initial pose ---
echo "[4/6] Setting initial pose..."
docker exec autoware_demo bash -c "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && \
    ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
        header: {frame_id: \"map\"},
        pose: {
            pose: {position: {x: 81382.7, y: 49918.8, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.2924, w: 0.9563}},
            covariance: [1,0,0,0,0,0,0,1,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0.04]
        }
    }' --once" >/dev/null 2>&1
echo "  Initial pose set"
sleep 15

# --- Set goal ---
echo "[5/6] Setting goal (~200m ahead)..."
docker exec autoware_demo bash -c "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && \
    ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{
        header: {frame_id: \"map\"},
        pose: {position: {x: 81491.6, y: 50099.8, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.497836, w: 0.867271}}
    }' --once" >/dev/null 2>&1
echo "  Goal set"
sleep 10

# --- Engage ---
echo "[6/6] Engaging autonomous driving..."
docker exec autoware_demo bash -c "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && \
    ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}' --once" >/dev/null 2>&1

# Check state
sleep 5
STATE=$(docker logs autoware_demo 2>&1 | grep "AutowareState" | tail -1 || echo "unknown")
echo ""
echo "=== Result: ${STATE} ==="
echo ""
echo "Waiting 60s for demo recording..."
sleep 60

# Stop recording
if [ "${RECORD_VIDEO}" = "true" ]; then
    kill -INT ${FFMPEG_PID} 2>/dev/null || true
    wait ${FFMPEG_PID} 2>/dev/null || true
    echo ""
    echo "Video saved: ${VIDEO_OUT}"
    ls -lh "${VIDEO_OUT}"
fi

echo ""
echo "=== Demo complete ==="
echo "Containers still running. Stop with: docker rm -f awsim_demo autoware_demo"
