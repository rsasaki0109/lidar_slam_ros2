#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# AWSIM + Autoware Quick Start Demo
# ============================================================
# 使い方: 2つのターミナルで順番に実行
#
# ターミナル1: AWSIM を起動
#   bash run_awsim_autoware_demo.sh awsim
#
# ターミナル2: Autoware を起動
#   bash run_awsim_autoware_demo.sh autoware
#
# ターミナル3 (オプション): 自動運転を開始
#   bash run_awsim_autoware_demo.sh engage
# ============================================================

AWSIM_DIR="${AWSIM_DIR:-/workspace/ai_coding_ws/awsim}"
AWSIM_BIN="${AWSIM_BIN:-${AWSIM_DIR}/awsim_labs_v1.6.1/awsim_labs.x86_64}"
MAP_PATH="${AWSIM_MAP_PATH:-${AWSIM_DIR}/sample_map/nishishinjuku_autoware_map}"
AUTOWARE_IMAGE="${AUTOWARE_IMAGE:-ghcr.io/autowarefoundation/autoware:universe-cuda}"

# CycloneDDS configuration for AWSIM <-> Autoware communication
setup_dds() {
    sudo sysctl -w net.core.rmem_max=2147483647 2>/dev/null || true
    sudo sysctl -w net.ipv4.ipfrag_time=3 2>/dev/null || true
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728 2>/dev/null || true
    sudo ip link set lo multicast on 2>/dev/null || true
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="${HOME}/cyclonedds.xml"
}

case "${1:-help}" in
  awsim)
    echo "=== Starting AWSIM ==="
    echo "Binary: ${AWSIM_BIN}"
    setup_dds
    # AWSIM bundles its own ROS 2 libs (Humble).
    # Host Jazzy environment conflicts, so unset ROS variables.
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
    unset PYTHONPATH LD_LIBRARY_PATH
    "${AWSIM_BIN}" -force-vulkan
    ;;

  autoware)
    echo "=== Starting Autoware (Docker) ==="
    echo "Map: ${MAP_PATH}"
    setup_dds

    # Allow X11 forwarding for Docker
    xhost +local:docker 2>/dev/null || true

    docker run -it --rm \
      --net=host \
      --gpus all \
      -e NVIDIA_DRIVER_CAPABILITIES=all \
      -e DISPLAY="${DISPLAY}" \
      -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      -e CYCLONEDDS_URI=/cyclonedds.xml \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "${HOME}/cyclonedds.xml:/cyclonedds.xml:ro" \
      -v "${MAP_PATH}:/autoware_map:ro" \
      -v "${HOME}/autoware_data:/root/autoware_data:rw" \
      "${AUTOWARE_IMAGE}" \
      bash -c "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && \
        ros2 launch autoware_launch e2e_simulator.launch.xml \
          vehicle_model:=awsim_labs_vehicle \
          sensor_model:=awsim_labs_sensor_kit \
          map_path:=/autoware_map"
    ;;

  engage)
    echo "=== Engaging autonomous driving ==="
    setup_dds
    docker run --rm --net=host \
      -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      -e CYCLONEDDS_URI=/cyclonedds.xml \
      -v "${HOME}/cyclonedds.xml:/cyclonedds.xml:ro" \
      "${AUTOWARE_IMAGE}" \
      bash -c "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && \
        ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}' --once"
    ;;

  help|*)
    echo "Usage: bash $0 {awsim|autoware|engage}"
    echo ""
    echo "  awsim     - Launch AWSIM simulator"
    echo "  autoware  - Launch Autoware in Docker (with sample map)"
    echo "  engage    - Send engage command for autonomous driving"
    echo ""
    echo "Run 'awsim' first, then 'autoware' in another terminal."
    echo "Once both are running and Autoware shows ready, run 'engage'."
    ;;
esac
