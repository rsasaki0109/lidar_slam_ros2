#!/usr/bin/env bash
# Builds the browser demo's WebAssembly module: RKO-LIO odometry (Thirdparty/rko_lio core and
# driver) and web/slam_web.cpp, single-threaded, with web/tbb_shim standing in for oneTBB.
#
#   web/build_wasm.sh --include-dir DIR --out-dir DIR [--target web|node]
#
# DIR (--include-dir) holds the header-only dependencies: Eigen/, sophus/, tsl/ (robin-map)
# and nlohmann/. --target web (default) writes slam_web.js + slam_web.wasm for the page;
# --target node writes a single slam_web_node.js for web/replay_node.js.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INCLUDE_DIR=""
OUT_DIR=""
TARGET="web"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --include-dir) INCLUDE_DIR="$(cd "$2" && pwd)"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --target) TARGET="$2"; shift 2 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "${INCLUDE_DIR}" && -n "${OUT_DIR}" ]] || { echo "need --include-dir and --out-dir" >&2; exit 2; }
command -v em++ >/dev/null || { echo "em++ (Emscripten) not found" >&2; exit 2; }
mkdir -p "${OUT_DIR}"

RKO="${REPO_ROOT}/Thirdparty/rko_lio"
SOURCES=(
  "${RKO}/rko_lio/core/lio.cpp"
  "${RKO}/rko_lio/core/bump_image_map.cpp"
  "${RKO}/rko_lio/core/direct_visual_odometry.cpp"
  "${RKO}/rko_lio/core/photometric_features.cpp"
  "${RKO}/rko_lio/core/photometric_image.cpp"
  "${RKO}/rko_lio/core/voxel_hash_map.cpp"
  "${RKO}/rko_lio/core/voxel_down_sample.cpp"
  "${RKO}/rko_lio/core/process_timestamps.cpp"
  "${RKO}/rko_lio/core/preprocess_scan.cpp"
  "${RKO}/rko_lio/driver/driver_config.cpp"
  "${RKO}/rko_lio/driver/offline_driver.cpp"
  "${RKO}/rko_lio/driver/point_cloud.cpp"
  "${REPO_ROOT}/web/slam_web.cpp"
)
CXXFLAGS=(-std=c++20 -O3 -msimd128 -msse -DNDEBUG -DSOPHUS_USE_BASIC_LOGGING -DSLAM_WEB_NO_MAIN
          -I"${REPO_ROOT}/web/tbb_shim" -I"${RKO}" -I"${INCLUDE_DIR}")
EXPORTS="['_slam_create','_slam_add_imu','_slam_add_scan','_slam_output_count','_slam_output_pose',\
'_slam_output_point_count','_slam_output_points','_slam_clear_outputs','_malloc','_free']"
LDFLAGS=(-O3 -msimd128 -s ALLOW_MEMORY_GROWTH=1 -s MODULARIZE=1 -s EXPORT_NAME=createSlamModule
         -s "EXPORTED_FUNCTIONS=${EXPORTS}"
         -s "EXPORTED_RUNTIME_METHODS=['ccall','HEAPU8','HEAPF64','HEAPF32']")

OBJ_DIR="$(mktemp -d)"
trap 'rm -rf "${OBJ_DIR}"' EXIT
pids=()
for src in "${SOURCES[@]}"; do
  em++ "${CXXFLAGS[@]}" -c "${src}" -o "${OBJ_DIR}/$(basename "${src%.cpp}").o" &
  pids+=($!)
done
for pid in "${pids[@]}"; do wait "${pid}"; done

case "${TARGET}" in
  web) em++ "${OBJ_DIR}"/*.o "${LDFLAGS[@]}" -s ENVIRONMENT=web,worker -o "${OUT_DIR}/slam_web.js" ;;
  node) em++ "${OBJ_DIR}"/*.o "${LDFLAGS[@]}" -s ENVIRONMENT=node -s SINGLE_FILE=1 \
          -o "${OUT_DIR}/slam_web_node.js" ;;
  *) echo "unknown --target ${TARGET}" >&2; exit 2 ;;
esac
ls -l "${OUT_DIR}"/slam_web*
