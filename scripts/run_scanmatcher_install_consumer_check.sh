#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_scanmatcher_install_consumer_check.sh [options]

Options:
  --install-prefix <dir>  Installed workspace prefix to inspect (default: ./install).
  --work-dir <dir>        Use this temporary build workspace root.
  --keep-work-dir         Keep an automatically-created workspace root.
  --help                  Show this help.

The check starts from /opt/ros/$ROS_DISTRO and uses only the installed
scanmatcher CMake package and its declared dependencies.  It compiles a
downstream translation unit that includes scanmatcher_component.h, so a
concrete implementation header accidentally leaked from the public include
surface fails before a downstream package is built.
EOF
}

fail() {
  echo "error: $*" >&2
  exit 1
}

WORK_DIR=""
INSTALL_PREFIX="${REPO_ROOT}/install"
KEEP_WORK_DIR=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-prefix)
      [[ $# -ge 2 ]] || fail "--install-prefix requires a path"
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --work-dir)
      [[ $# -ge 2 ]] || fail "--work-dir requires a path"
      WORK_DIR="$2"
      shift 2
      ;;
    --keep-work-dir)
      KEEP_WORK_DIR=true
      shift
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

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${ROS_SETUP}" ]] || fail "ROS setup not found: ${ROS_SETUP}"

INSTALL_PREFIX=$(cd -- "${INSTALL_PREFIX}" 2>/dev/null && pwd) || \
  fail "install prefix does not exist: ${INSTALL_PREFIX}"
SCANMATCHER_CONFIG="${INSTALL_PREFIX}/scanmatcher/share/scanmatcher/cmake/scanmatcherConfig.cmake"
SCANMATCHER_HEADER="${INSTALL_PREFIX}/scanmatcher/include/scanmatcher/scanmatcher_component.h"
[[ -f "${SCANMATCHER_CONFIG}" ]] || fail "scanmatcher config not found: ${SCANMATCHER_CONFIG}"
[[ -f "${SCANMATCHER_HEADER}" ]] || fail "scanmatcher header not found: ${SCANMATCHER_HEADER}"

# This is the regression being guarded: a downstream package must not need the
# concrete default-plugin implementation package merely to include scanmatcher.
if grep -Fq 'lidarslam_default_plugins/ndt_omp_registration.hpp' "${SCANMATCHER_HEADER}"; then
  fail "public scanmatcher header still includes the concrete NDT implementation"
fi

# Do not inherit a sourced overlay.  CMake is given the two prefixes directly,
# and the repository install is deliberately not sourced.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
  PYTHONPATH ROS_PACKAGE_PATH
set +u
source "${ROS_SETUP}"
set -u

if [[ -z "${WORK_DIR}" ]]; then
  WORK_DIR=$(mktemp -d "/tmp/lidarslam-scanmatcher-consumer.XXXXXX")
  WORK_DIR_OWNED=true
else
  mkdir -p -- "${WORK_DIR}"
  WORK_DIR=$(cd -- "${WORK_DIR}" && pwd)
  WORK_DIR_OWNED=false
fi

case "${WORK_DIR}" in
  "${REPO_ROOT}"|"${REPO_ROOT}"/*)
    fail "--work-dir must be outside the repository"
    ;;
esac

cleanup() {
  local status=$?
  if [[ "${KEEP_WORK_DIR}" == true || "${WORK_DIR_OWNED}" == false ]]; then
    echo "scanmatcher installed consumer work directory: ${WORK_DIR}" >&2
  else
    rm -rf -- "${WORK_DIR}"
  fi
  exit "${status}"
}
trap cleanup EXIT

mkdir -p "${WORK_DIR}/src" "${WORK_DIR}/build"
cat > "${WORK_DIR}/src/consumer.cpp" <<'EOF'
#include <memory>

#include <scanmatcher/scanmatcher_component.h>

int main()
{
  std::shared_ptr<graphslam::ScanMatcherComponent> component;
  return component ? 1 : 0;
}
EOF

cat > "${WORK_DIR}/src/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.16)
project(scanmatcher_installed_consumer LANGUAGES C CXX)

# PCL's optional VTK config references MPI::MPI_C when VTK is installed with
# MPI support.  Discovering the system target here avoids an unrelated CMake
# package-config failure while keeping the consumer's declared scanmatcher
# dependency path unchanged.
find_package(MPI QUIET COMPONENTS C)
find_package(scanmatcher REQUIRED)
find_package(ament_cmake REQUIRED)
if(NOT scanmatcher_INCLUDE_DIRS)
  message(FATAL_ERROR "scanmatcher did not export include directories")
endif()

add_executable(scanmatcher_header_consumer consumer.cpp)
target_compile_features(scanmatcher_header_consumer PRIVATE cxx_std_17)
ament_target_dependencies(scanmatcher_header_consumer scanmatcher)
EOF

PREFIX_PATH="${INSTALL_PREFIX};/opt/ros/${ROS_DISTRO}"
cmake -S "${WORK_DIR}/src" -B "${WORK_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${PREFIX_PATH}" \
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE \
  -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=FALSE \
  >"${WORK_DIR}/configure.log" 2>&1 || {
    tail -n 160 "${WORK_DIR}/configure.log" >&2 || true
    fail "installed scanmatcher consumer configure failed"
  }
cmake --build "${WORK_DIR}/build" --parallel 2 \
  >"${WORK_DIR}/build.log" 2>&1 || {
    tail -n 160 "${WORK_DIR}/build.log" >&2 || true
    fail "installed scanmatcher consumer compilation failed"
  }

HEADER_SHA256=$(sha256sum "${SCANMATCHER_HEADER}" | awk '{print $1}')
cat > "${WORK_DIR}/receipt.txt" <<EOF
installed_scanmatcher_consumer_proof=pass
ros_setup=${ROS_SETUP}
install_prefix=${INSTALL_PREFIX}
scanmatcher_config=${SCANMATCHER_CONFIG}
scanmatcher_header=${SCANMATCHER_HEADER}
scanmatcher_header_sha256=${HEADER_SHA256}
public_default_plugin_header_include=absent
repository_install_sourced=false
consumer_cxx_standard=17
EOF

cat "${WORK_DIR}/receipt.txt"
echo "installed scanmatcher consumer proof passed"
