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
  bash scripts/run_registration_plugin_consumer_check.sh [options]

Options:
  --work-dir <dir>       Use this temporary workspace root.
  --keep-work-dir        Keep an automatically-created workspace root.
  --help                 Show this help.

The check starts from /opt/ros/$ROS_DISTRO, builds the public interface and
loader in an isolated underlay, copies the fake external plugin into a second
overlay, and runs the external Identity discovery/lifetime test.
EOF
}

fail() {
  echo "error: $*" >&2
  exit 1
}

WORK_DIR=""
KEEP_WORK_DIR=false
while [[ $# -gt 0 ]]; do
  case "$1" in
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

# Do not source this repository's install space.  Clearing inherited overlay
# variables also makes an invocation from an already-sourced shell independent
# of the current workspace.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
  PYTHONPATH ROS_PACKAGE_PATH
set +u
source "${ROS_SETUP}"
set -u

if [[ -z "${WORK_DIR}" ]]; then
  WORK_DIR=$(mktemp -d "/tmp/lidarslam-registration-consumer.XXXXXX")
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
    echo "consumer proof work directory: ${WORK_DIR}" >&2
  else
    rm -rf -- "${WORK_DIR}"
  fi
  exit "${status}"
}
trap cleanup EXIT

run_logged() {
  local label="$1"
  shift
  local log_file="${WORK_DIR}/${label}.log"
  echo "running ${label}"
  if ! "$@" >"${log_file}" 2>&1; then
    echo "${label} failed; log: ${log_file}" >&2
    tail -n 120 "${log_file}" >&2 || true
    return 1
  fi
}

BASE_ROOT="${WORK_DIR}/base"
EXTERNAL_ROOT="${WORK_DIR}/external"
mkdir -p "${BASE_ROOT}/src" "${EXTERNAL_ROOT}/src"

cp -a "${REPO_ROOT}/lidarslam_plugin_interfaces" "${BASE_ROOT}/src/"
cp -a "${REPO_ROOT}/lidarslam_registration_loader" "${BASE_ROOT}/src/"
cp -a "${REPO_ROOT}/lidarslam_fake_registration_plugins" "${EXTERNAL_ROOT}/src/"

echo "clean consumer source: ${ROS_SETUP}"
echo "repository install sourced: false"

run_logged base_build colcon \
  --log-base "${WORK_DIR}/colcon-log/base" \
  build \
  --base-paths "${BASE_ROOT}/src" \
  --build-base "${BASE_ROOT}/build" \
  --install-base "${BASE_ROOT}/install" \
  --packages-select lidarslam_plugin_interfaces lidarslam_registration_loader \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

BASE_SETUP="${BASE_ROOT}/install/setup.bash"
[[ -f "${BASE_SETUP}" ]] || fail "base install setup was not created"
set +u
source "${BASE_SETUP}"
set -u

run_logged external_build colcon \
  --log-base "${WORK_DIR}/colcon-log/external" \
  build \
  --base-paths "${EXTERNAL_ROOT}/src" \
  --build-base "${EXTERNAL_ROOT}/build" \
  --install-base "${EXTERNAL_ROOT}/install" \
  --packages-select lidarslam_fake_registration_plugins \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14

EXTERNAL_SETUP="${EXTERNAL_ROOT}/install/setup.bash"
[[ -f "${EXTERNAL_SETUP}" ]] || fail "external install setup was not created"
set +u
source "${EXTERNAL_SETUP}"
set -u

LOADER_TEST="${BASE_ROOT}/build/lidarslam_registration_loader/test_registration_plugin_loader"
[[ -x "${LOADER_TEST}" ]] || fail "loader test was not built: ${LOADER_TEST}"

run_logged external_identity_test \
  "${LOADER_TEST}" \
  --gtest_filter=RegistrationPluginLoader.DiscoversInstalledFakeExternalClasses:RegistrationPluginLoader.LoadsExternalPluginAndKeepsLoaderAlive

cat > "${WORK_DIR}/receipt.txt" <<EOF
clean_external_consumer_proof=pass
ros_setup=${ROS_SETUP}
base_packages=lidarslam_plugin_interfaces,lidarslam_registration_loader
external_package=lidarslam_fake_registration_plugins
external_cxx_standard=14
gtest_filter=RegistrationPluginLoader.DiscoversInstalledFakeExternalClasses:RegistrationPluginLoader.LoadsExternalPluginAndKeepsLoaderAlive
repository_install_sourced=false
EOF

cat "${WORK_DIR}/receipt.txt"
echo "clean external registration consumer proof passed"
