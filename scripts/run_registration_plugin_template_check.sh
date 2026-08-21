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
TEMPLATE_PACKAGE=lidarslam_registration_plugin_template
TEMPLATE_CLASS="${TEMPLATE_PACKAGE}/Identity"

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_registration_plugin_template_check.sh [options]

Options:
  --work-dir <dir>       Use this temporary workspace root.
  --keep-work-dir        Keep an automatically-created workspace root.
  --help                 Show this help.

The check starts from /opt/ros/$ROS_DISTRO, builds the public interface and
shell loader in an isolated underlay, copies the author template into a second
overlay, and runs its C++14 contract and pluginlib lifetime tests.
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

# The proof must be independent of a caller's sourced workspace. The
# repository may be mounted read-only; all colcon outputs, including logs,
# live below WORK_DIR.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
  PYTHONPATH ROS_PACKAGE_PATH
set +u
source "${ROS_SETUP}"
set -u

if [[ -z "${WORK_DIR}" ]]; then
  WORK_DIR=$(mktemp -d "/tmp/lidarslam-registration-template.XXXXXX")
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
    echo "template proof work directory: ${WORK_DIR}" >&2
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
cp -a "${REPO_ROOT}/examples/${TEMPLATE_PACKAGE}" "${EXTERNAL_ROOT}/src/"

echo "template proof source: ${ROS_SETUP}"
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

run_logged template_build colcon \
  --log-base "${WORK_DIR}/colcon-log/template" \
  build \
  --base-paths "${EXTERNAL_ROOT}/src" \
  --build-base "${EXTERNAL_ROOT}/build" \
  --install-base "${EXTERNAL_ROOT}/install" \
  --packages-select "${TEMPLATE_PACKAGE}" \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DBUILD_TESTING=ON

EXTERNAL_SETUP="${EXTERNAL_ROOT}/install/setup.bash"
[[ -f "${EXTERNAL_SETUP}" ]] || fail "template install setup was not created"
set +u
source "${EXTERNAL_SETUP}"
set -u

CONTRACT_TEST="${EXTERNAL_ROOT}/build/${TEMPLATE_PACKAGE}/test_registration_plugin_template"
LOADER_TEST="${EXTERNAL_ROOT}/build/${TEMPLATE_PACKAGE}/test_registration_plugin_template_loader"
[[ -x "${CONTRACT_TEST}" ]] || fail "template contract test was not built: ${CONTRACT_TEST}"
[[ -x "${LOADER_TEST}" ]] || fail "template loader test was not built: ${LOADER_TEST}"

run_logged template_contract_test "${CONTRACT_TEST}"
run_logged template_loader_test "${LOADER_TEST}"

cat > "${WORK_DIR}/receipt.txt" <<EOF
m1_template_proof=pass
ros_setup=${ROS_SETUP}
base_packages=lidarslam_plugin_interfaces,lidarslam_registration_loader
template_package=${TEMPLATE_PACKAGE}
template_class=${TEMPLATE_CLASS}
plugin_cxx_standard=14
loader_test_cxx_standard=17
contract_tests=test_registration_plugin_template,test_registration_plugin_template_loader
metadata_class_id=${TEMPLATE_CLASS}
metadata_license=BSD-2-Clause
plugin_manifest=${EXTERNAL_ROOT}/install/${TEMPLATE_PACKAGE}/share/${TEMPLATE_PACKAGE}/registration_plugins.xml
plugin_library=${EXTERNAL_ROOT}/install/${TEMPLATE_PACKAGE}/lib/lib${TEMPLATE_PACKAGE}.so
repository_install_sourced=false
EOF

cat "${WORK_DIR}/receipt.txt"
echo "registration plugin template proof passed"
