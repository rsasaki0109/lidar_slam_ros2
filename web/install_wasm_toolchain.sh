#!/usr/bin/env bash
# Installs Emscripten and gathers the header-only dependencies of web/build_wasm.sh into one
# include directory (Ubuntu 24.04, as on the GitHub runners):
#   web/install_wasm_toolchain.sh INCLUDE_DIR
set -euo pipefail
INCLUDE_DIR="${1:?usage: install_wasm_toolchain.sh INCLUDE_DIR}"
SUDO=""
[[ "$(id -u)" -eq 0 ]] || SUDO="sudo"
${SUDO} apt-get update -q
${SUDO} env DEBIAN_FRONTEND=noninteractive apt-get install -y -q --no-install-recommends \
  emscripten libeigen3-dev robin-map-dev nlohmann-json3-dev git ca-certificates
# Emscripten writes its system-library cache next to the package.
${SUDO} chown -R "$(id -u):$(id -g)" /usr/share/emscripten/cache
mkdir -p "${INCLUDE_DIR}"
SOPHUS_DIR="${INCLUDE_DIR}/.sophus-1.22.10"
[[ -d "${SOPHUS_DIR}" ]] || git clone -q --depth 1 -b 1.22.10 https://github.com/strasdat/Sophus.git "${SOPHUS_DIR}"
ln -sfn /usr/include/eigen3/Eigen "${INCLUDE_DIR}/Eigen"
ln -sfn "${SOPHUS_DIR}/sophus" "${INCLUDE_DIR}/sophus"
ln -sfn /usr/include/tsl "${INCLUDE_DIR}/tsl"
ln -sfn /usr/include/nlohmann "${INCLUDE_DIR}/nlohmann"
em++ --version | head -1
