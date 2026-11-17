#!/usr/bin/env bash
# Build the C++ libraries the lidarslam wheel bundles, from Ubuntu 24.04's source packages.
#
# The wheel must map like the source install on Ubuntu 24.04 (apt's PCL, g2o, FLANN, oneTBB,
# Eigen), so this builds the same upstream versions from the same orig tarballs, configured like
# Debian builds them: -O2, no SSE/AVX/-march flags beyond the x86-64 baseline, and Debian's g2o
# no-sse patch. Only what the core uses is built: PCL common/octree/kdtree/search/
# sample_consensus/filters/features/2d/io/registration without VTK, OpenNI, pcap, PNG or Qt
# (docs/roadmap/ros-free-core-2026-11.md, M6 step 2); g2o core/stuff/slam3d/solver_eigen without
# OpenGL; Boost filesystem/iostreams/system for PCL io.
#
# Runs in the manylinux_2_28 container of cibuildwheel (before-all) and installs into
# LIDARSLAM_WHEEL_DEPS (default: .wheel-deps in the repository, which CI caches). A stamp file
# named after this script's hash skips the build when the prefix is already complete.
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
PREFIX=${LIDARSLAM_WHEEL_DEPS:-${REPO_ROOT}/.wheel-deps}
JOBS=${LIDARSLAM_WHEEL_JOBS:-$(nproc)}
STAMP="${PREFIX}/.complete-$(sha256sum "${BASH_SOURCE[0]}" | cut -c1-16)"
UBUNTU_POOL=${LIDARSLAM_UBUNTU_POOL:-http://archive.ubuntu.com/ubuntu/pool}

if [[ -f "${STAMP}" ]]; then
  echo "lidarslam wheel dependencies: using ${PREFIX}"
  exit 0
fi

if command -v dnf >/dev/null 2>&1; then
  dnf install -y zlib-devel bzip2-devel xz-devel libzstd-devel lz4-devel pkgconf-pkg-config \
    patch xz bzip2 curl
fi

WORK=$(mktemp -d)
trap 'rm -rf "${WORK}"' EXIT
# Start from an empty prefix; it can be a mount point (the cached volume), so clear its contents.
mkdir -p "${PREFIX}"
find "${PREFIX}" -mindepth 1 -delete
export CMAKE_PREFIX_PATH="${PREFIX}"
export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig:${PREFIX}/share/pkgconfig:${PKG_CONFIG_PATH:-}"
export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"

# fetch <pool path or URL> <sha256>: download one source tarball and check its hash. Pool paths
# are Ubuntu 24.04's source packages (the hashes are the Checksums-Sha256 of their .dsc files).
fetch() {
  local path=$1 sha=$2 out url
  out="${WORK}/$(basename "${path}")"
  url=${path}
  [[ "${path}" == https://* ]] || url="${UBUNTU_POOL}/${path}"
  curl -fsSL --retry 5 --retry-delay 5 -o "${out}" "${url}"
  echo "${sha}  ${out}" | sha256sum -c - >&2
  echo "${out}"
}

# unpack <tarball>: extract into its own directory and print the source root
unpack() {
  local dir
  dir="${WORK}/src-$(basename "$1")"
  mkdir -p "${dir}"
  tar -xf "$1" -C "${dir}"
  find "${dir}" -mindepth 1 -maxdepth 1 -type d | head -n 1
}

CMAKE_COMMON=(
  -DCMAKE_INSTALL_PREFIX="${PREFIX}"
  -DCMAKE_INSTALL_LIBDIR=lib
  -DCMAKE_PREFIX_PATH="${PREFIX}"
  -DCMAKE_BUILD_TYPE=None
  -DCMAKE_C_FLAGS=-O2
  -DCMAKE_CXX_FLAGS=-O2
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  -DBUILD_SHARED_LIBS=ON
)

cmake_install() {
  local src=$1
  shift
  cmake -S "${src}" -B "${src}/build" "${CMAKE_COMMON[@]}" "$@"
  cmake --build "${src}/build" -j "${JOBS}"
  cmake --install "${src}/build"
}

# Eigen 3.4.0 (eigen3 3.4.0-4build0.1), header-only.
src=$(unpack "$(fetch universe/e/eigen3/eigen3_3.4.0.orig.tar.bz2 \
  b4c198460eba6f28d34894e3a5710998818515104d6e74e5cc331ce31e46e626)")
cmake_install "${src}" -DBUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_PKGCONFIG=ON

# tsl::robin_map 1.2.1 (robin-map 1.2.1-1), header-only.
src=$(unpack "$(fetch universe/r/robin-map/robin-map_1.2.1.orig.tar.xz \
  34c720250cfc11ada269e8779991c85e260cfc865c0af3a6aab30a457a48aa75)")
cmake_install "${src}"

# oneTBB 2021.11.0 (onetbb 2021.11.0-2ubuntu2).
src=$(unpack "$(fetch universe/o/onetbb/onetbb_2021.11.0.orig.tar.xz \
  83e84fd87d6b34f0d920753c2be327a74d1f327802bdce691a844b0b0bc25db3)")
cmake_install "${src}" -DTBB_TEST=OFF -DTBB_EXAMPLES=OFF -DTBB4PY_BUILD=OFF -DTBB_STRICT=OFF

# Boost 1.83.0 (Ubuntu ships boost1.83 1.83.0): the libraries PCL io links. The upstream release
# tarball, because Debian's orig tarball lacks the generated b2 parser (jamgram.cpp) that Debian
# regenerates with bison; Boost does not touch the map's numbers.
src=$(unpack "$(fetch https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2 \
  6478edfe2f3305127cffe8caf73ea0176c53769f4bf1585be237eb30798c3b8e)")
(
  cd "${src}"
  ./bootstrap.sh --prefix="${PREFIX}" --with-libraries=filesystem,iostreams,system
  ./b2 -j "${JOBS}" -d0 variant=release link=shared threading=multi cxxflags=-O2 \
    --prefix="${PREFIX}" --libdir="${PREFIX}/lib" install
  # b2 installs only the headers of the libraries built; install all of them, as libboost-dev
  # does (3D-BBS includes boost/functional/hash).
  cp -a boost "${PREFIX}/include/"
)

# FLANN 1.9.2 (flann 1.9.2+dfsg-2build1), with OpenMP as Debian builds it.
src=$(unpack "$(fetch universe/f/flann/flann_1.9.2+dfsg.orig.tar.xz \
  9ef9e74820c0eb7136200fb7e2e9ac5caff1defe339351b4e8e7aec19bb35de8)")
cmake_install "${src}" -DBUILD_C_BINDINGS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF \
  -DBUILD_CUDA_LIB=OFF -DUSE_MPI=OFF -DUSE_OPENMP=ON

# g2o 20230806 (g2o 0~20230806-4.1build1) with Debian's no-sse patch.
src=$(unpack "$(fetch 'universe/g/g2o/g2o_0%7E20230806.orig.tar.xz' \
  52acdc696d6d8d3e2679cf99c33545c082fe266b544108665f1042d9cc77ae96)")
debian=$(fetch 'universe/g/g2o/g2o_0%7E20230806-4.1build1.debian.tar.xz' \
  8bee951833756621ac8bb99bac032f410a800cbcc9b340617624720298077ff1)
tar -xf "${debian}" -C "${src}"
patch -d "${src}" -p1 < "${src}/debian/patches/no-sse.patch"
cmake_install "${src}" -DG2O_USE_CHOLMOD=OFF -DG2O_USE_CSPARSE=OFF -DG2O_USE_OPENMP=OFF \
  -DG2O_USE_OPENGL=OFF -DG2O_USE_LOGGING=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF \
  -DBUILD_UNITTESTS=OFF -DG2O_BUILD_BENCHMARKS=OFF -DG2O_FAST_MATH=OFF \
  -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_SLAM2D_TYPES=OFF -DG2O_BUILD_SLAM2D_ADDON_TYPES=OFF \
  -DG2O_BUILD_DATA_TYPES=OFF -DG2O_BUILD_SCLAM2D_TYPES=OFF -DG2O_BUILD_SLAM3D_TYPES=ON \
  -DG2O_BUILD_SLAM3D_ADDON_TYPES=OFF -DG2O_BUILD_SBA_TYPES=OFF -DG2O_BUILD_ICP_TYPES=OFF \
  -DG2O_BUILD_SIM3_TYPES=OFF

# PCL 1.14.0 (pcl 1.14.0+dfsg-1), the modules the core uses, flags as Debian (PCL_ENABLE_SSE=OFF).
src=$(unpack "$(fetch universe/p/pcl/pcl_1.14.0+dfsg.orig.tar.xz \
  d084703d01397fa83c314b76e5e80a63ed310c5f57094365a993285bb734e7a9)")
pcl_off=()
for module in apps tools examples global_tests geometry keypoints ml outofcore people recognition \
  segmentation simulation stereo surface tracking visualization cuda gpu benchmarks; do
  pcl_off+=("-DBUILD_${module}=OFF")
done
cmake_install "${src}" -DPCL_ENABLE_SSE=OFF -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_MARCHNATIVE=OFF \
  -DWITH_VTK=OFF -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_PCAP=OFF -DWITH_PNG=OFF \
  -DWITH_QHULL=OFF -DWITH_CUDA=OFF -DWITH_LIBUSB=OFF -DWITH_QT=NO -DWITH_OPENGL=OFF \
  -DWITH_ENSENSO=OFF -DWITH_DAVIDSDK=OFF -DWITH_DSSDK=OFF -DWITH_RSSDK=OFF -DWITH_RSSDK2=OFF \
  -DBoost_NO_SYSTEM_PATHS=ON -DBOOST_ROOT="${PREFIX}" \
  -DBUILD_common=ON -DBUILD_octree=ON -DBUILD_kdtree=ON -DBUILD_search=ON \
  -DBUILD_sample_consensus=ON -DBUILD_filters=ON -DBUILD_features=ON -DBUILD_2d=ON \
  -DBUILD_io=ON -DBUILD_registration=ON "${pcl_off[@]}"

touch "${STAMP}"
echo "lidarslam wheel dependencies: built into ${PREFIX}"
