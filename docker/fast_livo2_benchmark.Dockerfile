# Reproducible FAST-LIVO2 ROS 1 benchmark image.
#
# The historical benchmark image was layered on an untracked local
# ``hdl_localization_noetic:local`` image.  This recipe owns the complete
# dependency closure needed by the pinned FAST-LIVO2 checkout instead.
FROM docker.io/library/ros:noetic-ros-base@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

ARG FAST_LIVO2_REVISION=0d2c0346107b75b59934975adec9a6eeeb913c64
ARG RPG_VIKIT_REVISION=6c886c8e5d83997806e00294826d528cea3581dd
# FAST-LIVO2's upstream README and the historical image use this Sophus pin.
# It is intentionally kept as a named build argument until the upstream
# project publishes a durable full-length ref for this legacy release.
ARG SOPHUS_REVISION=a621ff2e56c56c839a6c40418d42c3c254424b5c
ARG BENCHMARK_CPU_THREADS=8

ENV DEBIAN_FRONTEND=noninteractive \
    BENCHMARK_CPU_ONLY=1 \
    OMP_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    OPENBLAS_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    MKL_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    TBB_NUM_THREADS=${BENCHMARK_CPU_THREADS}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
 && apt-get install --no-install-recommends -y \
      build-essential ca-certificates cmake git ninja-build pkg-config \
      python3-catkin-tools python3-pip time \
      libboost-all-dev libeigen3-dev libgoogle-glog-dev libomp-dev \
      libsuitesparse-dev libopencv-dev libpcl-dev \
      ros-noetic-cmake-modules \
      ros-noetic-cv-bridge ros-noetic-eigen-conversions \
      ros-noetic-image-transport ros-noetic-message-generation \
      ros-noetic-nav-msgs ros-noetic-pcl-conversions ros-noetic-pcl-ros \
      ros-noetic-rosbag ros-noetic-roscpp ros-noetic-roslaunch \
      ros-noetic-rospy ros-noetic-sensor-msgs ros-noetic-std-msgs \
      ros-noetic-tf ros-noetic-visualization-msgs \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/fast_livo_ws/src \
 && git clone --filter=blob:none https://github.com/hku-mars/FAST-LIVO2.git \
      /opt/fast_livo_ws/src/FAST-LIVO2 \
 && git -C /opt/fast_livo_ws/src/FAST-LIVO2 checkout "${FAST_LIVO2_REVISION}" \
 && git clone --filter=blob:none https://github.com/xuankuzcr/rpg_vikit.git \
      /opt/fast_livo_ws/src/rpg_vikit \
 && git -C /opt/fast_livo_ws/src/rpg_vikit checkout "${RPG_VIKIT_REVISION}"

# Build the legacy Sophus release required by FAST-LIVO2.  The two source
# edits are the same compatibility edits used by the historical image.
RUN git clone --filter=blob:none https://github.com/strasdat/Sophus.git /tmp/Sophus \
 && git -C /tmp/Sophus checkout "${SOPHUS_REVISION}" \
 && sed -i \
      -e 's/unit_complex_\.real() = 1\.;/unit_complex_.real(1.);/' \
      -e 's/unit_complex_\.imag() = 0\.;/unit_complex_.imag(0.);/' \
      /tmp/Sophus/sophus/so2.cpp \
 && cmake -S /tmp/Sophus -B /tmp/Sophus/build -GNinja \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
 && cmake --build /tmp/Sophus/build --parallel 2 \
 && cmake --install /tmp/Sophus/build \
 && rm -rf /tmp/Sophus \
 && install -d /usr/local/lib/cmake/Sophus \
 && printf '%s\n' \
      'set(Sophus_FOUND TRUE)' \
      'set(Sophus_INCLUDE_DIRS /usr/local/include)' \
      'set(Sophus_LIBRARIES /usr/local/lib/libSophus.so)' \
      > /usr/local/lib/cmake/Sophus/SophusConfig.cmake

# The upstream CMakeLists enables -march=native/-ffast-math on the host.  A
# benchmark image must not silently encode host-specific CPU instructions, so
# retain Release optimization while replacing those flags with a portable
# deterministic floating-point contract.
RUN sed -i \
      -e 's/-march=native -mtune=native -funroll-loops/-ffp-contract=off/g' \
      -e 's/-ffast-math/-ffp-contract=off/g' \
      /opt/fast_livo_ws/src/FAST-LIVO2/CMakeLists.txt \
 && source /opt/ros/noetic/setup.bash \
 && cd /opt/fast_livo_ws \
 && catkin_make -DCMAKE_BUILD_TYPE=Release -j2

RUN printf '%s\n' \
      '#!/usr/bin/env bash' \
      'set -euo pipefail' \
      'source /opt/ros/noetic/setup.bash' \
      'source /opt/fast_livo_ws/devel/setup.bash' \
      'exec "$@"' \
      > /ros_entrypoint.sh \
 && chmod +x /ros_entrypoint.sh

LABEL org.opencontainers.image.source="https://github.com/hku-mars/FAST-LIVO2" \
      benchmark.fast_livo2.revision="0d2c0346107b75b59934975adec9a6eeeb913c64" \
      benchmark.rpg_vikit.revision="6c886c8e5d83997806e00294826d528cea3581dd" \
      benchmark.sophus.revision="a621ff2e56c56c839a6c40418d42c3c254424b5c" \
      benchmark.base_image="docker.io/library/ros:noetic-ros-base" \
      benchmark.base_digest="sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d" \
      benchmark.cpu_policy="cpu_only;threads=8;native_flags=disabled"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
