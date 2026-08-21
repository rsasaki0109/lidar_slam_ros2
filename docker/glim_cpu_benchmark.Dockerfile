FROM docker.io/library/ros:jazzy-ros-base@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

ARG GTSAM_REVISION=2f3e56c0ddbd3a1aa54ed043643b553d26a069f6
ARG GTSAM_POINTS_REVISION=9d32e7dbecf6015560d84b4901d6b0a6f483ec46
ARG GLIM_REVISION=faa264a1bce1bda406f73457e35511f56cdc2eaa
ARG GLIM_ROS2_REVISION=4a9e7a4cb084967c8525a1be529ad3ba2a118ae7

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential ca-certificates cmake git libboost-all-dev libeigen3-dev \
    libfmt-dev libmetis-dev libnanoflann-dev libomp-dev libopencv-dev \
    libspdlog-dev ninja-build python3-colcon-common-extensions time \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-rosbag2 \
    ros-jazzy-tf2-ros \
  && rm -rf /var/lib/apt/lists/*

RUN git clone --filter=blob:none https://github.com/borglab/gtsam.git /src/gtsam \
  && git -C /src/gtsam checkout "${GTSAM_REVISION}" \
  && cmake -S /src/gtsam -B /src/gtsam/build -GNinja \
       -DCMAKE_BUILD_TYPE=Release \
       -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
       -DGTSAM_BUILD_TESTS=OFF \
       -DGTSAM_WITH_TBB=OFF \
       -DGTSAM_USE_SYSTEM_EIGEN=ON \
       -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  && cmake --build /src/gtsam/build --parallel 2 \
  && cmake --install /src/gtsam/build

RUN git clone --filter=blob:none https://github.com/koide3/gtsam_points.git /src/gtsam_points \
  && git -C /src/gtsam_points checkout "${GTSAM_POINTS_REVISION}" \
  && cmake -S /src/gtsam_points -B /src/gtsam_points/build -GNinja \
       -DCMAKE_BUILD_TYPE=Release \
       -DBUILD_TESTS=OFF \
       -DBUILD_WITH_CUDA=OFF \
       -DBUILD_WITH_MARCH_NATIVE=OFF \
  && cmake --build /src/gtsam_points/build --parallel 2 \
  && cmake --install /src/gtsam_points/build \
  && ldconfig

RUN git clone --filter=blob:none https://github.com/koide3/glim.git /opt/glim_ws/src/glim \
  && git -C /opt/glim_ws/src/glim checkout "${GLIM_REVISION}" \
  && git clone --filter=blob:none https://github.com/koide3/glim_ros2.git /opt/glim_ws/src/glim_ros2 \
  && git -C /opt/glim_ws/src/glim_ros2 checkout "${GLIM_ROS2_REVISION}" \
  && . /opt/ros/jazzy/setup.sh \
  && colcon build --base-paths /opt/glim_ws/src \
       --build-base /opt/glim_ws/build \
       --install-base /opt/glim_ws/install \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_CUDA=OFF \
                    -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_MARCH_NATIVE=OFF

RUN printf '%s\n' '#!/usr/bin/env bash' 'set -e' \
    'source /opt/ros/jazzy/setup.bash' \
    'source /opt/glim_ws/install/setup.bash' 'exec "$@"' \
    > /ros_entrypoint.sh \
  && chmod +x /ros_entrypoint.sh

ARG BENCHMARK_CPU_THREADS=8

ENV BENCHMARK_CPU_ONLY=1 \
    OMP_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    OPENBLAS_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    MKL_NUM_THREADS=${BENCHMARK_CPU_THREADS} \
    TBB_NUM_THREADS=${BENCHMARK_CPU_THREADS}

LABEL org.opencontainers.image.source="https://github.com/koide3/glim" \
      benchmark.glim.revision="faa264a1bce1bda406f73457e35511f56cdc2eaa" \
      benchmark.glim_ros2.revision="4a9e7a4cb084967c8525a1be529ad3ba2a118ae7" \
      benchmark.gtsam.revision="2f3e56c0ddbd3a1aa54ed043643b553d26a069f6" \
      benchmark.gtsam_points.revision="9d32e7dbecf6015560d84b4901d6b0a6f483ec46" \
      benchmark.base_image="docker.io/library/ros:jazzy-ros-base" \
      benchmark.base_digest="sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f" \
      benchmark.cpu_policy="cpu_only;threads=8"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
