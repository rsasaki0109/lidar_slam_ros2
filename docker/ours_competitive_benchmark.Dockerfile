# Reproducible ours benchmark image for the pinned ROS 2 Jazzy track.
FROM docker.io/library/ros:jazzy-ros-base@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

ARG OURS_REVISION=866f733677e92ecb08d67126e463da99dd140d46
ARG OURS_REPOSITORY=https://github.com/rsasaki0109/lidar_slam_ros2.git
ARG NDT_OMP_SUBMODULE_REVISION=497411279593eb261a3e3d04cdcbb4717af33ca3
ARG RKO_LIO_GITLINK_REVISION=622b74778a41f753d47aa5918043755ebcbd4c75
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
      ca-certificates git python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ours_ws

# The build context intentionally contains only this recipe.  Fetch the
# declared source revision inside the image so the build-required ndt_omp
# gitlink is present; a host checkout, archive, or dirty working tree must
# never become benchmark input.  The rko_lio gitlink is intentionally not
# initialized: its pinned object is unavailable from the public mirror and it
# is not part of this BUILD_TESTING=OFF backend package target.
RUN git clone --no-checkout "$OURS_REPOSITORY" /opt/ours_ws/src/lidar_slam_ros2 \
 && git -C /opt/ours_ws/src/lidar_slam_ros2 checkout --detach "$OURS_REVISION" \
 && git -C /opt/ours_ws/src/lidar_slam_ros2 submodule sync --recursive \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 ls-tree HEAD Thirdparty/ndt_omp_ros2 | awk '{ print $3 }')" = "$NDT_OMP_SUBMODULE_REVISION" \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 ls-tree HEAD Thirdparty/rko_lio | awk '{ print $3 }')" = "$RKO_LIO_GITLINK_REVISION" \
 && git -C /opt/ours_ws/src/lidar_slam_ros2 submodule update --init --recursive -- Thirdparty/ndt_omp_ros2 \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 rev-parse HEAD)" = "$OURS_REVISION" \
 && test -z "$(git -C /opt/ours_ws/src/lidar_slam_ros2 status --porcelain=v1 --untracked-files=all)" \
 && test -z "$(git -C /opt/ours_ws/src/lidar_slam_ros2 submodule status --recursive -- Thirdparty/ndt_omp_ros2 \
      | awk '$1 ~ /^[-+U]/ { print; bad=1 } END { exit bad }')" \
 && test "$(git -C /opt/ours_ws/src/lidar_slam_ros2 submodule status --recursive -- Thirdparty/rko_lio \
      | awk '{ print substr($1, 1, 1) }')" = "-" \
 && test ! -f /opt/ours_ws/src/lidar_slam_ros2/Thirdparty/rko_lio/package.xml \
 && colcon list --base-paths /opt/ours_ws/src/lidar_slam_ros2 \
      | awk '$1 == "rko_lio" { found=1 } END { exit found }'

RUN apt-get update \
 && if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
 && rosdep update --rosdistro jazzy \
 && source /opt/ros/jazzy/setup.bash \
 && rosdep install -r -y --from-paths /opt/ours_ws/src/lidar_slam_ros2 \
      --ignore-src --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --base-paths /opt/ours_ws/src/lidar_slam_ros2 \
      --packages-up-to lidarslam graph_based_slam \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

LABEL org.opencontainers.image.source="https://github.com/rsasaki0109/lidar_slam_ros2" \
      benchmark.ours.repository="https://github.com/rsasaki0109/lidar_slam_ros2.git" \
      benchmark.ours.revision="866f733677e92ecb08d67126e463da99dd140d46" \
      benchmark.ndt_omp.submodule_revision="497411279593eb261a3e3d04cdcbb4717af33ca3" \
      benchmark.rko_lio.gitlink_revision="622b74778a41f753d47aa5918043755ebcbd4c75" \
      benchmark.rko_lio.initialized="false" \
      benchmark.base_image="docker.io/library/ros:jazzy-ros-base" \
      benchmark.base_digest="sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f" \
      benchmark.cpu_policy="cpu_only;threads=8"

CMD ["bash"]
