# lidarslam_ros2 — prebuilt SLAM workspace + one-command demo.
#
#   mkdir -p "$PWD/lidarslam_output"
#   docker run --rm \
#     -e LIDARSLAM_HOST_UID="$(id -u)" -e LIDARSLAM_HOST_GID="$(id -g)" \
#     -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
#     ghcr.io/rsasaki0109/lidar_slam_ros2:humble
#
# The default command downloads a public Livox MID-360 driving bag
# (Zenodo 14841855, CC-BY 4.0, 517 MB) and runs RKO-LIO + graph_based_slam
# headless, saving an Autoware-ready map under /lidarslam_ws/output.
# Any other command runs inside the sourced workspace, e.g.:
#
#   docker run --rm -it ghcr.io/rsasaki0109/lidar_slam_ros2:humble \
#     ros2 launch lidarslam lidarslam.launch.py
#
# Build (from a checkout with submodules): docker build -t lidar_slam_ros2 .
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core AS builder
ARG ROS_DISTRO
ARG LIDARSLAM_SOURCE_REVISION=
ARG LIDARSLAM_SOURCE_DIRTY=

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /lidarslam_ws
COPY . .

# Same dependency set as .github/workflows/main.yml (default workflow).
RUN apt-get update \
  && if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
  && rosdep update --rosdistro "${ROS_DISTRO}" \
  && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
  && rosdep install -r -y \
    --from-paths \
      lidarslam \
      lidarslam_msgs \
      scanmatcher \
      graph_based_slam \
      Thirdparty/ndt_omp_ros2 \
      Thirdparty/rko_lio \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
  && { apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-rosbag2-storage-sqlite3" \
    || apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-rosbag2-storage-default-plugins"; } \
  && apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-rosbag2-storage-mcap" \
    python3-scipy \
  && rm -rf /var/lib/apt/lists/*

# Same package selection as scripts/run_default_ci_checks.sh (the Thirdparty
# tree carries extra research packages whose deps are not installed here).
# No --symlink-install: the runtime stage receives only install/.
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
  && colcon build --packages-up-to lidarslam rko_lio \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DLIDARSLAM_SOURCE_REVISION:STRING="${LIDARSLAM_SOURCE_REVISION}" \
      -DLIDARSLAM_SOURCE_DIRTY:STRING="${LIDARSLAM_SOURCE_DIRTY}" \
  && . install/setup.sh \
  && lidarslam-map --version \
  && python3 docker/collect_runtime_apt_packages.py \
    --install-root /lidarslam_ws/install \
    --ros-distro "${ROS_DISTRO}" \
    --output /tmp/lidarslam-runtime-packages.txt \
    --report /tmp/lidarslam-runtime-packages.json

FROM ros:${ROS_DISTRO}-ros-core AS runtime
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV DEMO_DATA_DIR=/lidarslam_ws/datasets/mid360_public
ENV DEMO_OUTPUT_DIR=/lidarslam_ws/output/mid360_demo

WORKDIR /lidarslam_ws

COPY --from=builder \
  /tmp/lidarslam-runtime-packages.txt \
  /tmp/lidarslam-runtime-packages.txt
RUN test -s /tmp/lidarslam-runtime-packages.txt \
  && apt-get update \
  && xargs -r apt-get install -y --no-install-recommends \
    < /tmp/lidarslam-runtime-packages.txt \
  && rm -f /tmp/lidarslam-runtime-packages.txt \
  && rm -rf /var/lib/apt/lists/*

COPY --from=builder /lidarslam_ws/install/ /lidarslam_ws/install/
COPY docker/entrypoint.sh /lidarslam_ws/docker/entrypoint.sh

RUN test -x /lidarslam_ws/docker/entrypoint.sh \
  && test -x /lidarslam_ws/install/lidarslam/bin/lidarslam-map \
  && test -x /lidarslam_ws/install/lidarslam/share/lidarslam/product/scripts/run_docker_demo.sh \
  && . /lidarslam_ws/install/setup.sh \
  && lidarslam-map --version

ENTRYPOINT ["/lidarslam_ws/docker/entrypoint.sh"]
CMD ["bash", "/lidarslam_ws/install/lidarslam/share/lidarslam/product/scripts/run_docker_demo.sh"]
