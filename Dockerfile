# lidarslam_ros2 — prebuilt SLAM workspace + one-command demo.
#
#   docker run --rm -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
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
FROM ros:${ROS_DISTRO}-ros-core
ARG ROS_DISTRO

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
# No --symlink-install: a symlinked install/ dangles once build/ is removed.
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
  && colcon build --packages-up-to lidarslam rko_lio \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log

ENTRYPOINT ["/lidarslam_ws/docker/entrypoint.sh"]
CMD ["bash", "scripts/run_docker_demo.sh"]
