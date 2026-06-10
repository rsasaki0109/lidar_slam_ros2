#!/usr/bin/env bash
# Source ROS + the prebuilt lidarslam workspace, then run the given command.
set -e

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /lidarslam_ws/install/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /lidarslam_ws/install/setup.bash
fi

exec "$@"
