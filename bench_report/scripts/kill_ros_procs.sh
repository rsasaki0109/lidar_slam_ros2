#!/usr/bin/env bash
# Stops leftover pipeline processes (ros2 launch/bag CLIs, offline_node, graph_based_slam_node,
# static_transform_publisher, the barrier script). Patterns are anchored to the executable path so the
# caller's own shell never matches.
pats=('^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 (bag|launch|service)'
      '^/home/user/ws/[^ ]*/lib/rko_lio/offline_node'
      '^/home/user/ws/[^ ]*/lib/graph_based_slam/graph_based_slam_node'
      '^/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher'
      '^bash /home/user/ws/[^ ]*/wait_for_offline_output_subscribers.sh')
list() { for p in "${pats[@]}"; do pgrep -f "$p"; done | sort -u; }
for sig in INT INT KILL; do
  pids=$(list); [[ -z "${pids}" ]] && break
  kill -"${sig}" ${pids} 2>/dev/null; sleep 4
done
left=$(list); echo "left: ${left:-none}"
