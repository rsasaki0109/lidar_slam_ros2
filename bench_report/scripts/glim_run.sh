#!/usr/bin/env bash
# Usage: [GLIM_OVERRIDES=<dir with the 4 override json files>] glim_run.sh <out_dir> [bag]
# Native equivalent of scripts/glim_container_run.sh (v1 path) of lidar_slam_ros2: GLIM's default config
# with the four configs/glim/hilti2022_cpu overrides, then
#   /usr/bin/time -v ros2 run glim_ros glim_rosbag <bag> -p config_path:=... -p auto_quit:=true -p dump_path:=<out>/dump
# The only change is config_logging.json's log_dir (/out/logs in the container -> <out>/logs).
out=$1; bag=${2:-/home/user/data/hilti2022/exp04_ros2}
rm -rf "${out}"; mkdir -p "${out}"
set +u; source /opt/ros/jazzy/setup.bash; source /home/user/glim_ws/install/setup.bash; set -u
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
export ROS_HOME="${out}/ros_home" ROS_LOG_DIR="${out}/ros_log" ROS_DOMAIN_ID=38
cfg="${out}/glim_config"
cp -a /home/user/glim_src/glim/config "${cfg}"
for f in config.json config_sensors.json config_ros.json config_logging.json; do
  cp ${GLIM_OVERRIDES:-/home/user/lidar_slam_ros2/configs/glim/hilti2022_cpu}/${f} "${cfg}/${f}"
done
sed -i "s|\"/out/logs\"|\"${out}/logs\"|" "${cfg}/config_logging.json"
/usr/bin/time -v -o "${out}/process_time.txt" \
  ros2 run glim_ros glim_rosbag "${bag}" --ros-args \
    -p config_path:="${cfg}" -p auto_quit:=true -p dump_path:="${out}/dump" > "${out}/glim.log" 2>&1
echo "$?" > "${out}/process_exit_status.txt"
wall=$(awk -F': ' '/Elapsed \(wall clock\)/ {print $2}' "${out}/process_time.txt")
echo "exit=$(cat "${out}/process_exit_status.txt") wall=${wall} maxrss_kb=$(awk -F': ' '/Maximum resident/ {print $2}' "${out}/process_time.txt") traj_lines=$(wc -l < "${out}/dump/traj_imu.txt" 2>/dev/null)"
