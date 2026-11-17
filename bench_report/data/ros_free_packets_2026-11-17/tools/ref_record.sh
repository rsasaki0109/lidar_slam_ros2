#!/usr/bin/env bash
# Reference clouds: velodyne_transform_node (56fc178d overlay) on Leo Drive bag1's three packet streams.
set -eo pipefail
BAG=/home/user/data/leo_drive/bag1
OUT=/home/user/bench/m7c/ref_clouds
OV=/home/user/bench/velo56/install
source /opt/ros/jazzy/setup.bash
source $OV/setup.bash
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1
CAL=$OV/velodyne_pointcloud/share/velodyne_pointcloud/params/VLP16db.yaml
cat > /home/user/bench/m7c/velodyne_params.yaml <<YAML
/**:
  ros__parameters:
    calibration: $CAL
    model: VLP16
    min_range: 0.9
    max_range: 200.0
    view_direction: 0.0
    fixed_frame: ""
    target_frame: ""
    organize_cloud: false
YAML
cat > /home/user/bench/m7c/qos.yaml <<YAML
/sensing/lidar/front/velodyne_packets: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}
/sensing/lidar/left/velodyne_packets: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}
/sensing/lidar/right/velodyne_packets: {reliability: reliable, durability: volatile, history: keep_last, depth: 10}
YAML
pids=()
for s in front left right; do
  ros2 run velodyne_pointcloud velodyne_transform_node --ros-args -r __node:=transform_$s \
    -r velodyne_packets:=/sensing/lidar/$s/velodyne_packets -r velodyne_points:=/ref/$s/points \
    --params-file /home/user/bench/m7c/velodyne_params.yaml > /home/user/bench/m7c/transform_$s.log 2>&1 &
  pids+=($!)
done
sleep 5
ros2 bag record -s mcap -o $OUT /ref/front/points /ref/left/points /ref/right/points /sensing/imu/imu_data > /home/user/bench/m7c/record.log 2>&1 &
rec=$!
sleep 8
ros2 bag play $BAG --rate 0.25 --qos-profile-overrides-path /home/user/bench/m7c/qos.yaml --topics /sensing/lidar/front/velodyne_packets /sensing/lidar/left/velodyne_packets /sensing/lidar/right/velodyne_packets /sensing/imu/imu_data > /home/user/bench/m7c/play.log 2>&1
sleep 10
kill -INT $rec; sleep 5
kill -INT "${pids[@]}"; sleep 3
pkill -f "__node:=transform_" || true
ros2 bag info $OUT | grep -E "Topic:|Count"
