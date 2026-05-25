# Jetson + MID-360 Robot Scope

This project scope focuses on running the existing ROS 2 LiDAR SLAM pipeline on
Jetson-class computers with a Livox MID-360 mounted on legged robots.

For the operator command sequence, use the
[Jetson MID-360 Robot Runbook](jetson-mid360-robot-runbook.md).

The target robots are quadruped and biped platforms that can provide stable
power, time-synchronized MID-360 packets or point clouds, and a base-frame
definition for mapping. The first supported product path is mapping and map
validation, not locomotion control.

## In Scope

- Jetson deployment notes for the existing `RKO-LIO + graph_based_slam` path.
- Livox MID-360 point-cloud and IMU input using the tracked MID-360 presets.
- Quadruped and biped robot frame conventions:
  `map -> odom -> base_link -> livox_frame`.
- Offline rosbag2 mapping from robot logs.
- Saved map outputs:
  `pointcloud_map/`, `map_projector_info.yaml`, `pose_graph.g2o`, and `map.pcd`.
- Benchmark and validation evidence using the existing MID-360 cross-validation
  wrapper.
- Operational checks for topic names, frame names, static extrinsics, CPU/GPU
  load, thermals, and storage throughput on Jetson-class hardware.

## Out Of Scope For The First Pass

- Whole-body control, gait generation, footstep planning, and balance recovery.
- Real-time autonomy stacks that consume the map while walking.
- Wheel odometry or vehicle-speed fusion.
- Custom Livox driver development beyond documenting the expected topics and
  frames.
- RTK/GNSS-first workflows for outdoor vehicle mapping.
- Hardware-specific kernel, CUDA, or JetPack tuning unless it blocks the SLAM
  path directly.

## Primary Pipeline

Use the existing RKO-LIO launch as the default entry point:

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/livox/lidar \
  imu_topic:=/livox/imu \
  base_frame:=base_link \
  lidar_frame:=livox_frame \
  imu_frame:=livox_frame \
  main_param_dir:=/path/to/lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  rko_param_file:=/path/to/lidarslam/param/rko_lio_mid360.yaml
```

For repeatable evaluation, use:

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh
```

For an arbitrary robot bag, start with the repository preflight:

```bash
python3 scripts/preflight_autoware_map_bag.py /path/to/rosbag2
```

For the robot-specific report with MID-360 frame arguments:

```bash
python3 scripts/preflight_mid360_robot_bag.py /path/to/rosbag2
```

To preflight and run the MID-360 mapping path with the tracked presets:

```bash
bash scripts/run_mid360_robot_map.sh /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --base-frame base_link \
  --lidar-frame livox_frame \
  --imu-frame livox_frame
```

Use `--dry-run` first in the field to inspect the selected topics, robot
profile checks, and launch arguments without starting SLAM.

If the bag contains Livox/MID-360-style topics, the preflight should emit the
`rko_lio_graph_mid360_preset` recommendation. To let the repository select and
run that path automatically:

```bash
python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2
```

## Minimum Robot Contract

The robot integration should provide:

- `sensor_msgs/msg/PointCloud2` from the MID-360.
- `sensor_msgs/msg/Imu` from the MID-360 or a body-mounted IMU.
- A stable `base_link` frame.
- A calibrated static transform between `base_link` and `livox_frame`.
- Rosbag2 logs with enough storage bandwidth to preserve point-cloud timing.
- A known time source for LiDAR and IMU timestamps.

## Jetson Bringup Checklist

Use this checklist before spending field time on a quadruped or biped run.

Host and ROS:

- Jetson has the target Ubuntu / ROS 2 distro installed and sourced.
- The workspace builds in Release mode.
- `rosdep install --from-paths src --ignore-src -r -y` has no unresolved
  runtime dependencies.
- `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
  completes on the Jetson or in the deployment image.
- `ros2 doctor` and `ros2 topic list` work without DDS discovery issues.

MID-360 input:

- The Livox driver publishes a `sensor_msgs/msg/PointCloud2` topic, usually
  `/livox/lidar`.
- The IMU topic is present, usually `/livox/imu`.
- Point-cloud and IMU timestamps advance monotonically during robot motion.
- The point-cloud frame is set consistently, usually `livox_frame`.

Robot frames:

- `base_link` is fixed to the robot body frame used for mapping.
- `livox_frame` is the MID-360 optical/body frame used by the point cloud.
- A measured static transform from `base_link` to `livox_frame` is available.
- The launch does not silently fall back to an identity transform unless the
  sensor is intentionally treated as the base frame.
- Use the [Jetson MID-360 Static TF Worksheet](jetson-mid360-static-tf-worksheet.md)
  before recording the target mapping route.

Jetson runtime:

- CPU clocks and thermal mode are set for repeatable runs.
- NVMe or equivalent storage is used for bag recording when possible.
- Free disk space covers the expected point-cloud recording rate.
- A separate terminal watches CPU load, memory, temperature, and throttling.
- The robot can record at least one stationary bag and one short walking bag
  before a long mapping run.

Use the host-readiness report before recording:

```bash
python3 scripts/check_jetson_mid360_host_readiness.py \
  --bag-dir /path/to/bag_storage \
  --output-dir output/mid360_robot_test
```

This records Jetson model metadata, ROS/tool availability, storage headroom,
thermal state, available memory, and CPU governor state.

## MID-360 Bag Preflight

Record a short stationary bag first:

```bash
bash scripts/run_mid360_robot_field_session.sh \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --bag-root /path/to/bag_storage \
  --run-id stand_01 \
  --duration-sec 30 \
  --output-dir output/mid360_robot_test \
  --dry-run
bash scripts/record_mid360_robot_bag.sh \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --bag-root /path/to/bag_storage \
  --run-id stand_01 \
  --duration-sec 30 \
  --dry-run
bash scripts/record_mid360_robot_bag.sh \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --bag-root /path/to/bag_storage \
  --run-id stand_01 \
  --duration-sec 30
bash scripts/check_mid360_robot_recording.sh \
  --bag /path/to/bag_storage/stand_01 \
  --robot-profile /path/to/bag_storage/stand_01_profile.yaml \
  --output-dir output/mid360_robot_test
```

Inspect the recorded metadata:

```bash
ros2 bag info /path/to/bag_storage/stand_01
python3 scripts/preflight_autoware_map_bag.py /path/to/bag_storage/stand_01
python3 scripts/preflight_mid360_robot_bag.py /path/to/bag_storage/stand_01 \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml
bash scripts/run_mid360_robot_map.sh /path/to/bag_storage/stand_01 \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --write-manifest \
  --dry-run
```

Expected preflight result:

- at least one `sensor_msgs/msg/PointCloud2` topic
- at least one `sensor_msgs/msg/Imu` topic
- metadata rate checks for the selected point cloud and IMU topics
- sampled `header.frame_id` checks for readable point cloud and IMU messages
- sampled TF connectivity checks for `base_frame -> lidar_frame` and
  `base_frame -> imu_frame`, when TF messages are readable
- a Livox/MID-360 recommendation using
  `lidarslam_mid360_rko_graph.yaml` and `rko_lio_mid360.yaml`
- profile expected topics present, when `--robot-profile` is provided
- a dry-run command that passes `base_frame`, `lidar_frame`, and `imu_frame`
- no missing `metadata.yaml` or empty message-count warnings

## Robot Profile

Use a robot profile to keep field commands repeatable. The default template is:

```bash
configs/mid360_robot/livox_mid360_default.yaml
```

Validate a profile before taking it to the robot:

```bash
python3 scripts/validate_mid360_robot_profile.py \
  configs/mid360_robot/livox_mid360_default.yaml
```

Profile fields:

```yaml
robot_name: livox_mid360_default
base_frame: base_link
lidar_frame: livox_frame
imu_frame: livox_frame
expected_pointcloud_topic: /livox/lidar
expected_imu_topic: /livox/imu
mount:
  xyz: [0.0, 0.0, 0.0]
  q_xyzw: [0.0, 0.0, 0.0, 1.0]
```

The planner uses the profile to select the expected topics. If the bag uses
different topic names, preflight fails instead of silently switching to another
topic. CLI frame arguments can still override the profile for one-off tests.

## Run Manifest

Before a real run, write a readiness report:

```bash
python3 scripts/check_mid360_robot_readiness.py /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --output-dir output/mid360_robot_test \
  --write-manifest
```

This writes:

- `output/mid360_robot_test/mid360_robot_readiness.json`
- `output/mid360_robot_test/mid360_robot_readiness.md`
- `output/mid360_robot_test/mid360_robot_run_plan.json`
- `output/mid360_robot_test/mid360_robot_run_plan.md`

Readiness status is `PASS`, `WARN`, or `FAIL`. A missing IMU, missing
PointCloud2, invalid profile, or expected-topic mismatch is `FAIL`; missing TF
metadata or no MID-360 preset recommendation is `WARN`.

Write a reproducible run plan before starting SLAM:

```bash
bash scripts/run_mid360_robot_map.sh /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --output-dir output/mid360_robot_test \
  --write-manifest \
  --dry-run
```

This writes:

- `output/mid360_robot_test/mid360_robot_run_plan.json`
- `output/mid360_robot_test/mid360_robot_run_plan.md`

The manifest records the bag path, output directory, selected topics, frames,
robot profile snapshot, preflight checks, and generated commands.

For a real run, also write the post-run diagnosis:

```bash
bash scripts/run_mid360_robot_map.sh /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --output-dir output/mid360_robot_test \
  --write-manifest \
  --write-diagnosis
```

This adds `autoware_map_diagnosis.md` and `autoware_map_diagnosis.json` to the
same output directory after the SLAM command exits.

If the frame names are not `base_link` / `livox_frame`, pass them explicitly:

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  main_param_dir:=lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  rko_param_file:=lidarslam/param/rko_lio_mid360.yaml \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/livox/lidar \
  imu_topic:=/livox/imu \
  base_frame:=base_link \
  lidar_frame:=livox_frame \
  imu_frame:=livox_frame
```

## Field Log Recipe

For quadruped and biped robots, prefer boring logs before aggressive walking.

1. Record 30 seconds stationary with the robot standing.
2. Record 30 seconds of slow body yaw in place, if the platform can do it
   safely.
3. Record a short out-and-back walk in a structured area with walls, poles, or
   other stable geometry.
4. Record the target mapping route after the short log is already confirmed.
5. Save the robot model, sensor mount measurement, and launch arguments next to
   the bag.

For each bag, keep a small manifest:

```yaml
robot_type: quadruped
compute: jetson_orin_nx
lidar: livox_mid360
pointcloud_topic: /livox/lidar
imu_topic: /livox/imu
base_frame: base_link
lidar_frame: livox_frame
imu_frame: livox_frame
mount_note: measured base_link to livox_frame static transform
route_note: indoor loop, slow walk, no autonomy
```

## Acceptance Criteria

A robot run is inside the supported scope when:

- The bag preflight identifies a MID-360/Livox-style point-cloud topic.
- The RKO-LIO frontend publishes `/rko_lio/odometry` and `/rko_lio/frame`.
- `graph_based_slam` consumes the frontend odometry and frame clouds.
- `/map_save` produces a loadable pointcloud-map output.
- The trajectory and map can be inspected in RViz or the documented browser
  proof path.
- The run can be repeated with the same parameter files and produces comparable
  map quality.

## Remaining Work Items

- Add a known-good Jetson Orin launch profile after a real robot run is
  captured.
- Track one representative robot bag as a non-release research profile.
