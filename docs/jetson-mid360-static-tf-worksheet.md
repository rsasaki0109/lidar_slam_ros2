# Jetson MID-360 Static TF Worksheet

Use this worksheet when mounting a Livox MID-360 on a quadruped or biped robot.
The goal is to make the `base_link -> livox_frame` transform explicit before
running the MID-360 SLAM preset.

## Frame Contract

Use this frame shape unless the robot already has a stronger convention:

```text
map -> odom -> base_link -> livox_frame
```

- `base_link`: robot body frame used for mapping.
- `livox_frame`: MID-360 point-cloud frame.
- `imu_frame`: MID-360 IMU frame. Use `livox_frame` when the driver publishes
  LiDAR and IMU in the same sensor frame.

## Measurement Record

Record the measured translation from `base_link` origin to the MID-360 frame:

```yaml
base_frame: base_link
lidar_frame: livox_frame
translation_m:
  x: 0.0
  y: 0.0
  z: 0.0
rotation_quat_xyzw:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
measurement_note: replace with mount measurement and axis convention
```

Axis sanity checks:

- Positive `x` points forward in the robot body convention.
- Positive `y` points left in the robot body convention.
- Positive `z` points up in the robot body convention.
- A stationary bag does not show obvious roll/pitch inversion.
- A slow yaw-in-place bag rotates around the expected vertical axis.

## Launch Usage

Pass frames explicitly when running a robot bag:

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

If the mount is not identity, publish the measured static transform from the
robot bringup or from a dedicated static transform publisher. Keep that launch
file next to the bag manifest.

## Bag Checks

Before a full mapping route:

```bash
python3 scripts/preflight_mid360_robot_bag.py /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml
```

The report should show:

- `PointCloud2` topic detected.
- `Imu` topic detected.
- Livox/MID-360 preset recommendation available.
- A launch command using the measured frame names.

After measuring the mount, copy
`configs/mid360_robot/livox_mid360_default.yaml` to a robot-specific file and
replace `mount.xyz` / `mount.q_xyzw` with the measured transform.

Validate the edited profile:

```bash
python3 scripts/validate_mid360_robot_profile.py configs/mid360_robot/<robot>.yaml
```
