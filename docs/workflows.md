# Operator Workflows

This page keeps the procedural details that do not need to stay in the top-level
README.

## Build Prerequisites

- `scanmatcher` depends on
  [`ndt_omp_ros2`](https://github.com/rsasaki0109/ndt_omp_ros2)
- clone with submodules:

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

- build and run the default checks:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
bash scripts/run_default_ci_checks.sh
```

## Main Entry Points

| Goal | Entrypoint |
| --- | --- |
| Autoware pointcloud-map quickstart | `bash scripts/run_autoware_quickstart.sh` |
| Full dogfood flow | `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20` |
| Standard NTU VIRAL benchmark | `bash scripts/run_rko_lio_graph_benchmark.sh` |
| MID360 cross-validation benchmark | `bash scripts/run_rko_lio_mid360_crossval_benchmark.sh` |
| Release/readiness gate | `bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10` |

## Required Input Topics

### Public default path: `RKO-LIO + graph_based_slam`

Launch:

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/os_cloud_node/imu
```

Required inputs:

- `lidar_topic`: `sensor_msgs/msg/PointCloud2`
- `imu_topic`: `sensor_msgs/msg/Imu`

Optional inputs:

- `/gnss/fix`: `sensor_msgs/msg/NavSatFix` when `graph_based_slam use_gnss:=true`

Internal wiring in this launch:

- `RKO-LIO` publishes odometry on `/rko_lio/odometry`
- `RKO-LIO` publishes submap source clouds on `/rko_lio/frame`
- `graph_based_slam` consumes those via `odom_input` and `cloud_input`

Not currently supported in the public path:

- wheel odometry / vehicle speed topic fusion

GNSS note:

- GNSS is added as translation-only pose-graph constraints in the backend
- when covariance is present, edge weight is scaled from `position_covariance`
- `NavSatFix` does not standardize RTK fix status, so `graph_based_slam`
  treats low horizontal covariance as `RTK-like`
- default threshold: `gnss_rtk_fix_max_horizontal_stddev_m = 0.3`

### Classic path: `scanmatcher + graph_based_slam`

Launch:

```bash
ros2 launch lidarslam lidarslam.launch.py \
  input_cloud:=/points_raw \
  imu_topic:=/imu
```

Required inputs:

- `input_cloud`: `sensor_msgs/msg/PointCloud2`
- TF from `robot_frame_id` to the LiDAR frame

Optional inputs:

- `imu_topic`: `sensor_msgs/msg/Imu` when `scanmatcher use_imu:=true`
- odom TF into `odom_frame_id` when `scanmatcher use_odom:=true`
- `/gnss/fix`: `sensor_msgs/msg/NavSatFix` when backend `use_gnss:=true`

Internal wiring in this launch:

- `scanmatcher` publishes `lidarslam_msgs/msg/MapArray` on `map_array`
- `graph_based_slam` subscribes to `map_array`

### Backend only: `graph_based_slam`

Launch:

```bash
ros2 launch graph_based_slam graphbasedslam.launch.py
```

Default required input:

- `map_array`: `lidarslam_msgs/msg/MapArray`

Optional backend aids:

- `/imu`: `sensor_msgs/msg/Imu` when `use_imu_preintegration:=true`
- `/gnss/fix`: `sensor_msgs/msg/NavSatFix` when `use_gnss:=true`

Alternative direct-input mode used by the RKO-LIO launch:

- `odom_input`: `nav_msgs/msg/Odometry`
- `cloud_input`: `sensor_msgs/msg/PointCloud2`

Useful GNSS weighting parameters:

- `gnss_info_weight`
- `gnss_use_covariance_weighting`
- `gnss_covariance_min_variance_m2`
- `gnss_covariance_max_variance_m2`
- `gnss_rtk_fix_max_horizontal_stddev_m`
- `gnss_rtk_fix_weight_scale`
- `gnss_non_rtk_weight_scale`

Inspect a bag before enabling GNSS weighting:

```bash
python3 scripts/inspect_navsatfix_covariance.py /path/to/rosbag2 --topic /gnss/fix
```

## Run `RKO-LIO + graph_based_slam`

The main launch entrypoint is:

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/os_cloud_node/imu
```

Useful parameter files:

- default graph backend: `graph_based_slam/param/graphbasedslam.yaml`
- default scanmatcher frontend: `lidarslam/param/lidarslam.yaml`
- NTU VIRAL RKO-LIO profile: `lidarslam/param/rko_lio_ntu_viral.yaml`
- MID360 tuned profile: `lidarslam/param/lidarslam_mid360_rko_graph.yaml`

## Save Maps

Save the current map at any time with:

```bash
ros2 service call /map_save std_srvs/srv/Empty
```

Typical outputs:

- `map.pcd`
- `pose_graph.g2o`
- `pointcloud_map/pointcloud_map_metadata.yaml`
- `pointcloud_map/*.pcd`
- `map_projector_info.yaml`

## Autoware Map Output Notes

`graph_based_slam` always writes `map_projector_info.yaml`.

- without GNSS: `projector_type: Local`
- with GNSS and a stable origin: `projector_type: LocalCartesian` plus
  `map_origin`

To stage an existing run into an Autoware map bundle:

```bash
bash scripts/prepare_autoware_map_from_graph_slam.sh \
  output/bench_rko_lio_ntu_viral_loopgate_20260324 \
  /tmp/autoware_maps/ntu_viral_loopgate
```

To open the staged map through Autoware's map loaders:

```bash
bash scripts/run_autoware_pointcloud_map_viewer_docker.sh \
  /tmp/autoware_maps/ntu_viral_loopgate \
  /tmp/autoware_core \
  /tmp/autoware_map_runtime_ws
```

For the short supported path, use
`bash scripts/run_autoware_quickstart.sh` instead.

## Loop Closure Notes

`graph_based_slam` supports two loop-candidate sources:

- distance-based revisit search
- built-in GPL-free Scan Context place recognition

The backend validates candidates geometrically before adding a loop edge and
keeps only the best local edge inside the configured dedup window.

To regenerate the README loop-area zoom figure used for visual inspection of
closing-segment duplication:

```bash
python3 scripts/generate_readme_loop_zoom_figure.py
```

## Benchmark And Dataset Pointers

Recommended public benchmark:

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
```

Current MID360 cross-validation path:

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh
```

The public benchmark and release-report flow is documented in
[benchmarking.md](benchmarking.md).

## Related Docs

- [Autoware Quickstart](autoware-quickstart.md)
- [Benchmarking And Release Gate](benchmarking.md)
- [Comparison](comparison.md)
