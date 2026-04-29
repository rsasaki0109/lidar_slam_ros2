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

Optional 3D-BBS support:

- `Thirdparty/3d_bbs` is a small MIT-licensed vendor tree with `COLCON_IGNORE`.
- `graph_based_slam` builds its CPU 3D-BBS sources automatically when
  `GRAPH_BASED_SLAM_ENABLE_3D_BBS=ON` and the vendor headers are present.
- Runtime use is still off by default; enable it with
  `use_3d_bbs_for_scan_context: true` in the graph parameter YAML or with the
  MID360 benchmark wrapper option shown in the benchmarking docs.
- To force-disable the optional build, pass
  `--cmake-args -DGRAPH_BASED_SLAM_ENABLE_3D_BBS=OFF`.

## Main Entry Points

| Goal | Entrypoint |
| --- | --- |
| Autoware pointcloud-map quickstart | `bash scripts/run_autoware_quickstart.sh` |
| Full dogfood flow | `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20` |
| Standard NTU VIRAL benchmark | `bash scripts/run_rko_lio_graph_benchmark.sh` |
| KITTI Odometry small_gicp evaluation | `bash scripts/run_kitti_odometry_benchmark.sh --sequence 00 --small-gicp --force-prepare` |
| KITTI Odometry small_gicp sweep | `bash scripts/sweep_kitti_small_gicp.sh --dataset "$KITTI_ODOMETRY_ROOT" --sequences "00 05 07"` |
| MID360 cross-validation benchmark | `bash scripts/run_rko_lio_mid360_crossval_benchmark.sh` |
| Mixed-quality open-data GNSS smoke | `bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh --bag /path/to/rosbag2 --applanix-msg-dir /tmp/applanix/applanix_msgs/msg --verify-map` |
| Mixed-quality open-data GNSS benchmark | `bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh --bag /path/to/rosbag2 --applanix-msg-dir /tmp/applanix/applanix_msgs/msg --verify-map` |
| Leo Drive classic-path suite | `bash scripts/run_open_data_classic_path_benchmark_suite.sh --applanix-msg-dir /tmp/applanix/applanix_msgs/msg --verify-map` |
| Packet IMU deskew validation matrix | `bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh --applanix-msg-dir /tmp/applanix/applanix_msgs/msg` |
| Dynamic-object-filter save-map benchmark | `bash scripts/run_dynamic_object_filter_benchmark.sh` |
| MID360 place-recognition comparison | `bash scripts/run_place_recognition_benchmark.sh` |
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

### KITTI / LiDAR-only evaluation path

KITTI Odometry Velodyne sequences do not include IMU. Use this path for
LiDAR-only evaluation and frontend tuning, not as the public default workflow.

Download and run one sequence:

```bash
bash scripts/download_kitti_odometry.sh --velodyne
export KITTI_ODOMETRY_ROOT="$PWD/datasets/KITTI_odometry"
bash scripts/run_kitti_odometry_benchmark.sh --sequence 00 --small-gicp --force-prepare
```

Sweep several `small_gicp` parameter sets:

```bash
bash scripts/sweep_kitti_small_gicp.sh \
  --dataset "$KITTI_ODOMETRY_ROOT" \
  --sequences "00 05 07"
```

The KITTI wrappers write prepared rosbag2 data and benchmark artifacts under
`output/` by default. The raw KITTI dataset belongs under `datasets/`, which is
local-only and ignored by Git.

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

- `gnss_topic`
- `gnss_info_weight`
- `gnss_use_covariance_weighting`
- `gnss_covariance_min_variance_m2`
- `gnss_covariance_max_variance_m2`
- `gnss_rtk_fix_max_horizontal_stddev_m`
- `gnss_rtk_fix_weight_scale`
- `gnss_non_rtk_weight_scale`

Optional save-time dynamic-object filter:

- affects only the map written by `/map_save`
- does not change live odometry, loop closure, or the published working map
- useful when repeated passes observe parked/static structure consistently but
  transient objects appear only once

Parameters:

- `use_dynamic_object_filter`
- `dynamic_object_filter_voxel_size`
- `dynamic_object_filter_min_observations`
- `dynamic_object_filter_temporal_window`
- `dynamic_object_filter_max_range_from_sensor_m`

Inspect a bag before enabling GNSS weighting:

```bash
python3 scripts/inspect_navsatfix_covariance.py /path/to/rosbag2 --topic /gnss/fix
```

For Leo Drive open-data driving bags that only expose Applanix raw GNSS status,
inspect `GSOF50` instead:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
python3 scripts/inspect_applanix_gsof50_quality.py /path/to/rosbag2 \
  --topic /lvx_client/gsof/ins_solution_rms_50 \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg
```

If the bag has `GSOF49/50` but no `/gnss/fix`, generate a sidecar rosbag2 that
publishes only `sensor_msgs/msg/NavSatFix`:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
python3 scripts/convert_applanix_gsof_to_navsatfix_bag.py \
  --input /path/to/rosbag2 \
  --output /tmp/applanix_navsatfix_bag \
  --gsof49-topic /lvx_client/gsof/ins_solution_49 \
  --gsof50-topic /lvx_client/gsof/ins_solution_rms_50 \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --force
```

If you want to test the same Applanix raw messages as `sensor_msgs/msg/Imu`,
generate an IMU sidecar too:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
python3 scripts/convert_applanix_gsof_to_imu_bag.py \
  --input /path/to/rosbag2 \
  --output /tmp/applanix_imu_bag \
  --gsof49-topic /lvx_client/gsof/ins_solution_49 \
  --gsof50-topic /lvx_client/gsof/ins_solution_rms_50 \
  --output-topic /imu \
  --frame-id base_link \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --force
```

Then play the original bag together with the generated sidecar bag:

```bash
ros2 bag play /path/to/rosbag2 --clock
ros2 bag play /tmp/applanix_navsatfix_bag
```

For an end-to-end open-data GNSS smoke with automatic `/map_save`, use:

```bash
bash scripts/run_open_data_gnss_smoke.sh \
  --bag /path/to/rosbag2 \
  --verify-map
```

`run_open_data_gnss_smoke.sh` auto-detects the `NavSatFix` topic from
`--gnss-bag` when provided, otherwise from `--bag`.

For Leo Drive driving bags that expose LiDAR as
`velodyne_msgs/msg/VelodyneScan` and GNSS quality as Applanix `GSOF49/50`,
use the packet-to-PointCloud2 wrapper instead:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

That wrapper will:

- prefer same-bag native `sensor_msgs/msg/NavSatFix` / `sensor_msgs/msg/Imu`
  topics when they exist
- otherwise generate a `NavSatFix` sidecar bag from `GSOF49/50`
- optionally generate an `Imu` sidecar bag from `GSOF49/50`
- extract a local `TUM` reference from `GSOF49` with `extract_applanix_gsof49_reference.py`
- build a minimal `velodyne_pointcloud` overlay on demand with
  `bash scripts/prepare_velodyne_pointcloud_overlay.sh`
- convert `VelodyneScan` packets into `sensor_msgs/msg/PointCloud2`
- run `lidarslam.launch.py`, call `/map_save`, and optionally verify the output

To benchmark the same `driving_30_kmh` bag as a four-way classic-path
comparison, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_classic_path_benchmark_suite.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

That suite writes:

- `classic_path_report.md`
- `classic_path_report.json`
- `classic_path_report.svg`

To rerun the current MID360 place-recognition comparison entrypoint, use:

```bash
bash scripts/run_place_recognition_benchmark.sh
```

That wrapper reruns the distance-only baseline and a `Scan Context` candidate,
then emits:

- `place_recognition_report.md`
- `place_recognition_report.json`

For packet IMU deskew, the important caveat is runtime sensitivity. On the real
Leo Drive `all-sensors-bag1` and `all-sensors-bag6` front-lidar cases, native
`/sensing/imu/imu_data` works when the packet benchmark runs at `rate=1.0`.
Current reference numbers are:

- `bag1_front`, `no_imu`: `APE RMSE 0.248 m`
- `bag1_front`, `imu`: `APE RMSE 0.251 m`
- `bag6_front`, `no_imu`: `APE RMSE 0.422 m`
- `bag6_front`, `imu`: `APE RMSE 0.365 m`

The benchmark wrapper therefore auto-selects `rate=1.0` when `--use-imu=true`
and `--rate` is omitted. To validate the same A/B automatically on the default
front-lidar cases, run:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg
```

That matrix runs both `no_imu` and `imu` at `rate=1.0` for determinism and
writes per-case outputs plus:

- `packet_imu_deskew_validation.md`
- `packet_imu_deskew_validation.json`

The default acceptance criteria are:

- `no_imu` path coverage >= `0.95`
- `imu` path coverage >= `0.95`
- `imu_rmse / no_imu_rmse <= 1.10`
- `imu_matched_poses / no_imu_matched_poses >= 0.80`
deskew and `34.089 m` with `--imu-rotation-use-orientation false`. That is why
the public packet path still keeps `--use-imu false` by default.

If a bag carries NavSatFix messages whose header stamps do not track ROS time,
the backend now falls back to receive time when the skew exceeds
`gnss_header_stamp_max_skew_sec` (default `30 s`). That makes `all-sensors-bag6`
attach GNSS edges again, but its native `/gnss/fix` still disagrees with the
`GSOF49` reference enough that it is better suited to georeferenced smoke tests
than to clean GNSS cross-validation.

If you still want to test packet-based IMU deskew with a real static TF, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-imu true \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --robot-frame-id base_link \
  --imu-frame-id base_link \
  --verify-map
```

That path uses:

- `convert_applanix_gsof_to_imu_bag.py`
- `extract_static_transform_from_bag.py`
- `PointCloud2.time`-based deskew in `scanmatcher`
- `--imu-rotation-use-orientation false` for the gyro-only rotation variant

To turn the same real open-data path into a benchmark artifact with
`traj_raw.tum`, `traj_corrected.tum`, and `metrics.json`, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
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
