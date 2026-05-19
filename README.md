# lidarslam_ros2

[![CI](https://github.com/rsasaki0109/lidarslam_ros2/actions/workflows/main.yml/badge.svg?branch=develop)](https://github.com/rsasaki0109/lidarslam_ros2/actions/workflows/main.yml)
[![License: BSD-2-Clause](https://img.shields.io/badge/License-BSD--2--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![ROS 2: Humble | Jazzy](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)](#support-matrix)

ROS 2 LiDAR SLAM. Frontend is `RKO-LIO` (MIT), backend is `graph_based_slam` (BSD-2). Output is an Autoware-compatible `pointcloud_map/` directory plus `map_projector_info.yaml`. No GPL components on the default workflow.

`develop` tracks the current v2 alpha line. Latest tagged public beta: [v0.2.2 Release Notes](docs/releases/v0.2.2.md).

![Autoware-compatible pointcloud_map rendered by Autoware map loaders](lidarslam/images/autoware_map_loader_proof.png)

## Install

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If you already cloned without `--recursive`:

```bash
git -C src/lidarslam_ros2 submodule update --init --recursive
```

## Quickstart

The public quickstart downloads NTU VIRAL `tnp_01` (~580 s outdoor bag) and runs RKO-LIO + graph_based_slam end to end, producing an Autoware-loadable map.

```bash
cd src/lidarslam_ros2
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_autoware_quickstart.sh
python3 scripts/verify_autoware_map.py output/.../pointcloud_map
```

`verify_autoware_map.py` prints `map_verify: PASS` when the map can be loaded by Autoware map loaders.

For an arbitrary rosbag2:

```bash
bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2
```

## Running RKO-LIO + graph_based_slam Directly

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/os_cloud_node/imu

ros2 service call /map_save std_srvs/srv/Empty
```

To filter likely dynamic objects in the saved map, set `use_dynamic_object_filter: true` and tune `dynamic_object_filter_voxel_size`, `dynamic_object_filter_min_observations`, `dynamic_object_filter_temporal_window`, and `dynamic_object_filter_max_range_from_sensor_m` before calling `/map_save`. On Leo Drive `bag6` this cuts saved points by about 50 % while map verification still passes.

![Dynamic-object filter size and verification summary](lidarslam/images/dynamic_object_filter_bag6_summary.svg)

## Required input topics for the main public path

| Launch path | Required | Optional |
| --- | --- | --- |
| `lidarslam rko_lio_slam.launch.py` | LiDAR `PointCloud2` on `lidar_topic`, IMU on `imu_topic` | `NavSatFix` on `gnss_topic` (default `/gnss/fix`) when `use_gnss:=true` |
| `lidarslam lidarslam.launch.py` | `PointCloud2` on `input_cloud`, TF from `robot_frame_id` to the LiDAR frame | IMU on `imu_topic` when `scanmatcher use_imu:=true`, odom TF when `scanmatcher use_odom:=true`, GNSS when backend `use_gnss:=true` |
| `graph_based_slam graphbasedslam.launch.py` | `lidarslam_msgs/MapArray` on `map_array` | IMU on `/imu` when `use_imu_preintegration:=true`, GNSS when `use_gnss:=true` |

The backend GNSS topic is configurable via `gnss_topic` (default `/gnss/fix`). Wheel- or vehicle-speed fusion is not in the public path yet. Inspect covariance with `scripts/inspect_navsatfix_covariance.py`; Applanix conversion details are in [docs/workflows.md](docs/workflows.md).

## Features

- Loop closure with built-in Scan Context (re-implemented locally, GPL-free), plus opt-in BEV / SOLiD / STD/BTC-style Triangle descriptors.
- Optional MIT-licensed 3D-BBS loop verification (vendored, disabled at runtime by default).
- Optional GNSS georeferencing writes `map_projector_info.yaml` for direct Autoware loading; GNSS edges can use covariance-based weighting.
- AWSIM to Autoware autonomous-driving demo on the map you just built, with lanelet2 auto-generation from the SLAM trajectory (multi-segment, shared boundary nodes, structurally validated before Autoware loads it).
- NIS-driven auto-scale for the adjacent-edge information weight so the backend self-tunes between strong and weak LIO datasets.
- Save-time dynamic-object filter (Leo Drive bag6 example above).
- Report helpers for benchmarks, GNSS, cleanup, dynamic filtering, place recognition, and submission bundles.

## Benchmarks and release gate

Per-dataset pass / target thresholds live in `scripts/release_profiles.yaml`. The gate emits `WARN` for research-track datasets (`report_only_until: v0.4`) and only release-track profiles without `report_only_until` can block a release.

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
bash scripts/run_release_readiness_checks.sh --fail-on-profiles
```

The legacy single-threshold mode is still supported:

```bash
bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10
```

Release track:
- NTU VIRAL `tnp_01` outdoor GT (current default 0.952 m, best 0.870 m; gate `WARN`, `report_only_until: v0.4`).
- KITTI Odometry 00 / 05 / 07 LO baseline non-regression (`bash scripts/run_kitti_00_05_07_report.sh`).
- Autoware-compatible `pointcloud_map` + lanelet2 + AWSIM to Autoware E2E demo.

Research track (`report_only_until: v0.4`, does not block release):
- MID-360 vs GLIM cross-validation (current default 3.641 m, best 3.590 m; solid-state research dataset).
- Leo Drive applanix/velodyne open-data cross-validation.

Detail in [docs/comparison.md](docs/comparison.md), [docs/benchmarking.md](docs/benchmarking.md), `scripts/release_profiles.yaml`, `output/benchmark_summary.md`, and `output/latest_report.html`.

## AWSIM autonomous-driving pipeline

```bash
bash scripts/test_awsim_setup.sh
bash scripts/run_awsim_selfmade_map_demo.sh
```

For map packaging, lanelet2 generation, and terminal-by-terminal bringup see [docs/awsim-autonomous-driving-tutorial.md](docs/awsim-autonomous-driving-tutorial.md).

## Docs

- [AWSIM autonomous-driving tutorial](docs/awsim-autonomous-driving-tutorial.md)
- [Autoware-compatible map authoring](docs/autoware-map-authoring.md)
- [Autoware quickstart](docs/autoware-quickstart.md)
- [Autoware Foxglove](docs/autoware-foxglove.md)
- [Operator workflows](docs/workflows.md)
- [Benchmarking and release gate](docs/benchmarking.md)
- [Comparison](docs/comparison.md)
- [v0.2.2 release notes](docs/releases/v0.2.2.md)
- [Contributing](CONTRIBUTING.md)
- [Changelog](CHANGELOG.md)
- [Releasing](RELEASING.md)

Preview the doc site locally with `python3 -m mkdocs serve`.

## Support matrix

| ROS 2 distro | Ubuntu | Scope |
| --- | --- | --- |
| Humble | 22.04 | default workflow build and package tests in CI |
| Jazzy  | 24.04 | default workflow build and package tests in CI; Autoware dogfood exercised locally |

## License policy

The default public workflow excludes GPL-only frontend/backend components. `graph_based_slam` is BSD-2-Clause; `RKO-LIO`, `DLIO`, and the optional vendored `3D-BBS` are MIT; `FAST_GICP` is BSD-3-Clause; the built-in Scan Context is implemented locally. `Thirdparty/lio-sam` and `Thirdparty/3d_bbs` are excluded from direct `colcon` package discovery via `COLCON_IGNORE`.

## Quality gates

```bash
bash scripts/run_default_ci_checks.sh
python3 scripts/verify_autoware_map.py <pointcloud_map_dir>
bash scripts/run_autoware_quickstart.sh
bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20
bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10
```

Command-level details, parameter pointers, and Autoware map output notes are in [docs/workflows.md](docs/workflows.md).
