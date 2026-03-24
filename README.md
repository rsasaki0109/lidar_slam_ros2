lidarslam_ros2
====

ROS 2 LiDAR SLAM focused on permissive-license pointcloud-map generation and
Autoware integration.

## Recommended Public Workflow

The recommended and dogfooded path in this repository is:

- frontend: `RKO-LIO`
- backend: `graph_based_slam`
- output: Autoware-compatible `pointcloud_map/` and `map_projector_info.yaml`

This is the path exercised in the public quickstart, benchmark flow, and
release/readiness gate.

## Why This Repo

- permissive default path: `graph_based_slam` (BSD-2-Clause), `scanmatcher`
  (project-local), `RKO-LIO` (MIT), `DLIO` (MIT), `FAST_GICP` (BSD-3-Clause)
- Autoware pointcloud-map flow is dogfooded end-to-end
- default benchmark path is tracked on `NTU VIRAL`
- current long-loop evidence is tracked on `MID360`
- optional GNSS georeferencing writes `map_projector_info.yaml`
- GPL-free Scan Context place recognition is available in
  `graph_based_slam`

## Quickstart

Build and run the default local checks:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
bash scripts/run_default_ci_checks.sh
```

Run the fixed public Autoware pointcloud-map quickstart:

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_autoware_quickstart.sh
```

## Docs

- [Autoware Quickstart](docs/autoware-quickstart.md)
- [Operator Workflows](docs/workflows.md)
- [Benchmarking And Release Gate](docs/benchmarking.md)
- [Comparison](docs/comparison.md)
- [v0.2.0 Release Notes](docs/releases/v0.2.0.md)
- [Contributing](CONTRIBUTING.md)
- [Changelog](CHANGELOG.md)
- [Releasing](RELEASING.md)

## Current Snapshot

| Dataset | Published configuration | Reference kind | APE RMSE (m) | Autoware map verify |
| --- | --- | --- | --- | --- |
| `NTU VIRAL tnp_01` | current default | `ground_truth` | `0.952` | `PASS` |
| `NTU VIRAL tnp_01` | best observed | `ground_truth` | `0.870` | `PASS` |
| `MID360` | current default | `cross_validation` | `3.641` | `PASS` |
| `MID360` | best observed | `cross_validation` | `3.590` | `PASS` |

More detail lives in [docs/comparison.md](docs/comparison.md),
[docs/benchmarking.md](docs/benchmarking.md), `output/benchmark_summary.md`,
and `output/latest_report.html`.

## Main Entrypoints

Run the public Autoware quickstart:

```bash
bash scripts/run_autoware_quickstart.sh
```

Run `RKO-LIO + graph_based_slam` directly:

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/os_cloud_node/imu
```

Save the current map:

```bash
ros2 service call /map_save std_srvs/srv/Empty
```

Run the standard benchmark path:

```bash
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
```

Run the local readiness gate:

```bash
bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10
```

## License Policy

The default public workflow is restricted to permissive-license components.

- `graph_based_slam`: BSD-2-Clause
- `scanmatcher`: project-local frontend/backend code in this repository
- `RKO-LIO`: MIT
- `DLIO`: MIT
- `FAST_GICP`: BSD-3-Clause
- built-in `Scan Context`: implemented locally to avoid GPL dependencies

`Thirdparty/lio-sam` is excluded from default `colcon` package discovery via
`COLCON_IGNORE`.

## Support Matrix

| ROS 2 distro | Ubuntu | Scope |
| --- | --- | --- |
| Humble | 22.04 | default workflow build and package tests in CI |
| Jazzy | 24.04 | default workflow build and package tests in CI; Autoware pointcloud-map dogfood exercised locally |

## Quality Gates

The main checks for the public path are:

- `bash scripts/run_default_ci_checks.sh`
- `python3 scripts/verify_autoware_map.py <pointcloud_map_dir>`
- `bash scripts/run_autoware_quickstart.sh`
- `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20`
- `bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10`

For the command-level details, parameter-file pointers, and Autoware map output
notes, see [docs/workflows.md](docs/workflows.md).
