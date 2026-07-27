# Product Contract

`lidarslam_ros2` is an offline ROS 2 map-authoring product. Its maintained
product promise is:

> Given a compatible rosbag2 and an explicit sensor configuration, produce a
> diagnosable, reproducible point-cloud map bundle that can be checked before
> it is loaded by Autoware-compatible map tooling.

This contract separates the supported product surface from the repository's
larger research surface. A feature existing in the repository does not by
itself make that feature part of the product contract.

## Supported outcome

A successful product-path run produces an output directory containing:

- `pointcloud_map/` and `pointcloud_map_metadata.yaml`;
- `map_projector_info.yaml`;
- `traj_corrected.tum`;
- `verify_autoware_map.log`;
- `autoware_map_diagnosis.md`;
- `autoware_map_diagnosis.json`;
- `run_manifest.json`, with input/output checksums and execution identity;
- `lanelet2_map.osm` when lanelet generation is enabled and succeeds.

`verify_autoware_map.py` must report `PASS` before the point-cloud bundle is
treated as valid. Generated lanelets are a starting point for map authoring,
not surveyed road semantics; review them before use.

The versioned machine-readable contracts are published with the
[golden-path CLI documentation](golden-path-cli.md#versioned-json-contracts).
Existing output directories are immutable to the runner: operators must choose
a new name, and in-progress work is isolated under a `.partial` sibling. The
only exception is explicit `--resume` of an incomplete schema-v2 lifecycle:
the runner revalidates the original execution identity and completes terminal
post-processing without rerunning SLAM or replacing map artifacts.
Signal handling, preserved failure state, recovery actions, and still-open
Phase 3 gates are tracked in
[Operational reliability](operational-reliability.md).

## Official entrypoints

These are the only beginner-facing product entrypoints. Other scripts and ROS
launch files are advanced, benchmark, migration, or research interfaces.

| Goal | Official command | Contract |
| --- | --- | --- |
| Try the fixed public demo without a ROS workspace | `docker run --rm -v "$PWD/lidarslam_output:/lidarslam_ws/output" ghcr.io/rsasaki0109/lidar_slam_ros2:humble` | Downloads the tracked MID-360 demo and writes a headless map bundle |
| Map your own compatible rosbag2 | `lidarslam-map run <rosbag2_dir> --output-dir <dir>` | Preflights the bag, selects a compatible maintained profile, runs headless, verifies and diagnoses the output |
| Reproduce the fixed source-workspace quickstart | `bash scripts/download_ntu_viral_tnp01.sh && bash scripts/run_autoware_quickstart.sh` | Runs the tracked NTU VIRAL path and opens the bounded viewer flow |

For automation, use an explicit `--output-dir`; run `lidarslam-map doctor`
before a long run. The repo-local `./scripts/lidarslam` wrapper and installed
`ros2 run lidarslam lidarslam-cli` shim expose the same own-bag contract and do
not add beginner workflows. Installation details are in
[Distribution and installed CLI](distribution.md).

## Input contract

The general product path accepts a rosbag2 directory containing
`metadata.yaml`. The primary maintained profile requires:

- `sensor_msgs/msg/PointCloud2`;
- `sensor_msgs/msg/Imu`;
- valid, monotonic timestamps;
- a known transform between LiDAR, IMU and base frames;
- enough free disk space for the input bag, intermediate clouds and output map.

Topic presence is necessary but not sufficient. A bag with the right message
types can still require a sensor-specific point-time field, calibration,
frame override or parameter profile. The preflight report describes detected
capabilities; it does not certify calibration.

## Support tiers

| Tier | Scope | Evidence and expectation |
| --- | --- | --- |
| Validated | Tracked MID-360 Docker demo; NTU VIRAL Ouster/VN-100 quickstart | Fixed public input, parameters and expected artifacts; release documentation records measured accuracy |
| Maintained-compatible | Generic `PointCloud2 + Imu` through the beginner wrapper on ROS 2 Humble and Jazzy | Built and tested in CI; the operator supplies correct calibration, frames and sensor parameters |
| Evaluation | GNSS/Applanix packet paths, radar degeneracy presets, HILTI, coloured maps and optional loop detectors | Reproducible research or benchmark evidence exists, but it is not a universal plug-and-play hardware guarantee |
| Out of scope | Safety-certified localization, autonomous-driving validation, arbitrary proprietary bags, Windows native builds | No product support promise |

See [Support](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/SUPPORT.md)
for reporting requirements and
[Operator Workflows](workflows.md) for advanced configuration.

## Compatibility and change policy

- Humble on Ubuntu 22.04 and Jazzy on Ubuntu 24.04 are the supported source
  build targets.
- The latest tagged `0.x` release and `develop` receive fixes. Older `0.x`
  releases may require upgrading before a fix is provided.
- Product entrypoint flags and output filenames must not be removed silently.
  Deprecations are documented in release notes for at least one tagged release.
- New algorithms begin default-off. Promotion to a maintained preset requires
  paired real-data evidence and explicit negative checks.
- Offline determinism claims apply only to the runners and artifacts named in
  the corresponding release evidence; they do not imply bitwise-identical
  live ROS scheduling.

## Non-goals

The product contract does not promise:

- universal best accuracy on every dataset;
- automatic calibration of an unknown sensor rig;
- production localization or fail-operational autonomy;
- surveyed lanelet semantics;
- compatibility with every research script or optional dependency;
- that a verified file bundle is safe for deployment without operator review.

## Evidence and escalation

Accuracy claims and gates live in
[Benchmarking and Release Gate](benchmarking.md) and
[Comparison](comparison.md). Security-sensitive reports follow
[SECURITY.md](https://github.com/rsasaki0109/lidar_slam_ros2/security/policy);
usage questions and reproducible defects follow
[SUPPORT.md](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/SUPPORT.md).
