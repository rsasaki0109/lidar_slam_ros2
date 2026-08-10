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
- `first_map_validation_receipt.json` and `.md`, a privacy-bounded,
  issue-ready proof summary derived from the finalized run evidence;
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
The flagship installed workflow is also exercised by the
[pinned real-data E2E gate](real-data-e2e.md).

## Official entrypoints

These are the only three beginner-facing product workflows. Other scripts and ROS
launch files are advanced, benchmark, migration, or research interfaces.

| Goal | Official command | Contract |
| --- | --- | --- |
| Try the fixed public demo without a ROS workspace | [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace) | Downloads the tracked MID-360 demo with progress and writes a user-owned headless map bundle |
| Map your own compatible rosbag2 | `lidarslam-map run <rosbag2_dir> --guided` for people; `lidarslam-map run <rosbag2_dir> --output-dir <dir>` for automation | Preflights the bag, selects the same maintained profile, runs headless, verifies and diagnoses the output; guided mode adds explanation and confirmation only |
| Reproduce the fixed source-workspace quickstart | `bash scripts/download_ntu_viral_tnp01.sh && bash scripts/run_autoware_quickstart.sh` | Runs the tracked NTU VIRAL path and opens the bounded viewer flow |

For automation, use an explicit `--output-dir`; run `lidarslam-map doctor`
before a long run. The repo-local `./scripts/lidarslam` wrapper and installed
`ros2 run lidarslam lidarslam-cli` shim expose the same own-bag contract and do
not add beginner workflows. Installation details are in
[Distribution and installed CLI](distribution.md).

The `run --guided` mode is the recommended human path: it keeps the exact command
and profile selection visible before execution. Add `--yes` for a non-terminal
launcher, or use `run` directly when a script must remain non-interactive.

After a successful run, `lidarslam-map view <output_dir>` can open the
completed output in Autoware or Foxglove. Viewing is optional post-processing,
and its failure does not alter the map-run manifest.

## Input contract

The general product path accepts a rosbag2 directory containing
`metadata.yaml`. The primary maintained profile requires:

- `sensor_msgs/msg/PointCloud2`;
- `sensor_msgs/msg/Imu`;
- valid, monotonic timestamps;
- a known transform between LiDAR, IMU and base frames;
- enough free disk space for the input bag, intermediate clouds and output map;
  `run` enforces a 5 GiB output-filesystem reserve by default and accepts a
  deliberately sized `--min-free-space-gib` override.

Topic presence is necessary but not sufficient. A bag with the right message
types can still require a sensor-specific point-time field, calibration,
frame override or parameter profile. Before recommending RKO-LIO, the preflight
reads the first record on the selected `PointCloud2` topic and verifies
FLOAT32 `x`, `y`, and `z` plus a supported per-point timestamp field named
`t`, `timestamp`, `time`, or `stamps`. It does not certify timestamp units,
calibration, or TF connectivity. Before launch it also checks up to 100,000
`PointCloud2` and `Imu` records per selected topic for invalid or decreasing
`header.stamp` values. A bounded clean sample is not a full-bag monotonicity
proof, and the machine report distinguishes `sampled` from `passed`.

## Support tiers

| Tier | Scope | Evidence and expectation |
| --- | --- | --- |
| Validated | Tracked MID-360 Docker demo; NTU VIRAL Ouster/VN-100 quickstart | Fixed public input, parameters and expected artifacts; release documentation records measured accuracy |
| Maintained-compatible | Generic record-verified `PointCloud2 + Imu` through the beginner wrapper on ROS 2 Humble and Jazzy | Built and tested in CI; the operator supplies correct timestamp units, calibration, frames and sensor parameters |
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
Independent onboarding evidence follows the
[first-map validation contract](external-first-map-validation.md); its tracked
ledger remains separate from maintainer-operated demos and CI evidence.
The cross-phase completion claim is generated by the
[v1.0 readiness audit](v1-readiness.md), which fails closed over every product
dimension and the external ledger. Tagged publication and recovery assets are
separately verified by the read-only published-release audit; a GitHub Release
or image tag existing without its complete, cross-consistent evidence set is
not sufficient.
