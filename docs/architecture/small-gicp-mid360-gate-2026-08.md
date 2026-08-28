# SMALL_GICP / SMALL_VGICP MID-360 gate

Date: 2026-08-21

Status: full MID-360 frontend compatibility gate passed; production promotion remains opt-in / No-Go

## Scope and decision

This receipt compares the legacy same-translation-unit scanmatcher path with
the host-resident registration adapter for both optional `small_gicp`
selectors.  It is a frontend-only replay using the real
`ScanMatcherComponent`; it exports the received world-frame submaps to PCD and
does not run `graph_based_slam` or colored-map authoring.

For each selector, the legacy and host pairs produced byte-identical
trajectories, submap streams, and maps across two full 2,772-scan runs; the
two runs on each side were also identical.  The three-run geometric
map-quality reports were identical for the representative PCD of each
selector.  This closes the MID-360 compatibility/non-regression evidence for
the host-resident Small adapters.  It is not an absolute accuracy claim:
the pinned dataset has no paired ground truth, so APE is explicitly not
evaluated here.

The production default is unchanged.  `small_gicp` is still optional, the
host resolver is offline opt-in, and this receipt does not close the separate
independent external-DSO/ODR promotion gate.

## Input and isolated build

| item | value |
| --- | --- |
| rosbag2 | `/media/sasaki/aiueo1/datasets/mid360_public/driving_slam_mid360/extracted/rosbag2_2024_04_16-14_17_01/rosbag2_2024_04_16-14_17_01` |
| LiDAR topic | `/livox/lidar` |
| IMU topic | `/livox/imu` |
| bag duration | 277.166836670 s |
| LiDAR / IMU messages | 2,772 / 55,435 |
| bag metadata SHA-256 | `65d66875f49248e38ff14d80e6e749fb50606f6f80bd4be337160e3752691e9a` |
| parameters | `lidarslam/param/lidarslam.yaml` |
| parameter SHA-256 | `50a30f4442d450d2597e9c90a720ed249ee9f89b6f76b8b4f02aa9cea672d061` |
| paired GT | none in the pinned dataset |
| source HEAD | `0c08b58f8524ea8ee5288982ca4a1b86450161b2` (working tree changes retained) |

The dependency-enabled build used a read-only extraction of the Jazzy
`ros-jazzy-small-gicp-vendor` package under
`/tmp/small-gicp-vendor.7YYK8k/extracted/opt/ros/jazzy/opt/small_gicp_vendor`
and the isolated install space
`/tmp/small-gicp-build.2auPmA/install`.  The normal system installation was
not modified.  Before each run, the normal workspace was sourced for
`graph_based_slam`, then the isolated install was sourced last;
`ros2 pkg prefix scanmatcher` resolved to
`/tmp/small-gicp-build.2auPmA/install/scanmatcher`.

Each run used the same command shape:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /tmp/small-gicp-build.2auPmA/install/setup.bash
/usr/bin/time -v env ROS_DOMAIN_ID=<private-domain> ROS_LOCALHOST_ONLY=1 \
  ros2 run scanmatcher scan_matcher_offline_runner --ros-args \
  --disable-rosout-logs \
  --params-file lidarslam/param/lidarslam.yaml \
  -p async_map_update:=false \
  -p bag_path:=<pinned-bag> \
  -p cloud_topic:=/livox/lidar -p imu_topic:=/livox/imu \
  -p ndt_num_threads:=1 -p registration_method:=<SMALL_GICP|SMALL_VGICP> \
  -p max_clouds:=0 -p output_dir:=<run-dir> -p map_output_path:=<run-dir>/map.pcd
```

The host-resident runs additionally set
`registration_plugin_enable:=true` and selected
`lidarslam_builtin/SmallGicpPcl` or `lidarslam_builtin/SmallVGicpPcl`.  The
legacy runs left plugin injection disabled.  The method, parameter file,
target preparation, map export, and `ndt_num_threads:=1` were otherwise
identical.  Run domains were 201–208 and executions were sequential.

Raw runner logs and artifacts remain outside the repository in
`/tmp/small-gicp-mid360-full.4nLxP7`.

## Trajectory, submaps, and map bytes

Every run produced 2,772 poses.  The submap CSV contains 295 submaps for GICP
and 372 for VGICP (296 and 373 lines including the CSV header).  The hashes
below are full MD5 values; run 1 and run 2 were equal for every row, and the
legacy and host rows were equal for the same selector.

| selector | side | trajectory MD5 | submaps MD5 | map MD5 | map bytes | map SHA-256 |
| --- | --- | --- | --- | --- | ---: | --- |
| `SMALL_GICP` | legacy same-TU | `2e36e8d58cde280f2d12e7cad5cf850a` | `baaa92dcee874e7047a753e5bb7052de` | `947df4b495f92400137cb6a52c9a22ae` | 39,676,100 | `684d2983215ac76d91c108eec82916933e3d6854728e85104f4d1d5479644115` |
| `SMALL_GICP` | host `lidarslam_builtin/SmallGicpPcl` | `2e36e8d58cde280f2d12e7cad5cf850a` | `baaa92dcee874e7047a753e5bb7052de` | `947df4b495f92400137cb6a52c9a22ae` | 39,676,100 | `684d2983215ac76d91c108eec82916933e3d6854728e85104f4d1d5479644115` |
| `SMALL_VGICP` | legacy same-TU | `c6b1d03e2ced7acc694d2aa8880f070b` | `c975d42481af07077484109003069f27` | `f48f8e4c963675f7b57ca90ab7698647` | 49,469,494 | `ac7fd49dab3d0ea8f59c7c22c47cd235eac1c6d21c94aea6d0a677714e970b77` |
| `SMALL_VGICP` | host `lidarslam_builtin/SmallVGicpPcl` | `c6b1d03e2ced7acc694d2aa8880f070b` | `c975d42481af07077484109003069f27` | `f48f8e4c963675f7b57ca90ab7698647` | 49,469,494 | `ac7fd49dab3d0ea8f59c7c22c47cd235eac1c6d21c94aea6d0a677714e970b77` |

All `cmp` checks succeeded:

```text
GICP:  legacy run1 == run2; host run1 == run2; legacy == host
VGICP: legacy run1 == run2; host run1 == run2; legacy == host
          (trajectory_frontend.tum, submaps_frontend.csv, map.pcd)
```

## APE status

The pinned `driving_slam_mid360` artifact contains no paired trajectory ground
truth.  No substitute trajectory, cross-validation reference, or README
comparison was used, and `ape_from_tum.py` was not run.  APE is therefore
**not evaluated**, rather than treated as zero or inferred from hash equality.

## Pose-acceptance diagnostics

The representative legacy run logged 538 `POSE_REJECT` decisions for
`SMALL_GICP` and 332 for `SMALL_VGICP`.  The host paths produced the same
trajectory, submap, and map bytes, so this is not an adapter regression.
However, the rejection counts and missing paired GT prevent the compatibility
result from being treated as an absolute tracking-accuracy pass.  They are a
separate tuning/evaluation concern for any future default-method proposal.

## Geometric map quality

The existing geometric evaluator was run on one representative run-1 PCD per
selector.  Legacy and host PCDs were already byte-identical, so this avoids
duplicating the same map while preserving a three-run evaluator determinism
check:

```bash
bash scripts/run_map_quality_check.sh --setup install/setup.bash \
  --input <run>/map.pcd --output-dir <quality-dir> \
  --runs 3 --downsample 0.1
```

No threshold profile was passed.  The colored profile is not compatible with
this PCD-only evaluator, so no colored-map claim is made.  Both reports were
byte-identical across all three evaluator runs and matched between the
legacy/host PCDs for the same selector.

| selector | report SHA-256 | report MD5 | input / evaluated points | mean entropy [nats] | valid fraction | patches | thickness RMS mean / p95 [m] | planar coverage | occupied voxels |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `SMALL_GICP` | `78713a1480fdaaae439ff8fdcade3382fd669aa690386f56933127c68aae0099` | `4096fbb712ff465f8e973d416e7f2b33` | 2,738,983 / 2,339,162 | -1.540971384 | 0.891818951 | 42,862 | 0.049749374 / 0.108779698 | 0.458647584 | 163,893 |
| `SMALL_VGICP` | `02216af1c39602c08dba988f5ba7a342990e413a99be9bef253848b72d72cadd` | `2ebf0783a983267e92c20a25f8773c56` | 3,414,165 / 2,846,164 | -1.542615915 | 0.913876361 | 50,469 | 0.047897479 / 0.107767059 | 0.475914951 | 172,621 |

This is a real merged world-frame PCD evaluation, not a trajectory or submap
hash used as a map-quality proxy.

## Wall time and peak RSS

`/usr/bin/time -v` was collected for every replay.  RTF is wall time divided
by the 277.166836670 s bag duration; medians are across the two runs.

| selector / side | wall run 1 / run 2 [s] | median wall [s] | median RTF | peak RSS run 1 / run 2 [KiB] | max RSS [KiB] |
| --- | ---: | ---: | ---: | ---: | ---: |
| GICP legacy | 252.04 / 251.95 | 252.00 | 0.909182 | 754,772 / 751,356 | 754,772 |
| GICP host | 237.93 / 239.42 | 238.68 | 0.861124 | 740,804 / 754,416 | 754,416 |
| VGICP legacy | 158.34 / 157.01 | 157.68 | 0.568881 | 881,256 / 880,648 | 881,256 |
| VGICP host | 144.31 / 148.65 | 146.48 | 0.528490 | 839,248 / 895,548 | 895,548 |

Relative to the corresponding legacy median, host wall time changed by -5.29%
for GICP and -7.10% for VGICP.  Maximum RSS changed by -0.05% and +1.62%,
respectively.  These are two-run characterization measurements, not a
cross-machine benchmark claim.

## Gate interpretation and remaining work

This closes the MID-360 full-replay compatibility evidence for the
host-resident SMALL adapters: two-run determinism, cross-construction byte
identity, and three-run report-only geometric map-quality equality all pass.
APE is unassessed because this pinned dataset has no GT, and the pose-rejection
diagnostics above remain unresolved as an absolute-quality gate.  Neither
selector is promoted to the live default.

Remaining promotion gates are:

- keep `small_gicp` optional and absent from normal manifests/install spaces;
- perform an independent external-DSO/ODR binding check before treating a
  pluginlib DSO as production-equivalent;
- pin the vendor/toolchain matrix and define the absolute accuracy/profile
  policy with a dataset that has paired ground truth.

The production default and README comparison claims therefore remain unchanged.
