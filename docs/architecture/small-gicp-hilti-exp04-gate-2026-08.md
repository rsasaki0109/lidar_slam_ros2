# SMALL_GICP / SMALL_VGICP HILTI exp04 gate

Date: 2026-08-21
Status: full HILTI frontend compatibility gate passed; production promotion remains opt-in / No-Go

## Scope and decision

This receipt compares the legacy same-translation-unit scanmatcher path with
the host-resident registration adapter for both optional `small_gicp`
selectors.  It is a frontend-only replay: it uses the real
`ScanMatcherComponent`, exports the received world-frame submaps to PCD, and
does not run `graph_based_slam` or colored-map authoring.

The HILTI gate passed for this slice.  For each selector, the legacy and host
pairs produced byte-identical trajectories, submap streams, and maps across
two full 1,258-scan runs; the two runs on each side were also identical.  APE
and geometric map-quality reports were identical within each selector.  This
is a compatibility/non-regression result, not an absolute accuracy claim: the
APE values are reported below and no README comparison claim is changed.

The production default is unchanged.  `small_gicp` is still optional, the
host resolver is offline opt-in, and this receipt does not close the separate
MID-360 or independent external-DSO/ODR promotion gates.

## Input and isolated build

| item | value |
| --- | --- |
| rosbag2 | `/media/sasaki/aiueo1/datasets/hilti2022/exp04_ros2` |
| LiDAR topic | `/hesai/pandar` |
| IMU topic | `/alphasense/imu` |
| bag duration | 125.814128037 s |
| LiDAR / IMU messages | 1,258 / 50,198 |
| bag metadata SHA-256 | `f256bd10ec4a65fec68ab91455108ba73ac3791043f81e05846be93922d21100` |
| parameters | `configs/hilti2022/lidarslam_competitive_v2.yaml` |
| parameter SHA-256 | `53312d748bc6f6ba8f12fab2a11490c5dc2bcbb5b722f318b48500237aac3e17` |
| sparse control-point GT | `/media/sasaki/aiueo1/datasets/hilti2022/exp04_construction_upper_level_gt.txt` |
| GT SHA-256 | `38cf516e51113254e4ae0207c790f740b19dee08665063e0d8df7bd277040c20` |
| source HEAD | `0c08b58f8524ea8ee5288982ca4a1b86450161b2` (working tree changes retained) |

The dependency-enabled build used a read-only extraction of the Jazzy
`ros-jazzy-small-gicp-vendor` package under
`/tmp/small-gicp-vendor.7YYK8k/extracted/opt/ros/jazzy/opt/small_gicp_vendor`
and the isolated install space
`/tmp/small-gicp-build.2auPmA/install`.  The normal system installation was
not modified.  Before each run, the normal workspace was sourced for
`graph_based_slam`, then the isolated install was sourced last; `ros2 pkg
prefix scanmatcher` resolved to
`/tmp/small-gicp-build.2auPmA/install/scanmatcher`.

Each run used the same command shape:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /tmp/small-gicp-build.2auPmA/install/setup.bash
/usr/bin/time -v env ROS_DOMAIN_ID=<private-domain> ROS_LOCALHOST_ONLY=1 \
  ros2 run scanmatcher scan_matcher_offline_runner --ros-args \
  --disable-rosout-logs \
  --params-file configs/hilti2022/lidarslam_competitive_v2.yaml \
  -p async_map_update:=false \
  -p bag_path:=/media/sasaki/aiueo1/datasets/hilti2022/exp04_ros2 \
  -p cloud_topic:=/hesai/pandar -p imu_topic:=/alphasense/imu \
  -p ndt_num_threads:=1 -p registration_method:=<SMALL_GICP|SMALL_VGICP> \
  -p max_clouds:=0 -p output_dir:=<run-dir> -p map_output_path:=<run-dir>/map.pcd
```

The host-resident runs additionally set
`registration_plugin_enable:=true` and selected
`lidarslam_builtin/SmallGicpPcl` or `lidarslam_builtin/SmallVGicpPcl`.  The
legacy runs left plugin injection disabled.  The registration method,
parameters, target preparation, map export, and `ndt_num_threads:=1` were
otherwise identical.  Run domains were 191–198 and executions were
sequential.

Raw runner logs and artifacts remain outside the repository in
`/tmp/small-gicp-hilti-exp04-full.5JfQ2B`.

## Trajectory, submaps, and map bytes

Every run produced 1,258 poses.  The submap CSV contains 26 submaps for GICP
and 38 for VGICP (27 and 39 lines including the CSV header).  The hashes below
are full MD5 values; run 1 and run 2 were equal for every row, and the legacy
and host rows were equal for the same selector.

| selector | side | trajectory MD5 | submaps MD5 | map MD5 | map bytes | map SHA-256 |
| --- | --- | --- | --- | --- | ---: | --- |
| `SMALL_GICP` | legacy same-TU | `c6b98f87d0411a1167a4b26e285e90fa` | `23bc8057ad3b22b1bcdb4c1868c886d0` | `3e6dd027db8937396eacc31c84c3fa77` | 6,229,590 | `051e1e48fe04970e5014592548cd5b0d363460c1aef052bcd709b4af2918d1c2` |
| `SMALL_GICP` | host `lidarslam_builtin/SmallGicpPcl` | `c6b98f87d0411a1167a4b26e285e90fa` | `23bc8057ad3b22b1bcdb4c1868c886d0` | `3e6dd027db8937396eacc31c84c3fa77` | 6,229,590 | `051e1e48fe04970e5014592548cd5b0d363460c1aef052bcd709b4af2918d1c2` |
| `SMALL_VGICP` | legacy same-TU | `f69ae783894db9e19a9c9af86c17d4a0` | `6d26d745e119e423df026875b5f351dd` | `718a4679bd8b3ad6b06949f2a9f8f4d4` | 8,300,217 | `bb64790cd8d46248c26bd6939428cb94e1f7b745d3c056eb23a8aa0f6b2d3925` |
| `SMALL_VGICP` | host `lidarslam_builtin/SmallVGicpPcl` | `f69ae783894db9e19a9c9af86c17d4a0` | `6d26d745e119e423df026875b5f351dd` | `718a4679bd8b3ad6b06949f2a9f8f4d4` | 8,300,217 | `bb64790cd8d46248c26bd6939428cb94e1f7b745d3c056eb23a8aa0f6b2d3925` |

The corresponding `cmp` checks were all successful:

```text
GICP:  legacy run1 == run2; host run1 == run2; legacy == host
VGICP: legacy run1 == run2; host run1 == run2; legacy == host
          (trajectory_frontend.tum, submaps_frontend.csv, map.pcd)
```

## Historical APE

Each trajectory was evaluated with the historical interpolation contract:

```bash
python3 scripts/ape_from_tum.py --interpolate --max-time-diff 3.0 \
  --ref /media/sasaki/aiueo1/datasets/hilti2022/exp04_construction_upper_level_gt.txt \
  --est <run>/trajectory_frontend.tum --out <run>/ape_historical_interpolate.txt
```

All eight evaluations paired 7/7 sparse GT points, rejected zero reference
points, and had a maximum time gap of 0.10001611709594727 s.  Values are the
same for both runs and both sides of each selector:

| selector | legacy RMSE [m] | host RMSE [m] | mean [m] | max [m] |
| --- | ---: | ---: | ---: | ---: |
| `SMALL_GICP` | 2.4603031262 | 2.4603031262 | 2.0223185756 | 5.1233999500 |
| `SMALL_VGICP` | 5.4281371179 | 5.4281371179 | 4.1206470365 | 9.7411837986 |

These sparse-trajectory numbers are evidence of equality between the two
construction paths, not a claim that either optional method meets an absolute
accuracy target.

## Geometric map quality

The existing geometric evaluator was run on each selector/side's run-1 PCD:

```bash
bash scripts/run_map_quality_check.sh --setup install/setup.bash \
  --input <run>/map.pcd --output-dir <quality-dir> \
  --runs 3 --downsample 0.1
```

No threshold profile was passed.  This is the common report-only comparison;
the colored HILTI profile is not compatible with this PCD-only evaluator, so
no colored-map claim is made.  Each report was byte-identical across its three
evaluator runs, and legacy/host reports matched for the same selector.  The
three evaluator runs used one representative PCD per selector because the
legacy and host PCDs were already byte-identical.

| selector | report MD5 | input / evaluated points | mean entropy [nats] | valid fraction | patches | thickness RMS mean / p95 [m] | planar coverage | occupied voxels |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `SMALL_GICP` | `d5db958c0a4ceba2824d9b47870fc5fd` | 422,178 / 289,007 | -1.414567095 | 0.872065382 | 4,259 | 0.063222265 / 0.111913923 | 0.537872785 | 19,809 |
| `SMALL_VGICP` | `19c22f6e8ff0e6dc04fa82bfaa2f731b` | 562,785 / 392,695 | -1.322258998 | 0.874757764 | 5,824 | 0.066757276 / 0.112348438 | 0.486041330 | 27,151 |

The report is a real merged world-frame PCD evaluation, not a trajectory or
submap hash used as a map-quality proxy.

## Wall time and peak RSS

`/usr/bin/time -v` was collected for every replay.  RTF is wall time divided
by the 125.814128037 s bag duration; medians are across the two runs.

| selector / side | wall run 1 / run 2 [s] | median wall [s] | median RTF | peak RSS run 1 / run 2 [KiB] | max RSS [KiB] |
| --- | ---: | ---: | ---: | ---: | ---: |
| GICP legacy | 152.79 / 139.85 | 146.32 | 1.162985 | 210,792 / 204,124 | 210,792 |
| GICP host | 137.46 / 135.80 | 136.63 | 1.085967 | 213,976 / 204,656 | 213,976 |
| VGICP legacy | 104.58 / 107.38 | 105.98 | 0.842354 | 257,168 / 252,884 | 257,168 |
| VGICP host | 101.08 / 111.16 | 106.12 | 0.843466 | 258,616 / 257,860 | 258,616 |

Relative to the corresponding legacy median, host wall time changed by -6.62%
for GICP and +0.13% for VGICP.  Maximum RSS changed by +1.51% and +0.56%,
respectively.  These are two-run characterization measurements, not a
benchmark claim.

## Gate interpretation and remaining work

This closes the HILTI exp04 full-replay compatibility evidence for the
host-resident SMALL adapters: two-run determinism, cross-construction byte
identity, historical APE equality, and report-only geometric map-quality
equality all pass.  It does not promote either selector to the live default.

Remaining promotion gates are:

- keep `small_gicp` optional and absent from normal manifests/install spaces;
- repeat the same full evidence on the pinned MID-360 sequence;
- perform an independent external-DSO/ODR binding check before treating a
  pluginlib DSO as production-equivalent;
- pin the vendor/toolchain matrix and decide the absolute accuracy/profile
  policy separately from this non-regression receipt.

The production default and README comparison claims therefore remain unchanged.
