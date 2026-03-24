# Comparison

This page is the public comparison snapshot for `lidarslam_ros2 v0.2.0`.

It is intentionally scoped to workflows that are actually exercised in this
repository. It is not trying to be a universal ranking of every LiDAR SLAM
system.

## Capability Comparison

| Workflow | Role in this repo | License stance in the public path | Frontend / backend shape | Loop closure in the documented path | Autoware pointcloud-map path |
| --- | --- | --- | --- | --- | --- |
| `lidarslam_ros2` default | recommended public workflow | non-GPL default | `RKO-LIO` frontend + `graph_based_slam` backend | yes | yes |
| `RKO-LIO` raw | odometry baseline | non-GPL default | LIO frontend only | no | no |
| `KISS-ICP` baseline | comparison baseline | external comparison only | LiDAR odometry only | no | no |
| `LIO-SAM` | research reference | excluded from the default release path | tightly coupled factor-graph SLAM | yes | no supported path in this repo |

## Local Benchmark Snapshot

These numbers come from local artifacts currently checked under `output/`.

| Dataset | Published configuration | Reference kind | APE RMSE (m) | Autoware map verify | Notes |
| --- | --- | --- | --- | --- | --- |
| `NTU VIRAL tnp_01` | current default | `ground_truth` | `0.952` | `PASS` | default public benchmark path |
| `NTU VIRAL tnp_01` | best observed | `ground_truth` | `0.870` | `PASS` | loop-gated backend run |
| `MID360` | current default | `cross_validation` | `3.641` | `PASS` | current documented tuned path |
| `MID360` | best observed | `cross_validation` | `3.590` | `PASS` | rerun with the same tuned backend family |

Source artifacts:

- `output/benchmark_summary.md`
- `output/latest_report.html`
- `output/stress_validation_report_20260325.md`

## Current Default Position

The public `v0.2.0` position is:

- default workflow: `RKO-LIO + graph_based_slam`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- release gate: `bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10`
- current MID360 default tuning:
  `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
  `loop_edge_dedup_index_window=20`, `loop_edge_info_weight=200`

## Interpretation

Safe claims:

- the default path is benchmarked on `NTU VIRAL`
- the pointcloud-map flow is dogfooded into Autoware
- the backend has current long-loop evidence on `MID360`

Unsafe claims:

- that this repo is already the universal winner on every dataset
- that the current default path is fully validated against every aggressive
  motion edge case
- that lanelet generation is part of the supported release scope

## Release Scope Reminder

`v0.2.0` is a public `v2 beta` release for:

- ROS 2 pointcloud-map generation
- non-GPL default workflow
- Autoware pointcloud-map loading

It is not yet claiming full production maturity.
