# Comparison

This page is the public comparison snapshot for `lidarslam_ros2 v0.2.2`.

It is intentionally scoped to workflows that are actually exercised in this
repository. It is not trying to be a universal ranking of every LiDAR SLAM
system.

## Strategic Position

This repository is deliberately positioned as:

- a ROS 2 pointcloud-map authoring stack
- a benchmarkable mapping workflow
- a non-GPL public path for reusable map artifacts

It is not primarily positioned as:

- the smallest possible LiDAR odometry package
- a localization reliability research platform
- a universal winner on every SLAM benchmark

The intended differentiation is operational:

- generate pointcloud maps
- keep map metadata and georeference outputs usable
- verify saved bundles
- compare runs with tracked metrics and reports
- standardize submission artifacts for repeatable evaluation

That is the product layer this repository is trying to own.

## Capability Comparison

| Workflow | Role in this repo | License stance in the public path | Frontend / backend shape | Loop closure in the documented path | Pointcloud-map authoring / verification |
| --- | --- | --- | --- | --- | --- |
| `lidarslam_ros2` default | recommended public workflow | non-GPL default | `RKO-LIO` frontend + `graph_based_slam` backend | yes | yes |
| `RKO-LIO` raw | odometry baseline | non-GPL default | LIO frontend only | no | no |
| `KISS-ICP` baseline | comparison baseline | external comparison only | LiDAR odometry only | no | no |
| `LIO-SAM` | research reference | excluded from the default release path | tightly coupled factor-graph SLAM | yes | no supported path in this repo |

## Differentiators

The public differentiators currently exercised in this repository are:

- non-GPL default workflow
- saved-map verification tooling
- GNSS-aware `map_projector_info.yaml` export
- save-time dynamic-object cleanup
- tracked benchmark/report artifacts
- real open-data packet-path evidence
- a focused `map_authoring_report` that summarizes benchmark, georeference,
  cleanup, and fallback-path evidence in one place
- a standard submission-bundle helper that collects `pointcloud_map/`,
  `map_projector_info.yaml`, `metrics.json`, trajectories, logs, focused reports,
  and a generated `map_qa_summary.md`

Those are stronger differentiators for map authoring and evaluation than for
pure odometry novelty.

## Local Benchmark Snapshot

These numbers come from local artifacts currently checked under `output/`.

| Dataset | Published configuration | Reference kind | APE RMSE (m) | Autoware map verify | Notes |
| --- | --- | --- | --- | --- | --- |
| `NTU VIRAL tnp_01` | current default | `ground_truth` | `0.952` | `PASS` | default public benchmark path |
| `NTU VIRAL tnp_01` | best observed | `ground_truth` | `0.870` | `PASS` | loop-gated backend run |
| `MID360` | current default | `cross_validation` | `3.641` | `PASS` | current documented tuned path |
| `MID360` | best observed | `cross_validation` | `3.590` | `PASS` | rerun with the same tuned backend family |
| `MID360` | Scan Context candidate | `cross_validation` | `3.816` | `PASS` | fair current-code comparison; still opt-in |
| `MID360` | experimental BEV-assisted rerank | `cross_validation` | `3.607` | `PASS` | sensor-agnostic rerank of distance candidates; still opt-in |

Source artifacts:

- `output/benchmark_summary.md`
- `output/latest_report.html`
- `output/stress_validation_report_20260325.md`

## Current Default Position

The public `v0.2.2` position is:

- default workflow: `RKO-LIO + graph_based_slam`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- release gate: `bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10`
- map-cleanup benchmark: `bash scripts/run_dynamic_object_filter_benchmark.sh`
- classic-path suite: `bash scripts/run_open_data_classic_path_benchmark_suite.sh`
- place-recognition suite: `bash scripts/run_place_recognition_benchmark.sh`
- current MID360 default tuning:
  `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
  `loop_edge_dedup_index_window=20`, `loop_edge_info_weight=200`

## Interpretation

Safe claims:

- the default path is benchmarked on `NTU VIRAL`
- the pointcloud-map flow is dogfooded into Autoware
- the backend has current long-loop evidence on `MID360`
- the repository already provides reusable comparison artifacts for
  dynamic-filtering, classic-path open-data runs, and place-recognition
- the built-in GPL-free `Scan Context` path is now benchmarked and improves the
  fair current-code `MID360` rerun baseline, but it is still documented as
  opt-in
- the experimental submap-BEV path currently works better as a
  distance-candidate rerank than as a standalone loop source

Unsafe claims:

- that this repo is already the universal winner on every dataset
- that this repo should be judged primarily as a localization-research stack
- that the current default path is fully validated against every aggressive
  motion edge case
- that lanelet generation is part of the supported release scope

## Release Scope Reminder

`v0.2.2` is a public `v2 beta` release for:

- ROS 2 pointcloud-map generation
- non-GPL default workflow
- Autoware pointcloud-map loading

It is not yet claiming full production maturity.
