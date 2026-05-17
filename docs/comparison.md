# Comparison

This page is the public comparison snapshot for `lidarslam_ros2 v0.2.2` and
the in-flight `v0.3` track on `develop`.

It is intentionally scoped to workflows that are actually exercised in this
repository. It is not trying to be a universal ranking of every LiDAR SLAM
system.

## Release Track vs Research Track

`v0.3` separates evaluation into two tiers so the release gate stops squashing
heterogeneous datasets onto a single APE threshold:

- **Release track** — datasets where the release-profile gate enforces a
  pass threshold. The current release-track datasets are `Newer College
  math-hard` (hard gate) and the KITTI Odometry 00/05/07 LO baseline
  comparison (non-regression).
- **Research track** — datasets that are reported but do not block release
  (`report_only_until: v0.4` in `scripts/release_profiles.yaml`). The current
  research-track datasets are `NTU VIRAL tnp_01`, `MID-360` vs GLIM, the
  Leo Drive applanix/velodyne open-data cross-validation, and the experimental
  loop descriptor / NIS auto-scale paths.

The distinction is exercised by `scripts/run_release_readiness_checks.sh`,
which evaluates each profile in `scripts/release_profiles.yaml` and emits
`PASS` / `FAIL` / `WARN` / `TARGET_MET` / `NO_DATA` per dataset. Profiles with
`report_only_until` emit `WARN` instead of `FAIL` until the named release.

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

### Release-track datasets

| Dataset | Configuration | Reference kind | APE RMSE (m) | Profile gate | Notes |
| --- | --- | --- | --- | --- | --- |
| `NTU VIRAL tnp_01` | current default | `ground_truth` | `0.952` | `WARN` (pass ≤ 1.00, target 0.30, `report_only_until: v0.4`) | outdoor long-loop GT |
| `NTU VIRAL tnp_01` | best observed   | `ground_truth` | `0.870` | `WARN` (same)                                                | loop-gated backend run |

The Newer College `math-hard` profile is the only hard gate; numbers are not
checked in to this repo and are reported separately on the long-form benchmark
notes. The KITTI Odometry 00/05/07 LO baseline comparison is wired through
`scripts/run_kitti_00_05_07_report.sh` and emits a non-regression report under
`output/kitti_dev_<timestamp>/kitti_dev_report.md`.

### Research-track datasets (report-only until v0.4)

| Dataset | Configuration | Reference kind | APE RMSE (m) | Profile gate | Notes |
| --- | --- | --- | --- | --- | --- |
| `MID-360` | current default                  | `cross_validation` vs GLIM | `3.641` | `WARN` (pass ≤ 4.00, target 1.00) | solid-state LiDAR, non-360° FOV |
| `MID-360` | best observed                    | `cross_validation` vs GLIM | `3.590` | `WARN` (same)                     | rerun with same tuned backend family |
| `MID-360` | Scan Context candidate           | `cross_validation` vs GLIM | `3.816` | `WARN`                            | fair current-code comparison; still opt-in |
| `MID-360` | experimental BEV-assisted rerank | `cross_validation` vs GLIM | `3.607` | `WARN`                            | sensor-agnostic rerank of distance candidates; still opt-in |
| Leo Drive (applanix/velodyne) | current default | `cross_validation` vs Applanix GSOF49 | varies per bag | `WARN` (pass ≤ 1.50, target 0.50) | open-data Velodyne packet path |

`MID-360` and Leo Drive remain instructive evidence but no longer feed the
release gate; the v0.3 default release path is judged on Autoware-compatible
map authoring + Newer College + KITTI Odometry LO baseline.

Source artifacts:

- `output/benchmark_summary.md`
- `output/latest_report.html`
- `output/stress_validation_report_20260325.md`
- `scripts/release_profiles.yaml` (profile definitions)
- `output/kitti_dev_<timestamp>/kitti_dev_report.md` (KITTI LO baseline)

## Current Default Position

The public `v0.2.2` position is:

- default workflow: `RKO-LIO + graph_based_slam`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- release gate (legacy): `bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10`
- release gate (`v0.3`): `bash scripts/run_release_readiness_checks.sh --fail-on-profiles`
  using `scripts/release_profiles.yaml` (per-dataset pass/target thresholds)
- map-cleanup benchmark: `bash scripts/run_dynamic_object_filter_benchmark.sh`
- classic-path suite: `bash scripts/run_open_data_classic_path_benchmark_suite.sh`
- place-recognition suite: `bash scripts/run_place_recognition_benchmark.sh`
- KITTI Odometry dev split: `bash scripts/run_kitti_00_05_07_report.sh`
- AWSIM → Autoware E2E demo: `bash scripts/run_awsim_selfmade_map_demo.sh`
- research-track MID360 default tuning (kept for parity with prior numbers):
  `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
  `loop_edge_dedup_index_window=20`, `loop_edge_info_weight=200`

## Interpretation

Safe claims:

- the default path is benchmarked on `NTU VIRAL` and reports on `MID-360`
- the pointcloud-map flow is dogfooded into Autoware end-to-end via AWSIM
- the repository already provides reusable comparison artifacts for
  dynamic-filtering, classic-path open-data runs, and place-recognition
- the release gate is now data-aware (per-dataset pass/target thresholds) so
  hard datasets (`MID-360`, `NTU VIRAL`) can be reported without being
  forced to one global APE threshold
- the built-in GPL-free `Scan Context` path is now benchmarked and improves the
  fair current-code `MID-360` rerun baseline, but it is still documented as
  opt-in
- the experimental submap-BEV path currently works better as a
  distance-candidate rerank than as a standalone loop source

Unsafe claims:

- that this repo is already the universal winner on every dataset
- that this repo should be judged primarily as a localization-research stack
- that the current default path is fully validated against every aggressive
  motion edge case
- that the `MID-360` research-track number (3.5–4.0 m vs GLIM) is anywhere
  near production accuracy on solid-state LiDAR

## Release Scope Reminder

`v0.2.2` is a public `v2 beta` release for:

- ROS 2 pointcloud-map generation
- non-GPL default workflow
- Autoware pointcloud-map loading

`v0.3` (in flight on `develop`) extends this with:

- Autoware-compatible lanelet2 auto-generation + multi-segment routing
  validation (`scripts/simple_lanelet2_generator.py --validate-structure`)
- dataset-profile release gate (`scripts/release_profiles.yaml`)
- KITTI Odometry t_rel / r_rel drift metric and 00/05/07 dev-split aggregator
- opt-in NIS-driven auto-scale for `adjacent_edge_info_weight`

`MID-360` and other solid-state LiDAR datasets are explicitly research track
until `v0.4`; they are reported but do not block release.
