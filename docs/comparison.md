# Comparison

This page is the public comparison snapshot for `lidarslam_ros2 v0.2.2` and
the in-flight `v0.3` track on `develop`.

It is intentionally scoped to workflows that are actually exercised in this
repository. It is not trying to be a universal ranking of every LiDAR SLAM
system.

## Release Track vs Research Track

`v0.3` introduced per-dataset release profiles so the gate stops squashing
heterogeneous datasets onto a single APE threshold. `v0.4` then **graduated the
former research-track profiles to blocking** (decision 2026-06-07,
`docs/roadmap/v0.4.md`):

- **Release track (blocking)** — a FAIL blocks the release. As of v0.4 this is
  every shipped profile: `Newer College math-hard` (ground truth),
  `NTU VIRAL tnp_01` (ground truth), `MID-360` vs GLIM (cross-validation), the
  Leo Drive applanix/velodyne open-data cross-validation, and the KITTI
  Odometry 00/05/07 LO baseline comparison (non-regression).
- **Interim caveat** — `MID-360` vs GLIM and the Leo Drive profile block on a
  *cross-validation* reference rather than ground truth, an interim weakness
  accepted knowingly. `MID-360` vs GLIM is slated to be replaced by a real
  ground-truth `ape_rmse_gt_m` profile in v0.5.
- **Research track (mechanism retained)** — `report_only_until` still downgrades
  a FAIL to WARN, but **no shipped profile uses it as of v0.4**. It remains
  available for any future dataset introduced mid-cycle.

The distinction is exercised by `scripts/run_release_readiness_checks.sh`,
which evaluates each profile in `scripts/release_profiles.yaml` and emits
`PASS` / `FAIL` / `WARN` / `TARGET_MET` / `NO_DATA` per dataset.

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

As of v0.4 every profile below is a blocking release-track profile. The current
numbers all sit under their `pass` thresholds, so graduation flips their status
from `WARN` to `PASS` without breaking the gate.

| Dataset | Configuration | Reference kind | APE RMSE (m) | Profile gate | Notes |
| --- | --- | --- | --- | --- | --- |
| `NTU VIRAL tnp_01` | current default | `ground_truth` | `0.952` | `PASS` (pass ≤ 1.00, target 0.30) | outdoor long-loop GT |
| `NTU VIRAL tnp_01` | best observed   | `ground_truth` | `0.870` | `PASS` (same)                     | loop-gated backend run |
| `MID-360` | current default                  | `cross_validation` vs GLIM | `3.641` | `PASS` (pass ≤ 4.00, target 1.00) | solid-state LiDAR, non-360° FOV |
| `MID-360` | best observed                    | `cross_validation` vs GLIM | `3.590` | `PASS` (same)                     | rerun with same tuned backend family |
| `MID-360` | Scan Context candidate           | `cross_validation` vs GLIM | `3.816` | `PASS`                            | fair current-code comparison; still opt-in |
| `MID-360` | experimental BEV-assisted rerank | `cross_validation` vs GLIM | `3.607` | `PASS`                            | sensor-agnostic rerank of distance candidates; still opt-in |
| Leo Drive (applanix/velodyne) | current default | `cross_validation` vs Applanix GSOF49 | varies per bag | `PASS` (pass ≤ 1.50, target 0.50) | open-data Velodyne packet path |

The Newer College `math-hard` profile (ground truth) is the tightest gate
(pass ≤ 0.10); its numbers are not checked in to this repo and are reported
separately on the long-form benchmark notes. The KITTI Odometry 00/05/07 LO
baseline comparison is wired through `scripts/run_kitti_00_05_07_report.sh` and
emits a non-regression report under
`output/kitti_dev_<timestamp>/kitti_dev_report.md`.

`MID-360` and Leo Drive now **do** feed the release gate (graduated in v0.4),
blocking on their cross-validation thresholds; `MID-360` vs GLIM is slated to be
replaced by a real ground-truth profile in v0.5.

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
