# Comparison

This page is the public comparison snapshot for the
`lidarslam_ros2 v0.9.1` release candidate. It is not yet published; the
historical `v0.9.0` release and tag remain immutable.

It is intentionally scoped to workflows that are actually exercised in this
repository. It is not trying to be a universal ranking of every LiDAR SLAM
system.

## Release Track vs Research Track

`v0.3` introduced per-dataset release profiles so the gate stops squashing
heterogeneous datasets onto a single APE threshold. `v0.4` then **graduated the
former research-track profiles to blocking** (decision 2026-06-07,
`docs/roadmap/v0.4.md`):

- **Release track (blocking)** — a FAIL blocks the release. As of v0.5 this is:
  `Newer College math-hard` (ground truth), `NTU VIRAL tnp_01` (ground truth),
  the two `mid360_gt_rtkslam_construction_*` profiles (**total-station ground
  truth**, graduated in v0.5), the Leo Drive applanix/velodyne open-data
  cross-validation, and the KITTI Odometry 00/05/07 LO baseline comparison
  (non-regression).
- **Report-only** — `MID-360` vs GLIM was demoted in v0.5 (decision D-GT-2,
  `docs/roadmap/v0.5.md`): cross-validation against another SLAM estimate
  measures agreement, not accuracy, and the same sensor is now gated on real
  total-station checkpoints. It stays as a regression canary. The Leo Drive
  profile keeps its cross-validation caveat (separate track). New profiles
  introduced mid-cycle (e.g. the outdoor Stadtgarten pair) soak as report-only
  before graduating.

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

These numbers come from generated benchmark artifacts under local `output/`
directories. `output/` is ignored by git; use the commands in
[Benchmarking And Release Gate](benchmarking.md) to regenerate the reports.

### Release-track datasets

The current blocking profiles below sit under their `pass` thresholds. The
Stadtgarten pair remains report-only while its outdoor evidence soaks.

| Dataset | Configuration | Reference kind | APE RMSE (m) | Profile gate | Notes |
| --- | --- | --- | --- | --- | --- |
| `NTU VIRAL tnp_01` | current default | `ground_truth` | `0.952` | `PASS` (pass ≤ 1.00, target 0.30) | outdoor long-loop GT |
| `NTU VIRAL tnp_01` | best observed   | `ground_truth` | `0.870` | `PASS` (same)                     | loop-gated backend run |
| `MID-360` RTK-SLAM Construction Hall 2 | indoor default | `ground_truth` (total station, 16 chkpt) | `0.086` (median 0.064, 16/16) | `PASS` (pass ≤ 0.30, target 0.15) | dense odometry scored, 2.0 s association contract |
| `MID-360` RTK-SLAM Construction Hall 1 | indoor default | `ground_truth` (total station, 16 chkpt) | `0.321` (median 0.163, 16/16) | `PASS` (pass ≤ 0.55, target 0.30) | hardest indoor hall (published baselines ~0.22) |
| `MID-360` RTK-SLAM Stadtgarten 2 | sequence-specific compatibility preset | `ground_truth` (total station, 19 chkpt) | `0.426` (median 0.264, 19/19) | report-only soak (pass ≤ 1.20) | legacy voxel order + 105 m range; shared default unchanged |
| `MID-360` RTK-SLAM Stadtgarten 1 | outdoor config | `ground_truth` (total station, 36 chkpt) | `0.838` (median 0.511, 36/36) | report-only soak (pass ≤ 2.20) | 26 min / ~1 km park loop; raw odometry, no GNSS / loop closure |
| `MID-360` | current default                  | `cross_validation` vs GLIM | `3.641` | report-only since v0.5 (D-GT-2)   | solid-state LiDAR, non-360° FOV |
| `MID-360` | best observed                    | `cross_validation` vs GLIM | `3.590` | report-only (same)                | rerun with same tuned backend family |
| `MID-360` | Scan Context candidate           | `cross_validation` vs GLIM | `3.816` | report-only                       | fair current-code comparison; still opt-in |
| `MID-360` | experimental BEV-assisted rerank | `cross_validation` vs GLIM | `3.607` | report-only                       | sensor-agnostic rerank of distance candidates; still opt-in |
| Leo Drive (applanix/velodyne) | current default | `cross_validation` vs Applanix GSOF49 | varies per bag | `PASS` (pass ≤ 1.50, target 0.50) | open-data Velodyne packet path |

The Newer College `Maths-Hard` profile (official ICP-to-survey-map ground
truth) is the tightest gate (pass ≤ 0.10). Its form-gated, CC BY-NC-SA inputs
and generated numbers are not checked in to this repo; the exact acquisition,
calibration, and rerun contract is documented in
[Benchmarking And Release Gate](benchmarking.md#newer-college-maths-hard).
The KITTI Odometry 00/05/07 LO
baseline comparison is wired through `scripts/run_kitti_00_05_07_report.sh` and
emits a non-regression report under
`output/kitti_dev_<timestamp>/kitti_dev_report.md`.

As of v0.5 the MID-360 release evidence is **real ground truth**: the two
RTK-SLAM Construction Hall profiles block on SE(3)-aligned total-station
checkpoint RMSE (`ape_rmse_gt_m`), the same metric family as NTU VIRAL and
Newer College. `MID-360` vs GLIM is report-only (regression canary, D-GT-2);
Leo Drive still blocks on its cross-validation threshold. Dataset, metric
definition and attribution:
`docs/research/rtkslam-total-station-gt-methodology.md`.

Source artifacts:

- `output/benchmark_summary.md` (generated locally)
- `output/latest_report.html` (generated locally)
- `output/stress_validation_report_<YYYYMMDD>.md` (generated locally)
- `scripts/release_profiles.yaml` (profile definitions)
- `output/kitti_dev_<timestamp>/kitti_dev_report.md` (generated locally, KITTI LO baseline)

## Current Default Position

The current tagged-release position is:

- default workflow: `RKO-LIO + graph_based_slam`
- official product entrypoints: the three commands in
  [`docs/product-contract.md`](product-contract.md)
- release gate: `bash scripts/run_release_readiness_checks.sh --fail-on-profiles`
  using `scripts/release_profiles.yaml` (per-dataset pass/target thresholds)
- map-cleanup benchmark: `bash scripts/run_dynamic_object_filter_benchmark.sh`
- classic-path suite: `bash scripts/run_open_data_classic_path_benchmark_suite.sh`
- place-recognition suite: `bash scripts/run_place_recognition_benchmark.sh`
- KITTI Odometry dev split: `bash scripts/run_kitti_00_05_07_report.sh`
- research-track MID360 default tuning (kept for parity with prior numbers):
  `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
  `loop_edge_dedup_index_window=20`, `loop_edge_info_weight=200`

## Interpretation

Safe claims:

- the default path is benchmarked on `NTU VIRAL` and reports on `MID-360`
- the pointcloud-map flow is dogfooded into Autoware end-to-end (map loaders)
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

`v0.9.1` is the current release candidate. The maintained product boundary is
offline rosbag2-to-verified-map authoring through the three official
entrypoints. Lanelet generation remains operator-reviewed, and evaluation-tier
sensor, GNSS, radar, coloured-map, and optional loop-detector paths do not
become universal hardware guarantees merely because benchmark evidence exists.

The authoritative supported outcome, support tiers, compatibility policy, and
non-goals are in [`docs/product-contract.md`](product-contract.md). Product
maturation toward v0.9 and v1.0 follows
[`docs/roadmap/v0.9.md`](roadmap/v0.9.md).
