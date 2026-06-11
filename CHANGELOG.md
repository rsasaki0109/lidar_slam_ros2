# Changelog

## 0.6.0 - 2026-06-12

The deterministic core / ROS shell refactor (roadmap `docs/roadmap/v0.6.md`,
PRs #237–#262). Same bag + same config now provably produces the same map:
the offline runners replay recorded inputs through the same cores the live
nodes use and the release gate enforces byte-identical results across runs.

### Highlights

- **Deterministic offline mapping (backend)** — `graph_slam_offline_runner`
  replays a recorded odometry bag through submap creation, event-driven
  BackendCore loop search and pose-graph optimization with no executor and
  no wall clock; 3 runs are *byte-identical* (loop edges + optimized
  trajectory) on both gate substrates (MID-360: APE 7.262851264566504 to the
  last digit; NTU VIRAL: 9 loop edges, Leica-GT APE 0.8460511516520233).
  Before the refactor the same input produced different loop edges every run.
- **Deterministic offline mapping (frontend)** — `scan_matcher_offline_runner`
  drives the real `ScanMatcherComponent` in lockstep over a raw sensor bag
  (intra-process, drained executor, synchronous map update); 3 runs over the
  full NTU tnp_01 bag are byte-identical (5795 poses, 133 submaps) — the
  first determinism coverage for the scanmatcher (NDT) frontend.
- **Event-driven loop search by default** — the backend now searches loops on
  submap arrival instead of a wall-clock timer; the v0.4
  `deterministic_loop_scheduling` experiment is retired (its negative result
  motivated this architecture). The legacy timer path stays available behind
  `event_driven_loop_search: false` for one release.
- **Release gate hard-enforces determinism** — opt-in
  `--offline-determinism-bag` / `--frontend-determinism-bag` stages in
  `run_release_readiness_checks.sh` fail the gate on any byte-level mismatch.
- **Real bug fixes surfaced by the refactor** — silent ~6% input drop under
  load (QoS KeepLast(1) + best-effort), IMU/GNSS pose-graph edges weighting
  the wrong error blocks (GNSS georeferencing never actually pulled), missing
  tie-breaks in candidate ranking, GNSS yaw alignment with gauge release
  (first georeferenced maps).
- **God objects decomposed with characterization tests** —
  `graph_based_slam_component.cpp` 3421 → 2265 lines behind 19 tested pure
  headers; `scanmatcher_component.cpp` 2140 → 1694 lines with 4 new tested
  pure headers (pose prediction, pose acceptance incl. the tracking state
  machine, IMU processing, map-update policy). Package test count grew to
  1100+.

## 0.5.0 - 2026-06-11

First version aligned across `VERSION`, the four core `package.xml` files and
the release docs, prepared for the first rosdistro (bloom) binary release.
Covers the v0.4 and v0.5 roadmap milestones on top of 0.3.0.

### Highlights

- **Real ground truth on Livox MID-360 (v0.5 milestone)** — all four RTK-SLAM
  sequences measured against total-station checkpoints; the two indoor
  Construction Hall profiles now block releases (pass 0.30 m / 0.55 m), the
  outdoor Stadtgarten pair soaks as report-only with a dedicated outdoor
  preset (`double_downsample: false`: 3.9 m → 0.835 m), and the
  `mid360_vs_glim` cross-validation gate is demoted to a report-only canary.
- **Deterministic loop scheduling (v0.4 milestone)** — opt-in
  `deterministic_loop_scheduling` parameter (default off) that deterministically
  catches up unqueried submaps instead of latest-only wall-clock scheduling;
  the research-track release profiles graduated to blocking gates.
- **One-command everything** — ghcr Docker demo image
  (`docker run ... ghcr.io/rsasaki0109/lidar_slam_ros2:humble` downloads a
  public MID-360 bag and produces the map headless), the beginner bag-to-map
  runner, and the Autoware bundle now emits `lanelet2_map.osm` alongside
  `pointcloud_map/` + `map_projector_info.yaml`.
- **Photoreal map deliverables** — camera-colored point-cloud map flythrough
  (README hero GIF) with occlusion-aware robust colorization, plus the
  documented optional 3DGS pipeline (gsplat, Apache-2.0).
- **rosdistro release prep** — per-package `CHANGELOG.rst`, SPDX
  `BSD-2-Clause` license tags, versions aligned at 0.5.0, and a bloom runbook
  (`docs/rosdistro-release.md`).

## 0.3.0 - 2026-05-25

Public `v3` release. Carries forward the non-GPL default workflow from
`v0.2.x` and adds end-to-end AWSIM × Autoware, the Livox MID-360 operator
toolkit, an opt-in STD/BTC-style triangle descriptor research stack with
variance-validated defaults, dogfood wrapper measurement plumbing, and a
user-friendly README.

### Highlights

- **AWSIM × Autoware end-to-end pipeline** — sample-map and self-made-map
  demos, one-command wrappers, lanelet2 generation from TUM trajectory; see
  `docs/awsim-autonomous-driving-tutorial.md`
- **Livox MID-360 operator toolkit** — Jetson-class robot-side recording →
  SLAM → Autoware map workflow with host preflight, bag stamp rewriter,
  recording / production-candidate sessions, public-dataset map runner, and
  dashboards
- **Opt-in STD/BTC-style triangle descriptor stack** — BSD-2 implementation
  with `edge_3d` keypoint extractor for narrow-FOV / indoor, fine-grained ROS
  params, a diagnostic `triangle_descriptor_skip_ransac` flag, and an
  empirically validated MID-360 default of `max_pairs: 16`; default
  `use_triangle_descriptor: false` on every preset
- **Dogfood wrapper measurement plumbing** — frame overrides,
  quiescence-based offline completion, graph-drain wait, corrected-path
  capture, and reference-TUM APE evaluation
- **User-friendly README** — 5-minute "try the public default" path, badges,
  grouped feature lists, progressive disclosure of research tracks

### Research closeout

- 3-dataset variance evidence (NTU / MID-360 / Newer College) shows the
  triangle stack does not yet provide a reproducible APE win — it is opt-in
  research only
- single-run APE claims on MID-360 triangle ablations are unreliable
  (variance can exceed observed |Δ| by 8x); all reports in this release ran
  ≥3 runs with mean ± std + `|Δ|/σ`
- MID-360-specific `+1 m` APE drift was traced to RANSAC compute cost, not
  the act of enabling the pipeline; `max_pairs: 32 → 16` on the MID-360 yaml
  eliminates it (U-shaped sweep; `=16` is the empirical sweet spot)
- full narrative: `docs/research/triangle-stack-2026-05-summary.md`

### Notes

- the recommended public workflow remains `RKO-LIO + graph_based_slam` with
  distance-based loop closure
- the default release path remains non-GPL and focused on pointcloud-map
  generation for Autoware-compatible workflows
- AWSIM and the triangle stack are additive — they do not change the default
  public path

## 0.2.2 - 2026-03-28

Public `v2 beta` patch release focused on release stability and cross-distro CI
consistency.

### Highlights

- fixed Humble/Jazzy style and include-path mismatches that appeared after the
  `0.2.1` release-prep refresh
- kept the public `RKO-LIO + graph_based_slam` workflow, reports, and release
  metadata aligned on `develop`
- validated the patched release scope with green `docs`, `humble`, `jazzy`,
  `release readiness`, and threshold-guard workflows

### Notes

- this is a patch release over `0.2.1`, not a scope expansion
- public defaults and known limits remain unchanged from `0.2.1`

## 0.2.1 - 2026-03-28

Public `v2 beta` refresh focused on map-authoring workflow hardening and
clearer fallback-path positioning.

### Highlights

- GNSS-aware graph optimization now uses covariance-based weighting and has
  real open-data validation for both direct `NavSatFix` bags and Applanix
  sidecar conversion
- packet-path IMU deskew was hardened around `PointCloud2.time` handling and
  validated on real open data with a repeatable matrix report
- save-time dynamic-object filtering now has cross-dataset validation on Leo
  Drive `bag1` and `bag6`, with roughly `50%` saved-point reduction while
  keeping verification `PASS`
- classic-path fallback benchmarking now includes GNSS-only, IMU, and
  velocity-based odom-prior comparisons, with a tracked validation report that
  keeps dataset-specific knobs out of the public default
- exploratory place-recognition work is now explicitly closed out: distance
  remains the public default, while Scan Context, BEV rerank, and SOLiD stay
  opt-in or experimental
- map-authoring reporting and submission-bundle tooling were extended so maps,
  metrics, logs, and focused reports can be packaged and compared more
  repeatably

### Notes

- the recommended public workflow is still `RKO-LIO + graph_based_slam`
- the default release path remains non-GPL and focused on pointcloud-map
  generation rather than full production autonomy stacks

## 0.2.0 - 2026-03-25

Public beta candidate for the `v2` release line.

### Highlights

- recommended default workflow narrowed to permissive-license components
- `RKO-LIO + graph_based_slam` established as the dogfooded default path
- graph backend improved with better adjacent edges, loop dedup, robust kernels,
  multi-candidate validation, and safer state handling
- Autoware-compatible pointcloud-map export hardened with
  `map_projector_info.yaml` and bundle verification
- end-to-end Autoware dogfood flow added:
  `rosbag2 -> SLAM -> map save -> Autoware map loaders -> rviz2`
- benchmark reporting, HTML report generation, and release/readiness gate added
- CI expanded with default workflow checks and release-readiness fixture jobs
- contribution guide, Autoware quickstart, benchmarking guide, and issue
  templates added for external reports
- fixed public Autoware entrypoint added: `scripts/run_autoware_quickstart.sh`
- comparison page and checked-in release notes added for public `v2 beta`
- MID360 current default tuned to `voxel_size=0.5`, `max_range=80.0`,
  `search_submap_num=5`, `loop_edge_dedup_index_window=20`,
  `loop_edge_info_weight=200`

### Notes

- this release is suitable for public beta / developer preview distribution
- the default workflow remains focused on pointcloud-map generation for
  Autoware; lanelet generation is intentionally out of scope
