# Changelog

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
