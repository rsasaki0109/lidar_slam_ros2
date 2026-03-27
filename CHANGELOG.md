# Changelog

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
