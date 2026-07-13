# Plane re-observation constraints in the pose graph

Date: 2026-07-13

## Why this factor exists

Loop registration constrains a full relative SE(3) pose, but a repeated wall,
floor or ceiling observation does not. One plane constrains two rotations and
translation along its normal. Translation within the plane and rotation about
the normal remain unobservable. Encoding only those observable directions
avoids inventing information in corridors and other rank-deficient scenes.

The binary factor transforms each keyframe's local plane
`n_local · x_local + d_local = 0` into the world frame and minimizes:

```text
r_normal = n_world_from × n_world_to
r_offset = d_world_from - d_world_to
```

The normal sign is aligned before evaluating the residual. A Huber kernel
limits the effect of a wrong plane association.

## Data path

1. Deterministically voxel-downsample each offline submap cloud.
2. Reuse `associatePlaneFeatures` to group planar patches observed by multiple
   poses.
3. Fit each local observation from its `PointCluster`, rejecting weak support,
   excessive thickness and degenerate covariance.
4. Reject candidate pairs whose initial world-plane normal differs by more
   than 2 degrees or whose signed offset differs by more than 3 cm. This gate
   removes different parallel walls and mixed patches before the robust kernel.
5. Connect the earliest observation to a bounded number of temporally
   separated revisits. This creates a sparse star rather than a dense all-pairs
   graph.
6. Add the 4D binary edges beside odometry, loop, IMU and GNSS constraints in
   `optimizePoseGraph`.

The feature is default-off, preserving historical offline-runner artifacts.
Enable an A/B run with:

```bash
bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/backend_input \
  --runs 3 --save-maps \
  --param use_plane_revisit_constraints:=true \
  --param plane_revisit_cloud_downsample:=0.20 \
  --param plane_revisit_min_pose_separation:=5
```

The runner writes `plane_revisit_report.yaml` with feature/observation counts
and factor chi-square before and after optimization. The determinism script
requires this report to be byte-identical across runs when present.

## Parameters

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `use_plane_revisit_constraints` | `false` | Opt in without changing existing runs |
| `plane_revisit_cloud_downsample` | `0.20` m | Bound CPU and memory cost |
| `plane_revisit_min_pose_separation` | `5` | Exclude short-baseline pseudo-revisits |
| `plane_revisit_max_constraints_per_feature` | `4` | Bound graph density |
| `plane_revisit_normal_info_weight` | `10` | Normal alignment information |
| `plane_revisit_offset_info_weight` | `10` | Plane-offset information |
| `plane_revisit_root_voxel_size` | `2.0` m | World patch extraction scale |
| `plane_revisit_max_octree_depth` | `1` | Avoid fragmenting a revisit into tiny patches |
| `plane_revisit_max_plane_thickness` | `0.08` m | Extraction thickness gate |
| `plane_revisit_min_planarity_ratio` | `4.0` | Extraction planarity gate |
| `plane_revisit_min_points_per_observation` | `20` | Per-keyframe support gate |
| `plane_revisit_max_initial_normal_error_deg` | `2.0` deg | Reject inconsistent normals before graph insertion |
| `plane_revisit_max_initial_offset_error_m` | `0.03` m | Reject a different parallel surface before graph insertion |

## Verification completed

- A two-pose wall fixture with 0.30 m normal-direction drift converges to the
  correct offset while its tangential y/z coordinates remain unchanged.
- An 8 degree plane-normal rotation error converges without adding a forbidden
  rotation-about-normal constraint.
- Sparse builder tests verify temporal separation, per-feature caps, local
  offset fitting, weak-support rejection and non-planar rejection.
- A six-pose, three-orthogonal-plane trajectory fixture reduces position RMSE
  from more than 0.15 m to less than 0.002 m (over 50x) against weak odometry.
- A different-parallel-wall fixture is rejected by the initial residual gate.
- The complete pose-graph suite passes 13/13 tests, and the offline runner
  builds with the factor extraction path linked in.
- PLY/PCD point-cloud I/O plus the combined A/B gate pass 48/48 focused Python
  tests, so the runner's PCD maps feed Scan-to-BIM without an external converter.

## Real-data A/B gate: MID-360 public driving sequence

The final gate uses the recorded 277 s MID-360 backend input (2,765 paired
odometry/cloud messages, 640 submaps, 1,079.3 m travel). One previously
verified loop edge is replayed in both arms, so plane factors are the only
changed graph input. The metadata SHA-256 is
`7506c02e98a7b86ce10a1baa7e9d7c754a9301113443fe4f5cbceec45af78ef0`;
the fixed-edge CSV SHA-256 is
`5e30036d9190933e9659805caac032cc675ba968700a096c788f12560026c6d9`.

GLIM is an independent SLAM cross-validation trajectory, not survey ground
truth. Its APE therefore supports an A/B comparison but must not be presented
as absolute accuracy. Both arms match the same 218 timestamps after SE(3)
Umeyama alignment.

| Metric | OFF | ON | Change |
| --- | ---: | ---: | ---: |
| GLIM cross-validation APE RMSE (m) | 1.107267 | 0.751511 | **-32.13%** |
| planar thickness mean (m) | 0.0434823 | 0.0433933 | **-0.20%** |
| planar thickness P95 (m) | 0.110545 | 0.110310 | **-0.21%** |
| planar coverage | 0.446315 | 0.446632 | **+0.07%** |
| BIM Coverage mean | 0.105909 | 0.132391 | **+25.00%** |
| BIM Distance RMSE mean (m) | 0.051127 | 0.046690 | **-8.68%** |
| BIM Distance P95 mean (m) | 0.094805 | 0.087181 | **-8.04%** |
| BIM Distribution mean | 0.993700 | 0.993485 | -0.000216 absolute |

This outdoor holdout yields six observed slab elements and no walls or rooms;
the BIM row validates upstream geometric fit, while exp01/exp07 and the closed
synthetic room remain the authoritative wall/topology gates. Distribution is
within the frozen 0.001 absolute non-regression tolerance.

The ON report records 2,536 candidate constraints, rejects 2,441 at the initial
residual gate, and inserts 95. Their chi-square falls from 0.503652 to 0.392116.
Three OFF runs and three ON runs produce byte-identical loop edges and
trajectories; the ON `plane_revisit_report.yaml` is also byte-identical. The map
quality report is independently byte-identical across three evaluations in
each arm. HILTI exp04 is the negative holdout: its maximum observation
separation is one submap, so it inserts zero factors and OFF/ON trajectory MD5
is identically `2e2331d4b8adbb8487bd1dbac5f1d280`.

The machine-readable verdict is
`/media/sasaki/aiueo/benchmarks/plane_revisit_mid360_20260713/plane_revisit_ab_gate.json`
(SHA-256
`49005e95ca0e97e738c6e719424f1f00ac51186d007ea0c19688b8c98f41bccb`),
with all nine checks passing.

## Reproduce the final verdict

Run the backend twice with the same bag, parameter file and fixed edge. The
scripts now find either a repository-local `install/` or the normal parent
colcon workspace automatically.

```bash
ROOT=/media/sasaki/aiueo/benchmarks/plane_revisit_mid360_20260713
BAG=/media/sasaki/aiueo/benchmarks/mid360_public/backend_input_20260713/backend_input
FIXED="$ROOT/on_root2_depth1_smoke/run1/loop_edges.csv"
GLIM=/media/sasaki/aiueo/benchmarks/mid360_public/driving_slam_loop_weight400_20260713/traj_corrected.tum

bash scripts/run_offline_determinism_check.sh \
  --bag "$BAG" --params lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  --runs 3 --output-dir "$ROOT/final_off_default" \
  --reference-tum "$GLIM" --ape-max-time-diff 0.05 \
  --param fixed_loop_edges_path:="$FIXED" \
  --param refine:=false --param use_plane_revisit_constraints:=false

bash scripts/run_offline_determinism_check.sh \
  --bag "$BAG" --params lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  --runs 3 --output-dir "$ROOT/final_on_default" \
  --reference-tum "$GLIM" --ape-max-time-diff 0.05 \
  --param fixed_loop_edges_path:="$FIXED" \
  --param refine:=false --param use_plane_revisit_constraints:=true
```

Generate OFF/ON maps with `--save-maps --param refine:=true`, run
`scripts/run_map_quality_check.sh --runs 3 --downsample 0.20`, and export BIM
directly from each `map_optimized.pcd`. Finally run:

```bash
python3 scripts/evaluate_plane_revisit_ab.py \
  --off-ape "$ROOT/final_off_default/run1/ape.txt" \
  --on-ape "$ROOT/final_on_default/run1/ape.txt" \
  --off-map-quality "$ROOT/final_map_quality_off/run1/map_quality_report.yaml" \
  --on-map-quality "$ROOT/final_map_quality_on/run1/map_quality_report.yaml" \
  --off-bim "$ROOT/bim/off/map_metrics.json" \
  --on-bim "$ROOT/bim/on_weight10/map_metrics.json" \
  --on-plane-report "$ROOT/final_on_default/run1/plane_revisit_report.yaml" \
  --output "$ROOT/plane_revisit_ab_gate.json"
```

The factor remains default-off because only one positive real-data sequence has
completed the full trajectory/map/BIM gate. Enabling it is now a documented,
bounded opt-in rather than an unvalidated default behavior.
