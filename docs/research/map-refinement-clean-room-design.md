<!-- Design input for docs/roadmap/v0.7.md Phase 2. Drafted 2026-06-12 from the published papers only (clean-room constraint: the GPL reference implementations hku-mars/HBA and hku-mars/BALM must never be read while implementing this). -->

# Design Note: Offline Hierarchical Plane-Based Global Map Refinement

## Scope and Licensing

This is a clean-room design for a BSD-2-compatible implementation. The implementation must be written from the published mathematics and local engineering choices only. Do not inspect, copy, translate, or link GPL source from `hku-mars/HBA`, `hku-mars/BALM`, or related GPL implementations.

Primary paper references used for the design direction:

- BALM2 paper: <https://arxiv.org/abs/2209.08854>
- HBA paper: <https://arxiv.org/abs/2209.11939>
- Adaptive plane extraction paper: <https://arxiv.org/abs/2305.00287>

## Goal

Add an offline post-pose-graph refinement stage to `lidarslam-ros2`.

The existing deterministic offline backend runner, `graph_slam_offline_runner`, already replays a recorded odometry bag, creates submaps, performs loop search, runs g2o pose-graph optimization, and writes optimized TUM poses plus submap clouds. The new stage runs after pose-graph optimization and refines the submap poses using plane-feature bundle adjustment. It should improve map surface consistency and, when the environment has stable planes, reduce APE against total-station ground truth.

The stage is offline only. It is allowed to spend minutes, but it must preserve this repository's determinism contract.

## Inputs and Outputs

Inputs:

- `std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>`: one downsampled cloud per submap, expressed in the submap local frame.
- `std::vector<Eigen::Isometry3d>`: post-g2o optimized submap poses, mapping submap frame to global/map frame.
- Refinement config: voxelization, hierarchy, optimizer, robust-loss, and determinism settings.

Outputs:

- Refined `std::vector<Eigen::Isometry3d>` with the same indexing as the input poses.
- Diagnostics: plane counts, rejected-feature counts, cost history, LM damping history, pose delta stats, determinism hash, and convergence status.
- Optional refined TUM trajectory and refined submap clouds.

The refiner must not mutate the input clouds or input poses in place. Return a new pose vector.

## Math Core

### Plane Cost Without Explicit Plane Variables

For a candidate plane feature `f`, let its global points be `p_j in R^3`, with count `n`. Define:

```text
s = sum_j p_j
Q = sum_j p_j p_j^T
c = s / n
A = Q - s s^T / n
```

`A` is the unnormalized scatter matrix around the centroid.

The point-to-plane least-squares cost after eliminating the plane parameters is:

```text
E_f = min_{||u||=1, d} sum_j (u^T p_j + d)^2
    = min_{||u||=1} u^T A u
    = lambda_min(A)
```

The minimizing plane normal is the unit eigenvector of `A` associated with the smallest eigenvalue. The offset is `d = -u^T c`. These are analytical transient values, not optimization variables.

Total objective:

```text
E(T_0 ... T_{N-1}) = sum_f w_f rho(lambda_min(A_f) / n_f)
                  + pose-prior terms
```

Use `lambda_min(A_f) / n_f` for scale-normalized robust loss, while keeping the raw scatter form internally for derivatives. Pose-prior terms keep the solution close to the g2o trajectory in weakly constrained regions and fix gauge freedom.

### Point Cluster Coordinates

For each plane feature `f` and observing submap pose `i`, precompute a local homogeneous point cluster:

```text
C_fi = sum_{x in feature f observed in submap i} [x; 1] [x; 1]^T
     = [ Q_local  s_local
         s_local^T n_local ]
```

where `x` is expressed in submap `i`'s local frame. This is a symmetric `4x4` matrix; only 10 doubles plus count are needed.

For pose `T_i in SE(3)` represented as a homogeneous `4x4` matrix:

```text
H_fi = T_i C_fi T_i^T
H_f  = sum_i H_fi
```

Extract `Q`, `s`, and `n` from `H_f`, then compute `A_f = Q - s s^T / n`.

This makes cost and gradient evaluation independent of raw point count. Per optimizer iteration, the solver enumerates non-empty `(feature, pose)` clusters, not individual points. Hessian assembly is `O(observing_pose_pairs)` per feature, bounded by local window size in the hierarchy.

### Pose Perturbation

Use left perturbations with twist order `[translation, rotation]`:

```text
T_i(delta) = Exp(delta_hat) T_i
delta = [rho_x rho_y rho_z phi_x phi_y phi_z]^T
```

Let `G_r` be the `4x4` Lie algebra generator for component `r`. Translation generators have `e_r` in the top-right column. Rotation generators have the corresponding skew basis in the top-left block.

For the current transformed cluster `H = T C T^T`:

```text
H_r = G_r H + H G_r^T
P_rs = 0.5 * (G_r G_s + G_s G_r)
H_rs = P_rs H + H P_rs^T + G_r H G_s^T + G_s H G_r^T
```

`H_rs` is only nonzero when both derivatives apply to the same pose cluster. Mixed pose derivatives still enter through centroid subtraction in `A`.

For any derivative `D` of `H`, extract `Q_D` and `s_D`. Then:

```text
A_a  = Q_a - (s_a s^T + s s_a^T) / n

A_ab = Q_ab
     - (s_ab s^T + s s_ab^T + s_a s_b^T + s_b s_a^T) / n
```

For different poses, `Q_ab = 0` and `s_ab = 0`, but the final two centroid terms remain.

### Eigenvalue Gradient and Hessian

Let eigenvalues of `A` be ascending:

```text
mu_0 <= mu_1 <= mu_2
```

Let `u_0` be the eigenvector for `mu_0`. For a non-degenerate plane feature, require a minimum eigen-gap:

```text
mu_1 - mu_0 > eigen_gap_min
```

The first derivative of the plane cost is:

```text
d mu_0 / d x_a = u_0^T A_a u_0
```

The second derivative is:

```text
d^2 mu_0 / d x_a d x_b =
    u_0^T A_ab u_0
  + 2 * sum_{m=1,2}
      (u_m^T A_a u_0) * (u_m^T A_b u_0) / (mu_0 - mu_m)
```

For robust normalized cost `rho(z)` with `z = mu_0 / n`:

```text
g_a = w_f * rho'(z) * (1 / n) * mu_a

H_ab = w_f * [
    rho'(z)  * (1 / n)   * mu_ab
  + rho''(z) * (1 / n^2) * mu_a * mu_b
]
```

This is the exact second-order expression for the eliminated plane-feature cost under the chosen perturbation. Use it in a damped Newton/LM solver. If the Hessian is indefinite or the step is not a descent step, increase LM damping and retry. A conservative fallback is to use a PSD approximation by dropping the eigenvector-curvature term and adding stronger pose priors, but the first implementation should target the exact Hessian with robust damping.

### Gauge and Priors

Plane-only BA has gauge freedom. For each local BA window:

- Fix the first pose in the window, or equivalently apply a very strong prior to it.
- Optimize relative poses for the remaining window poses.

For full/global refinement:

- Keep the first submap fixed to preserve the map frame.
- Add soft priors to every pose against the post-g2o pose:
  - translation sigma default: `0.15 m` to `0.50 m`
  - rotation sigma default: `1 deg` to `3 deg`
- Allow config to disable all-pose priors only for controlled tests.

Priors are also important in vegetation-heavy or corridor-like scenes where plane constraints are rank deficient.

## Adaptive Voxelization for Plane Extraction

### Pipeline

For a BA problem over a set of poses:

1. Transform candidate points into the current anchor/global frame using the current initial poses.
2. Insert points into deterministic root voxels of size `root_voxel_size`.
3. Process root voxels in lexicographic integer key order.
4. For each voxel, run a PCA planarity test.
5. If planar, keep it as a candidate feature.
6. If not planar, split into eight octree children and recurse.
7. Stop on `min_voxel_size`, `max_depth`, or `min_points`.
8. Reject non-planar terminal leaves.
9. Merge coplanar leaf planes within the same root voxel.
10. For each accepted plane group, build per-submap point clusters `C_fi`.

### Planarity Test

For a point set, compute scatter eigenvalues ascending:

```text
lambda_0 <= lambda_1 <= lambda_2
```

Accept a candidate plane only if all hold:

```text
lambda_0 / max(lambda_2, eps) < planarity_ratio
lambda_1 / max(lambda_2, eps) > min_surface_ratio
point_count >= min_plane_points
observing_submap_count >= min_observing_submaps
```

The second condition rejects line-like features.

Use a quarter-split consistency check for false-positive planes:

1. Let `u_0`, `u_1`, `u_2` be the eigenvectors.
2. Treat `u_0` as normal and `u_1`, `u_2` as in-plane axes.
3. Move the split center from the centroid along `u_0` by `normal_split_offset * sqrt(lambda_0 / n)` to avoid splitting a thick plane into two layers.
4. Split points into four quarters by signs along `u_1` and `u_2`.
5. Reject if any quarter has fewer than `min_quarter_points`.
6. Compute each quarter's smallest scatter eigenvalue `lambda_0_q`.
7. Accept only if quarter thicknesses are consistent:

```text
max_q(lambda_0_q / n_q) <= quarter_thickness_ratio * max(lambda_0 / n, noise_floor^2)
```

Default starting parameters:

```text
root_voxel_size:          4.0 m
min_voxel_size:           0.5 m
max_depth:                4
min_plane_points:         30
min_quarter_points:       6
min_observing_submaps:    2, prefer 3
planarity_ratio:          0.03 to 0.08 local, 0.05 to 0.12 global
min_surface_ratio:        0.05
quarter_thickness_ratio:  2.0 to 4.0
noise_floor:              0.02 m
```

### Plane Merging

Within one root voxel, sort accepted leaf planes by leaf key. Merge deterministically into groups. Two planes may merge if:

```text
abs(n_a^T n_b) > cos(merge_angle)
abs(n_a^T (c_b - c_a)) < merge_distance
```

Canonicalize normals before comparison by making the largest absolute component positive. After each tentative merge, recompute group scatter and rerun the planarity test. Reject the merge if the combined group becomes non-planar.

Suggested defaults:

```text
merge_angle:     5 deg
merge_distance:  0.05 m to 0.15 m
```

### Degenerate Cases

Reject or downweight:

- Features observed by only one pose. They do not constrain relative pose.
- Features with unstable eigenvectors: `lambda_1 - lambda_0 <= eigen_gap_min`.
- Line-like features: `lambda_1 / lambda_2` too small.
- Volumetric vegetation or clutter: planarity ratio too high after max depth.
- Very large disconnected planes caused by voxel merging. Keep merging local to one root voxel.
- Planes dominated by one submap. Require minimum observing-submap count and optionally a max per-submap point fraction.
- Ground-only windows. They constrain `z`, roll, and pitch, but weakly constrain `x`, `y`, and yaw; rely on priors and pose graph constraints.

## HBA-Style Hierarchy

### Layer Model

Layer 0 is the original submap sequence.

Each upper layer is built from overlapping local BA windows over the previous layer:

```text
window_size = K
stride = S
S < K
```

For each window:

1. Anchor the first pose.
2. Extract/adapt plane features inside the window.
3. Run local plane BA over the `K` poses.
4. Store optimized relative poses and the local Hessian/information.
5. Aggregate the optimized window clouds into a keyframe expressed in the anchor frame.
6. The keyframe becomes one node in the next layer.

Use only accepted planar feature points to construct upper-layer keyframes by default. This reduces upper-layer memory and avoids carrying unstructured clutter into global BA.

### Top-Down Fusion Graph

After bottom-up construction, build a global pose graph that fuses constraints from all layers:

- Bottom-layer nodes: original submap poses.
- Upper-layer nodes: keyposes generated from local windows.
- Intra-window BA factors: relative pose constraints from local BA.
- Inter-layer equality/anchor factors: tie an upper-layer keypose to the corresponding lower-layer anchor pose.
- Adjacent/keypose factors from upper layers.
- Optional original g2o trajectory priors or relative factors to prevent degradation in weak geometry.

Solve this graph with the repository's existing deterministic g2o path if possible. Then propagate corrections back to bottom-layer poses. Iterate bottom-up BA and top-down graph for a fixed small number of outer iterations, usually `2`.

First implementation recommendation:

```text
outer_iterations: 2
local_lm_iterations: 8 to 15
top_graph_iterations: existing g2o default, fixed count preferred
```

### Layer Counts for 100 to 700 Submaps

For submaps, each node already covers more area than a raw scan, so use larger windows than scan-level HBA.

Recommended default:

```text
K = 16
S = 8
```

Approximate layer sizes:

```text
N = 100:  layer0 100 -> layer1 13 -> top
N = 300:  layer0 300 -> layer1 38 -> top
N = 640:  layer0 640 -> layer1 80 -> layer2 10 -> top
N = 700:  layer0 700 -> layer1 88 -> layer2 11 -> top
```

Policy:

- `N <= 120`: one global BA or one local layer plus top graph.
- `120 < N <= 350`: two-level hierarchy, top BA/graph on `~15` to `45` keyposes.
- `350 < N <= 700`: three-level hierarchy, top BA/graph on `~8` to `15` keyposes.
- Keep top-layer BA under `~50` poses. If it grows larger, add a layer.

Alternative conservative default:

```text
K = 20
S = 10
```

For `640` submaps this gives `640 -> 64 -> 7`, which is attractive for a 1 km release-gate dataset.

## Optimizer Design

Use a deterministic block solver:

1. Fixed pose ordering by submap index or layer-local index.
2. Dense block Hessian for local windows.
3. Sparse block Hessian only for the top fusion graph if needed.
4. LM solve:

```text
(H + lambda * diag_clamp(H) + H_prior) delta = -g
```

5. Apply step using `T <- Exp(delta_hat) T`.
6. Recompute cost.
7. Accept only if cost decreases and all pose deltas are finite.
8. If rejected, increase damping by a fixed factor.
9. If accepted, decrease damping by a fixed factor.

Suggested defaults:

```text
initial_lambda:        1e-4
lambda_up_factor:      10
lambda_down_factor:    0.3
max_lambda:            1e12
min_cost_decrease:     1e-9 relative
max_step_translation:  0.25 m local, 0.50 m global
max_step_rotation:     3 deg local, 5 deg global
```

If a step exceeds max step limits, scale the full update vector deterministically before evaluation.

## Integration

### `graph_slam_offline_runner`

Plug in after g2o pose-graph optimization and before final TUM/submap-cloud export:

```text
recorded odometry bag
  -> submap creation
  -> loop search
  -> g2o pose-graph optimization
  -> plane-based map refinement
  -> refined TUM trajectory
  -> refined submap clouds
  -> APE release-gate evaluation
```

Keep the existing optimized output as a baseline artifact:

```text
trajectory_g2o.tum
trajectory_refined.tum
submaps_g2o/
submaps_refined/
map_refinement_diagnostics.yaml
```

Config flag:

```yaml
map_refinement:
  enabled: true
  mode: hierarchical_plane_ba
```

If refinement fails validation, the runner should be configurable to either:

- fail the release gate, or
- emit diagnostics and fall back to the g2o trajectory.

For release gates, compare both g2o APE and refined APE. The default gate should require refined APE to be no worse than g2o by a configured tolerance.

### Standalone CLI

Add a standalone ROS-free CLI, for example:

```text
map_refiner \
  --poses input_optimized.tum \
  --submap_dir submaps_g2o \
  --output_poses trajectory_refined.tum \
  --output_submap_dir submaps_refined \
  --config map_refinement.yaml \
  --diagnostics map_refinement_diagnostics.yaml
```

Useful flags:

```text
--no_hierarchy
--max_submaps N
--check_determinism_runs 2
--write_plane_debug_clouds
--write_cost_history
```

The CLI should sort input files by explicit submap index, not filesystem order.

## Determinism Requirements

Byte-identical output across repeated runs is a first-class requirement.

Nondeterminism risks and required controls:

- Filesystem order: never rely on directory iteration order. Parse submap indices and sort.
- Voxel maps: do not allow `unordered_map` iteration order into results. Store voxel records in a vector and sort by integer key before processing.
- Floating-point accumulation: accumulate points, clusters, gradients, and Hessians in fixed order. Prefer single-threaded accumulation for v1. If threaded later, use fixed shards and deterministic pairwise reduction.
- PCL filters: avoid filters whose output depends on hash iteration. If downsampling is needed here, implement deterministic voxel centroid/downsample logic.
- Eigen decomposition: use `Eigen::SelfAdjointEigenSolver`; sort eigenpairs explicitly; canonicalize eigenvector signs.
- Plane normal sign: canonicalize by largest absolute component positive before merge tests or debug output.
- Close eigenvalues: skip features with insufficient eigen-gap. They cause unstable normals and Hessians.
- Robust loss: no random sampling, no RANSAC, no stochastic trimming.
- Window processing: process windows in ascending `(layer, window_start)` order. Parallel local BA is allowed only if final graph assembly is sorted and independent.
- Hessian assembly: fixed block ordering; write symmetric blocks once, then mirror deterministically.
- Linear solver: use deterministic Eigen dense `LDLT` or `LLT` for local windows. For sparse graph solve, use the repository's deterministic g2o path with fixed ordering.
- LM control flow: fixed iteration limits, fixed damping update factors, no time-budget termination.
- Invalid points: filter NaN/Inf in deterministic point-index order and report counts.
- Output formatting: fixed precision, `C` locale, fixed line endings, sorted YAML keys.
- CPU/compiler: do not use `-ffast-math`. Disable Eigen internal multithreading for deterministic runs. Byte-identical guarantees are required across runs on the same build and machine; cross-machine byte identity needs separately pinned compiler and CPU flags.

Add a diagnostics hash over:

```text
config
input pose bytes after parsing
submap file names and sizes
accepted plane feature descriptors
final refined poses
```

## Header-Only Module Decomposition

Place ROS-free implementation headers under:

```text
graph_based_slam/include/graph_based_slam/
```

Suggested headers:

- `map_refinement_types.hpp`
  - Config structs, diagnostics structs, result types, aligned pose vector aliases.

- `se3_lie.hpp`
  - `Exp`, `Log`, left perturbation, generators `G_r`, pose delta limits.

- `point_cluster.hpp`
  - Homogeneous cluster storage, deterministic accumulation, transform, merge, extraction of `Q/s/n`.

- `scatter_eigen_cost.hpp`
  - Scatter matrix construction, plane eigen-cost, gradient/Hessian formulas, robust loss application.

- `adaptive_voxel_plane_extractor.hpp`
  - Deterministic root voxelization, octree split, PCA tests, quarter test, plane merge.

- `plane_feature_association.hpp`
  - Converts accepted voxel planes into per-pose point clusters. Owns raw point-to-feature assignment for one BA problem.

- `ba_problem.hpp`
  - Pose blocks, feature blocks, priors, cost/gradient/Hessian assembly.

- `lm_solver.hpp`
  - Deterministic damped Newton/LM loop, acceptance policy, step limiting.

- `hba_pyramid.hpp`
  - Layer/window construction, local BA orchestration, keyframe aggregation, top-down factor generation.

- `map_refiner.hpp`
  - Public pure C++ API:
    ```cpp
    MapRefinementResult refineMap(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>& clouds,
        const AlignedIsometry3dVector& initial_poses,
        const MapRefinementConfig& config);
    ```

- `map_refiner_io.hpp`
  - TUM parsing/writing, PCD loading/writing, deterministic file ordering. Keep ROS-free.

Add only thin `.cpp` files for CLI and runner integration.

## Test Plan

Use gtest characterization tests in `graph_based_slam/test`.

Core math tests:

- Cluster equivalence: point-enumerated scatter and cluster-transformed scatter match within tolerance.
- Feature elimination: `lambda_min(scatter)` equals explicit least-squares point-to-plane residual using analytical plane.
- Gradient check: compare analytical gradient to central finite differences on fixed synthetic clusters.
- Hessian check: compare analytical Hessian to central finite differences of the gradient.
- Gauge test: applying the same global transform to all poses leaves plane costs unchanged.
- Degenerate feature tests: single-pose plane has no useful relative constraint; line-like and volumetric features are rejected.

Plane extraction tests:

- Synthetic wall, ground, and corner planes with known labels.
- Curved surface should split until rejected or become small local planes.
- Outlier slab/vegetation-like points should fail quarter consistency.
- Plane merging should merge coplanar leaves and reject perpendicular or offset leaves.
- Shuffled point order must produce identical accepted plane descriptors.

Optimizer tests:

- Two or more submaps observing one plane: constrained components improve, unconstrained components remain governed by priors.
- Three orthogonal planes with known perturbed poses: recover poses within tolerance.
- Multi-window synthetic map: hierarchy result is close to one-shot BA for small `N`.
- Rejection path: deliberately bad initial pose should increase LM damping and not emit NaNs.

Determinism contract tests:

- Run the same refiner twice and compare serialized refined poses byte-for-byte.
- Insert voxels/features in different orders and verify identical output.
- Shuffle input PCD file listing and verify identical output after index sorting.
- Run with `Eigen::setNbThreads(1)` and verify diagnostics hash stability.

Integration tests:

- Small offline-runner fixture with generated submaps and a known loop correction.
- Verify files are emitted:
  - g2o trajectory
  - refined trajectory
  - diagnostics YAML
- Verify fallback behavior when refinement has too few valid planes.

## Expected Impact for 640 Submaps / 1 km

Assumptions:

- Post-g2o trajectory is already close enough for voxel plane associations.
- Downsampled submaps contain stable man-made or ground/wall plane structure.
- `N ~= 640`, `K = 16`, `S = 8`, hierarchy `640 -> 80 -> 10`.

Expected accuracy:

- Translation APE RMSE improvement: typically `5%` to `25%` when stable planes are abundant.
- Map plane thickness improvement: often more visible than APE, roughly `10%` to `40%` on walls/ground.
- Little or no improvement in vegetation-heavy or weakly planar areas.
- Possible APE regression if false plane associations dominate, so release gates must compare refined APE against the g2o baseline.

Expected compute:

- Plane extraction dominates when clouds are large.
- For `640` submaps with `5k` to `20k` downsampled points each, expect roughly `3M` to `13M` points.
- Deterministic single-thread v1 target: about `5` to `25` minutes on a desktop CPU, depending on point count and plane density.
- Memory target: `1` to `4 GB`, mostly raw/downsampled points, feature assignment, and cluster storage.
- Local BA windows are small: `K=16` gives `96` pose DoF per local dense solve before anchoring. This is tractable.

## Main Risks

- Few stable planes: outdoor vegetation, open roads, and sparse scenes may not constrain enough DoF.
- Wrong associations: if post-g2o error exceeds voxel size or local geometry repeats, BA can polish the wrong map.
- Degenerate geometry: ground-only or corridor-only windows need priors and pose-graph constraints.
- Dynamic objects: cars, pedestrians, and moving vegetation can create false planes.
- Memory pressure: storing raw points for reclustering plus cluster descriptors can grow quickly.
- Convergence: exact Hessian can be indefinite; LM damping and step limits are mandatory.
- Determinism erosion: unordered containers, parallel reductions, PCL internals, and filesystem order are the most likely causes.
- Metric mismatch: optimizing map consistency can slightly worsen total-station APE if the total-station frame/noise and LiDAR surface model disagree. Keep the g2o result and require gate-based acceptance.
