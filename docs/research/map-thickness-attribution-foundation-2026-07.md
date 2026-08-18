# Map-thickness attribution foundation (2026-07)

## Decision

Map thickness is now treated as a hierarchical error budget, not as one
aggregate score. The first implementation is report-only and has no effect on
the saved map or trajectory. It decomposes planar point-to-plane residual
energy into four deterministic components:

1. within one LiDAR scan;
2. between scans in one submap;
3. between submaps in one revisit epoch; and
4. between revisit epochs.

This measurement must precede the dense map rebuild and frontend changes. A
candidate is developed against the component it can physically improve and is
still judged by the frozen aggregate map, trajectory, localization, resource,
and determinism gates.

## Mathematical contract

The shared adaptive voxel plane extractor supplies deterministic planar
patches and their source point indices. For each patch, signed residuals use
the patch normal and centroid. Nested group means then give the exact ANOVA
identity

```text
SSE_total = SSE_within_scan
          + SSE_between_scan_within_submap
          + SSE_between_submap_within_revisit
          + SSE_between_revisit.
```

The report emits RMS values normalized by the total planar point count,
fractions normalized by total SSE, and an explicit numerical closure error.
The output is meaningful only when planar points and non-zero residual energy
exist and the identity closes within the fixed floating-point tolerance.

This is attribution, not causal proof. In particular, an incorrect plane
association can move energy between components. Aggregate planar coverage and
the existing map-quality profiles remain mandatory companion evidence.

## Stable interchange contract

The CLI input is an unquoted numeric CSV with the exact header

```text
x,y,z,scan_id,submap_id,revisit_id
```

Coordinates must be finite. IDs are signed 64-bit integers. The full
`(revisit_id, submap_id, scan_id)` tuple identifies a scan, so local IDs may be
reused by different sessions without being merged. The CLI writes a
fixed-order, fixed-precision YAML schema and returns a distinct exit status
when a syntactically valid input does not produce a meaningful measurement.

## Verified synthetic result

One synthetic plane gives every level an analytically known symmetric offset:
1 cm within-scan, 2 cm between-scan, 4 cm between-submap, and 8 cm
between-revisit. The implementation recovers all four RMS components to
`1e-12`, closes the total SSE, preserves tuple identity when local scan IDs are
reused, rejects non-finite CSV coordinates, and reproduces report lines byte
for byte. Six focused tests pass.

## Real-data adapter boundary

The frozen HILTI exp04 backend bag is available at
`/media/sasaki/aiueo/benchmarks/hilti_exp04_backend_fixed_20260713/backend_input`.
It contains 1,258 exactly timestamp-paired `/rko_lio/odometry` and
`/rko_lio/frame` messages. The existing offline runner reduces those to 44
submap anchors and retains only the cloud from each selected anchor. Therefore
its current retained clouds cannot distinguish within-scan thickness from
between-scan thickness.

The next adapter must consume every paired scan. It will:

- assign monotonically increasing scan IDs in exact timestamp order;
- reproduce the runner's submap-distance decision and assign intervening scans
  to the active submap anchor;
- transform local scan points with the corresponding dense frontend pose;
- define revisit epochs from loop-independent spatial trajectory overlap with
  distance and exit hysteresis, then freeze those thresholds in the report;
- deterministically voxel-reduce each scan before retaining points, with raw
  and retained counts reported; and
- emit both a provenance manifest and the attribution YAML, while keeping the
  feature default-off.

Loop edges must not define the measurement groups: changing a loop detector
would otherwise change both the candidate and its ruler. Revisit segmentation
therefore uses only the frozen dense input trajectory.

## exp04 attribution result

The adapter is now integrated into `graph_slam_offline_runner` behind
`save_map_thickness_attribution=false`. With the feature enabled and CSV
materialization disabled, the frozen exp04 bag produced:

| measurement | result |
|---|---:|
| paired scans | 1,258 |
| physical submap anchors | 44 |
| nested `(revisit, submap)` groups | 46 |
| raw finite points | 43,895,054 |
| retained points after per-scan 0.15 m voxels | 10,812,762 |
| planar points / coverage | 2,513,108 / 23.24% |
| planar patches | 70,544 |
| total residual RMS | 0.05342 m |
| within-scan share / RMS | 35.91% / 0.03201 m |
| between-scan share / RMS | 40.22% / 0.03387 m |
| between-submap share / RMS | 18.50% / 0.02298 m |
| between-revisit share / RMS | 5.37% / 0.01238 m |
| closure error at report precision | 0 |
| elapsed / maximum RSS (isolated run) | 13.78 s / 1,257,184 KiB |

The 46 nested groups do not contradict the 44 physical anchors: two revisit
epoch boundaries occur while a physical submap remains active, so the ANOVA
hierarchy splits those observations at the boundary. The three trajectory
epochs start at scan IDs 0, 898, and 1087, at dense travel distances 0,
64.053, and 77.233 m. Both later starts are spatial returns near the origin.

Two independent replays produced byte-identical attribution YAML with SHA-256
`d054b34920ad8e949d3d3c73b3bbaf908d886f851e2e6f6a6fe3d2a86447657b`.
The existing raw and optimized trajectory files and loop-edge CSV also match
between runs. The raw trajectory retains its previously frozen SHA-256
`3e94a3482b67e56de515cb830fcd374dd0b6cab5ffb7c150e1ee414c5922b390`.

The attribution RMS is not the existing map-quality mean-thickness metric;
it is the square-root residual energy used solely for the hierarchical error
budget. The existing mean/p95/coverage gates remain authoritative for
candidate adoption.

The measured distribution is actionable: 76.13% of residual energy lies at
within-scan or between-scan levels, while 23.87% lies at submap/revisit levels.
Dense pose propagation is still implemented next because all later frontend
and backend pose improvements need a scan-rate map rebuild path. The result
predicts that deskew/timestamp work and fixed-lag frontend optimization will
ultimately carry the largest exp04 thickness opportunity.

## Dense pose-refined rebuild foundation

The runner now also has a separate default-off
`save_dense_pose_refined_map` path. At each graph anchor it computes the
world-side correction

```text
C_i = T_optimized_i * inverse(T_raw_i).
```

For every frontend scan timestamp it linearly interpolates the translation of
`C` and uses quaternion SLERP for its rotation, then applies
`T_dense_corrected(t) = C(t) * T_dense_raw(t)`. Corrections clamp outside the
anchor range. Numerically identity anchor corrections are snapped to exact
identity so a zero-loop negative holdout does not acquire rounding-only map
changes. Four focused tests pin exact identity, anchor-range clamping,
translation/rotation interpolation, and timestamp validation.

The exp04 zero-loop holdout generated 1,258-pose raw and optimized dense
trajectories plus globally 0.1 m voxelized raw and optimized maps. Both map
arms contain 6,887,588 points. The raw/optimized trajectory pair is byte
identical, as is the raw/optimized PCD pair. Two independent rebuilds also
match byte for byte; the PCD SHA-256 is
`5ca52f47be899a3a7ab7ea47fd36e55d1f82ee3ec8c94c97c3a670a59056480a`.
The isolated run took 46.42 s with 1,933,640 KiB maximum RSS; the repeat
without attribution took 40.65 s with 1,933,016 KiB maximum RSS.

Simple all-scan accumulation is correctly rejected as a map candidate on
exp04. The existing evaluator measured mean thickness 0.06026 m, p95
0.11973 m, and planar coverage 0.07226. It exposes scan-rate pose and deskew
noise that the historical 44-anchor map does not accumulate. Therefore this
stage is infrastructure for applying real corrections, not a default-on
density change. Promotion requires a positive loop/refinement substrate and
the later uncertainty-aware fusion stage; point count alone is not evidence
of map improvement.

### Construction Seq2 positive correction probe

The same path was exercised on the frozen Construction Seq2 backend bag with
the five previously verified loop edges fixed by CSV. The current worktree
produced 5,864 paired scans, 225 graph anchors, 46,162,639 retained local
points, a 5,549,609-point raw map, and a 5,615,171-point optimized map.

Dense corrected APE against all 16 surveyed positions improved from
0.154238 m to 0.145206 m (-5.86%), so continuous correction propagation is
effective and passes the 2% trajectory non-regression gate. Map geometry did
not follow the trajectory improvement:

| metric | dense raw | dense optimized | change |
|---|---:|---:|---:|
| mean thickness (m) | 0.095916 | 0.096680 | +0.80% |
| p95 thickness (m) | 0.124099 | 0.124810 | +0.57% |
| planar coverage | 0.184699 | 0.182776 | -1.04% |

This confirms that global trajectory error and local scan/surface error must
be treated separately. Better graph poses cannot rescue equal-weight fusion
of every noisy scan.

Two complete positive-probe rebuilds were byte identical. Raw PCD SHA-256 is
`704dbe6300d93507780136554164afdbffc10916a593ba8882879dbd29adfc08`;
optimized PCD SHA-256 is
`0a732d99730e732db1218a4ad5eb88c20104f9eb981d2f58ef988e1a5b1d6a42`.
The first run took 110.77 s and 3,408,908 KiB maximum RSS; the second took
134.03 s and 3,409,536 KiB. Against the 599.064 s sensor duration, both are
well inside the RTF 5 and RSS 8 GiB long-horizon limits.

Dense correction/rebuild infrastructure remains default-off but is now ready
to evaluate frontend improvements. The next stage targets deskew and timestamp
synchronization because the attribution evidence assigns 35.91% of exp04
residual energy within individual scans. Fixed-lag multi-scan optimization
then targets the additional 40.22% between scans. Uncertainty-aware surfel
fusion remains necessary before any all-scan final map can be promoted.

## Piecewise gyro deskew probe

The first within-scan candidate replaces the single interval-mean angular
velocity used for rotational deskew with deterministic, zero-order-hold
integration of every bias-corrected IMU gyro sample. Translation deskew and
the ICP prediction remain unchanged, and the feature is default-off behind
`piecewise_gyro_deskew=false`.

On HILTI exp04, a same-build default-off/on comparison over all 1,258 poses
and seven surveyed control points produced a strong trajectory improvement:

| metric | default-off | piecewise gyro | change |
|---|---:|---:|---:|
| APE RMSE (m) | 0.071560 | 0.052948 | -26.01% |
| APE mean (m) | 0.066569 | 0.050340 | -24.38% |
| APE maximum (m) | 0.114177 | 0.072368 | -36.62% |
| processing RTF | 1.144 | 1.380 | +20.63% |

The first candidate trajectory and the later complete-capture trajectory are
byte identical with SHA-256
`bb3226f772526a3bb1d24b7d6334b576bc8a2ad62f429363701dec9cba851e70`.
The RTF remains well below the long-horizon limit of 5.

The direct thickness result is much smaller. The complete fixed backend input
at
`/media/sasaki/aiueo/benchmarks/hilti_exp04_piecewise_gyro_20260722_v5/backend_input`
contains exactly 1,258 odometry messages and 1,258 deskewed frames. Against
the frozen default-off attribution:

| metric | default-off | piecewise gyro | change |
|---|---:|---:|---:|
| total attribution RMS (m) | 0.053415 | 0.053227 | -0.35% |
| within-scan RMS (m) | 0.032010 | 0.031925 | -0.27% |
| between-scan RMS (m) | 0.033874 | 0.034438 | +1.66% |
| planar coverage | 0.232421 | 0.233904 | +0.64% |

The candidate trajectory is shorter and changes loop-independent revisit
segmentation from three epochs to two, so the lower revisit component is not
treated as a deskew win. The targeted within-scan component is effectively
flat. Two offline replays produced byte-identical attribution reports with
SHA-256
`97f34439a537924434c4429737669d449a8b6c92179e0db39935c935a651f1b3`.
The isolated attribution run took 13.95 s and 1,251,128 KiB maximum RSS.

Capturing the high-rate offline output exposed a transport issue in the
benchmark harness: volatile publishers with depth one and rosbag2 defaults
could lose up to ten early or queued frames. The capture-only profile now
waits three seconds before replay, uses publisher depth 128, and the recorder
uses reliable keep-all QoS, a 1 GiB cache, and MCAP fast-write mode. These
settings do not change estimator math; the resulting trajectory remains byte
identical to the original candidate run.

Decision: retain piecewise gyro deskew as a default-off trajectory candidate,
but do not promote it as the map-thickness reform. Its -0.27% targeted gain is
far below the final -30% mean and -25% p95 objectives. Proceed to fixed-lag
multi-scan optimization, which targets the larger between-scan component.

## Fixed-lag multi-scan probe

The default-off fixed-lag frontend keeps six scans, builds quality-gated
scan-to-scan factors to three neighbors, fixes the oldest pose, and rebuilds
an adjustable active-map layer. A Bonxai lifetime defect found during the
first run was fixed by recreating its accessor after every clear; 1,000
clear/reuse cycles now pass under test.

The conservative complete exp04 capture contains all 1,258 odometry/cloud
pairs. Relative to piecewise gyro alone, it changed the attribution as follows:

| metric | piecewise gyro | conservative fixed lag | change |
|---|---:|---:|---:|
| total attribution RMS (m) | 0.053227 | 0.052725 | -0.94% |
| within-scan RMS (m) | 0.031925 | 0.031436 | -1.53% |
| between-scan RMS (m) | 0.034438 | 0.034231 | -0.60% |
| planar coverage | 0.233904 | 0.235334 | +0.61% |

Its APE RMSE was 0.053965 m, +1.92% versus piecewise gyro, leaving only
0.08 percentage points of margin under the 2% trajectory-regression gate.
RTF was 2.07. The result is a valid but weak thickness candidate and is not
ready for promotion.

Tracking and refined mapping were subsequently isolated. The proven primary
map/update path remains untouched, while marginalized and active refined maps
are separate. The optimizer can fix both the oldest and newest poses, and the
ROS wrapper delays each cloud/odometry pair until its pose is marginalized.
The final active window is explicitly flushed at offline shutdown. This makes
the map evaluated by the backend the same map represented by the trajectory
artifact, rather than retrospectively editing a trajectory that was never
published.

Several negative probes established necessary safety invariants. A stronger
history-oriented setting produced +8.21% APE. A per-solve cap also allowed
small corrections to accumulate, and lowering the factor weight increased
accepted windows and worsened drift. The implementation now caps total
displacement from each frame's primary odometry pose. It also copies the
exact primary pose back into online state when the newest pose is fixed;
allowing an approximately 1e-14 m optimizer round trip was enough for later
nonlinear ICP to diverge by as much as 6.4 cm. The explicit copy restores a
byte-identical primary tracking trajectory with SHA-256
`bb3226f772526a3bb1d24b7d6334b576bc8a2ad62f429363701dec9cba851e70`.

The final safe exp04 profile uses a 0.05 scan-factor weight, fixes the newest
pose, and limits total history deformation to 0.5 mm/0.005 degree. Against a
same-build piecewise control it produced:

| metric | piecewise gyro | finalized fixed lag | change |
|---|---:|---:|---:|
| APE RMSE (m) | 0.052948 | 0.052914 | -0.064% |
| total attribution RMS (m) | 0.053227 | 0.053235 | +0.0146% |
| within-scan RMS (m) | 0.031925 | 0.031930 | +0.0158% |
| between-scan RMS (m) | 0.034438 | 0.034443 | +0.0145% |
| planar coverage | 0.233904 | 0.233895 | -0.0036% |
| processing RTF | 1.370 | 1.867 | +36.3% |

The frozen backend capture at
`/media/sasaki/aiueo/benchmarks/hilti_exp04_fixed_lag_finalized_20260722_v2/backend_input`
contains exactly 1,258 clouds and 1,258 odometry messages. Two independent
attribution replays are byte identical with SHA-256
`01491fa51254bf8ddfea433b8d557f501fff7a7e176d7e4406209956ac3cb934`.
Decision: retain the default-off fixed-lag and finalized-output infrastructure,
but reject it as an exp04 thickness improvement. Its safe operating region is
effectively neutral, so the next material thickness candidate is
uncertainty-aware surfel fusion.

## Probabilistic surfel fusion probe

The default-off offline map builder now retains scan identity, sensor origin,
and the RKO-LIO odometry pose covariance. It first constructs a deterministic
0.1 m centroid map, then estimates probabilistic surfels from coarser 0.3 m
support voxels. Observations are collapsed to one centroid per scan before
fusion, range/incidence and pose uncertainty determine normal-direction
information, and a Huber weight limits outliers. Every valid support surfel
projects its constituent 0.1 m centroids onto the estimated plane. Invalid
support falls back to the exact centroid. Projected points remain 0.1 mm inside
their original output voxel, including after PointXYZ float serialization, so
the method cannot appear thinner by deleting occupied cells.

The first fine-voxel-only probe fused 9.68% of occupied voxels and changed mean
thickness by only -0.023%. Giving plane estimation 0.3 m spatial support raised
the fusion fraction to 36.75%. An initial boundary clamp placed double-precision
points exactly on voxel faces; PointXYZ conversion rounded some into adjacent
cells and caused 4.91% downsample collisions. That result was discarded. A
float-round-trip regression test now enforces the interior-voxel invariant.

The corrected exp04 result uses the finalized 1,258-pair fixed-lag backend bag.
Both maps contain 6,879,479 points and the frozen evaluator differs by only
three evaluated points (+0.000044%):

| metric | centroid baseline | probabilistic surfel | change |
|---|---:|---:|---:|
| mean thickness (m) | 0.060004 | 0.057191 | -4.69% |
| p95 thickness (m) | 0.119445 | 0.117506 | -1.62% |
| planar coverage | 0.072607 | 0.080042 | +10.24% |
| evaluated points | 6,879,475 | 6,879,478 | +0.000044% |

The baseline PCD SHA-256 is
`bad542335e025626f93efc648b62eedb9c64c19d82b2e98835ca7a9f29773e21`;
the fused PCD SHA-256 is
`f6d1ac3639c55f9b1c56a5d1fbd0aa2cdaacc6e0472c2bab705c1c65f563693d`.
Both hashes and the diagnostic YAML hash reproduced across independent runs.
The repeat took 41.21 s with 3,700,192 KiB maximum RSS. Against the 217.297 s
bag duration, RTF is 0.190; both resource gates pass comfortably.

Decision: retain the scan-aware surfel builder as the first material,
coverage-positive thickness candidate, but keep it default-off. It does not
meet the final -30% mean / -25% p95 targets and has only one-sequence evidence.
Proceed to scan-persistence dynamic-observation removal, then evaluate the
combined map on unseen datasets before considering promotion.

### Frozen 0.5 m support profile and unseen holdouts

The initial 0.3 m support result above was followed by a development-only
support-scale ablation. A 0.5 m support voxel improved exp04 substantially;
0.7 m mixed distinct surfaces, regressed mean thickness, and was rejected. A
missing safety condition was also fixed before freezing: valid surfels now
require a minimum middle-eigenvalue ratio of 0.05, so line-like support cannot
be misclassified as a plane. The frozen profile is therefore 0.1 m scan/output
voxels, 0.5 m support, one grid phase, maximum small-eigenvalue ratio 0.10, and
minimum middle-eigenvalue ratio 0.05.

Without any holdout tuning, the profile produced:

| dataset | mean thickness change | p95 change | coverage change |
|---|---:|---:|---:|
| exp04 development | -18.34% | -14.00% | +123.34% |
| exp01 unseen | -18.80% | -24.45% | +161.47% |
| exp07 unseen | -11.59% | -19.61% | +103.48% |
| macro mean | -16.24% | -19.35% | +129.43% |

Point counts are invariant before serialization and differ by at most 11 points
after float PCD round-trip and evaluator downsampling across maps containing
3.46--19.69 million points. Root-voxel counts and density remain invariant.
The exp01 baseline plane coverage is below the evaluator's meaningfulness floor
(3.97%); fusion raises it to 10.39%, so its thickness change is supportive but
not treated as a standalone promotion result.

The first exp01 build exposed a long-sequence memory failure: a tree node and
heap vector per occupied voxel reached 9,742,676 KiB RSS. The builder now stores
observations in one flat array and uses deterministic sort/run grouping twice,
reusing the same memory for support fusion. exp04 PCD hashes remained exactly
unchanged. Exp01 then fell to 6,728,076 KiB, took 47.35 s for a 693.18 s bag
(RTF 0.068), and reproduced the pre-refactor fused PCD SHA-256
`1b23d96ff50c089538a1e3aa26669244707be8041e1637e40e40b8ab1e0d1f9a`.
The frozen exp04 fused PCD SHA-256 is
`6c5aec79c847e52b2de1cd88bae192e5d5917806e673143720325d0a03524a77`.

An eight-phase half-voxel overlap probe increased fused coverage from 30.64%
to 64.59%, but independent per-voxel phase selection introduced surface seams:
exp04 improvement fell to -12.51% mean and -6.85% p95. It is rejected; one
phase remains frozen. Uniformly blending all valid phase projections also
failed (-12.16% mean, -8.13% p95), so the failure is not merely winner
selection. A looser maximum small-eigenvalue ratio of 0.20 reached -17.02%
mean and -26.58% p95 with +258% coverage, but cross-validated NDT fitness
regressed 3.06% mean and 0.79% p95. It is rejected as geometric overreach.

Chaining the existing point-count-preserving planar consolidation after the
frozen surfel map was also rejected at the applicability gate. Only 0.61% of
points had strict local-plane support, below its 10% minimum, so it returned
the input exactly. This confirms that the next candidate needs a coherent
multi-cell surface field, not another independent local projection pass.

A deterministic single-thread NDT residual evaluator sampled every 100th
exp04 scan (13/13 converged), initialized at the recorded odometry pose. Mean
fitness changed from 0.0131189 to 0.0132634 m^2 (+1.10%); p95 improved only
0.57%. Both baseline and fused reports reproduced byte-for-byte. The required
-10% NDT residual gate therefore fails.

Final surfel decision: keep the implementation and frozen profile default-off.
It improves thickness and coverage on all three datasets, preserves the
trajectory byte-for-byte, and passes determinism, RSS, and RTF gates, but misses
the macro -30%/-25% thickness targets and the NDT -10% target. No default-on
promotion is justified.

### Hierarchical support completion

A second support scale was added without changing the frozen single-scale
path. Setting `probabilistic_surfel_secondary_support_voxel_size=0` is the
compatibility mode: the exp04 baseline and fused PCDs and both trajectory
artifacts remain byte-for-byte identical to the frozen profile. A direct
0.5 m then 0.3 m projection was rejected because the weaker fine plane
overwrote good coarse projections: relative to the centroid baseline it
reached only -16.35% mean, -11.69% p95, and +89.3% coverage.

The accepted experimental semantics are hierarchical instead: a valid 0.5 m
plane owns its output voxel, and 0.3 m support may project only voxels that
fell back at the coarse scale. It never deletes an occupied 0.1 m cell. The
same frozen fusion thresholds, one grid phase, and scan/output resolution are
used on all datasets without holdout tuning:

| dataset | mean thickness change | p95 change | coverage change |
|---|---:|---:|---:|
| exp04 development | -19.69% | -15.48% | +131.09% |
| exp01 unseen | -20.60% | -27.69% | +175.15% |
| exp07 unseen | -13.38% | -21.98% | +112.94% |
| macro mean | -17.89% | -21.72% | +139.73% |

This improves the frozen single-scale macro by 1.65 percentage points mean,
2.37 points p95, and 10.30 points coverage. Exp01 peaked at 7,461,844 KiB RSS
and completed map construction in 53.87 s for a 693.18 s sequence (RTF 0.078),
so the 8 GB and RTF 5 resource gates still pass. Two independent exp04 runs
matched for both maps, both trajectories, the loop CSV, and the diagnostic
YAML. The fused PCD SHA-256 is
`305dd4586e3d65d9b7dc3bbc5af473861f2e7ac32c828518bb7e8a10c0831157`.

The stronger map does not pass the complete NDT gate. With an even-scan map
and 63 unseen odd-scan queries, fitness changed by +0.71% mean and +0.41% p95
relative to the centroid map. From a 0.5 m and 2 degree perturbation, however,
translation p95 improved 12.35%, rotation mean 10.23%, and rotation p95 12.06%;
translation mean improved only 4.64%. This is useful convergence-basin
evidence, but it cannot substitute for the required -10% residual.

Decision: hierarchical completion becomes the new best research candidate,
but remains default-off. The macro mean and p95 goals and the NDT residual
goal remain open.

### Scan-persistence dynamic-removal negative probe

A separate default-off output tested the simplest scan-rate persistence rule:
within 30 m, retain a 0.1 m voxel only when at least three distinct scans span
at least three scan IDs; preserve all observations beyond 30 m because reliable
visibility testing is unavailable there. The implementation is deterministic,
has synthetic tests for short-lived removal, separated observations, and the
far-range exception, and never replaces the surfel-only output.

On exp04 it removed 4,718,480 of 6,879,479 voxels (68.59%). This was not a
valid dynamic/static separation:

| metric | surfel only | persistence filtered | change |
|---|---:|---:|---:|
| points | 6,879,479 | 2,160,999 | -68.59% |
| mean thickness (m) | 0.057191 | 0.072172 | +26.20% |
| p95 thickness (m) | 0.117506 | 0.130933 | +11.43% |
| planar coverage fraction | 0.080042 | 0.119027 | +48.71% |
| plane patches | 38,504 | 15,335 | -60.17% |

The apparent coverage-fraction increase comes from a much smaller denominator
while most plane patches disappear. Decision: reject occupancy persistence as
a thickness candidate and keep it default-off as negative evidence. Static
surfaces at view boundaries are frequently observed only briefly, so a future
dynamic filter must model expected visibility, occlusion, and free-space rays
instead of treating missing repetition as motion. Proceed to global surface
optimization with the coverage-positive surfel map intact.

That visibility-aware follow-up is now implemented as a separate default-off
artifact. A 0.1 m voxel is eligible only when it is observed in at most two
nearby scans. The filter queries range returns five and fifteen scans before
and after the observation interval in deterministic 0.25 degree spherical
bins. It removes the voxel only after two measured returns lie at least 0.5 m
behind it. A closer return is treated as occlusion, and a missing angular
sample provides no negative evidence. Range images are rebuilt one scan at a
time rather than retained globally, preserving the long-sequence memory model.

On exp04, 4,029,575 voxels were eligible, 1,428,980 had a measured same-bin
return, 612,535 received at least one free-space contradiction, and only 63,712
(0.93% of the map) passed the two-vote removal gate. This fixes the naive
filter's catastrophic deletion rate, but the hierarchical surfel result still
changed by +0.203% mean thickness, +0.449% p95, and -0.377% coverage. A stricter
one-scan-only, 0.125 degree profile removed just 8,088 voxels (0.12%) and was
still slightly negative: +0.031% mean, +0.055% p95, -0.026% coverage.
Synthetic tests pin actual free-space removal, near-return occlusion, and
bitwise scan-order invariance. Decision: retain the visibility implementation
as diagnostic infrastructure, but reject point deletion as a thickness stage.

### Dense propagation of global surface BA

The existing hierarchical plane BA refines submap anchors after the offline
runner has already written dense maps. A default-off bridge now propagates an
accepted BA correction through all scan timestamps, writes the resulting dense
trajectory, and rebuilds the same probabilistic surfel map. This closes the
previous implementation gap between global plane optimization and the map that
is actually evaluated.

On exp04 the combined BA+surfel map changed the surfel-only result by mean
thickness -0.41%, p95 +0.12%, and coverage +1.38%. More importantly, dense APE
RMSE worsened from 0.052914 m to 0.065065 m (+22.96%), far outside the 2%
trajectory gate. Runtime was 81.52 s (RTF 0.375) and maximum RSS was 3,706,160
KiB. Decision: the propagation infrastructure is valid and remains
default-off, but the current BA profile is rejected. Internal plane-cost
acceptance is not sufficient; any future BA acceptance must include an
external trajectory/non-regression guard or a much stronger pose prior.

The pose-prior boundary was then measured explicitly. Translation/rotation
sigmas of 0.01 m/0.002 rad limited APE regression to 0.045%, but made the map
change effectively zero. At 0.03 m/0.005 rad APE regressed 2.04% and failed.
The intermediate 0.025 m/0.004 rad profile passed APE at +1.00%, yet improved
the surfel map by only 0.059% mean and 0.130% p95 while reducing coverage
0.120%. Strong priors therefore make this BA safe by making it immaterial;
the stage remains default-off and is not part of the best candidate.

Inspection of those runs exposed a prior implementation defect: each
overlapping BA window used the already-updated pose from the preceding window
as its new prior center. Even the strongest profile could therefore accumulate
about 7.8 cm mean dense correction. The solver now accepts a separate prior
reference trajectory, and every local/global window is anchored to the
original pose-graph solution. A stride-one overlap regression test bounds the
maximum correction below 2 mm under a strong synthetic prior.

With absolute priors, the weak exp04 profile reduced its APE regression from
22.96% to 11.12%, demonstrating that the accumulation fix is material but not
sufficient. The 0.03 m/0.005 rad boundary profile passed the external APE gate
at +1.06%. Its hierarchical surfel map, however, changed by +0.030% mean,
+0.007% p95, and -0.242% coverage. The corrected conclusion is therefore the
same but better founded: anchor-level plane BA is ineffective inside the safe
trajectory envelope and harmful outside it. It remains default-off.

### Scan-rate surface residual probe

The anchor-level result motivated a separate scan-rate translation refiner.
It deterministically downsamples every scan, constructs 0.5 m probabilistic
plane supports, collapses the supported point-to-plane observations into a
3-by-3 normal equation per scan, and solves one block-tridiagonal system with
absolute-pose and temporal-smoothness priors. Rotation is deliberately held
fixed. The solver accepts only a finite objective decrease, caps each total
translation correction, and returns the input trajectory exactly when the
scene is featureless or the solve is invalid. The corrected scan poses then
rebuild the same hierarchical 0.5 m then fallback-only 0.3 m surfel map.

Focused synthetic tests pin residual reduction, correction capping,
bitwise-exact featureless fallback, determinism, and exclusion of a target
scan parity from its reference surface. The offline path remains behind
`save_scan_surface_refined_surfel_map=false` and writes a separate trajectory,
centroid/surfel maps, and diagnostic report; it cannot replace the existing
surfel output.

The frozen exp04 candidate used a 0.02 m absolute prior, 0.01 m temporal
smoothness sigma, and 0.01 m correction cap. It constrained 1,256 of 1,258
scans and reduced its internal surface objective by 0.112%, with 0.316 mm RMS
and 1.672 mm maximum translation corrections. External APE RMSE improved
0.289%, so the 2% trajectory non-regression gate passed. Against the best
hierarchical surfel map, mean and p95 thickness each improved only 0.029%,
while coverage changed by -0.003%.

One pre-declared relaxed-prior probe used 0.05 m absolute and 0.02 m temporal
sigmas while retaining the 0.01 m cap. Correction RMS rose to 1.001 mm and the
internal objective reduction to 0.353%; APE still improved 0.195%. The map did
not respond materially: mean thickness improved 0.029%, p95 regressed 0.058%,
and coverage regressed 0.055%. The isolated relaxed run took 34.30 s and
2,663,960 KiB maximum RSS, comfortably inside the resource gates.

The final probe removed the self-reference explicitly. Each support voxel
builds independent even- and odd-scan surfels, and constrains each scan only
against the opposite parity. This raises the measured surface RMS from about
1.29 mm to 3.58 mm and constrains all 1,258 scans, confirming that the cross-fit
ruler exposes disagreement hidden by the all-scan surface. With the frozen
priors, correction RMS was 1.202 mm and APE improved 0.177%. The resulting map
still changed by only +0.003% mean thickness, -0.075% p95, and -0.132%
coverage. Runtime was 35.56 s and maximum RSS was 2,663,744 KiB.

Decision: reject scan-rate surface refinement as a thickness candidate and
keep it default-off as deterministic research infrastructure. The optimizer's
centroid plane residual, including its cross-fit form, is weakly coupled to the
neighborhood thickness evaluator; reducing it does not attack the remaining
map error. Because the map improvement is immaterial, the NDT localization
gate and unseen-dataset expansion are not run for this candidate. The
hierarchical surfel map remains the best research result.

### Connected field and surface-consolidation probes

A separate connected-field artifact tested whether seams between independent
0.5 m support cells caused the remaining thickness. It merges only 26-neighbor
surfels whose normals and mutual plane distances agree, refits a local plane
from their covariances, preserves every occupied output voxel, and never
crosses a perpendicular synthetic corner. The strict 8 degree/4 cm profile
found only three mergeable neighborhoods among 172,756 valid primary support
cells and changed 64 of 6.88 million output voxels. Even a 20 degree/10 cm
development probe changed only 2,609 voxels and was thickness-neutral.

Bounded propagation into unsupported neighbor cells was then tested. A single
nearby plane reached 1,130,395 fallback voxels at a 4 cm distance gate, but
regressed mean thickness 0.72%, p95 0.60%, and coverage 1.20%. Tightening the
gate to 1 cm still regressed all three metrics. Requiring two mutually
compatible support planes fixed the corner ambiguity but reached only 3,269
fallback voxels; mean changed -0.009%, p95 was unchanged, and coverage changed
+0.005%. Decision: keep the deterministic implementation default-off as
negative infrastructure, but reject both connected smoothing and neighbor
extension as thickness stages.

The next probe removed the fine-voxel clamp from supported projections, then
deterministically re-voxelized at 0.1 m. This explicitly consolidates multiple
thickness layers that project onto the same surface cell. Synthetic tests pin
an 8-to-4 layer merge and bitwise input-order invariance. On exp04 it merged
894,434 of 6,879,486 points (13.0%) and produced:

| comparison | mean thickness | p95 thickness | planar coverage |
|---|---:|---:|---:|
| versus best hierarchical surfel | -33.02% | +1.22% | +1.28% |
| versus original centroid baseline | -46.21% | -14.44% | +134.05% |

The mean result is material, but it is not sufficient evidence because p95
misses the target and consolidation changes sampling density. An even-scan map
was therefore evaluated with 63 unseen odd-scan NDT queries. Relative to the
same-run centroid map, fitness regressed 3.63% mean and 0.84% p95; recovered
translation error regressed 3.73% mean and 2.70% p95. Rotation error improved,
but cannot compensate for the failed residual and translation gates. Runtime
was 19.69 s and maximum RSS 2,912,080 KiB.

Decision: surface consolidation remains default-off and is rejected as a map
candidate. It demonstrates that the clamp is a real mean-thickness limiter,
but removing layers without improving the unsupported tail trades away NDT
localization geometry. It is not expanded to unseen datasets. The next
measurement must separate supported and fallback thickness before changing
support scale or sampling again.

### Supported/fallback tail attribution and tertiary support

A default-off diagnostic now partitions the final hierarchical map without
changing it: every output voxel appears exactly once in either the surfel-
supported PCD or the centroid-fallback PCD. On exp04 the supported partition
contained 3,555,938 points with p95 thickness 0.08385 m and 33.32% planar
coverage. The fallback partition contained 3,323,548 points with p95 thickness
0.13096 m and only 9.95% planar coverage. The remaining tail is therefore
localized to unsupported geometry rather than the already-projected surfels.

A third fallback-only support scale was added without changing the existing
0.5-to-0.3 path when set to zero. A broad 0.7 m scale added support to 604,057
exp04 voxels but mixed structures: mean regressed 0.52% and p95 0.27%. The
intermediate 0.4 m scale added support to 511,235 voxels and improved mean,
p95, and coverage on exp04. The same frozen 0.5, 0.3, then 0.4 m order was
applied without tuning to exp01 and exp07:

| dataset | mean change vs centroid | p95 change vs centroid | coverage change |
|---|---:|---:|---:|
| exp04 development | -19.86% | -15.86% | +133.01% |
| exp01 unseen | -20.78% | -28.47% | +178.49% |
| exp07 unseen | -13.62% | -22.71% | +114.81% |
| macro mean | -18.09% | -22.34% | +142.10% |

Relative to the preceding two-scale best, every dataset improves all three
metrics: mean by 0.21--0.29%, p95 by 0.45--1.07%, and coverage by 0.83--1.21%.
Exp01 completed in 64.17 s for a 693.18 s sequence (RTF 0.093) and peaked at
7,465,368 KiB RSS, within both resource gates. Two independent exp04 builds
matched byte-for-byte for the fused PCD, report, and both trajectories. The
new fused PCD SHA-256 is
`e6d0c7cb3f2f20b59a934e512340509450ba1fd4ae4accee31efe0c28b82f35d`.

The NDT gate remains negative. With an even-scan tertiary map and 63 unseen
odd-scan queries, fitness changed by +1.03% mean and +0.45% p95 relative to the
centroid map. Translation and rotation recovery improved slightly, but the
required residual improvement is absent. Decision: the 0.4 m tertiary scale
becomes the new best default-off research candidate, but it remains well short
of the -30% mean, -25% p95, and -10% NDT final gates.

### Boundary-complete fallback support

The fallback attribution led to a boundary-complete support probe. Each of the
0.5, 0.3, and 0.4 m support scales is evaluated on all eight half-voxel XYZ
grid phases. Phase zero owns every support decision it can make; later phases
may fill only outputs still unsupported at that scale, and finer scales may
fill only outputs unsupported by every coarser scale. This removes grid-boundary
blind spots without averaging incompatible planes or overwriting stronger
support. Focused tests pin boundary filling, phase-zero ownership, and bitwise
scan/point-order invariance.

The frozen eight-phase profile produced:

| dataset | mean change vs centroid | p95 change vs centroid | coverage change |
|---|---:|---:|---:|
| exp04 development | -22.77% | -19.80% | +161.33% |
| exp01 unseen | -25.23% | -36.11% | +237.10% |
| exp07 unseen | -15.98% | -26.14% | +129.81% |
| macro mean | -21.33% | -27.35% | +176.08% |

The p95 and coverage gates pass, but the mean gate does not. Exp01 peaked at
7,468,092 KiB and ran at RTF 0.46. Two independent exp04 runs produced exact
PCD, report, and trajectory bytes; the repeated fused PCD SHA-256 is
`6d68ee06f369f6a25440b7d3f7b29e65592487538a42812e042c42c7205eefb7`.
The stage remains default-off because its even-map/odd-query NDT residual
regressed 1.26% mean and 0.62% p95.

The NDT failure was also tested as a target-representation problem. Combining
the centroid target with the supported surfel partition improved odd-query
fitness by 4.52% mean and 2.47% p95. Combining it with the complete fused map
improved mean by 4.56% and p95 by 2.69%. Deterministic secondary weights of 2
and 4 reduced the mean gain to 2.55% and 2.85%; a half-density secondary target
regressed mean 0.19% while improving p95 5.46%. Point-count weighting therefore
has a clear optimum near one equal secondary target and cannot reach the 10%
NDT gate. The evaluator records primary, secondary, used-secondary, and total
point counts so this negative result is reproducible.

### Tail-selective surface consolidation

Eight-phase support and unconstrained consolidation solve complementary parts
of the metric: the former preserves a strong p95 while the latter removes
enough layer duplication to pass the mean target. A selective consolidation
therefore starts from the occupancy-preserving fused map and releases the fine
voxel clamp only when the chosen surfel plane moves a centroid by at least a
configured distance. Zero retains the original all-supported consolidation;
a high threshold retains the fused map. Both endpoints and input-order
determinism are covered by focused tests.

Two exp04 development probes were pre-limited to 2 cm and 5 cm. The 2 cm
profile selected 4,421,933 of 5,781,763 supported outputs and achieved -37.38%
mean, -17.44% p95, and +192.13% coverage. The 5 cm profile selected 3,139,298
outputs and improved every metric to -40.66% mean, -20.92% p95, and +216.12%
coverage. The threshold was then frozen at 5 cm before the unseen runs:

| dataset | mean change vs same-run centroid | p95 change | coverage change |
|---|---:|---:|---:|
| exp04 development | -40.66% | -20.92% | +216.12% |
| exp07 unseen | -34.28% | -32.75% | +358.79% |
| exp01 unseen | -38.90% | -33.13% | +264.74% |
| macro mean | **-37.95%** | **-28.93%** | **+279.88%** |

This is the first candidate to pass both multi-sequence thickness gates while
increasing coverage on every sequence. It is a post-pose map representation,
so the baseline/fused trajectories are unchanged; the exp01 baseline and fused
PCDs are byte-identical before and after its memory optimization. The initial
exp01 build peaked at 8,167,992 KiB. Deferring the full double-precision output
map, holding intermediate projections at PointXYZ precision, and releasing the
41,076,537 keyed observations before re-voxelization reduced final max RSS to
7,719,268 KiB. The optimized run took 823.65 s for the roughly 693 s bag
(RTF about 1.19), passing the strict 8,000,000 KiB and RTF 5 gates.

Decision: 5 cm selective consolidation becomes the leading default-off map
candidate. Geometry, coverage, trajectory non-regression by construction,
determinism of the pose/fused path, and resources pass. Promotion remains
blocked by the independent -10% NDT residual gate and requires a repeated
optimized selective-map artifact check before any default-on decision.

The optimized exp04 authoring path was subsequently repeated twice from the
same backend bag. All nine output files were byte-identical, including the
selective PCD and report. The selective PCD SHA-256 is
`1660011b552a962c4a1fee945b7d88289c4659aa4fb213c6fd6c3c67b7278309`
and the surfel report SHA-256 is
`f7b397eef6b955702df51683f82058d14528cb492f8a479d228dc9d35919715c`.
The two runs took 249.11 s and 244.25 s and peaked at 2,768,492 KiB and
2,768,496 KiB. Re-evaluation against the same-run centroid baseline confirmed
-40.6582% mean thickness, -20.9237% p95 thickness, and +216.1258% planar
coverage. The optimized selective-map determinism requirement therefore
passes; the old pre-optimization selective PCD differs by one 5 cm threshold
decision, while its baseline/fused PCDs, trajectories, and loop edge remain
byte-identical to the optimized runs.

### Tangent-sampled NDT localization target

The localization gate was separated from the visualization map after the
selective map showed that lower thickness and a useful NDT sampling density
are different representations. The target generator starts from the
occupancy-preserving eight-phase fused map, partitions it into 1.0 m NDT
voxels, and accepts a voxel as planar when it contains at least six points and
`lambda0/lambda2 <= 0.10`, `lambda1/lambda2 >= 0.05`. For every point in an
accepted voxel it adds eight symmetric samples: `+/-t1`, `+/-t2` at 0.50 m and
four diagonal samples at `0.50/sqrt(2)`. It adds no normal-direction samples
and retains every original finite point.

The radius/diagonal profile was tuned only on exp04, frozen, and then applied
unchanged to unseen exp07 and exp01. Maps contain only even-indexed input scans
and the NDT evaluator queries odd-indexed scans (`stride=20`, `offset=1`):

| dataset | fitness mean | fitness p95 | translation mean | translation p95 |
|---|---:|---:|---:|---:|
| exp04 development | -10.8325% | -16.4684% | +5.4232% | -1.0469% |
| exp07 unseen | -13.6183% | -15.5802% | -5.4709% | -3.3299% |
| exp01 unseen | -8.1221% | -3.3962% | -2.0751% | +1.1258% |
| macro | **-10.8577%** | **-11.8149%** | **-0.7076%** | **-1.0837%** |

The standalone `ndt_localization_target` executable and shared header are now
wired into CMake. Focused tests pin planar sample counts, symmetry, unchanged
normal thickness, volumetric rejection, invalid-radius rejection, and byte
determinism. Two exp04 generations produced byte-identical PCDs with SHA-256
`bd3d967c4537e58d7d3f9c5402dcb22b820bdfb76861de37ffdbbcbccb7f2870`.
Their path-normalized YAML reports were also identical. The generated target
contained 5,734,059 points (4,008,867 original plus 1,725,192 samples), ran in
at most 0.75 s, and peaked at 507,312 KiB.

Generated-PCD evaluation reproduced the on-load evaluator result exactly on
exp04 and exp01. On exp07 the only difference was `6.1e-10 m^2` in mean fitness
from float PCD serialization; p95 was identical. A full exp01 target generation
used 19,691,414 input points, produced 22,019,654 points, took 3.50 s, and
peaked at 1,990,468 KiB. The target authoring stage therefore passes the
determinism and resource gates.

Decision: keep both selective visualization-map authoring and tangent NDT
target authoring **default-off**. The frozen macro gate passes, and the map
stage leaves APE/RPE unchanged by construction, but exp01 misses the NDT target
on an every-sequence interpretation and exp04 translation mean regresses by
more than 2% in the recovery proxy. Promotion requires another genuinely
held-out suite and an explicit trajectory-level localization APE/RPE gate;
aggregate evidence alone is not sufficient for a safe default-on change.

### Direct scan-localization trajectory APE/RPE

The residual evaluator now optionally writes every final registered pose as a
TUM trajectory (`--trajectory`). A shared serializer normalizes quaternion
sign, canonicalizes signed zero, rejects non-finite poses, and is covered by
focused deterministic tests. Two full exp04 tangent runs produced identical
629-line TUM and CSV bytes; the TUM SHA-256 is
`4db1d7131d2d20d1c8a93477262fda7109849b1a1c2dd4e64f25fad615cde426`.

For each sequence, the map contains even scans and every odd scan is localized
independently with the same odometry initial pose, source voxel, and NDT
settings. The resulting trajectories are associated to the HILTI control
points within 0.11 s and independently aligned with scale-free SE(3). RPE is
the translation-vector error between consecutive associated checkpoints after
alignment. This measures the localization output directly rather than using
the earlier delta-from-initial-pose proxy:

| dataset | checkpoints | APE RMSE | APE mean | RPE RMSE | RPE mean |
|---|---:|---:|---:|---:|---:|
| exp04 development | 7 | -8.8570% | -3.0679% | -11.4621% | -5.6925% |
| exp07 unseen | 6 | -3.0419% | -1.8007% | -4.5434% | -3.7839% |
| exp01 unseen | 13 | -9.5024% | -8.2134% | -8.9657% | -8.4957% |
| macro | 26 | **-7.1338%** | **-4.3607%** | **-8.3237%** | **-5.9907%** |

Every APE/RPE metric improves, so the 2% non-regression gate passes directly
on all three sequences. Full odd-scan NDT fitness mean improves 10.5170%,
13.8012%, and 7.9055% respectively (10.7413% macro); p95 improves 7.9597%,
11.2164%, and 6.7143% (8.6301% macro). The denser evaluation therefore
confirms the mean residual gate in aggregate but also confirms that exp01 does
not pass an every-sequence policy. The default-off decision remains unchanged;
the missing evidence is another genuinely untuned sequence that passes the
frozen target, not trajectory non-regression.

### exp02 multilevel held-out audit and density/pose tradeoff

The unchanged single-ring profile was next applied to HILTI exp02 before any
exp02 result was inspected. The frozen backend input contains 4,302 exactly
paired odometry/cloud scans over 752.13 s. Its even-scan map used 2,151 scans,
174 submaps, and 270.2 m of travel. Map authoring took 1,456.25 s (RTF 1.94)
and peaked at 6,620,184 KiB. The map contained 15,724,080 occupied output
voxels. Against its same-run centroid baseline, the unchanged 5 cm selective
map achieved:

| metric | baseline | selective | change |
|---|---:|---:|---:|
| mean thickness | 0.058157903 m | 0.034778160 m | **-40.2005%** |
| p95 thickness | 0.118127485 m | 0.084016766 m | **-28.8762%** |
| planar coverage | 0.057246789 | 0.210034211 | **+266.8926%** |

Thus geometry, coverage, RSS, and RTF generalize to the multilevel sequence.
The frozen eight-direction tangent target did not: all 2,151 unseen odd scans
converged, but mean NDT fitness improved only 8.9369%. APE RMSE changed
+0.6650% (within the 2% gate), while RPE RMSE improved 8.2654%.

exp02 was then explicitly converted from held-out evidence into a development
sequence. Default-off, planar-only density probes exposed a hard tradeoff:

| target probe | NDT mean | APE RMSE | RPE RMSE | decision |
|---|---:|---:|---:|---|
| original 8 directions | -8.9369% | +0.6650% | -8.2654% | NDT miss |
| +0.25 m inner ring | -9.6461% | +1.9968% | -5.7120% | boundary miss |
| 16 outer directions | -10.0942% | +2.5813% | -5.2055% | APE fail |
| inner ring + two midpoint pairs | -10.1077% | +3.6991% | -4.6847% | APE fail |

An inner radius of 0.10 m, an inner radius of 0.40 m, and five NDT iterations
also failed the residual gate. All added samples remain exactly in the local
tangent plane and the original profile is unchanged by default. The optional
infrastructure is retained for research, but none of the exp02-tuned density
profiles is promoted. The evidence says that increasing target density alone
crosses the residual gate by moving the optimum too strongly; the next method
must constrain or validate the pose update rather than add more points.

#### Deterministic SE(3) pose-update regularization

The density result localized the remaining error to accepting the optimized
NDT pose in full. `map_ndt_residual_report` therefore gained an explicit
`--pose-update-gain` in `[0, 1]`: translation is linearly interpolated and
rotation uses quaternion slerp from the odometry initial pose to the NDT
optimum. The default is exactly 1.0 and reproduces the old stride-20 fitness
and translation values exactly. Fitness for a guarded pose is recomputed from
the transformed query cloud; it is not copied from the unguarded optimum.

A gain of 0.99 appeared to pass against the old unguarded baseline, but failed
the fair comparison where both maps use the same guard: exp02 APE was 2.6046%
worse. A strong odometry prior at gain 0.03 passes the full 2,151-query exp02
comparison against a gain-0.03 centroid baseline:

| metric | baseline guard 0.03 | 16-direction guard 0.03 | change |
|---|---:|---:|---:|
| NDT mean fitness | 0.062138551 | 0.055563116 | **-10.5819%** |
| NDT p95 fitness | 0.151733801 | 0.136065770 | **-10.3260%** |
| APE RMSE | 0.089760653 m | 0.091244191 m | **+1.6528%** |
| RPE RMSE | 0.125954137 m | 0.127669685 m | **+1.3620%** |

The candidate used 2,733,332 KiB RSS and 425.59 s. A second full run produced
byte-identical 2,151-line TUM and CSV artifacts. The TUM SHA-256 is
`61c3863d1f9600d8eef3a5da5354ada4db7e526b09d03be0e285bd10e791fa9b`
and the CSV SHA-256 is
`0bd525dd1d0491cc7ce08a83cd0f3fb75f61436c4511b2aead924c45023db543`.

The same frozen gain and 16-direction target were then compared against a
same-gain baseline on the three earlier sequences:

| sequence | NDT mean | NDT p95 | APE RMSE | RPE RMSE |
|---|---:|---:|---:|---:|
| exp04 | -12.5419% | -12.2776% | +0.4849% | +0.2756% |
| exp07 | -16.0055% | -13.2546% | -0.0846% | -0.1231% |
| exp01 | -8.4185% | -10.1285% | +0.1185% | +0.1314% |
| exp02 | -10.5819% | -10.3260% | +1.6528% | +1.3620% |
| sequence macro | **-11.8870%** | **-11.4967%** | **+0.5429%** | **+0.4115%** |

The four-sequence macro and exp04/07/02 individual gates pass, while exp01
still misses the per-sequence mean NDT gate. The frozen profile and hashes are
in `docs/research/artifacts/map-thickness-ndt-guard03-candidate-2026-07-23.yaml`.
Decision: remain default-off. The exp02 contradiction is resolved, so exp03
may now be used exactly once as the next untouched holdout; do not tune on it.

#### exp03 untouched holdout result

The candidate manifest was frozen before capture with SHA-256
`d1d76b9dbd7c3c2d3fb5d276b44d371fe62bf4f73ce0daf46a4c5afcfb645b5f`.
No parameter was changed after observing exp03. The capture contains 3,095
exact odometry/cloud pairs over 160.537 s. Its even map uses 1,548 scans, 133
submaps, 207.1 m travel, and two loop edges. Map authoring took 212.09 s
(RTF 1.32) and 2,904,580 KiB RSS.

| exp03 geometry | baseline | candidate | change |
|---|---:|---:|---:|
| mean thickness | 0.052662440 m | 0.035356915 m | **-32.8612%** |
| p95 thickness | 0.106008994 m | 0.077071068 m | **-27.2976%** |
| planar coverage | 0.094449920 | 0.311300294 | **+229.5930%** |

The frozen 16-direction target and gain 0.03 were then evaluated on all 1,547
odd scans against a centroid target using the same gain:

| exp03 localization | baseline | candidate | change |
|---|---:|---:|---:|
| NDT mean fitness | 0.100425079 | 0.086414311 | **-13.9515%** |
| NDT p95 fitness | 0.317252726 | 0.266671698 | **-15.9434%** |
| APE RMSE | 0.937860163 m | 0.938092093 m | **+0.0247%** |
| RPE RMSE | 0.749940879 m | 0.750155336 m | **+0.0286%** |

The map was rebuilt once: nine primary artifacts were byte-identical. The NDT
target was regenerated from that repeat map and was byte-identical, and the
repeat candidate TUM/CSV were also byte-identical. The five-sequence macro is
now -12.2999% NDT mean, -12.3860% NDT p95, +0.4393% APE RMSE, and +0.3349%
RPE RMSE. Full hashes and evidence are frozen in
`docs/research/artifacts/map-thickness-exp03-holdout-result-2026-07-23.yaml`.

Decision: exp03 passes every adoption gate without tuning. The implementation
still remains default-off because exp01 is the only sequence missing the
individual mean-NDT threshold (-8.4185%); the final policy must explicitly
choose between an every-sequence gate and the already-passing macro gate.

#### exp21 second untouched holdout result

The unchanged pre-holdout manifest was next applied once to exp21 outside the
building. Its frozen input has 1,528 exact pairs over 340.445 s. The even map
uses 764 scans, 80 submaps, 123.7 m travel, and one loop edge. It ran in
168.13 s (RTF 0.49) at 3,401,168 KiB RSS.

| exp21 geometry | baseline | candidate | change |
|---|---:|---:|---:|
| mean thickness | 0.058783005 m | 0.038111589 m | **-35.1656%** |
| p95 thickness | 0.118644469 m | 0.094931111 m | **-19.9869%** |
| planar coverage | 0.092257431 | 0.346325609 | **+275.3905%** |

The mean and coverage gates pass, but p95 misses the required 25% improvement.
The localization side passes strongly on all 764 odd scans:

| exp21 localization | baseline | candidate | change |
|---|---:|---:|---:|
| NDT mean fitness | 0.060477898 | 0.051064368 | **-15.5652%** |
| NDT p95 fitness | 0.150775282 | 0.129194013 | **-14.3135%** |
| APE RMSE | 0.237103548 m | 0.229894340 m | **-3.0405%** |
| RPE RMSE | 0.252519588 m | 0.246718852 m | **-2.2971%** |

No parameter was changed after this result. The six-sequence macro still
passes: geometry mean/p95 -37.0110/-27.1604%, coverage +268.5882%, NDT
mean/p95 -12.8441/-12.7073%, and APE/RPE -0.1407/-0.1038%. Full evidence is
in `docs/research/artifacts/map-thickness-exp21-holdout-result-2026-07-23.yaml`.

Decision: remain default-off under an every-sequence interpretation. exp21
must remain evidence and must not become a tuning sequence. Future tail-p95
work must be developed on other data and validated on a new untouched dataset.

## Reform sequence after exp04 attribution

The implementation order remains evidence-driven:

1. dense pose-refined map rebuild for scan/submap/revisit energy;
2. deskew and timestamp synchronization for within-scan energy;
3. fixed-lag multi-scan frontend for between-scan energy;
4. uncertainty-aware surfel fusion and dynamic-observation removal;
5. global surface bundle adjustment for residual submap/revisit energy.

No stage becomes default-on from exp04 alone. Promotion still requires the
multi-dataset gates in the active long-horizon goal.
