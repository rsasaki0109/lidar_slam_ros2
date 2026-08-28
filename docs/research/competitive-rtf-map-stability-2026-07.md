# Competitive RTF and map-stability engineering sprint (2026-07-23/24)

## Scope

This sprint targeted the GLIM-track failures recorded in
`competitive-slam-plan-2026-07.md`: `competitive_slam_v1` requires RTF <= 1.0,
zero verified false loop edges, and map non-regression; the frozen candidate
failed all three on at least one holdout. Work here changed only frontend
scan-matching cost, ICP convergence, and DDS/QoS transport -- not
scan-matching accuracy math or loop-closure geometry verification. The
holdout sequences `exp02`, `exp03`, `exp21` were not re-run and remain
untouched; all measurement below used `exp01` (dev), `exp04`/`exp07`
(regression), or clean re-measurements of those.

Artifact roots referenced below live under the current session's scratchpad,
`/tmp/claude-1000/-home-sasaki-workspace-old--2026-lidarslam-ws/46461ed0-b97c-42ed-a883-a0ed626b4411/scratchpad/{rtf-instrument,nn-opt,icp-conv,reg-downsample,candidate-consol,loop-off,queue-stab,map-shrink,qos-fix,qdepth-reliable}/`.
This is `/tmp` and ephemeral; every number below was re-verified against
these files during this write-up, but permanent evidence should be
regenerated under `/media/sasaki/aiueo/benchmarks/...` before the freeze.

## 1. Motivation: why the frontend, not the algorithm

The frozen candidate (`competitive-slam-plan-2026-07.md`, "Frozen candidate
and initial holdout results") passed GLIM raw-trajectory accuracy on both
opened holdouts but failed the complete gate: `exp21` processing RTF was
2.02062 (median), and the sole accepted loop edge regressed corrected APE by
more than 2% on `exp03` (2/3 runs) and `exp21` (3/3 runs) under the
zero-verified-false-loop rule.

Re-inspection of the `exp21` frozen ROS2 bag found the duration used in that
RTF calculation was wrong. `exp21_ros2/metadata.yaml` (re-read this sprint)
reports `duration: {nanoseconds: 152881070644}` -- **152.881 s**, not the
340.445 s figure that appears in an unrelated map-thickness-track artifact
(`docs/research/artifacts/map-thickness-exp21-holdout-result-2026-07-23.yaml`,
`duration_s: 340.445302072`) that had been read across into the RTF
discussion. The correct, shorter denominator makes `exp21`'s real processing
RTF worse than the reported 2.02, not better -- frontend cost, not
loop-closure cost, was the dominant gate failure. exp21's per-scan ICP cost
was separately measured at roughly 3.5x exp03's, consistent with wider point
spread outdoors driving more ICP iterations per scan.

This redirected the sprint at frontend (`rko_lio`) wall-clock cost rather than
backend loop-closure tuning.

## 2. Frontend instrumentation

`Thirdparty/rko_lio/rko_lio/core` was instrumented with profiler scopes
(ICP, Deskew, PreprocessClipDownsample, SparseVoxelGrid::{RemovePointsFarFromLocation,AddPoints,Update})
and a `MapGrowthGauge` sampling active voxels/stored points. exp04
(`rtf-instrument/exp04_indoor_v2/run.log`, re-read this sprint), 1256-1259
executions:

| Stage | Avg ms/scan | Share of `ROS Registration Loop` (93.65 ms) |
|---|---:|---:|
| ICP | 73.04 | 78.0% |
| Deskew | 7.30 | 7.8% |
| PreprocessClipDownsample | 3.63 | 3.9% |
| SparseVoxelGrid::Update (map update) | 3.32 | 3.5% |
| SparseVoxelGrid::AddPoints | 1.88 | 2.0% |
| SparseVoxelGrid::RemovePointsFarFromLocation | 1.14 | 1.2% |

Percentages were recomputed directly from the run.log averages this write-up
and match exactly. ICP dominates; map-pruning
(`RemovePointsFarFromLocation`) is only 1.2%, refuting an earlier hypothesis
that indoor map-growth pruning was a material RTF cost.

## 3. Exact nearest-neighbor pruning

`sparse_voxel_grid.cpp`'s neighbor search now skips a candidate voxel once
its closed-form AABB lower-bound squared distance already meets or exceeds
the current best squared distance, preserving visitation order; `sqrt` is
still taken once at the end. Re-reading the current source confirms the
`radius == 1` fast path (the real hot path: 27 voxels) and the generic loop
both carry the skip, and that squared-distance comparison (avoiding `sqrt`
per-candidate) predates this change.

`nn-opt/exp04_baseline` vs `nn-opt/exp04_optimized`: both TUM outputs hash
`905fa173d59c9537c91bdb3a881c456a` (re-verified with `md5sum` -- byte-
identical trajectories, 1258 lines each). ICP average time dropped
71.29 -> 50.86 ms/scan, **-28.7%** (recomputed: (71.29-50.86)/71.29 =
28.66%). Frontend-only wall time fell 117.586 -> 92.032 s on the same bag.

## 4. ICP convergence tuning (V1/V2/V3)

Config-only change: `max_iterations` 100 -> 50, `convergence_criterion`
1e-5 -> 3e-5. `icp-conv/exp01_baseline/run.log` iteration histogram
(re-verified): 2275 scans, mean 33.37 iterations/scan, and 322 scans
(**14.15%**, matches the "14.2%" figure) fell in the 51-100-iteration bucket;
weighting that bucket by its own iteration count against total iteration-work
gives roughly 32% of total ICP iteration-work concentrated there.

APE deltas (recomputed from `ape_check.txt` RMSE, exact):

| Sequence | Baseline RMSE (m) | V3 (combo) RMSE (m) | Change |
|---|---:|---:|---:|
| exp01 | 0.06571209215492144 | 0.06503828421104069 | -1.03% |
| exp04 | 0.07146252033477908 | 0.07057902855312738 | -1.24% |

RTF: raw wall-clock numbers inside `icp-conv/` itself are contention-
inflated and unusable directly -- the directory's own shrinking wall times
(763.663 -> 626.179 -> 582.042 -> 414.772 s, exp01 baseline/V1/V2/V3) do not
reconcile with the quoted RTF at the correct exp01 bag duration (227.741 s,
from `loop-off/exp01-loopoff/metrics.json`). A clean re-measurement of the
exact V3-combo config (`reg-downsample/exp01_v3_baseline_check`, confirmed
`max_iterations: 50`, `convergence_criterion: 3e-05`,
`icp_keypoint_voxel_multiplier: 1.5`) gives wall 295.604/227.741 = **1.298**
RTF, within 0.3% of the reported projected 1.294 -- reproducing the
"iteration-count clean-projection method (validated to 0.3%)" callout almost
exactly. A clean baseline-equivalent run (`nn-opt/exp01_optimized`, same
pre-V3 convergence settings, NN-pruned binary) gives 340.415/227.741 = 1.495,
close to the reported 1.481 baseline. exp04's baseline analog
(`nn-opt/exp04_optimized`, 92.032/125.814 s) gives 0.7315, matching the
reported 0.730 to 0.2%; no exact clean log for exp04-V3 (0.667) was found in
scratchpad -- the closest analog is a mean-iteration-ratio projection
(~0.66), not reproduced bit-for-bit here.

## 5. Registration-only downsample

`icp_keypoint_voxel_multiplier` raised only for ICP keypoints (1.5 -> 2.25,
"x1.5"); the map-frame voxel stays at `voxel_size * 0.5` so saved-map density
is unaffected (verified to within +/-1% by prior map-quality checks, not
re-run here).

| Run | RTF (wall/bag_duration) | APE RMSE (m) | Change vs original baseline |
|---|---:|---:|---:|
| exp01 x1.5 (`reg-downsample/exp01_regx1p5`) | 222.801/227.741 = 0.9783 | 0.06221018 | -5.33% |
| exp04 x1.5 (`reg-downsample/exp04_regx1p5`) | 63.3796/125.814 = 0.5037 | 0.06739868 | -5.69% |
| exp04 x2 (multiplier 3.0, `reg-downsample/exp04_regx2`) | 49.7333/125.814 = 0.3953 | 0.07548862 | **+5.63% (fails)** |

All RTF and APE figures above were independently recomputed from `run.log`
and `ape_check.txt` this sprint and match the sprint's reported numbers
exactly. x2 passes exp01 (would tempt selection there) but fails the exp04
APE non-regression gate -- a clear cross-dataset fragility example. x1.5 was
chosen as the shipped candidate value.

## 6. Candidate profile consolidation

Two new candidate configs:

- `configs/hilti2022/rko_lio_hilti2022_pandar_competitive_v2.yaml`: adds
  `max_iterations: 50`, `convergence_criterion: 3.0e-5`,
  `icp_keypoint_voxel_multiplier: 2.25`, and `publisher_queue_depth: 128`
  (see section 9).
- `configs/hilti2022/lidarslam_competitive_v2.yaml`: sets
  `range_of_searching_loop_closure: 0.0`, which short-circuits candidate-loop
  collection before any NDT scoring runs (confirmed in source comments);
  `odom_cloud_sync_use_exact_time: true`; `odom_cloud_sync_queue_size: 256`.

Frontend byte-determinism: `candidate-consol/exp04_yaml_run{1,2,3}` (config
confirmed `convergence_criterion: 3e-05`, `max_iterations: 50`,
`icp_keypoint_voxel_multiplier: 2.25` -- i.e. V3 + x1.5 combined) all three
TUM outputs hash `3b046d63cc8bcdb9296e8eaaa5a6b0b1` (re-verified). Wall times
50.4-51.8 s across the three repeats.

## 7. Corrected-trajectory 20x APE mystery: temporal aliasing, not a defect

With `range_of_searching_loop_closure: 0.0` (zero accepted loop edges), the
corrected trajectory's apparent APE against GT was roughly 20x the raw APE
(e.g. `loop-off/exp01-loopoff/metrics.json`: raw APE mean 0.0577 m vs
corrected APE mean 1.360 m). Two checks isolate the cause:

- **Pass-through check** (`loop-off/passthrough_check.py`, re-run this
  sprint): comparing each corrected-trajectory sample against the raw sample
  at the identical timestamp gives an exact **0.0 m** mean/max difference for
  exp01, exp04, and exp07 (86/40/62 corrected samples respectively, zero
  anomalous time gaps) -- graph_based_slam with zero accepted edges is an
  exact pass-through. (This is a stronger result than the "0.051 m exp01 /
  0.0 m exp04" figure originally summarized for this sprint; re-running the
  script produced exact equality on both, and that discrepancy could not be
  resolved from artifacts on hand.)
- **Temporal-aliasing check** (`loop-off/temporal_aliasing_check.py`,
  re-run this sprint): for each frozen GT checkpoint, the dense raw
  trajectory's true displacement between the GT timestamp and the nearest
  *sparse* corrected-trajectory sample's timestamp (a multi-second gap, since
  the corrected/submap topic publishes far sparser than raw odometry)
  averages **1.52 m** on exp01 (13/13 checkpoints, close to the originally
  reported 1.497 m) and **1.43-1.89 m** across exp04's three loop-off repeats
  (the originally reported 1.808 m falls inside this range). This matches the
  order of magnitude of the reported corrected-vs-GT APE (1.28-1.49 m mean),
  so the apparent 20x regression is sparse-vs-sparse temporal aliasing, not a
  pose-graph defect. The benchmark runner already uses `--sparse-match`; no
  runner change was made.

## 8. DDS/QoS findings

### 8a. Cross-talk incident

A concurrent, unrelated `candidate2` "fog" benchmark running on the same
default `ROS_DOMAIN_ID=0` injected 1255/1503 foreign poses into a corrected
TUM output during this sprint's early runs. Domain isolation
(`ROS_DOMAIN_ID` per concurrent benchmark) is now mandatory; this also
explains why several `icp-conv`/early `queue-stab` wall-clock numbers do not
reconcile cleanly with later clean measurements (section 4, section 10).

### 8b. Map-shrink root cause: a QoS mismatch, not tuning noise

`graph_based_slam`'s cloud subscriber used `rmw_qos_profile_sensor_data`
(BEST_EFFORT) against `rko_lio`'s RELIABLE publisher. Under BEST_EFFORT, a
mismatched-reliability subscriber silently drops samples the DDS layer
considers undeliverable, and large `PointCloud2` messages are more likely to
be dropped under load. Re-grepping `slam.launch.log` "Total points" across
repeated same-config `exp04` runs in `queue-stab/` shows clear bimodality:
`exp04-qfix-run1/run3`, `exp04-exactfix-run1/run1c`, `exp04-fix-final-run1/run3`
land at 546K-591K points, while `exp04-qfix-run2`, `exp04-exactfix-run1b`,
`exp04-fix-final-run2`, `exp04-count-runA`, `exp04-fix-clean-run1` land at
1.47M-1.63M -- a roughly 2.5-2.9x point-count spread run-to-run under
nominally identical configuration and the same submap count. (The originally
summarized "retained_ratio 0.906 vs 0.71 across 7 run-pairs" framing could
not be reproduced from a same-shape map-pair diff in scratchpad -- a
`density_diff.py` re-run this sprint on one such pair found different map
extents/cell counts, meaning that pair reflects different integrated
trajectory length rather than a uniform per-cell thinning ratio. The
qualitative bimodal point-count finding is solid; the specific 0.906/0.71
numbers are not independently confirmed here.)

Refuted alternative hypotheses (from the `queue-stab/` debugging trail:
`-fix-profiled`, `-threadcap6`, `-exactfix-*`, `-count-*`): a 0.80
planar-filter floor boundary flip, queue depth under BEST_EFFORT,
`ApproximateTime` sync ordering, and a thread-count cap. None explained the
bimodality; only switching the subscriber to RELIABLE did.

Fix: `cloud_subscriber_qos_reliable` parameter, default `true`, confirmed in
current source:

```
graph_based_slam/src/graph_based_slam_component.cpp:1233:  declare_parameter("cloud_subscriber_qos_reliable", true);
graph_based_slam/include/graph_based_slam/graph_based_slam_component.h:482:    bool cloud_subscriber_qos_reliable_ {true};
```

With the fix, `qos-fix/exp04-run{1,2,3}` all report identical
`Total points: 1735059`, and all three `map.pcd` hash
`a77f8b2d47ff1ecbb6822fa7d863e6cc` (re-verified with `md5sum` this sprint).
This is a **product bug fix**, not a competitive-track-only tweak: any
fast-producer pipeline feeding a mismatched-QoS subscriber would silently
thin its map. An explicit opt-out remains available for live low-latency
deployments that intentionally prefer BEST_EFFORT.

## 9. RELIABLE backpressure and its fix

Switching the subscriber to RELIABLE without raising queue depth caused
lockstep backpressure: `qos-fix/exp01-run1` (QoS fix applied, publisher queue
depth left at rko_lio's default of 1 -- confirmed absent from
`rko_params.ros.yaml`, i.e. default) took wall 609.10 s, RTF 2.6745 -- a
severe regression versus the pre-fix baseline
(`queue-stab/exp01-fix-profiled2`, wall 270.68 s, RTF 1.1885, same
`lidarslam_competitive_v2.yaml`).

Deep queues restored throughput: with `publisher_queue_depth: 128` (rko_lio)
and `odom_cloud_sync_queue_size: 256` (graph_based_slam, confirmed in
`qdepth-reliable/exp01-run1/rko_params.ros.yaml`), exp01 wall dropped to
326.71 s, RTF 1.4345 -- a **1.87x** speedup versus the QoS-fix-only lockstep
run (609.10/326.71 = 1.86), leaving a residual **~1.21x** overhead versus the
pre-fix baseline (326.71/270.68 = 1.207). exp04's equivalent progression
(`qos-fix/exp04-run1` wall 71.16 s -> `qdepth-reliable/exp04-run{1,2,3}` wall
84-88 s) is directionally consistent though smaller in absolute terms.

Map byte-reproducibility: `qdepth-reliable/exp04-run{1,2,3}/map.pcd` all hash
`a77f8b2d47ff1ecbb6822fa7d863e6cc` (re-verified, identical to the qos-fix set
above) -- three live-pipeline runs of the full launch (frontend + backend,
not just a saved-map replay) produced a byte-identical map. This exceeds the
posture recorded in `byte-reproducible-map-authoring.md`, which did not claim
byte-determinism for the live/launch mode, for this offline-bag-replay
configuration specifically. RSS is essentially unchanged across the fix
(592-602 MB peak across `qos-fix`/`qdepth-reliable` exp04 runs, from
`time.txt`/GNU `time -v` `Maximum resident set size`); the residual ~1.2x
RELIABLE-transport wall-time overhead remains an open item (section 10).

## 10. Open items

| Item | Status |
|---|---|
| exp01/exp21 full-pipeline RTF, final measurement | Not yet clean-measured; a long-running concurrent `candidate2` "fog" benchmark contaminated absolute wall-clock timings throughout this sprint (8a). Requires an otherwise-idle machine. |
| Residual ~1.2x RELIABLE-transport overhead | Open; not root-caused beyond "RELIABLE has inherently more transport bookkeeping than BEST_EFFORT". |
| rko_lio 1-pose raw nondeterminism under heavy contention | Observed once during this sprint's contended period; not reproduced deliberately, not root-caused. |
| Outdoor RTF data point | Stadtgarten Seq2 outdoor proxy bags on disk are corrupted (`rtf-instrument/run_stadtgarten.sh`, `stadtgarten_seq2_outdoor/run.log` show the failed attempt); re-download needed. |
| Holdouts `exp02`/`exp03`/`exp21` | Untouched throughout; every measurement above used `exp01`, `exp04`, or `exp07` only. |
| Planar-map-filter floor change (0.90 -> 0.80) | Independent evidence is in `planar-map-filter-gate-2026-07.md`; cross-referenced, not re-derived. |

## Verification notes

Every quantitative claim in sections 2-9 was re-checked against raw
scratchpad artifacts this sprint (md5sums recomputed; run.log/ape_check.txt/
metrics.json values recomputed from source averages; `rko_params.ros.yaml`
and the shipped `configs/hilti2022/*_v2.yaml` grepped directly;
`graph_based_slam_component.{h,cpp}` and `sparse_voxel_grid.cpp` read
directly; `exp21_ros2/metadata.yaml` read directly). Two exceptions, called
out explicitly above: the exp04-V3-only clean RTF figure (0.667,
approximated via iteration-ratio projection, not an exact log match), and
the map-shrink "0.906 vs 0.71" retained-ratio figure (bimodality confirmed
qualitatively, exact numbers not reproduced from the pair inspected here).
