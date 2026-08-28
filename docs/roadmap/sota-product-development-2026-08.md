# SOTA Product Development Plan (2026-08)

This plan turns “best LiDAR SLAM OSS” into measurable engineering work. A
SOTA claim is always limited to a named dataset, metric, revision, and execution
condition. The project must not publish a blanket “best overall” claim.

## Phase 0: Current Baseline

| Area | Current evidence | Honest status | Next proof needed |
| --- | --- | --- | --- |
| HILTI `exp04` accuracy | lidarslam_ros2 0.0565 m median APE vs GLIM CPU 0.0866 m | Measured win in the existing three-run snapshot | Re-run once after the next frozen release candidate |
| HILTI `exp04` runtime | RTF 0.993 vs GLIM CPU 0.244 | GLIM wins | Profile the frontend/backend boundary and remove the largest bottleneck |
| HILTI `exp04` memory | 586.83 MB vs GLIM CPU 690.88 MB maximum peak RSS | Measured win in the existing snapshot | Preserve under the same-input release candidate |
| Multi-dataset research candidate | Voxel-SLAM v17 geomean APE 2.2836 m vs GLIM 5.0779 m | Promising, but not the default and not fresh-blind | Graduate only after default-path and holdout gates pass |
| FAST-LIVO2 comparison | Historical exp04 ROS1/paced snapshot scores 0.05298 m median APE, about 4.7 GiB RSS, and about 135 s mapper wall time | FAST is the current accuracy target, but the old pacing/input/runtime contract is not comparable to the current unpaced harness | Complete one bounded, same-input current-contract run |
| Reproducibility | Frozen profiles, hashes, reports, and fail-closed publication tooling exist | Strong infrastructure, incomplete current rival evidence | Produce a compact release evidence bundle |
| Extensibility | Versioned registration plugin API, clean consumer/template, optional FAST/Small adapters, and transactional loader exist | Humble/Jazzy absent/present builds and targeted runtime rollback gates pass | Add the matrix to CI and broaden real-bag plugin characterization |
| First use | Docker demo is one command; source demo is two commands | Good start, fragmented diagnostics | One doctor and one source first-map entrypoint |
| Continued use | Run diagnosis and map verification exist | Useful tools, fragmented workflow | Saved run manifest, resume, support bundle, migration guide |

The numbers above describe existing evidence, not a new 2026-08 measurement.
See [Comparison](../comparison.md) for revisions, limitations, and reproduction
details.

### 2026-08-28 registration dependency checkpoint

The campaign at
`/media/sasaki/aiueo1/benchmarks/registration-plugin-dependency-capture-20260828-decoded-bound-v8`
completed Humble/Jazzy × dependency absent/present exactly once per row.  All
four rows completed 19 phases with `PASS_REVIEW_REQUIRED`; the aggregate has
zero partial rows. This proves dependency capture only; it is not a runtime or
performance result.

The follow-up compatibility work passed all four intended legs: clean external
consumer plus author-template builds on Humble and Jazzy with optional
dependencies absent, and real `small_gicp` plus `fast_gicp` adapter builds and
direct-fixture tests on both distros with the dependencies present. The helper
CMake module is now exported by `lidarslam_plugin_interfaces`, so an external
plugin no longer reaches back into this repository's source tree. Humble also
exposed a portable-test bug: `PointXYZI` padding was being compared as if it
were public point data; the test now compares the four public float fields
bit-for-bit.

The actual DSO gate loads the built plugin, validates its contract, runs a load
session, and reports no host/plugin ODR collision. Targeted loader tests also
pass constructor rejection, activation rejection with lease preservation, and
post-commit rollback; scanmatcher resource failure restores the prior external
candidate, and backend preflight rejection remains fail-closed. These are
runtime compatibility results, not SLAM accuracy or performance claims.

### 2026-08-28 development checkpoint (`n=1`)

Two distinct configurations were each run once on the current dirty
development tree. The product-default profile completed in 79.94 s (RTF
0.635), used 606,760 KiB peak RSS, and scored 0.07156 m APE RMSE over all seven
control points. The competitive-v2 profile completed in 54.80 s (RTF 0.436),
used 591,372 KiB peak RSS, scored 0.06802 m over all seven control points, and
produced a verified Autoware map with 1,735,059 points across 32 tiles.

On the exact six-point common reference used by the existing GLIM snapshot,
competitive-v2 scores 0.04712 m versus GLIM's 0.08662 m three-run median. Its
577.51 MiB peak RSS is also below GLIM's 674.15 MiB median, but its 54.80 s
wall time remains above GLIM's 30.66 s median.

This is a development signal, not README claim evidence: the worktree and
rival source closure were not sealed and ours has only one measurement. It
shows the immediate priorities clearly: preserve the observed accuracy and
memory advantage, then reduce competitive-v2 wall time by at least 1.79x to
beat the GLIM median. RKO-LIO took 47.51 s of the run; its 1,256 ICP calls
averaged 21.60 ms, making frontend ICP the next measured bottleneck. The run
also exposed and fixed three harness blockers:
ROS 2 launch argument construction, leaked ThreadSanitizer instrumentation,
and ordinary-run completion being misclassified as M6A10 evidence failure.
Sparse graph anchors are now retained as `traj_corrected_sparse.tum`, while
`traj_corrected.tum` receives full-rate correction propagation before scoring.

The historical FAST-LIVO2 exp04 snapshot is a separate directional constraint,
not a fair current comparison. Its three-run all-seven-point APE median is
0.05298 m, so competitive-v2 would need roughly a 22% reduction from 0.06802 m
to beat it on this metric. That snapshot used a ROS1-converted bag and paced
playback, consumed about 4.7 GiB peak mapper RSS, and took about 135 s mapper
wall time. The next FAST run must use the current same-input, unpaced online-
compute and memory contracts before any cross-system claim; the old result is
nevertheless sufficient to prevent treating the GLIM-only win as overall SOTA.

A later quiescent development run on the same day validated the task-local
correspondence reduction: the authoritative 1,258-pose RKO trajectory is
byte-identical to the pre-change dump (`3ffde307...`), while mean ICP time moved
from 21.60 ms to 20.92 ms and RKO-LIO wall time moved from 47.51 s to 46.55 s.
The full pipeline completed in 52.74 s (RTF 0.419), used 599,800 KiB peak RSS,
and passed all eight Autoware map checks. Because this is `n=1`, the timing and
RSS deltas are directional rather than statistical claims; numerical identity
and removal of the shared hot-path atomic are the adoption grounds.

The optimized piecewise-gyro candidate also completed its single predeclared
development run. Scan-local prefix SO(3) integration reduced deskew from the
historical 62.32 ms to 8.89 ms per scan. The pipeline completed in 50.40 s
(RTF 0.401), used 593,292 KiB peak RSS, passed all eight map checks, and scored
0.05057 m interpolated APE over all seven checkpoints. That clears the 0.05307 m
all-seven accuracy target and is directionally below the historical FAST-LIVO2
median, but it scores 0.04745 m on the six-point common subset versus the frozen
competitive-v2 value of 0.04675 m (a 1.48% regression). The candidate therefore
remains ungraduated under its predeclared no-regression gate. No README claim is
authorized from these dirty-tree, single-run results.

## Measurement Policy

- Normal development and OSS comparison uses **one complete run per fixed
  condition (`n=1`)**.
- Repeat only when the run is anomalous, a regression must be isolated, or a
  release/publication result needs confirmation. Every table states its actual
  `n`; an `n=1` result is never presented as statistical evidence.
- Same-input comparisons freeze dataset hash, calibration, revision, build
  type, host, CPU/thread limits, warm-up policy, scorer, and completion rule.
- Failed and incomplete runs remain visible. A result is publishable only when
  all required artifacts and identities pass the publication gate.
- “SOTA” means a win on a named metric and benchmark. “Overall SOTA” is not a
  release label unless all predeclared accuracy, completion, runtime, memory,
  map-quality, and reproducibility gates pass.

## Priorities And Capacity

Use the following planning budget per milestone:

- 60% core SLAM accuracy, completion, runtime, and map quality.
- 25% first-use and continued-use product quality.
- 15% benchmark/release infrastructure.

Infrastructure work is pulled forward only when it blocks the next bounded
SLAM run, user workflow, or release decision.

## Delivery Phases

### Phase 1: First Useful Map

Target: a new user reaches a verified map in at most three commands, with a
median clone-to-first-map time of 15 minutes or less on a documented machine.

- Provide a read-only environment doctor with actionable fixes.
- Provide one source first-map command while retaining the one-command Docker
  path.
- Make missing ROS setup, tools, submodules, disk space, dataset, and output
  failures point to the exact next command.
- Test Humble/22.04 and Jazzy/24.04 clean environments.
- Record local, opt-in timing only; do not add network telemetry.

Exit gate: at least 90% of clean supported-environment trials produce a
verified map, and all failures leave an actionable diagnostic.

Implementation checkpoint: `lidarslam_doctor.py` and `run_first_map.sh` now
provide the unified diagnostic and first-map entrypoints. Clean Humble/Jazzy
trial evidence remains before this phase can pass its exit gate.

### Phase 2: Fair OSS Baseline

Freeze ours, GLIM, and FAST-LIVO2 revisions and run each condition once. Start
with HILTI `exp04`, then one MID-360 sequence and one fresh holdout. Capture APE,
completion, RTF, peak RSS, map-quality metrics, configuration, host identity,
and all failures in the same report.

Exit gate: a reviewer can reproduce the table from published commands and
artifacts, and the README generator refuses unsupported prose.

### Phase 3: Core SLAM Iteration

Profile before changing algorithms. Improve the largest observed bottleneck,
then run the fixed condition once. Candidate areas are frontend registration,
keyframe/submap policy, loop candidate ranking and verification, optimizer
scheduling, memory ownership, and deterministic offline I/O.

Exit gate: aggregate accuracy improves by at least 10% against the best pinned
rival without a completion, real-time, memory, or map-quality regression. If it
does not, document the rejected candidate and select the next bottleneck.

#### Runtime iteration log

- `2026-08-28 / icp-task-local-correspondence-count`: replace the shared
  per-correspondence atomic increment in RKO-LIO's deterministic TBB reduction
  with a task-local integer reduced alongside H/b/chi. The Release build and
  all RKO-LIO tests pass. A full exp04 execution under severe unrelated CPU
  load produced an authoritative 1,258-pose frontend dump byte-identical to the
  pre-change dump (`3ffde307...`), proving numerical equivalence for that
  sequence. A later quiescent `n=1` run preserved that exact dump, reduced mean
  ICP time from 21.60 ms to 20.92 ms, and completed RKO-LIO in 46.55 s. Adopt
  the simpler task-local reduction; do not present the one-run timing delta as
  a release performance claim.
- The same interference exposed a benchmark observation flaw: the ROS topic
  logger missed startup poses even though the offline node's atomic dump was
  complete. The benchmark now requires exactly one authoritative full-rate
  dump and uses it for scoring in every map-save mode.
- `2026-08-28 / competitive-piecewise-v1`: historical piecewise-gyro and
  fixed-lag candidates scored 0.052948 m and 0.052914 m respectively on all
  seven exp04 points, at non-realtime RTF 1.36--1.79. The current competitive-
  v2 raw and graph-corrected paths are position-identical (no accepted loop
  edges), so the FAST accuracy gap is frontend-only. A bounded candidate now
  adds only `piecewise_gyro_deskew=true` to competitive-v2, retaining its
  50-iteration budget, 2.25 keypoint-voxel multiplier, convergence threshold,
  and publisher queue. One quiescent run must beat or match 0.05307 m, remain
  RTF <= 1, preserve memory/map gates, and avoid regression on the six-point
  common subset before adoption.
- `2026-08-28 / competitive-piecewise-v1 result`: the optimized prefix
  integrator clears the all-seven, realtime, memory, and map gates at 0.05057 m
  interpolated APE, RTF 0.401, 593,292 KiB, and 8/8 map checks. It misses the
  strict common-six no-regression gate by 1.48% (0.04745 m versus 0.04675 m), so
  keep it as a research profile rather than graduating it. The next candidate
  must use a sensor/uncertainty-derived decision fixed before evaluation; it
  must not branch on exp04 time, checkpoint identity, or post-hoc GT tuning.
- `next / uncertainty-gated-zoh-v1 preregistration`: compare the mean-rate and
  prefix-integrated rotations at IMU boundaries and use a covariance-normalized
  discrepancy to select/blend them. Differences within a fixed three-sigma
  bound return the existing mean model directly; high-SNR within-scan angular
  motion may move continuously toward the piecewise result. Non-finite input,
  timestamp reversal, insufficient IMU coverage, or an abnormal sample gap
  fails closed to the mean model. The decision must use sensor calibration or
  a deterministic GT-blind static-initialization noise estimate, and record its
  mode, calibration/noise identity, decision counts, and fallback reasons.
  HILTI's checked-in calibration contains gyro bias but no noise density or
  clock-offset uncertainty, so no guessed constants are authorized. Before any
  run, freeze the estimator and require common-six APE <= 0.046755 m,
  all-seven APE <= 0.05307 m, 1,258 poses, RTF <= 1, memory acceptance, and all
  map checks. Apply the same rule without per-sequence thresholds to exp01,
  exp04, exp07, and the fresh holdout.

### Phase 4: Continued Use

- Save an immutable run manifest containing effective config, revisions, input
  identity, output inventory, and metrics.
- Resume interrupted offline sessions without silently mixing configurations.
- Export a privacy-safe diagnostic bundle with logs and redacted environment
  facts.
- Publish compatibility and config-migration notes for every stable release.
- Maintain task recipes for Autoware mapping, MID-360 field capture, large
  loops, GNSS georeferencing, and coloured maps.

Exit gate: an operator can reproduce, resume, diagnose, and migrate a mapping
job without reconstructing undocumented shell history.

Implementation checkpoint: schema-v2 portable run manifests now hash every
rosbag metadata/storage byte and bind the ordered records into a deterministic
input tree identity. A read-only verifier reopens that identity together with
the effective configs, revision/tracked binary-diff identity, and output
file hashes without a large-file size-only exception; a redacted
support-bundle exporter is also present. The public map runner rejects
non-empty output directories instead of silently mixing prior artifacts.
Identity-bound checkpoint/resume semantics and release migration notes remain
open.

### Phase 5: Evidence And Release

Publish only generated, qualified claims. Each row includes revision, input
hash, host/limits, scorer, measurement count, completion, APE, RTF, RSS, and map
quality. The release checklist verifies docs, source builds, supported distro
matrix, plugin consumer compatibility, and evidence bundle hashes.

### Phase 6: Extensible OSS Platform

Complete the versioned frontend/backend plugin contracts, lifecycle and
thread-safety rules, capability discovery, ABI/API compatibility policy,
out-of-tree consumer SDK, minimal plugin template, conformance tests, and
fail-closed fallback behavior. Keep the default installation small; advanced
algorithms remain optional plugins.

### Phase 7: Long-Term Differentiation

Long-term SLAM is the primary future differentiator: persistent map identity,
multi-session localization and merging, change detection, map aging, dynamic
object memory, bounded storage, and safe rollback. Application differentiation
includes fleet map maintenance, construction progress/change reports,
warehouse digital twins, disaster inspection, vegetation/infrastructure
monitoring, and privacy-preserving on-premise map updates.

These items are intentionally scheduled after the default single-session path
and evidence pipeline are reliable. Architecture boundaries and staged gates
are detailed in [Extensible OSS Architecture](extensible-architecture-2026-08.md).

## Milestone Scorecard

| Milestone | Product metric | SLAM/evidence metric |
| --- | --- | --- |
| M1 First map | ≤3 commands, p50 ≤15 min | verified Autoware bundle |
| M2 Repeat use | saved manifest + resume + diagnostic bundle | byte-identifiable inputs/config/outputs |
| M3 Competitive | one run per fixed condition | complete APE/RTF/RSS/map-quality table |
| M4 Release | supported clean-env success ≥90% | all publication and regression gates pass |
| M5 Platform | external plugin builds on Humble/Jazzy | no silent fallback; conformance suite passes |
| M6 Long term | repeatable multi-session workflow | change/aging/rollback metrics pass |
