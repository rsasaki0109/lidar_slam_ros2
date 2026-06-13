# Byte-Reproducible, Globally Refined Map Authoring

This document describes the headline engineering result of the v0.6 and v0.7
release arc for `lidarslam-ros2`: offline map authoring can now produce
byte-reproducible artifacts, apply a clean-room global refinement pass, and gate
the map that was just built against frozen quality thresholds. The intended
claim is narrow: same input bag, same build, same machine, same map bytes; then
a refined trajectory and map quality report whose acceptance rules are fixed
before the optimizer is allowed to compete against them.

## Provenance

The refinement design is clean-room. It was derived from published papers only:
BALM2 (`arXiv:2209.08854`), HBA (`arXiv:2209.11939`), and the plane extraction
paper (`arXiv:2305.00287`). The reference GPL implementations were not read.
This repository remains BSD-2-Clause.

The byte-identity claim is also intentionally scoped. The release gate claims
byte identity for the same source tree, same build, and same machine. It does
not claim cross-machine reproducibility. A cross-machine claim would require
pinning compilers, CPU flags, math libraries, and other numerical details that
are outside the current release gate.

## Why Determinism In SLAM At All

Most SLAM stacks are not deterministic at the artifact level.

Even when the algorithm is conceptually deterministic, the implementation often
contains thread scheduling effects, wall timers, callback races, unordered
container traversal, or floating-point reductions whose order changes between
runs. The resulting maps are usually close enough to look the same in a viewer,
but they are not byte-identical.

For interactive research this may be acceptable. For map authoring it is a
problem.

A map used as a deliverable for autonomous driving or Autoware is part of a
downstream build chain. If the same recorded input can produce a slightly
different map on each run, every later decision becomes ambiguous:

- a regression cannot be bisected cleanly;
- a quality gate cannot distinguish a real change from run-to-run noise;
- a reviewer cannot tell whether an "improvement" is algorithmic or accidental;
- a release cannot prove that the shipped artifact is the one that passed.

Determinism changes the engineering contract. Mapping stops being a manual
craft step and becomes a build step:

```text
same input bag -> bit-identical map -> enforceable in CI by md5
```

That is the foundation for the rest of the v0.7 work. A global optimizer and a
map-quality gate are only useful as release mechanisms if the input artifacts
they judge are themselves reproducible.

## v0.6: Core/Shell Refactor

The v0.6 work split both the backend and the frontend into two layers:

- a deterministic, single-threaded core;
- a thin ROS shell around that core.

The backend is `graph_based_slam`. The frontend is `scanmatcher`. In the online
ROS path they still run as ROS nodes. In the offline path, recorded inputs are
replayed through the pure core with controlled ordering.

Two offline runners are central:

- `graph_slam_offline_runner`
- `scan_matcher_offline_runner`

`graph_slam_offline_runner` consumes a recorded odometry-and-cloud backend input
bag and emits the backend artifacts. `scan_matcher_offline_runner` consumes the
raw sensor bag in lockstep and exercises the frontend determinism surface.

The important architectural change is that the deterministic core is no longer
driven by wall time. Loop search is event-driven. The legacy wall timer was
removed entirely in v0.7 Phase 0, after proving byte-identical behavior before
and after the removal.

The release gate runs each offline runner three times and requires
byte-identical outputs. The backend determinism stage checks artifacts including:

- `loop_edges.csv`
- `trajectory_optimized.tum`
- submap CSVs

The canonical md5s on the MID-360 substrate stayed stable across the v0.7
development arc:

| artifact | canonical md5 |
|---|---|
| backend loop edges | `feee9547...` |
| backend optimized trajectory | `92676db4...` |
| frontend output | `0ca41ec6...` |

The frontend canonical output corresponds to 800 poses and 11 submaps.

This is the first release result: the offline authoring path can be treated like
a deterministic compiler for maps, within the same-build and same-machine scope.

## v0.7 Phase 1: Measure Before Optimizing

Before adding a global refinement optimizer, v0.7 froze the map-quality
measurement layer.

The purpose was to avoid a common failure mode: building an optimizer and then
tuning the metric until the optimizer looks good. The v0.7 ordering was the
opposite. The map-quality metrics and their extraction profile were frozen
before optimizer work started.

The quality layer reports three main signals.

| metric | purpose |
|---|---|
| Mean Map Entropy (MME) | local map crispness using voxel-hash neighborhoods |
| plane thickness RMS | wall and planar-surface sharpness |
| planar coverage | how much of the map supports the plane-thickness score |

MME uses voxel-hash neighborhoods with radius `r=0.5m`.

Plane thickness RMS uses adaptive voxel plane extraction: octree splitting,
PCA-based planarity testing, and per-plane thickness scoring in the style of
`arXiv:2305.00287`.

The extraction profile is frozen in code:

| parameter | value |
|---|---:|
| thickness cap | `0.15m` |
| planarity ratio | `4.0` |
| minimum points | `10` |
| depth | `4` |
| coverage floor | `0.05` |

The profile also carries support values. Coverage, valid fractions, and an
explicit "not meaningful" state are part of the result. That matters because
low-planarity scenes can otherwise produce a single scalar that looks precise
but says little about map quality.

The baseline finding across all five gate substrates was simple: before
refinement, walls are roughly 9 cm RMS. On the indoor construction data, planar
coverage could be as low as 8.5%. That is a useful failure signature. Blur does
not only thicken the planes that survive extraction; it can also erase planes
from the fixed extractor entirely.

That is why planar coverage is included in the gate. If refinement is real,
coverage should often rise as blurred geometry crosses fixed extraction
thresholds. If only a scalar thickness score improves while support collapses,
the result is suspect.

## v0.7 Phase 2: Clean-Room Hierarchical Plane Bundle Adjustment

The v0.7 optimizer is a clean-room hierarchical plane bundle adjustment pass.

The cost is BALM2-style: for each planar cluster, minimize the smallest
eigenvalue of the plane scatter matrix. In compact form:

```text
cost = lambda_min
```

The implementation aggregates points into pose-local clusters. Each cluster is
represented as a `4x4` homogeneous matrix:

```text
H = sum T C T^T
```

This changes the computational shape of the problem. The cost, gradient, and
Hessian scale with the number of poses participating in a cluster instead of the
number of raw points. That is the difference between refining a map and merely
evaluating a dense point cloud.

The derivative implementation includes:

- exact first derivatives;
- exact second derivatives;
- the eigenvector-curvature term;
- cross-pose centroid coupling.

The solver uses Levenberg-Marquardt with gauge fixing through a reduced system.
It also keeps soft pose priors on the graph solution:

| prior | value |
|---|---:|
| translation sigma | `0.30 m` |
| rotation sigma | `2 deg` |

Steps are limited at the whole-vector level. The hierarchy uses overlapping
windows with window size 16 and stride 8. Each window has an anchored gauge.

The acceptance rule is conservative: if refinement does not strictly improve
the judged result, the pose-graph solution is returned bit-unchanged.

That rule matters for release engineering. It means enabling refinement cannot
silently degrade a substrate and still produce a new trajectory just because an
optimizer ran. The optimizer has to earn the right to change the artifact.

### A Cautionary Failure

The priors are not cosmetic.

In a synthetic corridor with periodic doorframes and priors disabled, the
optimizer found a repeated-geometry false minimum. It jumped 1.2 m to the wrong
alignment and made that wrong map crisper.

That is the dangerous version of map refinement: "polishing the wrong map."
The visual and planar metrics can improve locally while the trajectory becomes
physically wrong.

The production configuration keeps the soft priors enabled for real data. They
physically forbid that repeated-geometry jump while still allowing the optimizer
to remove the small residual blur left by the pose graph.

### Numeric Determinism Rules

The numeric core follows deterministic implementation rules:

- sorted voxel iteration;
- fixed accumulation order;
- no `-ffast-math`;
- canonicalized eigenvector signs;
- an eigen-gap floor.

These are not style preferences. They are what allow a floating-point optimizer
to remain part of a byte-reproducible authoring pipeline.

## v0.7 Phase 3: Thresholds, Holdout Hygiene, Then Default On

Phase 3 turned the metrics and optimizer into a release gate.

The backend-input bags were recorded for two RTK-SLAM construction-hall
sequences. The dataset uses total-station surveyed checkpoints and is public
under CC-BY 4.0.

Thresholds for the indoor blocking profile were selected only from tuning
substrates:

- `construction_seq1`
- the NTU evidence

The held-out construction sequence was not used to select those thresholds.
After the profile was fixed, `construction_seq2` was evaluated untouched as the
holdout. All five profile rows passed.

The indoor blocking profile rows are:

| row | blocking threshold |
|---|---:|
| thickness mean | `<= 0.085 m` |
| thickness p95 | `<= 0.15 m` |
| planar coverage | `>= 0.30` |
| MME | `<= -0.80 nats` |
| MME valid fraction | `>= 0.90` |

Outdoor scenes remain report-only. The current outdoor substrates are dominated
by vegetation, and the coverage baseline is 0.12-0.16. That is not yet a strong
enough support regime for a blocking refinement claim.

Refinement is now default-on in the offline runner. This was safe to do because
the pose-graph artifacts are byte-identical either way. The canonical backend
md5s remain the same for `loop_edges.csv` and `trajectory_optimized.tum`; the
refined trajectory appears alongside them.

The gate checks the refined map that the gate itself just built. It does not
check a pre-existing map artifact that could have been hand-polished outside the
pipeline.

## Evidence Table

The table below is the compact release evidence.

| substrate | metric | before (pose graph) | after (refined) |
|---|---|---:|---:|
| NTU VIRAL tnp_01 (Leica prism GT) | APE RMSE | 0.8460511516520233 m | 0.8190124186992095 m (-3.2%) |
| | thickness mean | 0.084944868 m | 0.074452257 m (-12.4%) |
| | thickness p95 | 0.116538522 m | 0.110470374 m |
| | planar coverage | 0.496509563 | 0.573336748 (+15.5%) |
| | MME | -0.970478040 | -1.110389903 |
| construction_seq1 (total-station GT, tuning) | APE RMSE | 0.7800603471740267 m | 0.7744167604040919 m (-0.7%) |
| | thickness mean | 0.077957119 m | 0.075093748 m (-3.7%) |
| | planar coverage | 0.377912074 | 0.394277537 (+4.3%) |
| | MME | -0.894974483 | -0.932375982 |
| construction_seq2 (total-station GT, HOLDOUT) | APE RMSE | 0.8376950131160659 m | 0.825250416881328 m (-1.5%) |
| | thickness mean | 0.082155760 m | 0.078891815 m (-4.0%) |
| | planar coverage | 0.423900020 | 0.424767096 |
| | MME | -0.915227013 | -0.942085924 |

The key reading is that every ground-truth substrate improves in both trajectory
and map quality.

That combination is important. A trajectory-only improvement could be a
localization result with no map-authoring consequence. A map-only improvement
could be a metric artifact if the trajectory became worse. Here the refined
trajectory improves and the judged map improves at the same time.

The coverage behavior is also important. On the NTU substrate, planar coverage
rises from 0.496509563 to 0.573336748. On `construction_seq1`, it rises from
0.377912074 to 0.394277537. On the held-out `construction_seq2`, it rises from
0.423900020 to 0.424767096.

That is the predicted signature of real crispening under a frozen extractor:
blurred geometry crosses fixed gates and becomes measurable as planar support.
The optimizer is not allowed to move the judge.

All reported numbers are three-run byte-identical. The refined trajectory md5
reproduces across independent invocations.

For the larger resource point, the observed run was:

```text
640 submaps / 1079 m = 4:38 wall / 539 MB peak RSS
```

### APE Methodology

The construction ground truth is based on total-station checkpoints. Those
checkpoints are taken while the platform is stationary, so they fall in the time
gaps of a submap-rate trajectory.

The APE score therefore uses linear interpolation at checkpoint timestamps:

```text
ape_from_tum.py --interpolate
```

The alignment is SE(3) Umeyama alignment. Each construction sequence has 15
pairs.

## Reproduce It

The full release gate runs the build, tests, APE profiles, both determinism
stages, and the blocking map-quality profile on the refined map produced by the
gate itself.

```bash
# full release gate: build + tests + APE profiles + both determinism stages
# + blocking map-quality profile on the gate-built refined map
bash scripts/run_release_readiness_checks.sh --fail-on-profiles \
  --offline-determinism-bag <backend_input bag> \
  --offline-determinism-map-quality-profile \
    configs/map_quality_profiles/indoor_construction.yaml \
  --frontend-determinism-bag <raw sensor bag> \
  --frontend-determinism-cloud-topic <points topic>

# one substrate, by hand
bash scripts/run_offline_determinism_check.sh --bag <backend_input> --runs 3 --save-maps
bash scripts/run_map_quality_check.sh --input <map.pcd> --output-dir out \
  --profile configs/map_quality_profiles/indoor_construction.yaml
```

The first command is the release-level contract. The second pair is the smaller
manual reproduction path for one substrate.

The determinism check is not a visual comparison. It is an artifact comparison.
The map-quality check is not run against a stored reference map. It is run
against the map produced by the current pipeline invocation.

That distinction is what makes the gate useful in CI.

## What This Is Not

This is not a cross-machine reproducibility claim.

The current byte-identity scope is same build and same machine. Cross-machine
identity would require a stricter toolchain and hardware contract than this
release asserts.

This is not a live-path refinement claim.

The online path is untouched by the refinement pass. The refinement work applies
to offline map authoring.

This is not an outdoor vegetation claim.

Outdoor vegetation-dominated scenes remain report-only until there is enough
evidence to set a blocking profile there. Current outdoor coverage baselines are
0.12-0.16, which is not the same support regime as the indoor construction
profile.

This is not a raw odometry accuracy claim.

The stack builds on RKO-LIO. The v0.7 claim is not that raw odometry is solved
in general. The claim is narrower and more useful for release engineering:
globally refined, quality-gated, byte-reproducible maps.

## Closing

The v0.6 result made offline SLAM artifacts reproducible enough to gate by md5.
The v0.7 result added frozen map-quality metrics, a clean-room global refinement
pass, holdout-validated blocking thresholds, and a default-on offline runner
that checks the refined map it just built.

For the full supporting material, see:

- `docs/research/map-quality-baseline.md` for the complete metric tables;
- `docs/research/map-refinement-clean-room-design.md` for the math and design;
- `docs/roadmap/v0.7.md` for the phased evidence chain;
- the README quickstart for running the stack.
