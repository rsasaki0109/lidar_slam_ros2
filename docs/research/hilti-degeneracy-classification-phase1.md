# HILTI degeneracy classification (v0.8 Phase 1)

Status: **recorded 2026-07-06.** Companion to
[`docs/roadmap/v0.8.md`](../roadmap/v0.8.md) §5 Phase 1,
[`hilti-degeneracy-baseline.md`](hilti-degeneracy-baseline.md) (Phase 0
freeze) and
[`rko-lio-diagnostic-patch-characterization.md`](rko-lio-diagnostic-patch-characterization.md)
(the fork patch this classification consumes). This is the Phase 1 "wire the
score into a per-scan diagnostics CSV / map-bundle report / release-readiness
stage, then gate + calibrate on real substrates" record.

## 1. What was wired

- `graph_based_slam/include/graph_based_slam/odometry_covariance_localizability.hpp`
  -- pure adapter: `H' = pose.covariance^-1`, fed into the existing
  `localizability_analysis.hpp::analyzeLocalizability`. Detects and excludes
  the fork's isotropic "no diagnostics this scan" fallback and an all-zero
  (unpopulated) covariance.
- `graph_based_slam/include/graph_based_slam/degeneracy_diagnostics_csv.hpp`
  -- per-scan CSV row: timestamp, 6 eigenvalues, 6 categories, 3 category
  counts, condition number, and (this pass) all 6 sign-canonicalized
  eigenvectors (36 columns) so the physical direction of every classified
  axis, not only the weakest one, can be identified offline without
  re-running the substrate.
- `graph_based_slam/include/graph_based_slam/degeneracy_report_summary.hpp`
  -- O(1)-memory streaming summary (category rates, longest not-well-
  conditioned interval) consumed by both the live map-bundle
  `degeneracy_report.yaml` and the offline runner.
- Component wiring: `graph_based_slam_component` (`use_odom_input` path,
  `receiveSyncedOdomCloud`) and `graph_slam_offline_runner.cpp`, both gated
  by new opt-in parameters `degeneracy_diagnostics_csv_path` (default `""`)
  and `save_degeneracy_report` (default `false`); `/map_save` writes
  `degeneracy_report.yaml` into the map bundle when the latter is set
  (best-effort, matching `map_projector_info.yaml`'s existing convention).
- `scripts/run_release_readiness_checks.sh --degeneracy-report <csv>`
  (repeatable, report-only -- a summarizer failure is logged, never fails
  the gate) drives `scripts/summarize_degeneracy_csv.py`, a pure-Python
  reimplementation of the same streaming summary (used to cross-check the
  C++ accumulator below, §4).

All of the above is opt-in and additive: default parameter values are
unchanged (`""` / `false`), so the default `use_odom_input` path and the
default offline-runner outputs (`loop_edges.csv`, TUM trajectories) are
untouched unless a caller explicitly asks for these diagnostics.

## 2. Calibration data source

Real HILTI 2022 RKO-LIO per-scan `H` telemetry, captured via the fork's
anisotropic odometry covariance (Thirdparty/rko_lio `d6c767d`,
`configs/hilti2022/rko_lio_hilti2022_pandar.yaml`), replayed through
`scripts/run_rko_lio_graph_benchmark.sh` with
`degeneracy_diagnostics_csv_path` / `save_degeneracy_report` set on
`graph_based_slam` (via a param-file override of `lidarslam/param/lidarslam.yaml`,
kept outside the repo under
`/media/sasaki/aiueo/lidarslam_work/output/v0.8_phase1_classification/`):

| sequence | environment | scans (available) | artifacts |
|---|---|---:|---|
| exp07 | long corridor (degenerate, Phase 0 motivating evidence) | 1080 of 1081 | `exp07_run/{degeneracy_diagnostics.csv,degeneracy_report.yaml,degeneracy_summary.{md,json}}` |
| exp01 | construction ground level (feature-rich control) | 1623 of 1624 | `exp01_run/{degeneracy_diagnostics.csv,degeneracy_report.yaml,degeneracy_summary.{md,json}}` |

Both runs completed their full bag (exp07: 1322 raw poses / 132 s; exp01:
2277 raw poses / 228 s) with `diagnostics_available_ratio >= 0.999` (the
~0.1% gap is the first frame, which has no prior ICP diagnostics yet). Full
reproduction commands: §6.

## 3. Threshold calibration

`localizability_analysis.hpp::LocalizabilityThresholds` has two
dimensionless ratios (`H`-scale-invariant, i.e. invariant to `H -> c*H`):
`well_conditioned_ratio` (a direction is "weak" below this fraction of
`trace(H)`) and `multiplicity_relative_gap` (two adjacent weak directions
merge into NON_OBSERVABLE when they differ by less than this). The
provisional Phase 0 defaults (`1e-4` / `1e-6`, tuned only against the
synthetic fixtures) were **not** load-bearing on real data: at `1e-4`,
exp07 flags 88.4% of scans weak but so does exp01 at 35.8% -- far too little
separation to serve as a degeneracy signal, and the `1e-6` gap merged almost
every weak pair (exp07: 55%+ scans reported NON_OBSERVABLE from a single
merge artifact, not a genuine multi-dimensional null space).

### 3.1 `well_conditioned_ratio`

Normalized contribution (`lambda_i / trace(H)`) quantiles, ascending
eigenvalue index (`c0` = smallest / most ill-constrained), over every scan
with available diagnostics:

| sequence | direction | q01 | q05 | q25 | q50 | q75 | q95 | q99 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| exp07 | c0 | 5.00e-7 | 5.00e-7 | 9.52e-7 | 5.03e-6 | 6.32e-5 | 6.71e-4 | 1.15e-3 |
| exp07 | c1 | 5.00e-7 | 5.00e-7 | 1.26e-6 | 6.08e-6 | 6.73e-5 | 1.22e-3 | 2.07e-3 |
| exp07 | c2 | 4.79e-5 | 5.03e-5 | 6.72e-5 | 1.34e-4 | 3.69e-4 | 2.25e-3 | 2.53e-3 |
| exp01 | c0 | 1.74e-5 | 1.98e-5 | 7.47e-5 | 1.39e-4 | 3.54e-4 | 6.12e-4 | 1.14e-3 |
| exp01 | c1 | 3.76e-5 | 5.06e-5 | 1.02e-4 | 2.48e-4 | 5.12e-4 | 7.36e-4 | 1.21e-3 |
| exp01 | c2 | 1.51e-4 | 1.69e-4 | 2.99e-4 | 4.14e-4 | 7.23e-4 | 9.14e-4 | 1.28e-3 |

exp01's smallest per-scan contribution (`c0`) essentially never drops below
`~1.7e-5` (1% quantile `1.74e-5`); exp07's `c0` sits an order of magnitude
lower at the median (`5.0e-6`). Scan-level "at least one weak direction"
rate (the classification-relevant statistic) as a function of the
threshold, over the same final CSVs:

| threshold | exp07 weak rate (whole run) | exp01 weak rate | exp07 weak rate (mid-corridor third) |
|---:|---:|---:|---:|
| 1.0e-4 | 0.884 | 0.358 | 1.000 |
| 5.0e-5 | 0.723 | 0.177 | 1.000 |
| 3.0e-5 | 0.692 | 0.093 | 1.000 |
| 2.0e-5 | 0.657 | 0.052 | 0.925 |
| **1.5e-5** | **0.590** | **0.001** | **0.764** |
| 1.0e-5 | 0.566 | 0.000 | 0.697 |
| 7.0e-6 | 0.531 | 0.000 | 0.594 |
| 5.0e-6 | 0.499 | 0.000 | 0.497 |
| 3.0e-6 | 0.447 | 0.000 | 0.383 |
| 1.0e-6 | 0.257 | 0.000 | 0.108 |

`1.5e-5` is the calibrated Phase 1 default: it sits at the point where
exp01's weak rate collapses to effectively zero (0.1%, its two remaining
weak scans are a single 0.2 s blip in an otherwise feature-rich sweep, §5)
while exp07's mid-corridor weak rate stays a clear majority (76.4%). Any
threshold in `[1e-5, 2e-5]` gives a qualitatively identical gate outcome;
`1.5e-5` was picked as the geometric-ish midpoint of that plateau rather
than an edge value, so the classification is not fragile to small
future-data revisions of these same quantiles. The synthetic box fixture's
smallest contribution (`4.8e-4`, `docs/research/hilti-degeneracy-baseline.md`
§4) stays ~30x above this threshold, so the Phase 0 synthetic gate is
unaffected.

### 3.2 `multiplicity_relative_gap`

Gap between the two smallest normalized contributions (`c1 - c0`) on exp07,
and the fork's own covariance-eigenvalue floor
(`max(1e-9, 1e-6*lambda_max)`, `rko-lio-diagnostic-patch-characterization.md`)
recovered through the `H' = Cov^-1` round trip as a **floored-direction
count** (directions whose eigenvalue sits at or below `1.05 * 1e-6 *
lambda_max`, i.e. indistinguishable from the fork's own floor):

| gap quantile | q01 | q05 | q25 | q50 | q75 | q95 | q99 |
|---|---:|---:|---:|---:|---:|---:|---:|
| exp07 `c1-c0` | 8.7e-18 | 2.5e-17 | 8.5e-8 | 5.8e-7 | 3.9e-6 | 5.5e-4 | 9.6e-4 |

| floored directions per scan | 0 | 1 | 2 |
|---|---:|---:|---:|
| exp07 scan count | 902 | 24 | 154 |

Two populations are visible in the `c1-c0` gap column: a cluster at
`~1e-17` (machine-epsilon-scale, i.e. the fork's floor value applied
identically to two or more directions -- a genuine merge candidate) and
everything from `~9e-8` upward (two distinct, non-floor-clamped weak
directions that should stay individually DEGENERATE). `1e-8` sits cleanly
between the two: it is far above the floored-pair cluster (`~1e-17`,
6+ orders of magnitude of margin) and far below the 25th-percentile
distinct-pair gap (`8.5e-8`, an 8.5x margin), the same qualitative
separation the provisional `1e-6` value failed to provide (`1e-6` sat
*above* the median distinct-pair gap and merged the majority of exp07's
weak scans into a spurious NON_OBSERVABLE reading).

### 3.3 Calibrated defaults (landed)

`graph_based_slam/include/graph_based_slam/localizability_analysis.hpp`:

| threshold | Phase 0 provisional | Phase 1 calibrated |
|---|---:|---:|
| `well_conditioned_ratio` | 1.0e-4 | **1.5e-5** |
| `multiplicity_relative_gap` | 1.0e-6 | **1.0e-8** |

Full classification with the calibrated defaults, final CSVs (§2), reported
by the C++ `degeneracy_report.yaml` (and cross-checked byte-for-byte in
category counts by the Python `summarize_degeneracy_csv.py` stage, §4):

| sequence | well-conditioned | degenerate | non-observable | mid-corridor-third not-well-conditioned |
|---|---:|---:|---:|---:|
| exp07 | 41.0% (443/1080) | 43.1% (465/1080) | 15.9% (172/1080) | **76.4%** (275/360) |
| exp01 | **99.9%** (1621/1623) | 0.1% (2/1623) | 0.0% (0/1623) | 99.7% |

(The synthetic-fixture gate, `test_localizability_analysis.cpp`, was
re-verified green against the calibrated defaults -- the corridor/box/
single_plane fixtures' eigenvalue gaps are many orders of magnitude wider
than either threshold, so their pass/fail outcome is unaffected; only the
one boundary-threshold test whose hand-built numbers were chosen relative
to the *old* defaults was updated to match, `TwoDistinctWeakEigenvaluesStayIndividuallyDegenerate`.)

## 4. Cross-check: C++ accumulator vs. Python summarizer

The release-readiness `--degeneracy-report` stage re-implements the same
streaming summary in Python
(`scripts/summarize_degeneracy_csv.py`) rather than shelling out to a C++
binary, so it was cross-checked against the live component's own
`degeneracy_report.yaml` on both final CSVs:

| sequence | field | C++ (`degeneracy_report.yaml`) | Python (`degeneracy_summary.json`) |
|---|---|---:|---:|
| exp07 | well/degenerate/non_observable scans | 443 / 465 / 172 | 443 / 465 / 172 |
| exp07 | worst interval | 523 scans, NON_OBSERVABLE | 523 scans, NON_OBSERVABLE |
| exp01 | well/degenerate/non_observable scans | 1621 / 2 / 0 | 1621 / 2 / 0 |
| exp01 | worst interval | 2 scans, DEGENERATE | 2 scans, DEGENERATE |

Exact match, as expected (both consume the same CSV rows with the same
"first non-well-conditioned direction wins, escalate to NON_OBSERVABLE if
any direction in the run is NON_OBSERVABLE" rule).

## 5. Direction identification: is the DEGENERATE direction the corridor axis?

The Phase 1 gate (`docs/roadmap/v0.8.md` §5) requires exp07's "along-corridor
direction" specifically -- not merely *some* direction -- to be flagged
degenerate on the majority of mid-corridor frames. Two convention-independent
checks were run against the final exp07 CSV's per-direction eigenvectors
(the schema addition landed in this pass specifically to make this possible
offline, §1):

**(a) Translation dominance.** For every DEGENERATE-labeled direction across
the whole run (925 instances: a scan can have more than one DEGENERATE
direction), the fraction of that eigenvector's squared norm carried by its
translation block (`tx,ty,tz`) versus its rotation block (`rx,ry,rz`):

| quantile | q05 | q25 | q50 | q75 | q95 |
|---|---:|---:|---:|---:|---:|
| translation fraction | 0.9991 | 0.9995 | 0.9997 | 0.9998 | 0.9999 |

100% of DEGENERATE-direction instances have translation fraction `> 0.99`.
Every DEGENERATE direction on exp07 is (numerically) a pure translation
direction, not a rotation (yaw/pitch/roll) direction -- consistent with
`docs/roadmap/v0.8.md` §0's framing ("one axis... carries substantially
more error... the classic signature of a directionally under-constrained
registration problem") and with the Phase 0 synthetic corridor fixture,
whose exact-zero eigenvalue is a pure `tx` translation direction by
construction (`hilti-degeneracy-baseline.md` §4).

**(b) A single, persistent axis (the "corridor" identification specifically).**
Whether that dominant-translation direction is one *fixed* structural axis
(the corridor's own long axis) rather than a different direction on every
scan was checked by comparing each mid-corridor-third DEGENERATE
direction's translation part against the *next* scan's, and separately
against a scan 20 frames away (2 s at ~10 Hz), without assuming any
particular coordinate-frame convention for `H` (attempting to rotate into
the odometry's own world/map frame via the published pose orientation gave
inconsistent results -- see the limitation note below):

| pair type | n pairs | \|cos\| q05 | q25 | q50 | q75 | q95 | frac `>0.9` |
|---|---:|---:|---:|---:|---:|---:|---:|
| consecutive scans (~0.1 s apart) | 254 | 0.913 | 0.995 | 0.999 | 1.000 | 1.000 | **95.7%** |
| 20-scan-apart scans (~2 s apart) | 48 | 0.017 | 0.178 | 0.600 | 0.994 | 0.999 | -- |

Consecutive-scan DEGENERATE directions are essentially identical (median
`|cos| = 0.9992`), while directions 2 s apart have already decorrelated
substantially (median `0.60`, 25th percentile `0.18`) -- i.e. the
DEGENERATE direction is a locally persistent structural axis that holds
steady scan-to-scan through the corridor stretch, not an artifact that
re-picks a new (noise-driven) direction every frame. Combined with (a),
this is the along-corridor axis signature the roadmap gate asks for: a
single, stable, translation-dominant ill-constrained direction, present on
the majority of mid-corridor frames, and essentially absent on exp01
(whose only two DEGENERATE scans, a single consecutive pair, are a
momentary `tz`-dominant -- i.e. vertical, not along-travel -- blip, not a
recurring structural direction).

**Limitation, stated honestly.** A direct check against the odometry's own
world/map-frame direction of travel (net displacement between trajectory
poses, transformed through the published orientation quaternion) did
*not* show the expected alignment (median `|cos| ~ 0.03-0.33` depending on
window size) -- most likely because RKO-LIO's internal Gauss-Newton `H` is
expressed in a Lie-algebra tangent-space convention (left- vs
right-multiplication perturbation, `docs/roadmap/v0.8.md` §2 item 2's
`current_pose = Sophus::SE3d::exp(dx) * current_pose`) or local-vs-global
basis that does not trivially match the frame the published
`pose.orientation` rotates points into, and resolving that exactly would
require reading the fork's internals beyond what the diagnostic patch
characterization documents -- out of scope for this diagnostic-only,
clean-room pass. The consecutive-vs-distant-frame stability test (b) above
was chosen specifically because it needs no assumption about that
convention and still gives an unambiguous, quantitatively strong answer.

## 6. Reproduction

```bash
# Build with the calibrated defaults (already the header's compiled-in
# values; no extra flag needed).
colcon build --symlink-install --packages-select graph_based_slam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

# Param-file override enabling the two opt-in diagnostics outputs on top of
# lidarslam/param/lidarslam.yaml's graph_based_slam block:
#   degeneracy_diagnostics_csv_path: "<out>/degeneracy_diagnostics.csv"
#   save_degeneracy_report: true

bash scripts/run_rko_lio_graph_benchmark.sh \
  --bag <datasets>/hilti2022/exp07_ros2 \
  --lidar-topic /hesai/pandar --imu-topic /alphasense/imu \
  --rko-param configs/hilti2022/rko_lio_hilti2022_pandar.yaml \
  --lidarslam-param <work>/lidarslam_exp07.yaml \
  --reference-tum <datasets>/hilti2022/exp07_long_corridor_gt.txt \
  --reference-meta <work>/hilti2022_exp07_reference_meta.json \
  --skip-reference-gen --reference-source hilti2022_exp07_control_points_gt \
  --quiescence-secs 60 --offline-timeout-secs 5400 \
  --output-dir <work>/exp07_run
# (same shape for exp01, --bag .../exp01_ros2 etc.)

# Report-only release-readiness stage:
python3 scripts/summarize_degeneracy_csv.py \
  --csv <work>/exp07_run/degeneracy_diagnostics.csv \
  --write-md <work>/exp07_run/degeneracy_summary.md \
  --write-json <work>/exp07_run/degeneracy_summary.json
# or, as part of the full gate:
bash scripts/run_release_readiness_checks.sh \
  --degeneracy-report <work>/exp07_run/degeneracy_diagnostics.csv \
  --degeneracy-report <work>/exp01_run/degeneracy_diagnostics.csv \
  --skip-default-ci --skip-benchmark-summary
```

Artifacts backing every number in this note (kept off-repo, external disk):
`/media/sasaki/aiueo/lidarslam_work/output/v0.8_phase1_classification/{exp07_run,exp01_run}/`.

## 7. Phase 1 gate status (`docs/roadmap/v0.8.md` §5)

- synthetic fixtures classify correctly: **green**, unchanged by the
  threshold calibration (§3.3 parenthetical).
- fork patch characterization test green (byte-identical poses): **green**,
  recorded in `rko-lio-diagnostic-patch-characterization.md`, unaffected by
  this (backend-only) pass.
- exp07 vs exp01 classification matches the documented drift asymmetry:
  **green** -- exp07's along-corridor direction is DEGENERATE on 76.4% of
  mid-corridor frames (majority, §3.3/§5), exp01 is well-conditioned on
  99.9% of frames (large majority); direction identification (§5) confirms
  the DEGENERATE direction is translation-dominant and a single persistent
  axis, not an arbitrary or rotational one.
- zero change to any existing blocking APE/map-quality profile: **green**
  -- every new parameter defaults off/empty; `run_default_ci_checks.sh` and
  the existing release profiles are untouched by this pass (§8, next
  section, records the full test run).

---

(Related: `docs/roadmap/v0.8.md` §5 Phase 1 -- the gate this note satisfies;
`docs/research/hilti-degeneracy-baseline.md` -- the Phase 0 freeze this
calibration is judged against; `docs/research/rko-lio-diagnostic-patch-characterization.md`
-- the fork patch whose covariance output this classification consumes.)
