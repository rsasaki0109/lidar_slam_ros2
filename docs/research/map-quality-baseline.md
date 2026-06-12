# Map-quality baseline (v0.7 Phase 1)

Status: recorded 2026-06-12 on the pre-refinement (v0.6.0-era) maps of the
five gate substrates. This table is the "before" side of every v0.7 Phase 2
refinement claim: the refinement gate requires these numbers to improve
without coverage loss (`docs/roadmap/v0.7.md`).

## Frozen metric extraction profile

The plane extraction profile used by the metrics is **frozen as the
defaults of `graphslam::map_quality::MapQualityConfig`** as of this
document. Phase 2 must not change it — the optimizer may not adjust its
own judge. Values (calibrated below):

| knob | value | rationale |
|---|---|---|
| `downsample` (CLI) | 0.10 m | bounds cost; matches the densest substrate leaf |
| `mme_radius` | 0.50 m | standard MME neighborhood |
| `mme_min_neighbors` | 8 | excludes isolated points from the entropy mean |
| `plane: max_plane_thickness` | 0.15 m | must *measure* blurry walls, not exclude them — with the strict 0.06 m default the pre-refinement maps' walls (≈ 9 cm RMS) were invisible and improvement could never register |
| `plane: min_planarity_ratio` | 4.0 | the binding gate in calibration; 6.0 rejected most real (blurred) planes, 3.0 admitted clutter (cs2 thickness mean jumped to 0.118 m) |
| `plane: min_points_per_plane` | 10 | admits small wall segments at depth 3–4 |
| `plane: max_octree_depth` | 4 | one level deeper than the BA-oriented default |
| `plane: quarter test` | on (tol 2.0) | verified non-binding on all substrates — kept as a clutter guard |
| `min_meaningful_planar_coverage` | 0.05 | all five substrates sit above it pre-refinement (min: 8.5%) |

Calibration evidence (construction_seq2): with the strict defaults the
thickness mean pinned at the 0.06 cap (only sneak-under patches accepted,
coverage 0.2%); thickness sweep saturated at 0.15 (cap no longer binding);
ratio 6→3 lifted coverage 1.9%→6.6% but degraded the measured population;
the quarter test changed nothing (binding analysis, not taste).

## Baseline table (pre-refinement maps, `--downsample 0.1`, frozen profile)

| substrate | map | points | MME (nats) ↓ | MME valid | patches | thickness mean (m) ↓ | thickness p95 (m) ↓ | planar coverage ↑ | meaningful |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| construction_seq2 (blocking, APE 0.154) | `output/rtkslam_cs2_run2/map.pcd` | 808,844 | −0.567506873 | 0.905 | 3,595 | 0.091690906 | 0.132638198 | 0.085346600 | true |
| construction_seq1 (blocking, APE 0.403) | `output/rtkslam_construction_seq1_run/map.pcd` | 946,462 | −0.581225185 | 0.901 | 4,350 | 0.087740857 | 0.131826050 | 0.084639694 | true |
| stadtgarten_seq2 (report-only, APE 0.835) | `output/rtkslam_stadtgarten_seq2_v1nodd/map.pcd` | 1,037,016 | −0.720378721 | 0.798 | 6,003 | 0.094053898 | 0.134546999 | 0.120050453 | true |
| stadtgarten_seq1 (report-only, APE 1.666) | `output/rtkslam_stadtgarten_seq1_nodd/map.pcd` | 2,195,507 | −0.773510128 | 0.838 | 16,270 | 0.091641271 | 0.133019576 | 0.163144624 | true |
| NTU tnp_01 (blocking, APE ≤ 1.00) | `output/bench_ntu_event_driven_live/map.pcd` | 222,367 | −0.989917390 | 0.944 | 2,972 | 0.084936508 | 0.127683566 | 0.312951112 | true |

Determinism: 3 consecutive CLI runs on construction_seq2 produced
md5-identical `map_quality_report.yaml` (`63a9a9e96cfd…`); the run_* gate
stage re-checks 3-run byte identity on every invocation.

## Reading the numbers

- **Walls are ≈ 9 cm thick RMS on every substrate.** That is the
  pre-refinement reality this track exists to change; it also explains why
  the strict (BA-grade) extraction profile saw almost nothing. The Phase 2
  plane BA targets exactly this number (and planar coverage, which rises
  as blurred geometry crosses the acceptance gates).
- Coverage ordering (NTU 31% > outdoor 12–16% > indoor 8.5%) is **map
  crispness, not scene structure** — the construction halls are the
  cluttered, scaffolding-heavy scenes *and* the maps are blurry at the
  1 m-voxel scale, so fewer voxels pass planarity. Expect indoor coverage
  to move the most under refinement.
- MME is not comparable across substrates (density and sensor differ); it
  is a same-substrate before/after metric only. Lower = crisper.
- The reproduction command for any row:
  `map_quality_report --input <map.pcd> --output-dir <dir> --downsample 0.1`
  (all other knobs are the frozen defaults).

## Refinement before/after (Phase 2 hard-gate evidence, 2026-06-13)

First real-data results of the offline plane-BA refinement
(`graph_slam_offline_runner ... -p refine:=true -p refine_save_maps:=true`,
frozen metric profile, `--downsample 0.1`). Backend-input bags exist for
two substrates; both passed the 3-run byte-identity gate **including the
refined artifacts** (`trajectory_refined.tum` + `map_refinement_report.yaml`,
and the refined trajectory md5 reproduced across independent invocations).

| substrate | metric | optimized (before) | refined (after) | Δ |
|---|---|---:|---:|---|
| NTU tnp_01 (Leica GT) | APE RMSE (m) | 0.8460511516520233 | **0.8190124186992095** | **−3.2 %** |
| | MME (nats) | −0.970478040 | **−1.110389903** | crisper |
| | thickness mean (m) | 0.084944868 | **0.074452257** | **−12.4 %** |
| | thickness p95 (m) | 0.116538522 | **0.110470374** | −5.2 % |
| | planar coverage | 0.496509563 | **0.573336748** | **+15.5 %** |
| MID-360 (GLIM cross-val ref) | APE RMSE (m) | 7.262851264566504 | 7.254892426501326 | −0.1 % |
| | MME (nats) | −1.612667608 | −1.620476494 | crisper |
| | thickness mean (m) | 0.044403193 | 0.044050759 | −0.8 % |
| | planar coverage | 0.550698372 | 0.545862322 | −0.9 % |

Reading: the NTU row is the headline — against total-station-grade GT the
refinement improves the *trajectory* (−3.2 %) and the *map* (thickness
−12.4 %) simultaneously, with planar coverage RISING (+15.5 %): blurred
geometry crossing the frozen acceptance gates is exactly the predicted
signature of real crispening, not metric gaming. The MID-360 deltas are
small because that reference is a cross-validation trajectory whose
global disagreement (meters) dominates; its slight coverage dip (−0.9 %)
keeps the per-substrate rollout discipline honest — outdoor/cross-val
profiles stay report-only in Phase 3.

Existing artifacts are untouched by the stage: `loop_edges.csv`
md5 `feee9547…` and `trajectory_optimized.tum` md5 `92676db4…` are
unchanged with `refine:=true`. Resources: 640 submaps / 1079 m in
4:38 wall / 539 MB peak RSS (NTU: 1:54 / 367 MB) — inside the design
note's budget.

## Threshold outlook (for Phase 3, not enforced yet)

Per-profile, baseline-relative: a refined map must not regress MME,
thickness mean, or coverage on its own substrate beyond noise; blocking
absolute thresholds are deferred until the Phase 2 deltas exist and a
holdout sequence validates them (`docs/roadmap/v0.7.md` Phase 3,
threshold hygiene).
