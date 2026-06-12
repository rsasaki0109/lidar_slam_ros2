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

## Threshold outlook (for Phase 3, not enforced yet)

Per-profile, baseline-relative: a refined map must not regress MME,
thickness mean, or coverage on its own substrate beyond noise; blocking
absolute thresholds are deferred until the Phase 2 deltas exist and a
holdout sequence validates them (`docs/roadmap/v0.7.md` Phase 3,
threshold hygiene).
