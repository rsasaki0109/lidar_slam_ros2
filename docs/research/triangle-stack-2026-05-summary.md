# Triangle Descriptor Stack — 2026-05 Research Summary

This doc is the operator-facing closeout for the triangle descriptor research
arc that landed across PRs #135-#189 (May 2026), with a focused write-up of
the variance / RANSAC-cost / max_pairs sweep that occurred 2026-05-24.

If you only want the production take-aways, read **Production take-aways**
below. If you want the research narrative (why the defaults are what they
are), read on.

## Production take-aways

- `use_triangle_descriptor: false` is the public default for **every** preset
  (`graphbasedslam.yaml`, `graphbasedslam_indoor.yaml`,
  `lidarslam_mid360_rko_graph.yaml`). The triangle pipeline is research /
  opt-in only.
- If you opt in on MID-360, the preset ships `triangle_descriptor_max_pairs: 16`
  (lowered from 32 in PR #186). Do **not** raise it to 32 (systematic
  +1 m APE drift) or lower it to 8 (drift returns). 16 is an
  **empirically validated sweet spot** for this preset.
- If you opt in on other presets (NTU outdoor 360°, Newer College indoor),
  `max_pairs` was never the problem — keep the existing defaults.
- Single-run APE claims on triangle ablation are unreliable. **Always 3-run +
  report mean ± std + `|Δ|/σ`** before claiming an APE improvement.

## Why every ablation needs ≥3 runs (NTU v5, PR #183)

The 2026-05-18 NTU v5 single-run reported "2 emit / 1 accept / Δ APE
-0.022 m". A 3-run repeat on the same code (post #159-#162, all default off)
got **0 emits across 3 runs** and mean Δ APE -0.019 ± 0.125 m
(`|Δ|/σ = 0.15`, within variance). Root cause is **wall-clock-driven
searchLoop scheduling**: triangle compute itself is deterministic given an
input submap, but the SLAM run's bag-play + RKO-LIO offline + map_array
publish timing jitter run-to-run, so the searchLoop tick fires against
different `latest_idx` values each run. Single-run claims sample one
realization of that timing.

(PR #183 retracted the 2026-05-18 "APE improvement" claim from plan.md §1.2.)

## Why triangle accept = 0 still costs APE on MID-360 (PR #184)

A 3-run MID-360 ablation (tuned config: `min_inliers=3, min_votes=6`) gave
**Δ APE +1.083 ± 0.128 m** (`|Δ|/σ = 8.5`, systematic regression) even though
triangle accept = 0/3. So merely *enabling* the triangle pipeline costs
~1 m APE on MID-360, with no compensating accepted loops. The same direction
appears in the default config (`min_inliers=5`) but is variance-bounded.

## Where the cost lives: it's the RANSAC, not the votes (PR #185)

A diagnostic ROS param `triangle_descriptor_skip_ransac` (default false)
runs `accumulateVotes` (O(N) hash lookup) and submap-id selection but
**skips `findLoopCandidate`** (O(N²) RANSAC). 3-run MID-360 tuned with
RANSAC OFF: Δ APE +0.604 ± 1.258 m → `|Δ|/σ = 0.48` (within variance).

| condition          | mean Δ APE [m] | std [m] | \|Δ\|/σ |
|--------------------|----------------|---------|---------|
| RANSAC ON (#184)   | +1.083         | 0.128   | 8.5     |
| RANSAC OFF (#185)  | +0.604         | 1.258   | 0.48    |

Conclusion: the dominant source of the +1 m drift is **RANSAC compute**, not
the act of enabling the pipeline. accumulateVotes alone is variance-bounded.

## The fix: max_pairs sweep, =16 is the sweet spot (PRs #186, #188)

Halving `max_pairs` halves the linear cost and cuts the O(N²)
`transformAgrees` work to 1/4.

| max_pairs | mean Δ APE [m] | std [m] | \|Δ\|/σ | cand mean APE [m] | classification |
|-----------|----------------|---------|---------|---------------------|----------------|
| 32 (PR #184) | +1.083 | 0.128 | 8.5 | 4.876 | systematic regression |
| **16 (PR #186)** | **-0.292** | 0.607 | **0.48** | **3.812** | **sweet spot ✓** |
| 8  (PR #188) | +0.768 | 0.167 | 4.6 | 4.644 | regression 再発 |

(baseline 9-run aggregate: 3.92 ± 0.40 m)

The relationship is U-shaped: both 32 and 8 give systematic regression; 16
is the only `max_pairs` value whose candidate mean APE sits inside the
baseline noise envelope. PR #186 made `max_pairs: 16` the MID-360 yaml
default.

The 8-run regression root cause is **not** explained by the RANSAC compute
cost hypothesis alone (8 should be cheaper than 16, so the drift should
shrink, not return). The current best candidates are:

- (A) Wall-clock floor effect: RANSAC finishes so fast that searchLoop's
  per-tick budget redistributes to other message handling, perturbing
  scheduling in a different direction
- (B) RANSAC consensus failure pattern: at max_pairs=8 the inner loop
  almost never finds consensus → different early-return paths → different
  wall-clock distribution
- (C) accumulateVotes / chosen_submap_id downstream thread contention

This is left open for future instrumentation work.

## The fix is MID-360-specific (PRs #187, #189)

- **Newer College math_hard** (`graphbasedslam_indoor.yaml`, `max_pairs=64`):
  2026-05-19 3-run had Δ APE +0.004 ± 0.022 m (variance-bounded). Newer
  code path was unchanged across the 7 PRs, so the data is still valid.
- **NTU VIRAL tnp_01** (`graphbasedslam.yaml`, `max_pairs=24`, PR #183
  5-run aggregate): Δ APE -0.039 ± 0.093 m (variance-bounded).
- **NTU skip_ransac 3-run direct test (PR #189)**: Δ APE -0.013 ± 0.047 m
  (vs RANSAC ON Δ -0.019 ± 0.125 m). Means agree; RANSAC compute does not
  shift NTU APE.

So `max_pairs` reduction was not applied to other presets — they don't
need it. The +1 m drift is genuinely MID-360-narrow-FOV-specific. Working
hypothesis for why: MID-360 has lower keypoint repeatability, which lets
the vote threshold fire more often per searchLoop tick, so RANSAC runs
more often, and its wall-clock cost is large enough to perturb the
downstream distance-loop verification timing.

## Side observation: RANSAC adds wall-clock jitter on every preset

NTU 3-run with RANSAC OFF (#189) gave std 0.047 m vs RANSAC ON std 0.125 m
(3x tighter). Even on NTU, where RANSAC compute doesn't shift the APE
mean, it does shift the **variance** of the SLAM run. This is a hint that
"map_array-driven searchLoop scheduling" or "RANSAC in a std::async"
would help reproducibility everywhere; it just only becomes APE-visible
on MID-360 today.

## Diagnostic flag remains

`triangle_descriptor_skip_ransac` (default false) stays in the tree
(PR #185) for future investigations into the same trade-off on different
datasets / configs. It is not for production use.

## PR table

| PR | Type | Headline finding |
|----|------|------------------|
| #183 | retract | NTU v5 single-run "emit improvement" claim was N=1 noise |
| #184 | meta | MID-360 triangle pipeline costs +1 m APE even with accept = 0 |
| #185 | diag flag | RANSAC compute (not votes / not just-enabling) is the dominant cost |
| #186 | fix | `max_pairs: 32 → 16` on MID-360 yaml eliminates the drift |
| #187 | generalize | Drift is MID-360-specific; Newer + NTU show no effect at higher max_pairs |
| #188 | sweep | U-shape: max_pairs=8 regression returns; 16 is the empirical sweet spot |
| #189 | confirm | NTU skip_ransac 3-run directly shows RANSAC compute has no APE effect on NTU |

## Outstanding research questions (not blocking release)

1. **max_pairs=8 regression root cause** — instrument searchLoop with
   per-tick wall-clock logs, run 8 vs 16 with the log on, see what
   distribution differs. Could be hypotheses (A), (B), or (C) above.
2. **RANSAC async / map_array-driven scheduling** — if RANSAC moved to a
   std::async or searchLoop ticked on map_array messages instead of
   wall-clock, both APE and variance should tighten on every preset
   (NTU side observation supports this).
3. **Newer College APE at current develop HEAD** — the 3-run baseline
   from 2026-05-19 is still trusted because Newer code path didn't
   change, but a confirmation run at the post-#189 HEAD would close the
   "did 7 PRs touch anything we missed?" question definitively.

## Files

- Triangle implementation: `graph_based_slam/include/graph_based_slam/triangle_descriptor*.hpp`
- ROS wiring + diagnostic flag: `graph_based_slam/src/graph_based_slam_component.cpp`
- MID-360 preset (the one that ships `max_pairs: 16`):
  `lidarslam/param/lidarslam_mid360_rko_graph.yaml`
- Other presets (unchanged): `graph_based_slam/param/graphbasedslam.yaml`,
  `graph_based_slam/param/graphbasedslam_indoor.yaml`
- Ablation outputs:
  - `output/triangle_ablation_ntu_v5_3run_20260524_083127/SUMMARY.md`
  - `output/triangle_ablation_mid360_3run_tuned_20260524_093504/SUMMARY.md`
  - `output/triangle_ablation_mid360_skipransac_20260524_101218/SUMMARY.md`
  - `output/triangle_ablation_mid360_maxpairs16_20260524_175503/SUMMARY.md`
  - `output/triangle_ablation_mid360_maxpairs8_20260524_213619/SUMMARY.md`
  - `output/triangle_ablation_ntu_v5_skipransac_20260524_222141/`
