# Triangle Descriptor Stack — 2026-05 Research Summary

This doc is the operator-facing closeout for the triangle descriptor research
arc that landed across PRs #135-#189 (May 2026), with a focused write-up of
the variance / RANSAC-cost / max_pairs sweep that occurred 2026-05-24.

> **v0.4 D1 closeout (2026-06-07).** The two "outstanding research questions"
> that the 2026-05-24 sweep left open — the `max_pairs=8` U-shape root cause and
> the RANSAC-async / map_array-driven scheduling fix — are now **answered /
> design-pinned** below (see *U-shape root cause: the tick-interval histogram*
> and *Determinism root cause + the scheduling fix*). This is a research
> closeout, **not** a default change: every preset still ships
> `use_triangle_descriptor: false`. The deterministic scheduling refactor has
> since **landed as an opt-in** (`deterministic_loop_scheduling`, default
> **false** — behaviour-identical when off); the one remaining piece is the
> 8-vs-16 head-to-head benchmark that would clear the flag to become a default.
> That validation needs benchmark-data access and is intentionally not attempted
> as an unvalidated behavior change here.

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
shrink, not return). The candidate hypotheses were:

- (A) Wall-clock floor effect: RANSAC finishes so fast that searchLoop's
  per-tick budget redistributes to other message handling, perturbing
  scheduling in a different direction
- (B) RANSAC consensus failure pattern: at max_pairs=8 the inner loop
  almost never finds consensus → different early-return paths → different
  wall-clock distribution
- (C) accumulateVotes / chosen_submap_id downstream thread contention

The tick-interval histogram below (2026-05-25) makes **(A)** the
best-supported explanation.

## U-shape root cause: the tick-interval histogram (2026-05-25, Q1 answered)

The instrumentation needed to settle this already exists: with `debug_flag_`
on, `searchLoop` logs `"searching Loop, num_submaps:%d"`
(`graph_based_slam_component.cpp` ~L1261) on every tick, so each tick's
wall-clock timestamp **and** its `latest_idx` (= `num_submaps - 1`) are
recoverable from the existing `slam.launch.log`. Binning the **inter-tick
interval** across the existing 3-run logs for each `max_pairs`
(`loop_detection_period = 1000 ms`) gives:

| max_pairs | 1.0–1.5 s | 1.5–2.5 s | 5.0+ s | mode |
|-----------|-----------|-----------|--------|------|
| 32 | 45% | 36% | **18% (long tail)** | 1.0–1.5 s |
| **16** | 28% | **48%** | 14% | **1.5–2.5 s** |
| 8 | **63%** | 18% | 16% | 1.0–1.5 s |

Read across the row, the two regressing values have qualitatively different
wall-clock signatures, and the sweet spot sits between them:

- **=32 (heavy RANSAC):** an 18% long tail of 5 s+ intervals — ticks are being
  *skipped* under wall-clock pressure. Skipped ticks starve the natural timing
  window of the distance-loop verification → APE drift.
- **=8 (RANSAC too light):** ticks fire back-to-back right after the 1 s timer
  (63% in the 1.0–1.5 s bin). `accumulateVotes` runs at high frequency, and
  that high-density firing is itself the distractor — hypothesis (A)'s
  wall-clock floor effect, in the opposite direction from =32.
- **=16 (balanced):** the modal interval lands in a healthy 1.5–2.5 s band —
  enough tick-skip to amortize RANSAC, not so much that distance-loop timing
  is starved.

So the U-shape is **not** monotone-in-compute: both extremes perturb
searchLoop's wall-clock schedule, just in opposite directions, and =16 is the
empirical sweet spot in *interval distribution* as well as in APE. This is
consistent with hypothesis (A) and inconsistent with a pure "cheaper RANSAC is
always better" model. It is the same wall-clock-scheduling mechanism that makes
single-run APE claims unreliable (the ≥3-run discipline above) — see the next
section for the architectural root and the fix that would remove it.

## The fix is MID-360-specific (PRs #187, #189)

- **Newer College math_hard** (`graphbasedslam_indoor.yaml`, `max_pairs=64`):
  2026-05-19 3-run had Δ APE +0.004 ± 0.022 m (variance-bounded). A
  **post-v0.3.0 3-run at HEAD (2026-05-25)** using the same base param
  (via the PR #192 `--skip-reference-gen` plumbing) gave
  Δ APE −0.0094 ± 0.0108 m, |Δ|/σ = 0.87 — still variance-bounded, but
  the mean now favors triangle and the candidate std roughly halved
  (0.025 → 0.010 m). See
  `output/triangle_ablation_newer_3run_at_v030_20260525_090437/SUMMARY.md`.
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

## Determinism root cause + the scheduling fix (Q2 answered / design-pinned)

Every variance result above — the ≥3-run discipline, the U-shape histogram, the
"RANSAC adds jitter on every preset" side observation — traces to **one**
architectural fact about how loop search is scheduled.

**Root cause.** `searchLoop` is driven by a **wall-clock timer**
(`create_wall_timer(loop_detection_period_, …)`,
`graph_based_slam_component.cpp` ~L1127), and on each tick it queries **only the
single latest submap**:

```cpp
const int latest_idx = num_submaps - 1;   // graph_based_slam_component.cpp:1434
```

The submaps themselves arrive asynchronously on the `map_array` topic, produced
by bag-replay + RKO-LIO at a wall-clock rate that jitters run-to-run (system
load, scheduler, I/O). So the number of submaps that have arrived **by the time
a given tick fires is itself timing-dependent**:

- When the timer fires *between* two submap arrivals, `latest_idx` advances by
  one and that submap gets queried against the database.
- When several submaps arrive *between* two ticks, only the **last** one becomes
  `latest_idx`; the intermediate submaps **are never queried as a `latest`** —
  their triangle / Scan-Context / BEV query against the database simply never
  happens.

The set of `(query, database)` pairs that loop closure ever evaluates is
therefore a function of wall-clock timing, not of the map. That is exactly why
the same code + same bag yields `{2, 3, 3}` inliers for submap id=16 across
three runs, why emit counts swing 0–5, and why single-run APE deltas are noise.
The triangle compute is deterministic *given an input pair*; **which pairs it is
handed is not.**

**The fix (design-pinned).** Make the query sequence depend on the map, not the
clock: track the last submap index already processed as a `latest`
(`last_searched_submap_idx_`, init −1) and, on each invocation, **catch up
deterministically** over every un-queried index instead of jumping to
`num_submaps - 1`:

```cpp
for (int latest = last_searched_submap_idx_ + 1; latest < num_submaps; ++latest) {
  // existing per-latest loop-search body, with latest_idx := latest
}
last_searched_submap_idx_ = num_submaps - 1;
```

Every submap is then queried exactly once as it becomes available, regardless of
whether the timer batched several arrivals into one tick — the `(query, db)`
sequence becomes a deterministic function of the submap stream. (The timer can
remain as the *trigger*; the determinism comes from the catch-up loop, not from
retiming the trigger. Equivalently, the trigger could move onto the `map_array`
callback so a tick fires per arrival.) Moving `findLoopCandidate` into a
`std::async` is the orthogonal half: it takes RANSAC's wall-clock cost off the
searchLoop hot path, which the NTU side observation predicts would tighten
variance on *every* preset.

**Why this is design-pinned, not implemented here.** The change is sound in
principle but carries real risk that must be paid down with data, not asserted:

1. `searchLoop` is a ~1300-line single function (`L1249–2551`) with `latest_idx`
   threaded through as a `const` local and many early `return`s; turning it into
   a per-latest catch-up loop is a sizeable extraction that needs its own
   careful review.
2. The payoff is **reproducibility**, whose validation *is* a benchmark: the
   only honest acceptance test is an **8-vs-16 head-to-head** (and an NTU /
   Newer 3-run) showing the interval histogram collapses toward a single mode
   and `|Δ|/σ` tightens. That requires benchmark-data access this session does
   not have. Landing the refactor *as a default* without that validation would
   replace a known, documented stochasticity with an unverified one.

So D1 closes the *research* questions (root cause understood, fix specified) and
the fix has **landed as an opt-in** (`deterministic_loop_scheduling`, default
off): `searchLoop` is split into a scheduler + a per-query `searchLoopForLatest`,
and the scheduler catches up over every un-queried submap index when the flag is
on. With the flag off the path is byte-for-byte the historical single-latest
query, so the public default is unchanged. Only the 8-vs-16 head-to-head — which
would clear the flag to become a default — is handed to a data-access follow-up.

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

## Research questions — closed out (v0.4 D1)

1. ~~**max_pairs=8 regression root cause**~~ — **answered 2026-05-25**. The
   tick-interval histogram (see *U-shape root cause* above) shows =32 and =8
   have opposite wall-clock signatures (=32 a 5 s+ tick-skip tail, =8 back-to-back
   1.0–1.5 s firing) with =16 in a healthy 1.5–2.5 s band. Best-supported
   explanation is hypothesis **(A)**, the wall-clock floor effect; the U-shape is
   not monotone-in-compute.
2. ~~**RANSAC async / map_array-driven scheduling**~~ — **root cause +
   fix design-pinned 2026-06-07** (see *Determinism root cause + the scheduling
   fix* above). The single architectural cause is that `searchLoop` is
   wall-clock-triggered and queries only `latest_idx = num_submaps - 1`, so
   timer-batched submap arrivals are skipped non-deterministically. Fix: a
   deterministic catch-up loop over un-queried submap indices (+ optional
   `std::async` RANSAC), shipped opt-in. **The catch-up scheduler has landed**
   (`deterministic_loop_scheduling`, default off); the **8-vs-16 validation** (and
   the optional `std::async` RANSAC half) is the benchmark-data follow-up before
   the flag could become a default.
3. ~~**Newer College APE at current develop HEAD**~~ — **answered 2026-05-25**
   via PR #192 (`--skip-reference-gen` plumbing). Post-v0.3.0 3-run gave
   Δ APE −0.0094 ± 0.0108 m (|Δ|/σ = 0.87), still variance-bounded but
   no regression introduced by the #183–#191 series; candidate variance
   ~halved vs the 2026-05-19 baseline.

All three research questions from the 2026-05-24 sweep are now closed, and the
Q2 opt-in deterministic-scheduling implementation has landed (default off). The
only remaining triangle work is the **8-vs-16 benchmark validation** that would
clear `deterministic_loop_scheduling` to become a default, tracked as a v0.4/v0.5
follow-up that needs data access.

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
