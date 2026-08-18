# Candidate 2: loop safety and runtime design (2026-07)

## Evidence from candidate 1

Candidate 1 performs event-driven loop search for every arriving submap after
the travel-distance gate. Its generic correction limits are 15 m and 45 deg,
and overlap gating is disabled. On frozen holdouts it accepted one different
edge per repetition. Two of three `exp03` edges and all three `exp21` edges
made dense graph-corrected checkpoint APE more than 2% worse.

Accepted-edge verifier statistics were:

| Sequence/run | correction (m/deg) | forward overlap | reverse overlap | mutual overlap | APE effect |
|---|---:|---:|---:|---:|---:|
| exp03/01 | 0.254 / 6.846 | 0.758 | 0.240 | 0.364 | -4.79% |
| exp03/02 | 2.225 / 14.161 | 0.814 | 0.229 | 0.357 | +24.26% |
| exp03/03 | 0.613 / 14.699 | 0.772 | 0.356 | 0.487 | +7.46% |
| exp21/01 | 1.261 / 8.151 | 0.733 | 0.051 | 0.096 | +166.76% |
| exp21/02 | 0.908 / 14.582 | 0.829 | 0.033 | 0.063 | +351.50% |
| exp21/03 | 3.627 / 14.939 | 0.779 | 0.054 | 0.101 | +128.88% |

By contrast, the Construction Seq2 development replay's verified loop
candidates had correction translation at most 0.432 m, correction rotation at
most 1.225 deg, forward overlap at least 0.765, and mutual overlap at least
0.315. Its five final loop edges improve dense APE by 6.38%.

## Candidate changes

Develop and select only on development/regression inputs; the consumed
`exp02`, `exp03`, and `exp21` holdouts must not be used as a candidate-2 tuning
gate.

1. Add an event-driven `loop_search_query_stride` so expensive registration is
   attempted only every Nth eligible submap. Start with stride 5 on development
   data. This directly addresses `exp21` candidate-1 RTF 2.02 while preserving
   event-order determinism.
2. Set the generic correction envelope to 0.5 m and 2 deg. This retains every
   verified Construction Seq2 loop candidate and rejects every harmful
   candidate-1 holdout edge. The holdout observation is diagnostic only; the
   actual threshold selection must pass the development replay.
3. Enable forward-overlap support at 0.70. Add a symmetric/mutual-overlap gate
   only if development ablation shows that the correction envelope is
   insufficient.
4. Preserve zero-loop behavior: when no edge passes, dense corrected output
   must equal the raw trajectory within numerical tolerance.

## Development acceptance gates

- Construction Seq2 retains at least one independently verified edge, has zero
  harmful edge, and does not regress its 0.143967 m corrected APE by more than
  2%.
- `exp04` and `exp07` regression trajectories do not regress by more than 2%.
- Backend replay RTF improves by at least 3% and is deterministic across three
  runs.
- Map mean thickness, p95 thickness, and planar coverage do not regress by more
  than 2%.
- Only after freezing candidate 2 may three new, previously unused holdout
  sequences be assigned and opened.
