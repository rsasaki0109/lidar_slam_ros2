# Determinism variance attribution (v0.6 Phase 0, 2026-06-11)

Phase 0 of the deterministic core/shell refactor
([`../roadmap/v0.6.md`](../roadmap/v0.6.md)): before touching the
architecture, measure **which layer owns the run-to-run variance** that the
D1 closeout exposed. Substrate: the GLIM MID-360 demo bag (277 s), the same
one the D1 bench used.

## Method

Three measurements, each isolating one layer:

1. **End-to-end** (frontend + transport + backend): the D1 bench arms
   (`output/d1_sched_bench_20260611/`, 3 runs each).
2. **Backend + transport, byte-identical input**: record the backend's exact
   input topics (`/rko_lio/odometry` + `/rko_lio/frame`) during one benchmark
   run, then replay that bag into `graph_based_slam` alone, 3 times
   (`scripts/run_backend_replay_variance.sh`; raw data
   `output/backend_replay_mid360_p0/`).
3. **Registration floor**: `test_registration_determinism.cpp` — pclomp NDT
   on a fixed synthetic scene, 5 repeats per thread count and a
   cross-thread-count comparison.

## Results

| Layer | APE behaviour | Loop-edge behaviour |
|---|---|---|
| End-to-end (D1 armA, off/16) | baseline APE 3.83–4.83 (σ 0.52); candidate σ 0.066–0.481 | accepted count stable (2) per run |
| Backend + transport (fixed input, 3 runs) | 5.748 / 5.792 / 5.849 (**σ 0.050**) | **a different edge every run**: 1↔577, 6↔583, 14↔582 (edge sets never identical) |
| Registration (NDT micro-bench) | — | bitwise repeatable at any fixed thread count (max dev = 0 for threads 1/2/4); threads 1 vs ≥2 differ by a *fixed, deterministic* 0.028° rotation, 0 m translation |

(The replay APE level ~5.8 m differs from the in-benchmark level ~3.8–4.3 m
because replay timing/pacing differs from the live pipeline and typically
accepts one loop instead of two; the harness measures replay-vs-replay
variance, not replay-vs-benchmark equality.)

## Attribution

1. **The frontend owns most of the APE variance.** With the input stream
   frozen, backend APE σ collapses from ~0.5 to 0.050 — an order of
   magnitude. RKO-LIO's own run-to-run variation (out of scope for v0.6;
   Thirdparty) plus live transport dominate the end-to-end number.
2. **The backend owns the loop-edge nondeterminism, and it is purely a
   scheduling/interleaving artifact.** Same bytes in, three different loop
   closures out (geometrically the same revisit, entered at a different
   submap index each run, hence the small APE spread). This is exactly the
   class of nondeterminism the Phase 2 event-driven core eliminates: process
   submaps in arrival order, query each one exactly once, and the chosen
   edge becomes a function of the input sequence.
3. **Registration will not betray the determinism contract.** ndt_omp NDT is
   bitwise repeatable at a fixed thread count — even multithreaded — and the
   thread-count-dependent difference is itself deterministic and tiny
   (0.028° on the synthetic scene). The Phase 2 core does **not** need to pin
   `ndt_num_threads: 1`; it needs to pin the *value* (a config already does)
   and the candidate evaluation order. Caveat: one synthetic scene; the
   identity-at-fixed-threads result should be re-checked on real submap
   pairs once the offline runner exists (cheap to add there).

## Phase 1/2 design consequences

- The hard Phase 2 gate (identical loop-edge sets across runs of the offline
  runner) is **achievable**: the only remaining nondeterminism sources at
  fixed input are the timer/arrival interleaving (removed by event-driven
  scheduling), the unprotected shared state (removed in Phase 1), and
  candidate-order/tie-break ambiguity (defined in Phase 1).
- Two backend warts found while building the harness, queued for Phase 1:
  - `doPoseAdjustment` writes `pose_graph.g2o` to the node's **CWD**
    (`graph_based_slam_component.cpp:2761`) regardless of any save-path
    parameter — the harness works around it with a per-run working
    directory.
  - The g2o file does not distinguish loop edges from the
    `num_adjacent_pose_cnstraints` adjacency fan-out (edges up to |i−j| ≤ 4
    at the default 5); consumers must window on |i−j| (the harness uses
    `--adjacent-window`, default 5). The offline runner should emit loop
    edges explicitly instead.

## Reproduce

```bash
# layer 2 (backend + transport)
bash scripts/run_backend_replay_variance.sh --mode record --output-dir output/backend_replay_mid360
bash scripts/run_backend_replay_variance.sh --mode replay --output-dir output/backend_replay_mid360 --runs 3
cat output/backend_replay_mid360/backend_replay_summary.md

# layer 3 (registration floor) — report lines grep "[determinism]"
colcon test --packages-select graph_based_slam \
  --ctest-args -R test_registration_determinism
```
