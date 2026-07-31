# Competitive-v2 / Candidate2 production gate (2026-08-01)

## Decision

- **competitive-v2 frontend:** pending clean isolated exp01/exp04 measurements.
- **Candidate2 backend:** **do not promote** under the frozen
  `competitive_slam_v1` contract. Only `spms_01` remains unused by candidate
  development, while the contract requires three assigned, frozen holdouts and
  three wins. In addition, the frozen manifest identifies a dirty source-tree
  hash rather than a reviewable commit, and its enabled planar-map refinement
  is not part of the isolated Candidate2 delta.

This decision separates implementation readiness from benchmark eligibility.
The code below is reviewable and builds in isolation; a successful regression
run cannot retroactively make a previously consumed sequence an unseen
holdout.

## Reviewable source boundary

| Area | Revision | Scope |
| --- | --- | --- |
| backend | `24fda87` | deterministic loop-search scheduling and unit test |
| backend transport | `45fe842` | exact-stamp odom/cloud pairing, reliable cloud QoS, and bounded queue headroom |
| frontend submodule | `60b861a`, `4619565` | exact sparse-voxel nearest-neighbour pruning, queue/config controls, and signed-boundary brute-force contract tests |
| production profile | `ef92aa7` | pinned frontend revision and competitive-v2 HILTI configuration |
| benchmark harness | `625f3d4` | explicit bag-end completion margin for strict completeness checks |
| holdout profiles | `4af54b2` | NTU-VIRAL frontend-v2 and trajectory-only Candidate2 safety settings |

The branch is `agent/kaizen-production-gate-20260801`, based on `3d44bc5`.
Benchmark/build directories are not source changes.

## Build and unit verification

- RKO-LIO core, `offline_node`, and `online_node_component`: built and linked
  from the isolated submodule worktree.
- `test_frontend_performance_contract`: 1/1 passed.
- `test_loop_search_schedule`: 3/3 passed.
- `graph_based_slam` production targets: colcon build passed in the isolated
  overlay.
- Overlay provenance was checked with `ros2 pkg prefix`; both `rko_lio` and
  `graph_based_slam` resolve inside the isolated worktree.

## Clean HILTI regression gate

Measurements must begin only with load1 <= 2.0 and sampled CPU busy <= 10%.
Runs performed while another benchmark owns the CPU are invalid.

| Sequence | Candidate RTF | Raw APE RMSE | Baseline RTF | Baseline APE RMSE | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| exp01 | pending | pending | 0.9783 | 0.06221018 m | pending |
| exp04 | pending | pending | 0.5037 | 0.06739868 m | pending |

The gate is RTF <= 1.0 and no accuracy regression above 2%.

### Measurement status

No value is accepted yet. The first unrelated Calibrex batch completed 200
trials, but a second 200-trial batch immediately began and occupied roughly
6.5 of 8 logical CPUs. The preflight correctly rejected launch (examples:
load1 3.28 and CPU busy 88.89%). The measurement process was never started,
so there is no contaminated candidate result to discard or accidentally cite.

The queued output root is
`competitive_ours/kaizen_clean_4af54b2_20260801`. It must record three
consecutive samples with load1 <= 2.0 and CPU busy <= 10% before exp01 starts.
Until that happens, competitive-v2 remains **not yet promotable**, rather than
failed.

## Holdout eligibility audit

| Dataset | Eligibility | Reason |
| --- | --- | --- |
| HILTI exp02/exp03/exp21 | consumed | explicitly used for candidate-1 loop/overlap development |
| HILTI exp01/exp04/exp07 | regression only | used during frontend/backend development |
| NTU-VIRAL rtp_01 | consumed | candidate runs, including a first-130-second run, already inspected |
| NTU-VIRAL tnp_02 | consumed | candidate runs, including a first-130-second run, already inspected |
| NTU-VIRAL spms_01 | retry-only | three infrastructure-failed attempts produced zero poses; no candidate metric was exposed, but the slot is not pristine/unopened |

The frozen Candidate2 manifest records correction caps of 0.5 m and 2 deg,
minimum overlap 0.76, and loop-search stride 1. Its formal profile requires
three frozen holdout slots and three wins. The local dataset history leaves
only one eligible sequence, so the formal promotion contract cannot be
satisfied without registering and freezing two genuinely new sequences before
examining their results.

The manifest pins repository revision `3d44bc5` and RKO-LIO revision
`689106a`, but separately fingerprints an 845-file dirty source tree. It also
enables `use_planar_map_filter`; that implementation belongs to the broader
backend research WIP and is absent from the isolated loop-scheduling commit.
Consequently, the frozen artifact cannot be reconstructed from its recorded
Git revisions alone. Candidate2 needs a new clean freeze after its map-export
delta is reviewed, even before the missing-holdout requirement is considered.

## Diagnostic evidence (not an unseen holdout)

The recoverable `rtp_01/ours_lio/run_01` trajectory completed 1739 frontend
poses and 99 graph anchors. Two loop edges were accepted: 28->94 and 37->98.
Their corrections were 0.0965848 m / 0.197824 deg and 0.133148 m / 0.531555
deg, with overlaps 0.809863 and 0.780715 respectively.

Post-hoc dense scoring using the frozen body-to-prism translation
(-0.293656, -0.012288, -0.273095) produced:

- raw APE RMSE: 1.8969631489 m
- dense corrected APE RMSE: 1.8929973655 m
- relative change: -0.209% (non-harmful)

This is useful safety evidence but is neither a complete formal run nor an
unused holdout.

## Unused holdout result

`spms_01`: pending clean isolated candidate retry. This is unused with respect
to candidate result/tuning evidence (all prior attempts produced zero poses),
but is reported as retry-only rather than as a pristine unopened slot.

## Required continuation

1. Wait for all unrelated Calibrex six-DoF batches to finish without stopping
   or altering them.
2. Run the queued exp01 once and exp04 three times from top revision
   `4af54b2` / RKO-LIO revision `4619565`.
3. Require complete trajectories, RTF <= 1.0, <=2% APE regression, and
   byte-identical exp04 frontend trajectories.
4. Run the scoped Candidate2 trajectory profile on `spms_01` and report it as
   retry-only diagnostic evidence. Do not call it a pristine formal holdout.
5. Update this document with the measured values. Candidate2 remains a no-go
   regardless of that diagnostic result until its dirty map-export delta is
   reviewable and two additional genuinely new holdouts are frozen.
