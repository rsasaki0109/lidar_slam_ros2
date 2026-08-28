# Degeneracy solve cross-sensor gate study (2026-07-14)

## Decision

Do not merge any of the tested stateless gates into RKO-LIO. Keep the Phase 2
solver default-off and unchanged. The experiments isolate repeated weak-space
freezing as the dominant MID-360 failure mechanism, but every stateless change
that made MID-360 safe also removed the HILTI exp07 corridor gain.

The next candidate must be stateful: constrain a weak direction only after the
same world-frame axis persists across consecutive scans. This follows the
existing Phase 1 evidence that exp07 has one persistent corridor axis, rather
than treating every small Hessian eigenvalue as equivalent.

## Motivation and literature check

The frozen Phase 2 candidate improved HILTI exp07 from 0.318107 m to
0.259018 m APE, but moved the public MID-360 trajectory 5.891015 m RMSE from
the unchanged RKO-LIO baseline. The implementation blends the
constant-velocity/IMU initial guess in isolated weak directions and freezes a
repeated weak eigenspace.

[X-ICP](https://doi.org/10.1109/TRO.2023.3335691) distinguishes controlled
updates from unchanged non-localizable directions and bases the decision on
fine-grained correspondence contributions. The later
[field analysis](https://arxiv.org/abs/2408.11809) reports that active
mitigation is useful without a reliable external estimate, while soft
constraints remain sensitive to heuristic tuning. These results support
gating the intervention itself instead of applying another global prior
weight.

## Cross-sensor A/B results

All MID-360 rows use 2,772 exactly timestamp-matched poses and SE(3) Umeyama
alignment against the unchanged default-off trajectory. HILTI rows use the six
official exp07 millimetre control points and the same nearest-timestamp scoring
as Phase 2.

| candidate | MID-360 delta RMSE (m) | HILTI exp07 APE (m) | verdict |
|---|---:|---:|---|
| Phase 2 frozen solve | 5.891015 | **0.259018** | cross-sensor hard fail |
| clamp blended update to geometric descent | 5.562912 | not run | insufficient |
| reject a dominant opposing prior | 2.175406 | 0.326772 | loses corridor gain |
| above + geometric fallback for repeated weak space | **0.200998** | 0.363435 | MID safer, HILTI regression |
| constrain only >99% translational weak axes | 5.852624 | not run | direction type does not separate sensors |

The repeated-weak geometric fallback removes 96.6% of the original MID-360
trajectory delta. This is strong evidence that zeroing repeated weak spaces,
not prior reversal alone, causes most of the narrow-FOV failure. It is not an
adoptable fix because exp07 becomes worse than default-off.

Artifacts:

- `/media/sasaki/aiueo/benchmarks/mid360_public/degeneracy_consistency_gate_20260714`
- `/media/sasaki/aiueo/benchmarks/mid360_public/degeneracy_consistency_gate_v2_20260714`
- `/media/sasaki/aiueo/benchmarks/mid360_public/degeneracy_consistency_gate_v3_20260714`
- `/media/sasaki/aiueo/benchmarks/mid360_public/degeneracy_translation_gate_20260714`
- `/media/sasaki/aiueo/benchmarks/hilti_exp07_degeneracy_consistency_gate_v2_20260714`
- `/media/sasaki/aiueo/benchmarks/hilti_exp07_degeneracy_consistency_gate_v3_20260714`

## Why translation fraction is insufficient

The frozen Phase 1 exp07 CSV was rechecked. All 925 isolated DEGENERATE
instances have translation fraction above 0.9957, and all 344 NON_OBSERVABLE
direction instances are above 0.9995. A 0.99 translation gate therefore
preserves the intended corridor class. It nevertheless leaves MID-360 almost
unchanged at 5.852624 m delta, proving that the MID failure also occurs in
translation-dominant weak directions.

## Next implementation gate

Track weak translation axes in the world-frame tangent space across scans:

1. match the current axis to the previous axis with absolute cosine similarity;
2. require a short consecutive-scan streak before prior blending or weak-space
   freezing is enabled;
3. reset the streak on a well-conditioned scan, category change, or axis jump;
4. fall back to the legacy geometric update while unconfirmed;
5. expose streak, matched cosine, and intervention counts in diagnostics.

This candidate was implemented and evaluated in
[`persistent-degeneracy-gate-2026-07.md`](persistent-degeneracy-gate-2026-07.md).
It reduced the MID-360 trajectory delta from 5.891 m to 0.530 m while keeping
HILTI exp07 near its default-off baseline, but did not meet the cross-sensor
adoption gate. Persistence alone is therefore retained as an opt-in research
path, not promoted to the default solve.

Tune only the persistence length and cosine threshold on HILTI exp07. Then run
MID-360 first as the hard safety holdout, followed by untouched HILTI exp01 and
exp04. Promotion still requires the Phase 2 determinism and resource gates.
