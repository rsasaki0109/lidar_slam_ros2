# Phase 8 RTK-SLAM plane-revisit validation

Date: 2026-07-14

## Decision

The three-dataset gate **rejects the current plane-revisit candidate**. The
factor remains opt-in: it improves MID-360 substantially and does not affect
the HILTI exp04 holdout, but Construction Seq2 position-only ATE regresses by
0.772%. Map geometry improves, so the next iteration should tune factor weight
or acceptance gates rather than discard plane association.

Machine-readable evidence is on the external SSD:

```text
/media/sasaki/aiueo/benchmarks/phase8/rtkslam_cs2_20260714/
  plane_revisit/{off,on}/manifest/cross_repo_benchmark.json
  three_dataset_candidate_regression.{json,md}
```

## Dataset and reference

RTK-SLAM Construction Seq2 contains 599.51 s of data, 5,878 PointCloud2
frames, and 119,819 IMU messages. Its downloaded database SHA-256 is
`9e808703a57d7be6afa6a37abb8f5d65c6566f71f4864cd4c24cb01f6ab82af5`.
The evaluation uses all 16 surveyed total-station positions; orientation was
not surveyed, so the profile forbids rotational RPE.

RKO-LIO produced 5,875 dense poses and 0.153779 m position-only ATE after SE(3)
alignment. The backend capture contains 5,865 frames and 5,864 odometry
messages. Its recovered MCAP was replayed three times per arm and produced
byte-identical loop-edge and optimized-trajectory artifacts in every run.

## OFF/ON result

Both arms use the same six loop edges, 225 graph anchors, and dense frontend
trajectory. ON extracts 1,788 plane patches and accepts 45 of 926 candidate
constraints after the initial-residual gate rejects 881.

| Metric | OFF | ON | Change | Gate |
| --- | ---: | ---: | ---: | :---: |
| position-only ATE (m) | 0.174372 | 0.175718 | +0.772% | fail |
| planar thickness mean (m) | 0.083571 | 0.083530 | -0.049% | pass |
| planar coverage | 0.284945 | 0.290064 | +1.796% | pass |
| processing/sensor time | 0.069904 | 0.072524 | +3.749% | pass |

The runtime denominator is the 599.064 s dense-trajectory timestamp span, not
the 1,308 s wall-clock span of the backend capture.

## Three-dataset conclusion

- MID-360: ATE -28.00%, translational RPE -7.62%, pass.
- HILTI exp04: trajectory and map unchanged, pass as negative holdout.
- Construction Seq2: geometry improves but surveyed-position ATE regresses,
  fail as the required second positive sequence.

The next candidate should keep the reproducible dataset contract and sweep the
normal/offset information weights plus initial offset gate. Promotion still
requires Construction Seq2 ATE to improve without exceeding the existing map
quality and runtime budgets.

## Follow-up: loop-edge attribution

Checkpoint-level attribution showed that the graph baseline's main regression
was not caused by the plane factor. Removing only distance-loop edge `57 -> 123`
changed graph ATE from 0.174372 m to 0.143967 m: 17.44% better than the six-edge
graph and 6.38% better than raw RKO-LIO. Removing any of the other five edges
made ATE worse, so a global loop-weight reduction is not an adequate fix.

The result is deterministic across three runs when the five verified edges are
frozen. Plane revisit on that corrected substrate changes ATE to 0.144433 m and
reduces planar coverage from 0.295084 to 0.291398, so the plane candidate still
fails independently of the bad loop. Use `analyze_sparse_checkpoint_errors.py`
to retain the per-checkpoint evidence instead of relying on aggregate ATE alone.

## Follow-up: cloud-overlap loop verifier

NDT fitness alone could not separate the harmful edge from the useful ones.
An aligned-source overlap metric now counts points with a target neighbor
within 0.5 m, after the cheaper fitness and correction gates pass. The bad
`57 -> 123` edge measured 0.648 overlap. Its possible replacements
`59 -> 123` and `59 -> 124` measured 0.710 and 0.752, while the lowest useful
edge (`54 -> 145`) measured 0.765. The other useful edges ranged from 0.794
to 0.955.

`loop_min_overlap_ratio=0.76` therefore produced exactly the five-edge oracle
set without a fixed-edge CSV. Three runs were byte-identical, and the dense
position-only ATE was 0.143967 m (6.38% better than raw and 17.44% better than
the original six-edge graph). Map quality was also deterministic and matched
the fixed five-edge result: mean planar thickness 0.083919 m and coverage
0.295084. Evidence is stored under:

```text
/media/sasaki/aiueo/benchmarks/phase8/rtkslam_cs2_20260714/
  overlap_probe_r05/
  overlap_gate070/
  overlap_gate076/
  overlap_gate076_probe/manifest/
```

The optimized implementation adds about 2.3 s to the dynamic offline backend
run on this sequence (roughly 5.7% including refinement), rather than the
initial diagnostic implementation's 20 s overhead.

## Cross-dataset overlap holdouts

The same `0.76 / 0.5 m` gate was then exercised through dynamic loop search,
not fixed-edge replay, on MID-360 and HILTI exp04. The result rejects promotion
as a global default:

- HILTI exp04 passed the negative holdout. Under the deliberately open
  `distance_loop_closure=20 m`, fitness `1.1` profile, the known false
  `2 -> 37` edge had overlap 0.594 and was rejected. The resulting zero-loop
  trajectory was byte-identical to the conservative baseline.
- MID-360 did not preserve the known edge. `6 -> 604` had overlap 0.742 and
  was rejected; the nearby `7 -> 604` candidate had overlap 0.797 and was
  accepted instead. Dense cross-validation ATE was 1.226013 m versus
  1.167254 m raw (+5.03%), so this is not a safe transfer despite keeping one
  loop edge.

Evidence is stored in:

```text
/media/sasaki/aiueo/benchmarks/phase8/loop_overlap_holdouts_20260714/
  hilti_exp04_open_gate_probe/
  mid360_probe/
```

Keep the generic YAML default disabled (`loop_min_overlap_ratio=0.0`) and use
0.76 only for the validated Construction Seq2 profile. A general verifier now
needs a density/FOV-aware score (for example multi-radius support or a mutual
source/target overlap), because no single 0.5 m threshold separates the
Construction substitute at 0.752 from the MID-360 edge at 0.742.

## Correction-adaptive overlap candidate

Bidirectional overlap was measured before choosing a replacement. Its harmonic
mean did not separate the classes: the Construction false edge scored 0.580,
the valid MID-360 edge 0.544, and valid Construction `54 -> 145` only 0.315.
The reverse direction is biased by the larger target aggregation window, so it
is retained only as a debug diagnostic rather than exposed as a gate.

The useful separation came from registration correction magnitude. The
Construction false edge and substitutes apply only 0.30--0.45 m translation,
the MID-360 edge applies 7.25 m, and the HILTI false edge applies 2.00 m but has
only 0.594 source overlap. The following explicit adaptive candidate was
therefore evaluated:

```yaml
loop_min_overlap_ratio: 0.76
loop_min_overlap_ratio_large_correction: 0.70
loop_overlap_large_correction_translation_m: 1.0
loop_overlap_max_distance_m: 0.5
```

| Dataset | Result | Artifact check |
| --- | --- | --- |
| Construction Seq2 | five verified edges; dense ATE 0.143967 m | edge and trajectory hashes match the three-run five-edge result |
| HILTI exp04 open gate | false `2 -> 37` rejected; zero loops | trajectory hash matches the conservative baseline |
| HILTI exp01 | no candidate passed the fitness gate; zero loops | OFF/ON APE 0.897399 m and trajectory hash `a05346e83a7a` match |
| HILTI exp07 | no candidate passed the fitness gate; zero loops | OFF/ON APE 0.618685 m and trajectory hash `ecd6d0e52b50` match |
| MID-360 | valid `6 -> 604` retained; dense ATE 1.214342 m | edge and trajectory hashes match the prior two-run dynamic baseline |
| KITTI 00 | valid `28 -> 176` retained; overlap 0.864002 | edge `5d0344e82042` and trajectory `b39e5cc9ff98` hashes match the established baseline |

This fixes the narrow-FOV transfer failure without weakening the strict gate
for small corrections. It is implemented consistently in the live component
and offline runner, but stays opt-in pending more than these three substrates.
Evidence is under:

```text
/media/sasaki/aiueo/benchmarks/phase8/
  mutual_overlap_probe_20260714/
  adaptive_overlap_gate_20260714/
  adaptive_overlap_gate_kitti00_20260714/current/
  hilti_overlap_crossval_20260714/{exp01,exp07}/
```

The exp01/exp07 holdouts establish non-regression on complete sequences, but
do not exercise the overlap threshold because the cheaper fitness gate rejects
all candidates. They therefore strengthen release coverage without by
themselves justifying a generic default-on setting. Their 2,277-pair and
1,322-pair frozen backend inputs were produced by
`scripts/run_hilti_overlap_crossval.sh`; the wrapper also emits machine-readable
OFF/ON comparisons and refuses to overwrite incomplete captures.

## HILTI fitness relaxation and support-distance probe

The exp01 zero-loop result was investigated rather than assuming its NDT
fitness scale merely needed normalization. This follows the motivation of
[Trimmed ICP](https://doi.org/10.1109/ICPR.2002.1047997), which applies least
trimmed squares to partially overlapping clouds, and
[Generalized-ICP](https://doi.org/10.15607/RSS.2009.V.021), which models local
planar structure probabilistically. Neither idea was promoted without a
fixed-input counterfactual.

| exp01 variant | loop edges | interpolated APE RMSE (m) | conclusion |
| --- | ---: | ---: | --- |
| conservative NDT fitness 0.7 | 0 | 0.897399 | baseline |
| NDT fitness 10, overlap disabled | 2 | 1.000332 | relaxed fitness regresses 11.5% |
| NDT fitness 10, adaptive overlap | 1 (`14 -> 79`) | 1.461211 | high-overlap alias regresses 62.8% |
| GICP fitness 10, adaptive overlap | 0 | 0.897399 | rejects exp01 aliases, but also rejects the valid KITTI edge |
| ScanContext, fitness 0.7, adaptive overlap | 0 | 0.897399 | descriptors find candidates but none pass geometric verification |

The harmful `14 -> 79` edge has NDT fitness 0.849210, correction 3.289 m /
7.933 deg, source overlap 0.746136, support RMSE 0.239223 m, and support p90
0.399387 m. Sparse-checkpoint RMSE worsens from 1.281948 m to 1.774295 m.
The valid KITTI `28 -> 176` edge has nearly the same support residuals
(0.232568 m RMSE, 0.369690 m p90), so support distance cannot safely classify
the alias. It is emitted as a density-independent diagnostic but is not an
acceptance gate. Cross-registration consensus was also rejected because GICP
failed to converge on the valid KITTI edge.

The evidence supports retaining the conservative NDT fitness gate. exp07 is a
104.8 m one-way corridor and produces no loop candidates even at fitness 10,
so loop-closure tuning cannot improve that sequence. Further exp01/exp07
accuracy work belongs in the odometry/degeneracy path, not a weaker graph
verifier. Probe artifacts are under:

```text
/media/sasaki/aiueo/benchmarks/phase8/
  hilti_fitness_probe_20260714/
  loop_support_probe_20260714/
```
