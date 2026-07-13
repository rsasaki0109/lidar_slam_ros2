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
