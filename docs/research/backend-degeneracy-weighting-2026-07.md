# Backend degeneracy covariance weighting (2026-07)

## Status

Implemented as a default-off Track-B candidate. Synthetic and optimizer-level
tests pass; real-bag adoption evidence is still pending. This candidate does
not replace the rejected frontend solve interventions and cannot improve a
zero-loop, zero-anchor trajectory by itself.

## Contract

RKO-LIO publishes its final ICP Hessian as an anisotropic
`nav_msgs/Odometry.pose.covariance`. The live component and deterministic
offline runner retain the covariance of every created submap. When
`use_degeneracy_covariance_weighting` is enabled, the backend:

1. reconstructs and classifies the six Hessian directions using the frozen
   Phase-1 localizability thresholds;
2. preserves the historical adjacent-edge information matrix in every
   well-conditioned direction;
3. scales isolated degenerate directions by
   `degeneracy_adjacent_edge_information_scale` (default `0.25`);
4. scales repeated non-observable subspaces by
   `non_observable_adjacent_edge_information_scale` (default `0.05`);
5. falls back exactly to the historical matrix when disabled, when covariance
   diagnostics are absent, or when the covariance is malformed.

The directional matrix is applied by congruence with the square root of the
existing base information matrix. This preserves the existing unified or
split translation/rotation balance instead of replacing it with the raw,
correspondence-count-dependent Hessian scale.

## Verification completed

- corridor covariance weakens only its isolated weak translation axis;
- a fully observable box preserves the legacy matrix;
- missing and isotropic fallback covariance preserve the legacy matrix;
- identical optimizer inputs remain deterministic;
- with a competing loop constraint, weakening an injected weak direction
  lets the loop correct more drift along that direction;
- live component and offline runner build successfully;
- GLIM, FAST-LIVO2, candidate provenance, per-sequence gate, result composer,
  and three-holdout suite tests pass together.

## Next real-data A/B

Use one frozen backend-input bag for both sides and fixed loop edges. Only the
flag changes:

```bash
ros2 run graph_based_slam graph_slam_offline_runner --ros-args \
  --params-file <graph.yaml> \
  -p bag_path:=<backend_input> -p output_dir:=<off> \
  -p use_degeneracy_covariance_weighting:=false

ros2 run graph_based_slam graph_slam_offline_runner --ros-args \
  --params-file <graph.yaml> \
  -p bag_path:=<backend_input> -p output_dir:=<on> \
  -p use_degeneracy_covariance_weighting:=true
```

Run Construction Seq2 first because it has five verified loops and independent
checkpoints. Require three byte-identical ON repetitions, no harmful loop,
corrected APE no worse than the default-off result, and map mean/p95 thickness
and coverage within the existing two-percent non-regression envelope. Then run
exp04/exp07 as safety checks: with no competing constraint, their optimized
trajectory must remain equal to the input trajectory within numerical
tolerance. Keep the candidate default-off until those gates pass.
