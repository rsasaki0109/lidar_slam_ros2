# Non-zero loop map bundle validation (2026-07-14)

## Scope

Validate `/map_save` with real accepted loop closures. Saved-map loading,
localization mode, and relocalization remain outside scope.

## Fixed input

- Dataset: RTK-SLAM Construction Seq2
- Backend bag: `phase8/rtkslam_cs2_20260714/backend_input`
- Submaps: 225
- Loop verifier: source overlap at least 0.76 within 0.5 m
- Independent position checkpoints: 16

The live ROS replay artifact is stored at:

`/media/sasaki/aiueo/benchmarks/rtkslam_cs2_live_bundle_20260714_v1/replay_run1`

## Result

`verify_map_bundle.py` reports `MAP_BUNDLE_OK` with 225 trajectory poses,
225 g2o vertices, and five loop-edge rows. Autoware map verification reports
eight passes and zero failures.

The saved `loop_edges.csv` and `trajectory_optimized.tum` SHA-256 hashes match
the established three-run deterministic offline result exactly:

- loop edges: `6361afcbf61d1af871a9aca42c77829bba4fe20f460b683035b78a0dfc0c2a56`
- optimized trajectory: `e73fe94c3c6b0cc824ab07334e1cc57f435935890f058904a4202fcffa1ef058`

Propagating the 225 graph corrections onto the 5,875-pose frontend trajectory
also produces a byte-identical dense trajectory. Position-only APE improves
from 0.153778917 m raw to 0.143967274 m with five verified loops (6.38%).

The saved map's three quality runs are byte-identical. Mean planar thickness
is 0.0839185 m, p95 is 0.1297403 m, planar coverage is 0.295080, and MME valid
fraction is 0.948441. Coverage narrowly misses the existing indoor blocking
profile's 0.30 threshold, so this run is retained as deterministic evidence,
not claimed as a blocking-profile pass.
