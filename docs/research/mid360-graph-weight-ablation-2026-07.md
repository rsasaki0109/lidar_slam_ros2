# MID-360 pose-graph loop-weight ablation (2026-07-13)

## Decision

Keep `loop_edge_info_weight: 200.0` as the default. A deterministic
`200 -> 400` ablation improves the fitted rotation residual of the one accepted
loop constraint, and does not regress the measured map geometry, but it has no
ground-truth trajectory evidence and covers only one public dataset. It therefore
does not meet `public_suite_v1`'s minimum of two improved datasets.

An accepted-edge residual is an optimization diagnostic, not a trajectory
accuracy metric. Lowering it must not be reported as lower ATE or drift without
an independent reference.

## Fixed input

Dataset: Zenodo `driving_slam_mid360`, 277.17 s, 2,772 Livox scans and 55,435
IMU messages. The frontend output was recorded once, then consumed directly by
`graph_slam_offline_runner`; ROS topic replay was rejected because subscriber
delivery changed the number and identity of accepted loops between runs.

Both candidates consumed the same 2,765 odometry/cloud pairs and produced 640
submaps over 1,079.3 m. Their `loop_edges.csv` files are byte-identical:

- SHA-256: `5e30036d9190933e9659805caac032cc675ba968700a096c788f12560026c6d9`
- one accepted edge: submap `6 -> 604`
- registration fitness: `0.4731357485638518`

The baseline was also run twice; accepted edges and optimized trajectories were
byte-identical across both runs. The candidate was run once after this fixed-input
contract had been established.

## Results

All geometry metrics use the same `map_quality_report` profile at 0.20 m
downsampling.

| metric | weight 200 | weight 400 | change |
|---|---:|---:|---:|
| loop rotation residual mean (deg) ↓ | 1.855587 | 1.469802 | **−20.79%** |
| loop translation residual mean (m) ↓ | 0.00005338 | 0.00003690 | −30.87% |
| optimized thickness mean (m) ↓ | 0.0434823 | 0.0434508 | −0.07% |
| optimized thickness p95 (m) ↓ | 0.110545 | 0.110463 | −0.07% |
| optimized planar coverage ↑ | 0.446315 | 0.447399 | +0.24% |
| refined thickness mean (m) ↓ | 0.0433376 | 0.0431428 | −0.45% |
| refined thickness p95 (m) ↓ | 0.110574 | 0.110327 | −0.22% |
| refined planar coverage ↑ | 0.443010 | 0.443890 | +0.20% |
| peak RSS (MiB) ↓ | 610.88 | 611.27 | +0.06% |
| wall time/run (s) ↓ | 461.62 | 519.17 | +12.47% |

The runtime row normalizes a two-run baseline driver and a one-run candidate
driver. It is recorded for completeness, not treated as a stable performance
claim; this parameter changes only the edge information matrix and the observed
runtime delta is likely host/run-order noise. Peak memory is effectively tied.

## Reproduction

Record the frontend once, then run the backend without ROS transport replay:

```bash
bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/backend_input \
  --params lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  --runs 2 --output-dir /path/to/weight200 --save-maps

bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/backend_input \
  --params /path/to/same-config-with-loop-edge-info-weight-400.yaml \
  --runs 1 --output-dir /path/to/weight400 --save-maps
```

Generate `pose_graph_loop_residuals.json` in each `run1` directory with
`analyze_pose_graph_loop_residuals.py`, and generate optimized/refined map quality
reports with `run_map_quality_check.sh`. Then freeze the decision artifact:

```bash
python3 scripts/compare_graph_slam_offline_ablation.py \
  --baseline-dir /path/to/weight200 --candidate-dir /path/to/weight400 \
  --baseline-runs 2 --candidate-runs 1 \
  --dataset driving_slam_mid360 \
  --parameter loop_edge_info_weight \
  --baseline-value 200 --candidate-value 400 \
  --improved-datasets 0 --minimum-improved-datasets 2 \
  --max-geometry-regression-percent 2 \
  --output /path/to/graph_weight_ablation.json
```

The resulting verdict is `DO_NOT_ADOPT`, specifically because the
minimum-improved-dataset gate is not met. The next valid promotion step is a
fixed-input run on a second dataset with independent trajectory ground truth,
not further tuning on this MID-360 sequence.
