# MID-360 map-refinement sweep (2026-07-13)

## Decision

Keep the offline refiner defaults (`cloud_downsample_voxel=0.10 m`, window
`16`, stride `8`). None of the three fixed-input candidates is a Pareto
improvement across plane thickness, planar coverage, entropy, and resources.

This is a rejection result, not a tuning recommendation. In particular, the
small `0.20 m` thickness-mean win must not be selected while hiding its coverage
loss and p95/entropy regressions.

## Isolation contract

All candidates consumed the same recorded `driving_slam_mid360` backend bag:
2,765 odometry/cloud pairs, 640 submaps, 1,079.3 m, and one accepted loop. Each
candidate matched the baseline artifacts byte-for-byte before refinement:

- `loop_edges.csv` SHA-256:
  `5e30036d9190933e9659805caac032cc675ba968700a096c788f12560026c6d9`
- `trajectory_optimized.tum` SHA-256:
  `9b8fd9faff62e7f8a20073373c7d85f5153bee0d18128a4e5ed957df351574ac`
- `map_optimized.pcd` SHA-256:
  `64b93f162d4b8affee697e15a7803c08cb4733adf3d2f60bef2dfbc82b04d803`

Only the refiner parameter named in each row changed. Map-quality extraction was
held at 0.20 m for every output.

## Results

| candidate | thickness mean (m) ↓ | Δ | p95 (m) ↓ | Δ | coverage ↑ | Δ | entropy (nats) ↓ | Δ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| default: voxel 0.10, 16/8 | **0.0433376** | — | 0.110574 | — | 0.443010 | — | **−1.664887** | — |
| window 32/stride 16 | 0.0437350 | +0.92% | 0.110850 | +0.25% | 0.442524 | −0.11% | −1.659809 | +0.31% |
| voxel 0.20, 16/8 | **0.0432970** | −0.09% | 0.110586 | +0.01% | 0.441512 | −0.34% | −1.664731 | +0.01% |
| voxel 0.05, 16/8 | 0.0433746 | +0.09% | **0.110493** | −0.07% | **0.443133** | +0.03% | −1.663921 | +0.06% |

Positive entropy delta means less negative and therefore worse. The 32/16 case
also increased mean correction magnitude (0.138→0.152 m) and correction jumps
at window boundaries, consistent with its geometry regression.

Peak RSS was effectively tied at about 611 MiB. Single-candidate wall times were
528–551 s versus 462 s/run for the earlier two-run baseline, but the runs were
sequential under changing host load. Record that observation; do not claim a
parameter runtime effect from it.

## Reproduction

`run_offline_determinism_check.sh` accepts repeatable ROS parameter overrides so
an ablation does not need a copied, silently drifting YAML:

```bash
bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/frozen/backend_input \
  --params lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  --runs 1 --output-dir /path/to/candidate --save-maps \
  --param refine_cloud_downsample:=0.20
```

The other commands used `refine_cloud_downsample:=0.05`, or both
`refine_window_size:=32` and `refine_window_stride:=16`. For every candidate:

```bash
bash scripts/run_map_quality_check.sh \
  --input /path/to/candidate/run1/map_refined.pcd \
  --output-dir /path/to/candidate/run1/map_quality_refined \
  --runs 1 --downsample 0.2
```

The next geometry change should address refinement acceptance/aggregation rather
than continue scalar tuning on this sequence. It must then pass the frozen
multi-dataset gate; this MID-360 sweep alone cannot promote a default.
