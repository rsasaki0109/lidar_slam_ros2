# HILTI 2022 exp04 graph-loop weight ablation (2026-07-13)

## Decision

Keep the current HILTI exp04 graph result with no accepted loop. A fixed
1,258-pair RKO-LIO backend bag makes the comparison reproducible, but the
lowest-fitness loop admitted by the diagnostic profile is a false constraint.
It worsens independent total-station APE at every tested information weight.

Do not adopt `distance_loop_closure=20.0`,
`threshold_loop_closure_score=1.1`, or a different loop information weight as
a HILTI default. The experiment is useful negative evidence: increasing the
weight makes the same bad edge progressively more damaging.

## Fixed input contract

- public dataset: HILTI SLAM Challenge 2022 exp04, construction upper level
- source: 1,258 PandarXT-32 scans and 50,198 Alphasense IMU samples, 125.8 s
- frontend: RKO-LIO with
  `configs/hilti2022/rko_lio_hilti2022_pandar.yaml`
- backend topics: `/rko_lio/odometry` and `/rko_lio/frame`
- pairing: exact header timestamp, 1,258 odometry/cloud pairs each
- raw trajectory: 1,258 poses, timestamp span
  1646304541.627159834--1646304667.323997974
- backend MCAP: 703,613,045 bytes, SHA-256
  `33b4f86df9054223412a8db1af8359416a631beb0f7754f780a0b7cbab59c56a`
- raw TUM SHA-256:
  `ceb356006a4514754c8c4df0431371f32b4df7bbcc5f235d614e0f7bb5480e5e`

The last raw hash above is the live TUM logger artifact. The RKO frontend
exited zero after 282.31 s wall time with 878,928 KiB peak RSS. The offline
graph runner reports all 1,258 pairs consumed, 44 submaps, 67.0 m travelled,
and no unpaired messages.

The first recording attempt was invalid: the benchmark wrapper interpreted a
pause at 139 clouds / 138 poses as completion. Its 120 s end margin covered
almost the entire 125.8 s bag. Quiet completion now additionally requires at
least 80% bag progress, so a short early writer pause cannot pass the gate.

## Baseline

The full raw trajectory scores at **0.0715599 m** SE(3)-aligned APE RMSE over
all seven total-station control points. Nearest-neighbour association uses a
0.05 s bound; all seven points match and the largest time gap is 0.04369 s.

With the repository `lidarslam.yaml` unchanged, two isolated graph runs
produce zero loops and byte-identical optimized trajectories (SHA-256
`3e94a3482b67e56de515cb830fcd374dd0b6cab5ffb7c150e1ee414c5922b390`).
The configured `distance_loop_closure=100.0` excludes the whole 67 m sequence
from loop search. That is conservative but correct for this measured case.

## Candidate and weight ablation

The trajectory physically revisits its start: submaps 0 and 43 are 1.077 m
apart after 63.9 m of intervening travel. Lowering only the travel exclusion
to 20 m activates registration. At threshold 0.7 every candidate is rejected.
The lowest observed fitness is 1.041556, for edge `2 -> 37`; a threshold of
1.1 admits exactly this one edge.

All weight rows have the same `loop_edges.csv` SHA-256
`ce986dc754f8efdf769c1faf5c14e2ac7a02704bdfeb73f637eace2d9ae2e3d0`.
Only `loop_edge_info_weight` changes. Sparse graph corrections are propagated
onto the 1,258-pose raw trajectory before association; the 44 graph anchors
are never directly interpolated across the sparse HILTI checkpoints.

| profile | accepted loops | APE RMSE (m) | mean (m) | max (m) | decision |
|---|---:|---:|---:|---:|---|
| raw / no loop | 0 | **0.071560** | 0.066569 | 0.114178 | keep |
| weight 20 | 1 | 0.347031 | 0.293193 | 0.492289 | reject |
| weight 100 | 1 | 0.623732 | 0.534215 | 0.855838 | reject |
| weight 400 | 1 | 0.770252 | 0.664042 | 1.045557 | reject |

The edge's reported correction is about 2.0 m and 14.4 degrees. Its fitness
alone is therefore not a safe acceptance signal. A stronger weight fits the
false constraint harder and monotonically worsens the independent GT metric.
Accuracy fails before runtime or memory can affect the adoption decision.

## Reproduction

Record one odometry/cloud pair for every RKO output scan, then verify the bag
before graph evaluation:

```bash
ros2 bag record --storage mcap -o backend_input \
  /rko_lio/odometry /rko_lio/frame

python3 scripts/odom_to_tum.py \
  --topic /rko_lio/odometry --output raw_full.tum --use-sim-time false

ros2 run rko_lio offline_node --ros-args \
  --params-file rko_params.ros.yaml \
  -p bag_path:=/path/to/exp04_ros2 \
  -p lidar_topic:=/hesai/pandar -p imu_topic:=/alphasense/imu \
  -p base_frame:=os_sensor -p publish_deskewed_scan:=true

ros2 bag info backend_input
```

Run the byte-reproducible baseline:

```bash
bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/backend_input --params lidarslam/param/lidarslam.yaml \
  --runs 2 --output-dir /path/to/graph_default
```

Run the rejected one-edge profile, appending one weight at a time:

```bash
bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/backend_input --params lidarslam/param/lidarslam.yaml \
  --runs 1 --output-dir /path/to/weight20 \
  --param distance_loop_closure:=20.0 \
  --param threshold_loop_closure_score:=1.1 \
  --param loop_edge_info_weight:=20.0

python3 scripts/densify_corrected_trajectory.py \
  --raw raw_full.tum --corrected /path/to/weight20/run1/trajectory_optimized.tum \
  --output /path/to/weight20/run1/trajectory_optimized_dense.tum \
  --max-anchor-offset 10.0
```

Repeat with weights 100.0 and 400.0, and score each dense result with
`scripts/ape_from_tum.py` using the HILTI exp04 control-point file and a 0.05 s
association bound.
