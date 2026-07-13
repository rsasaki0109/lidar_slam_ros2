# KITTI 00 localization_zoo → graph-SLAM loop (2026-07-13)

## Decision

The two repositories now share a fixed, timestamp-paired KITTI Odometry 00
evaluation input. Keep the graph loop fitness threshold at `0.7`. Relaxing it
to `1.5` admits one loop, but slightly worsens independent ground-truth APE, so
the candidate is **not adopted**.

On that rejected one-edge profile, raising loop information weight from 100 to
400 improves both fitted residual and APE relative to weight 100, but still
does not beat the threshold-0.7 default. Weight 400 therefore remains
non-default as well.

This is a second graph-SLAM dataset with trajectory ground truth. It confirms
that an accepted registration edge is not, by itself, evidence of a better
trajectory.

## Input contract

- public dataset: KITTI Odometry sequence 00, 4,541 scans, 454.0 s
- PCD source: `localization_zoo/dogfooding_results/kitti_seq_00_full`
- frontend trajectory: `TrICP_LO.txt`, produced without a GT seed
- official calibration: KITTI `sequences/00/calib.txt`
- reference: KITTI camera poses converted to the Velodyne body frame
- backend cloud sampling: deterministic acquisition-order stride 16
- backend topics: `/rko_lio/odometry` and `/rko_lio/frame`
- pairing: exact nanosecond timestamp, 4,541 odometry/cloud pairs each

The graph input bag contains every scan and pose; only the number of points in
each cloud is reduced. Both graph candidates consume the same 560,648,192-byte
SQLite bag (`SHA-256 4b64f7e648f3e5548a9ef964d83327e744755bb977d6a80d8aa82343e9cde242`).
The runner reports 4,541 paired scans, 356 submaps and 3,707.2 m travelled, with
no unpaired messages.

The full TrICP trajectory scores at 8.116305 m SE(3)-aligned APE over all 4,541
poses. Its TUM SHA-256 is
`c17d27047e92798d66c06ccc13a73565396761fa3e4ca9f78826099c29d445d7`.

## Frontend findings

The deterministic scanmatcher path was first validated independently: two
isolated runs on the stride-4 bag produced byte-identical 4,541-pose
trajectories (`e370eecf3c37cab8f992f78e885eb9d0`) and identical 92-submap CSVs
(`36a7d09aabe177659d87f00c410e0747`). Total wall time was 182.03 s and peak RSS
was 584,896 KiB.

That generic configuration was not accurate on KITTI: it travelled only 686.1
m versus the reference 3,723.2 m. The KITTI NDT configuration was also tested
on a fixed 1,000-frame prefix:

| motion gate | APE RMSE (m) | zero steps | Z extent (m) | decision |
|---|---:|---:|---:|---|
| 8 m/s baseline | 127.983 | 0 | 2.42 | reference only |
| 14 m/s | 120.016 | 666 | 46.93 | reject |
| 20 m/s | 114.088 | 539 | 42.97 | reject |

Widening the motion gate reduced some rejection symptoms but admitted bad
registrations and eventually froze the trajectory. No parameter change was
kept. TrICP-LO was therefore used as the independent frontend substrate for the
graph experiment.

The optional ROS small_gicp node initially could not be built because a normal
CMake imported target was passed to `ament_target_dependencies`. Linking
`small_gicp::small_gicp` directly fixes that build contract. Its online default
still lost tracking on the stride-4 bag, so it was not used for the result.

## Graph ablation

Both rows use `submap_distance_threshold=10.0`, one best loop candidate,
`refine=false`, and identical input. APE is evaluated at the 356 recorded
submap timestamps, never interpolated across sparse poses.

| metric | threshold 0.7 | threshold 1.5 | change |
|---|---:|---:|---:|
| accepted loops | 0 | 1 | +1 |
| raw APE RMSE (m) | 8.263387 | 8.263387 | 0 |
| optimized APE RMSE (m) | 8.263387 | 8.267352 | **+0.0480%** |
| optimized median error (m) | 6.224564 | 6.214412 | -0.163% |
| optimized max error (m) | 21.600065 | 21.610661 | +0.0491% |
| wall time (s/run) | 29.98 | 37.39 | report only |
| peak RSS (KiB) | 171,148 | 168,756 | -1.40% |

The admitted edge is submap `28 → 252`, registration fitness `0.865084`.
Although median error falls slightly, the primary RMSE and maximum error both
regress. It fails the accuracy gate and remains non-default. Runtime is not
treated as a candidate benefit because run order and host load were not
counterbalanced. The baseline's two isolated runs produced byte-identical loop
sets and optimized trajectories (`SHA-256
756245d53bec6f7a02a8fded0713a652350b46b5f12b5ba057fb41dd6ccf432a`).

## Loop-weight transfer check

The accepted edge was then frozen and only `loop_edge_info_weight` was changed.
This checks the direction previously seen on MID360 without changing candidate
discovery.

| metric | weight 100 | weight 400 | change |
|---|---:|---:|---:|
| edge set | `28 → 252` | `28 → 252` | identical |
| APE RMSE (m) | 8.267352 | 8.264587 | -0.0334% |
| loop rotation residual (deg) | 2.369562 | 2.067070 | -12.77% |
| loop translation residual (m) | 0.00053716 | 0.00013030 | -75.74% |

Weight 400 fits the frozen constraint better and recovers most of the APE
regression introduced by admitting it. It still loses to the threshold-0.7
default (8.263387 m). MID360 has no independent trajectory GT, and its earlier
comparison used weight 200 rather than 100, so these two observations cannot be
counted as two independent trajectory improvements under `public_suite_v1`.

## Reproduction

Create the fixed paired bag from localization_zoo output:

```bash
python3 scripts/pcd_sequence_to_rosbag2.py \
  --pcd-dir /path/to/localization_zoo/dogfooding_results/kitti_seq_00_full \
  --gt-matrices /path/to/localization_zoo/dogfooding_results/kitti_seq_00_full_evaluated_gt.txt \
  --estimate-matrices /path/to/localization_zoo/dogfooding_results/TrICP_LO.txt \
  --calib /path/to/KITTI_odometry/sequences/00/calib.txt \
  --output-dir /path/to/backend_tricp_stride16 \
  --point-stride 16 \
  --topic /rko_lio/frame --odom-topic /rko_lio/odometry
```

Run the frozen baseline and the rejected candidate:

```bash
bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/backend_tricp_stride16/rosbag2 \
  --params lidarslam/param/lidarslam_kitti_velodyne.yaml \
  --runs 2 --output-dir /path/to/threshold0p7 \
  --reference-tum /path/to/backend_tricp_stride16/ground_truth.tum \
  --param submap_distance_threshold:=10.0 \
  --param max_loop_candidate_count:=1 --param refine:=false

bash scripts/run_offline_determinism_check.sh \
  --bag /path/to/backend_tricp_stride16/rosbag2 \
  --params lidarslam/param/lidarslam_kitti_velodyne.yaml \
  --runs 1 --output-dir /path/to/threshold1p5 \
  --reference-tum /path/to/backend_tricp_stride16/ground_truth.tum \
  --param submap_distance_threshold:=10.0 \
  --param max_loop_candidate_count:=1 \
  --param threshold_loop_closure_score:=1.5 --param refine:=false
```

For the report-only weight transfer, append
`--param loop_edge_info_weight:=400.0` to the second command.

The wrappers assign a private localhost ROS domain per run and only reuse an
output carrying a `.complete` marker, preventing stale DDS participants or
partial trajectories from contaminating deterministic comparisons.
