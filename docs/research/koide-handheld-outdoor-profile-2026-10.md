# Handheld MID-360 outdoor profile (Koide outdoor_hard, 2026-10)

**Question:** where does the SLAM front end lose accuracy on handheld outdoor MID-360 data, and does a configuration-only change fix it?

**Data:** [Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133) (CC BY 4.0), `outdoor_hard_02a`, `02b` and `01b`.
- IMU acceleration is rescaled from g to m/s² (`tools/readme_media/scale_imu_bag.py`).
- Pipeline: `scripts/run_rko_lio_graph_benchmark.sh`, with `lidarslam/param/lidarslam.yaml` for the backend.
- Metrics: APE after Umeyama SE(3) alignment against the dataset ground truth (nearest pose within 0.05 s).
- No sequence produced a loop closure, so raw and corrected trajectories coincide. The numbers are front-end (RKO-LIO) odometry.

## Error split

Previous outdoor settings (`voxel_size` 1.0, no gravity alignment):

| Sequence | xy RMSE | z RMSE | 3D RMSE |
| --- | --- | --- | --- |
| 02a | 0.81 m | 0.64 m | 1.03 m |
| 02b | 0.29 m | 0.56 m | 0.63 m |
| 01b | 4.79 m | 0.54 m | 4.82 m |

On 02a and 02b, most of the error is vertical. Slow pitch/roll drift integrates into height while walking.

## Selection on 02a only

| RKO-LIO change | xy | z | 3D |
| --- | --- | --- | --- |
| none | 0.81 | 0.64 | 1.03 |
| `voxel_size: 0.5` | 0.70 | 0.73 | 1.01 |
| `voxel_size: 0.75` | – | – | 1.17 |
| `gravity_window_alignment` (20 s) | 1.11 | 0.20 | 1.13 |
| same, `gravity_alignment_gain: 0.02` | 0.97 | 0.35 | 1.03 |
| same, `gravity_window_sec: 40` | 0.95 | 0.26 | 0.99 |
| **`voxel_size: 0.5` + gravity alignment, 40 s** | **0.46** | **0.34** | **0.57** |

- **Gravity alignment's side effect:** it consistently trades horizontal for vertical error.
  - The correction rotates the optimized orientation after ICP, while the local map keeps the earlier, tilted geometry.
  - The next registrations absorb part of that mismatch horizontally.
  - The 40 s window makes the correction gentler. Folding gravity into the ICP objective instead is the principled follow-up.

## Unchanged check on 02b and 01b

| Sequence | Previous | New profile |
| --- | --- | --- |
| 02b | 0.63 m (z 0.56) | **0.37 m** (z 0.23) |
| 01b | 4.82 m (z 0.54) | 4.81 m (z 0.31) |

- Vertical error drops on all three sequences.
- **01b still fails horizontally (4.8 m) with either setting.** Its scans are often sparse or degenerate (the earlier completion sweep logged single-frame drops with 0–2 ICP keypoints), which needs separate work.

## Reference: GLIM odometry on the same sequences

- **Source:** GLIM v1.2.2 (GICP, container `glim-ros2:jazzy-v1.2.2`) runs from 2026-07-16, full sequences, 4 runs each.
- **Evaluation:** the GLIM pose output was evaluated with the same alignment and association as above.
- These runs were not repeated for this note, and the runtime configuration differs from ours. Read the comparison as indicative, not as a benchmark claim.

| Sequence | GLIM 3D RMSE (4 runs) | GLIM xy / z (run 1) | New profile |
| --- | --- | --- | --- |
| 02a | 0.72–0.80 m | 0.69 / 0.21 m | 0.57 m |
| 02b | 0.37–0.40 m | 0.34 / 0.17 m | 0.37 m |
| 01b | 0.30–0.37 m | – | 4.81 m |
