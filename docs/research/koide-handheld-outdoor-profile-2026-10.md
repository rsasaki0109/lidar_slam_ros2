# Handheld MID-360 outdoor profile (Koide outdoor_hard, 2026-10)

**Question:** where does the SLAM front end lose accuracy on handheld outdoor MID-360 data, and does a configuration-only change fix it?

**Data:** [Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133) (CC BY 4.0), `outdoor_hard_02a`, `02b` and `01b`; later `01a` and `outdoor_kidnap_a` / `_b`.
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
  - The 40 s window makes the correction gentler. Re-levelling the local map removes the mismatch; see [Re-levelling the local map](#re-levelling-the-local-map).

## Unchanged check on 02b and 01b

| Sequence | Previous | New profile |
| --- | --- | --- |
| 02b | 0.63 m (z 0.56) | **0.37 m** (z 0.23) |
| 01b | 4.82 m (z 0.54) | 4.81 m (z 0.31) |

- Vertical error drops on all three sequences.
- 01b still failed horizontally (4.8 m) with either setting.

## Scan gaps (01b)

- **Symptom:** the 01b heading error against the ground truth jumps from about 3° to about −65° within 1.3 s, 26 s into the run, and stays there. Short-window displacement magnitudes still match the ground truth, so the failure is heading, not translation.
- **Cause:**
  - The bag drops LiDAR scans for 1.2 s at that moment, and for 1.8 s later, while the IMU keeps streaming.
  - Past `max_scan_delta_sec` (default 1.0 s), RKO-LIO drops the scan and re-anchors at the new timestamp. The re-anchor keeps the pre-gap pose and discards the rotation integrated during the gap.
  - The operator was turning, so the next scans register against the map from a stale heading.
  - 02a also re-anchors once, at a 1.10 s gap.
- **Fix:** `max_scan_delta_sec: 3.0`. Scans after a gap of up to 3 s register from the IMU prediction instead. No re-anchor happens on any of the three sequences.

| Sequence | Profile without the change | With `max_scan_delta_sec: 3.0` |
| --- | --- | --- |
| 02a | 0.57 m (xy 0.46, z 0.34) | 0.57 m (xy 0.46, z 0.34) |
| 02b | 0.37 m (xy 0.28, z 0.23) | 0.37 m (xy 0.28, z 0.23) |
| 01b | 4.81 m (xy 4.80, z 0.31) | **0.32 m** (xy 0.28, z 0.16) |

- 10 s gives the same 01b result, since the longest gap is 1.8 s.
- RKO-LIO now re-anchors at the IMU-propagated pose instead of the pre-gap pose (rko_lio [#16](https://github.com/rsasaki0109/rko_lio/pull/16)). With that fix, 01b reaches 0.32 m (xy 0.29, z 0.13) with the default `max_scan_delta_sec` of 1.0 s and re-levelling on, so the setting is no longer needed to avoid the heading failure.
- 3 s keeps the drop for genuine long dropouts, where an IMU-only prediction is no longer trustworthy.

## Re-levelling the local map

- **Symptom:** with the profile above, the orientation is still tilted by about 1° against the ground truth (RMS 0.7–1.0° on all three sequences), and that tilt drives most of the vertical error.
- **Cause:**
  - The window measurement is right: on 02a the window tilt (1.4° at the end) matches the error against the ground truth.
  - A small correction is applied after every scan (02a: 2745 of 2745 scans).
  - The correction rotates only the new pose. The local map keeps the earlier tilt, so the next registrations pull it back.
- **Fix:** `gravity_alignment_relevel_map: true` (rko_lio [#15](https://github.com/rsasaki0109/rko_lio/pull/15)).
  - Once the window tilt reaches 0.29°, the full tilt is applied at once as a rigid rotation about the current position.
  - The rotation covers the pose, the previous pose, the local map and the gravity window.

| Sequence | xy / z / 3D before | xy / z / 3D re-levelled | Re-levels |
| --- | --- | --- | --- |
| 02a | 0.46 / 0.34 / 0.57 m | 0.47 / **0.23** / **0.53** m | 25 |
| 02b | 0.28 / 0.23 / 0.37 m | 0.28 / **0.19** / **0.34** m | 54 |
| 01b | 0.28 / 0.16 / 0.32 m | 0.28 / **0.13** / **0.31** m | 36 |

- Vertical error drops on all three sequences, and horizontal error is unchanged.
- **Cost:**
  - Each re-level rebuilds the local map, about 0.85 M points in 154 ms on average (max 271 ms).
  - Back to back under the same load, total CPU time was 599 s without re-levelling and 567 s with it, so the overhead is not measurable.
- Both modes reproduce identical APE on reruns.

## Held-out check (01a) and occluded scans

`outdoor_hard_01a` was not used for any of the choices above.

| Setting | xy | z | 3D |
| --- | --- | --- | --- |
| Previous outdoor settings | 0.58 m | 2.90 m | 2.96 m |
| Profile (with re-levelling) | 0.61 m | 1.44 m | 1.56 m |
| **Profile + `min_icp_keypoints: 100`** | 0.65 m | **0.31 m** | **0.72 m** |

- **Symptom:** both earlier settings jump about 7 m upward within 0.6 s at 126 s into the run, while the ground truth stays flat. The jump stays in the map.
- **Cause:**
  - The scan at 125.9 s keeps only 55 of its 20k points beyond 1 m. The sensor was most likely covered by a hand or the operator's body.
  - RKO-LIO refused only scans with fewer than 10 ICP keypoints, so ICP ran on a few dozen and slid vertically.
- **Fix:** `min_icp_keypoints: 100` (rko_lio [#18](https://github.com/rsasaki0109/rko_lio/pull/18), default 10). Such scans are skipped, and IMU propagation continues.
  - 10–20 scans per sequence are skipped.
  - 300 gives the same 01a result.

| Sequence | Profile | + `min_icp_keypoints: 100` |
| --- | --- | --- |
| 02a | 0.53 m | 0.55 m |
| 02b | 0.34 m | 0.34 m |
| 01b | 0.31 m | 0.31 m |

## Kidnap sequences: voxel size and keypoint threshold

`outdoor_kidnap_a` and `_b`: the operator covers the sensor and carries it elsewhere. Reference: the dataset reference trajectories (`outdoor_kidnap_{a,b}_reference.csv`), same alignment as above.

- **Symptom:** the profile above drifts 12.1 m (a) and 20.0 m (b). The previous outdoor settings gave 677 m and 0.47 m.
- **What happens:** about 740 scans of `_b` are skipped while the sensor is covered. 726 of them have fewer than 10 keypoints and are skipped at any threshold. Past `max_scan_delta_sec`, the state re-anchors every ~3 s at the IMU-propagated pose.
- **Negative result:** keeping only the IMU rotation (not translation) across long gaps made both worse (34 m / 29 m), because the operator keeps walking while the sensor is covered.

3D APE in m (– = not run):

| `voxel_size` | `min_icp_keypoints` | 02a | 02b | 01b | 01a | kidnap_a | kidnap_b |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.5 | 100 (previous profile) | 0.55 | 0.34 | 0.31 | 0.72 | 12.1 | 20.0 |
| 0.5 | 10 | – | – | – | – | 12.8 | 0.18 |
| 0.5 | 50 | 0.55 | 0.34 | 0.30 | 0.74 | 0.07 | 26.9 |
| 1.0 | 100 | – | – | – | – | 0.06 | 23.9 |
| 1.0 | 50 | 0.53 | 0.35 | 0.35 | 0.57 | 0.06 | 22.6 |
| 1.0 | 30 | 0.53 | 0.34 | 1.12 | 0.61 | 0.07 | 0.13 |
| **1.0** | **10 (new profile)** | **0.56** | **0.34** | **0.35** | **0.77** | **0.07** | **0.06** |

- **Fragile outcomes:** each sequence either tracks or fails by metres, and the outcome is not monotonic in the threshold. On 01b, a single scan with 30–40 keypoints decides between 0.35 m and 1.12 m. On kidnap_b, about 18 scans with 10–50 keypoints decide between 0.2 m and 20+ m.
- **Choice:** `voxel_size` 1.0 with the default threshold is the only combination tried that tracks all six sequences. It also lowers RKO-LIO CPU time on 02a from 393 s to 112 s (user time).
- **Cost:** the 01a slide described above returns partly (about 4 m upward at 126 s; z 0.31 → 0.62 m, 3D 0.72 → 0.77 m). 01b goes from 0.31 m to 0.35 m.
- No sequence is held out after this step: all six were used to choose it.

## Reference: GLIM odometry on the same sequences

- **Source:** GLIM v1.2.2 (GICP, container `glim-ros2:jazzy-v1.2.2`) runs from 2026-07-16, full sequences, 4 runs each.
- **Evaluation:** the GLIM pose output was evaluated with the same alignment and association as above.
- These runs were not repeated for this note, and the runtime configuration differs from ours. Read the comparison as indicative, not as a benchmark claim.

| Sequence | GLIM 3D RMSE (4 runs) | GLIM xy / z (run 1) | Profile, xy / z / 3D |
| --- | --- | --- | --- |
| 02a | 0.72–0.80 m | 0.69 / 0.21 m | 0.52 / 0.23 / 0.56 m |
| 02b | 0.37–0.40 m | 0.34 / 0.17 m | 0.31 / 0.15 / 0.34 m |
| 01b | 0.30–0.37 m | – | 0.32 / 0.16 / 0.35 m |
| 01a | not run | – | 0.46 / 0.62 / 0.77 m |

- GLIM still has slightly lower vertical error on 02a; ours is lower on 02b.
