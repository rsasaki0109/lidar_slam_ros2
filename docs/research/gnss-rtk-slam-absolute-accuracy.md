# RTK GNSS anchoring on RTK-SLAM Stadtgarten 2 (2026-11)

The GNSS constraint had been validated mechanically
([gnss-constraint-first-validation.md](gnss-constraint-first-validation.md)) but its absolute accuracy was
never measured. This note measures it against the 19 total-station checkpoints of RTK-SLAM
`stadtgarten_seq2` (CC-BY 4.0) and records what it took to get from metres to the published baselines.
Everything here was tuned on this one sequence; `stadtgarten_seq1` (30 GB) did not fit the machine and is the
open check.

## Metric

The RTK-SLAM "offline" protocol: the trajectory in ENU around the GNSS origin, converted to latitude and
longitude (WGS84), then to UTM 32N (GRS80), heights minus the 48.22 m geoid offset, the nearest pose within
2 s of each checkpoint, **no alignment**. Checkpoints sit at the rig's `base_center` (IMU
[-0.073, -0.023, -0.172]), so the LiDAR-frame trajectory is moved by [-0.062, 0.0, -0.216]. The same metric
gives FAST-LIO-SAM's published seq2 trajectory 0.099 m, as the dataset reports. SE3 is the same pairs after
a rigid fit.

| Published (seq2, offline) | absolute 3D | SE3 |
| --- | ---: | ---: |
| RTK GNSS alone (all fix types) | 11.989 m | 10.524 m |
| FAST-LIO-SAM | 0.099 m | 0.077 m |
| OKVIS-LVIG | 0.092 m | 0.080 m |

## Results

| Run (seq2) | absolute 3D | horizontal | vertical | SE3 |
| --- | ---: | ---: | ---: | ---: |
| RKO-LIO, deskew off, init off, no GNSS | — | — | — | 1.594 m |
| + GNSS (fed with `ros2 bag play`), default backend | 7.644 m | 2.903 m | 7.072 m | 5.968 m |
| deskew on (Livox `offset_time`), init off, GNSS | 6.132 m | 2.876 m | 5.416 m | 4.761 m |
| deskew on, init on, no GNSS | — | — | — | 0.296 m |
| deskew on, init on, GNSS, default backend | 1.356 m | 0.315 m | 1.319 m | 0.903 m |
| + interpolation, antenna offset, gravity prior | 0.379 m | 0.331 m | 0.183 m | 0.339 m |
| **+ odometry/anchor weights = `lidarslam_rtk_gnss.yaml`** | **0.100 m** | **0.052 m** | **0.086 m** | **0.074 m** |

The last row is the product path: `run_rko_lio_graph_benchmark.sh --bag-gnss` with
`lidarslam/param/lidarslam_rtk_gnss.yaml` and the Stadtgarten 2 RKO-LIO preset. Per checkpoint the
horizontal error is 0.01–0.09 m everywhere, including the 180 s at the end without an RTK fix; the
vertical error is within 0.10 m except the last two checkpoints (−0.14, −0.26 m), 160 s after the last fix.

## What mattered, in order

1. **RKO-LIO ignored the Livox `offset_time` field**, so the RTK-SLAM presets ran without deskew. The rig is
   lifted onto every checkpoint (up to 3–4 rad/s) and without deskew the odometry broke at 750–800 s
   (metres in 30 s). Reading `offset_time` (uint32 ns from the header) and turning deskew on:
   SE3 1.594 → 0.322 m.
2. **The rig starts static but pitched ~20°** and the preset skipped the static gravity initialisation, so the
   odometry frame was tilted: height fell 54 m over the run and the planar GNSS alignment could not fix it.
   `initialization_phase: true`: height drift −2.8 m over 876 s, SE3 0.296 m.
3. **The anchors tilted the graph.** Adjacent edges carry relative rotation weakly (information 1000 on the
   quaternion error is ~3.6° per edge), so the optimiser pitched submaps by up to 12° to fit the anchors and
   carried the tilt past the last fix (−26 m height at the end). A unary roll/pitch prior per submap from the
   gravity-aligned odometry (`gravity_prior_info`, `GravityPriorEdge`) took the vertical RMSE 1.14 → 0.18 m.
4. **Anchor geometry:** the antenna is 5.7 cm from the LiDAR origin (`gnss_antenna_offset_xyz`) and a 10 Hz fix
   can be 5 cm from the submap stamp at walking speed (`gnss_interpolation_max_gap_sec`). Together 1.36 →
   1.17 m before the gravity prior; both stay on.
5. **Odometry versus anchors.** Splitting the adjacent-edge information into translation 1e4 (~1 cm) and
   rotation 1e7 (~0.04°), the vertical GNSS weight 0.3 instead of the hard-coded 0.1, and a 2-sigma Huber
   kernel on the anchors: 0.379 → 0.100 m.

A replay of the pose-graph stage from a finished run's submap poses and the bag's fixes predicted the
product runs to within 1 mm of RMSE (0.378 vs 0.379 m, 0.099 vs 0.100 m), which is how the weights were
searched. Variants that did not help: 30 instead of 10 optimiser iterations (no change), including float and
DGPS fixes (horizontal better, vertical worse), a stronger gravity prior (3000: 0.142 m, the vertical end error grows).

## Map thickness

The same runs through `scripts/run_map_quality_check.sh` (frozen profile, `--downsample 0.1`):

| Map (seq2, whole run) | thickness mean | thickness p95 | planar coverage | entropy (nats) |
| --- | ---: | ---: | ---: | ---: |
| deskew off, init off | 0.0719 m | 0.118 m | 0.497 | −1.146 |
| deskew on, init on, GNSS v1 backend | 0.0561 m | 0.112 m | 0.543 | −1.448 |
| + interpolation, antenna, gravity prior | 0.0525 m | 0.107 m | 0.579 | −1.524 |
| **+ `lidarslam_rtk_gnss.yaml`** | **0.0440 m** | **0.104 m** | **0.616** | **−1.792** |

Deskew removes most of the blur; the stiffer odometry edges remove the bending the anchors used to put
between submaps that see the same surface.

## Coloured map

The README's Stadtgarten 2 recipe (`tools/colored_map/README.md`: 20–180 s, 640 views, pose refinement,
corridor carving, the K4 colouring settings with sky rejection; no dynamic-object cleaner on either side) run
on the old trajectory (deskew off, init off) and on the `lidarslam_rtk_gnss.yaml` one:

| Metric | old trajectory | new trajectory |
| --- | ---: | ---: |
| held-out epipolar error before pose refinement | 4.20 px | **1.09 px** |
| after pose refinement (views reset to the recorded pose) | 0.66 px (61) | **0.33 px (4)** |
| held-out colour error, RGB L2 mean / median / p90 | 47.6 / 32.9 / 108.6 | **45.1 / 30.7 / 103.1** |
| held-out points within 20 | 0.339 | **0.363** |
| coverage | 0.928 | **0.946** |
| chroma retention | 1.039 | **1.068** |
| colour roughness, planar voxels median / p90 | 5.42 / 12.59 | **4.65 / 11.84** |
| colour roughness, all voxels median / p90 | 4.73 / **12.89** | **4.56** / 13.65 |

The camera poses the trajectory hands to the colouring are about four times more consistent, so the pose
refinement starts closer and keeps almost every view; the colours follow, modestly. The calibration file's
camera time shift (−0.0206 s) instead of the recipe's 0 made it worse (2.29 px before refinement, held-out
mean 45.9), so the recipe keeps 0.

The README flythrough (`lidarslam/images/map_flythrough_stadtgarten.{webp,mp4}`) is now rendered from the new
trajectory with the recipe's dynamic-object cleaner put back. It removes 6.8 % of the points and moves every
appearance metric slightly the right way: coverage 0.951, planar roughness median 4.51. Held-out colour does
not change (RGB L2 mean 45.0). The old asset's map was tilted about 20° by the pitched start; the new one is
level.

## The construction presets (release gates)

Deskew and the static initialisation on the shared construction preset
(`configs/mid360_robot/rko_lio_rtk_slam_mid360.yaml`), same commit, the gate command of
`scripts/run_rtk_slam_accuracy_suite.py` with the map saved, map quality with the blocking
`indoor_construction` profile:

| Sequence | preset | SE3 RMSE (median, max) | thickness mean / p95 | planar coverage | entropy |
| --- | --- | ---: | ---: | ---: | ---: |
| construction_seq2 | deskew off, init off | 0.165 m (0.091, 0.526) | 0.082 / 0.123 m | 0.46 | −0.94 |
| | deskew only | 0.062 m (0.058, 0.122) | — | — | — |
| | **deskew + init** | **0.062 m (0.059, 0.110)** | **0.049 / 0.095 m** | **0.77** | **−1.59** |
| construction_seq1 | deskew off, init off | 0.280 m (0.145, 0.666) | 0.080 / 0.123 m | 0.40 | −0.90 |
| | **deskew + init** | **0.205 m (0.095, 0.640)** | **0.051 / 0.095 m** | **0.73** | **−1.59** |

Both sequences start static with the rig pitched ~19°. The published construction_seq2 baselines
are FAST-LIO-SAM 0.086 m and OKVIS 0.075 m. With the map saved the trajectory RMSE is unchanged to the last digit
(0.06219702536 m both ways), so the gate numbers come from the same deterministic run.

## Other Livox bags

The `offset_time` field is specific to how the RTK-SLAM bags were converted. The two public MID-360 bags
the repository downloads already carry a per-point time RKO-LIO read before: `t` (uint32 ns,
Hard Point Cloud `outdoor_kidnap_a`) and `timestamp` (float64 absolute ns, Driving SLAM). Their presets
(`configs/mid360_robot/rko_lio_mid360_*no_deskew.yaml`) turn deskew off by choice, and on the vehicle bag
that choice holds: `rko_lio_mid360_low_voxel_no_deskew.yaml` with deskew on gives a slightly worse map
(thickness 0.050 → 0.054 m, planar coverage 0.39 → 0.35, entropy −1.43 → −1.31 nats; GT-free,
`outdoor_vegetation` profile). Those presets stay as they are.

## Product changes

- RKO-LIO (fork): `offset_time` is a per-point time field; the offline node passes a bag NavSatFix topic
  through (`gnss_topic`), and waits for its subscriber like for odometry.
- Launch `bag_gnss:=true` / runner `--bag-gnss`; before this the offline path never fed GNSS to the graph.
- graph_based_slam: `gnss_interpolation_max_gap_sec`, `gnss_antenna_offset_xyz`, `gnss_robust_kernel_delta`,
  `gravity_prior_info`, `gnss_vertical_weight_scale` (all default to the old behaviour); the GNSS stamp check
  compares against the latest odometry stamp (the node clock before it), so offline bags keep their fix
  stamps without `gnss_header_stamp_max_skew_sec: 1e9`; the GNSS subscription keeps 1000 messages.
- Presets: `lidarslam/param/lidarslam_rtk_gnss.yaml`; the Stadtgarten 2 RKO-LIO preset turns deskew and
  the static initialisation on.

## Tried and not adopted (2026-11-05)

- **Loop closure.** With `lidarslam_rtk_gnss.yaml` no loop is accepted: the default
  `distance_loop_closure: 100` m excludes most revisits on this 413 m walk (the 695–767 s pass is within
  9–15 m of the RTK-fixed 623–662 s stretch after only ~25 m of travel), and the rest fail the 0.7 m²
  fitness gate. With 30 m (3 loops accepted), and additionally fitness 2.0 / range 25 m / overlap 0.5
  (6 loops), the result is the same 0.100 m to the millimetre at every checkpoint: the loops fall where the
  anchors already hold the graph, and the remaining error (785 s −0.14 m, 855 s −0.26 m in height) lies after
  the last revisit, where nothing can bound the extrapolation.
- **Plane BA refinement** (`graph_slam_offline_runner -p refine:=true` on a recorded seq2 backend input,
  scans thinned to 0.1 m, no GNSS): 33/33 windows accepted, checkpoint SE3 0.296 → 0.278 m, but the map does
  not get thinner (thickness 0.0449 → 0.0450 m, planar coverage 0.614 → 0.610, entropy −1.778 → −1.763).
  Deskew left little blur for it; it is not wired into the GNSS path.

## Open

- `stadtgarten_seq1` (and any other RTK site) to check the weights were not fitted to seq2's 19 checkpoints.
