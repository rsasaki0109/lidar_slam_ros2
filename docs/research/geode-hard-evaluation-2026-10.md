# GEODE Hard sequences: RKO-LIO v8 and the profile rivals (2026-10)

## Summary

GEODE ([thisparticle.github.io/geode](https://thisparticle.github.io/geode)) marks 18 of
its 64 sequences "Hard": flat surfaces, shield tunnels, urban tunnels and bridges. This
note runs the ENWIDE v8 candidate and the five profile rivals on the 12 of them that could
be downloaded. Google Drive's download limit has held back the other six so far:
Shield_tunnel3–5 on device γ, and Shield_tunnel7, 8 and 10 on device β.

- On these sequences every method fails almost everywhere. Only Shield_tunnel6 is tracked
  below 20 % 10 m RTE, by v8 (0.56 m), BIEVR-LIO (0.47 m) and v7 (4.12 m).
- Among the failures, v8 has the lowest median ATE (253 m). RKO-LIO, as v8 or v7, is
  best on 7 of the 12 sequences.
- v8's bump image registration is what makes it the best on the vehicle-mounted urban
  tunnels and bridges. The same registration hurts the handheld shield tunnels, where v7
  (point-to-point) is better.

No SOTA claim follows: the profile needs zero catastrophic failures, and no method has
that on GEODE Hard.

## Inputs

GEODE has three devices with the same Xsens MTi-30 IMU (`/imu/data`, 100 Hz):

| device | LiDAR | topic | sequences here |
|---|---|---|---|
| α | Velodyne VLP-16 | `/velodyne_points` | Urban_Tunnel01–03, bridge01–03 (vehicle) |
| β | Ouster OS1-64 | `/ouster/points` | Shield_tunnel9 (handheld) |
| γ | Livox AVIA | `/livox/lidar` (CustomMsg) | Shield_tunnel1, 2, 6, flat_surfaces_* (handheld) |

Every method gets the same data:

- **Inputs.** Every method uses the Xsens IMU and the dataset's `T_IMU_LiDAR`
  (`{alpha,beta,gamma}_config.yaml`).
- **Conversion.** `scripts/convert_geode_bag.py` writes the LiDAR and the IMU to a ROS 2
  bag for RKO-LIO and BIEVR-LIO.
  - Livox CustomMsg becomes PointCloud2 with a uint32 `t` (ns from the scan start).
  - The VLP-16 `time` field is in seconds relative to a stamp at the scan end, so most
    offsets are negative. FAST-LIO2 and Point-LIO expect offsets from the scan start and
    would leave the earlier part of each scan undeskewed, which costs metres at 40 km/h.
    The converter moves the stamp to the first point and makes `time` non-negative. Every
    point's absolute time is unchanged.
  - `--ros1-out` writes the same normalized data as ROS 1 for the ROS 1 rivals. The γ and
    β rivals play the original bags.
- **β metadata.** `metadata_beta.json` lists `pixel_shift_by_row` for only 62 of 64 rows.
  All 64 are `round(beam_azimuth_deg * 1024 / 360)`, the rule the listed rows follow and
  ENWIDE's `os_enwide.json` follows for all 128 rows.
- **Ground truth.** The trajectories are not always in time order.
  `scripts/prepare_geode_ground_truth.py` sorts them.
  - The shield tunnels have laser-tracker positions at about 3 Hz.
  - The urban tunnels and bridges have GNSS/INS, with gaps of up to 70 s inside the
    tunnels; those gaps are not scored.
  - The flat surfaces have Vicon.
  - The lever arm between the IMU and the ground-truth point is not published, so the IMU
    trajectory is scored directly.
- **Scoring.** `score_position_only_trajectory.py` aligns with SE(3) and reports ATE and
  10 m RTE, as on ENWIDE.

Configurations (`configs/geode/`):

- `rko_lio_{alpha,beta,gamma}_v8.yaml` is ENWIDE v8 with GEODE's sensor settings only. It
  has photometric terms on β only, because they need an Ouster intensity image.
- v7 is the same configuration without the three v8 options.
- `rivals/` holds each rival's own configuration for the closest sensor. Only the topics,
  the extrinsic, the 0.65 m blind range and, for the VLP-16, `timestamp_unit` change.
- COIN-LIO needs an Ouster intensity image, so it runs on β only.

Development split, declared after the first three pilot runs:

- Development: flat_surfaces_smooth, Urban_Tunnel01 and Shield_tunnel9.
- Validation: the other sequences, which no configuration was tuned on.

## Results (12 sequences)

ATE in metres. Development sequences are marked *. A run that diverged until it stopped
producing poses is marked *incomplete*.

| sequence (device) | v8 | v7 | COIN-LIO | BIEVR-LIO | FAST-LIO2 | Point-LIO |
|---|---:|---:|---:|---:|---:|---:|
| Shield_tunnel1 (γ) | 303 | **78** | – | 995,841 | 295 | 94 |
| Shield_tunnel2 (γ) | 119 | **76** | – | 259,303 | 554 | 83 |
| Shield_tunnel6 (γ) | 0.56 | 4.12 | – | **0.47** | 39.9 | 32.2 |
| Shield_tunnel9 (β) * | 441 | **59** | 852 | 83.7 | 254 | 83.1 |
| Urban_Tunnel01 (α) * | 560 | 888 | – | **169** | 1010 | 983 |
| Urban_Tunnel02 (α) | **260** | 816 | – | 680 | 1158 | 1173 |
| Urban_Tunnel03 (α) | **160** | 1202 | – | 69,283 | 1721 | 149,732 |
| bridge01 (α) | **46** | 167 | – | 109 | 933 | 984 |
| bridge02 (α) | 246 | 885 | – | **190** | 1830 | 23,649 |
| bridge03 (α) | 1095 | **697** | – | 879 | 1091 | 1100 |
| flat_surfaces_aggressive (γ) | incomplete | incomplete | – | 9304 | 1047 | **4.38** |
| flat_surfaces_smooth (γ) * | 3.66 | 2.47 | – | **1.34** | 2229 | 2.77 |
| median | **253** | 432 | – | 435 | 1028 | 539 |
| runs under 20 % RTE | 1 | 1 | 0 | 1 | 0 | 0 |

## Why v8 fails in the shield tunnels

The ablations below are on the development sequences only, with ATE in metres:

| change to v8 | Shield_tunnel9 * | Urban_Tunnel01 * | flat_surfaces_smooth * |
|---|---:|---:|---:|
| v8 | 441 | **560** | 3.66 |
| without bump image registration | 68.5 | 895 | |
| v7 (without all three v8 options) | 59.1 | 888 | 2.47 |
| degeneracy-aware solve, ratio 0.01 | 546 | 1356 | |
| bump plus a gyro rotation prior, weight 30 / 300 | 127 / 207 | – / 662 | |
| bump plus point-to-point, bump weight 1 | **58.3** | 873 | **2.12** |
| bump plus point-to-point, bump weight 5 / 20 | 69.9 / 75.1 | 779 / 725 | |

- **Where the error comes from.** A per-scan log of v8 on Shield_tunnel9 shows the
  registration turning the pose by several degrees per scan after about 20 s, up to 11.7°
  in one scan. The Xsens gyro predicts that rotation to well under a degree.
- **Hypothesis.** A circular shield tunnel leaves the roll about its axis and the
  along-axis translation unobservable, and the bump images' periodic segment rings let
  the registration slip along them. The ring explanation has not been tested.
- **The fixes tried.** None of them keeps v8's gain on the vehicle sequences and removes
  the shield-tunnel failure: the generic degeneracy-aware solve, a gyro rotation prior,
  and mixing in point-to-point.

## v9: fall back to point-to-point when the bump terms turn the pose too far

A correctly registered scan stays close to the gyro prediction. The rotation correction
of v8's registration (degrees, per scan) separates the cases cleanly:

| development sequence | v8 result | median | 99th percentile | scans over 2° |
|---|---|---:|---:|---:|
| GEODE Urban_Tunnel01 | best of all methods | 0.15 | 1.07 | 0 % |
| ENWIDE FieldD | good | 0.40 | 1.60 | 0.3 % |
| ENWIDE RunwayD | good | 0.42 | 1.53 | 0.3 % |
| GEODE flat_surfaces_smooth | fails | 1.70 | 50.9 | 46 % |
| GEODE Shield_tunnel9 | fails | 5.16 | 31.3 | 68 % |

- **What v9 does.** v9 is v8 plus `bump_image_max_rotation_correction_deg: 2.0`
  (rko_lio #28). A scan whose bump result turns more than 2° away from the IMU guess is
  registered again without the bump terms.
- **How it was frozen.** The threshold was set on the six development sequences and
  frozen before any validation sequence was run.
- **Determinism fix.** rko_lio #27 makes the point-to-point system deterministic first.
  Without it, the chaotic fallback scans varied by about ±0.2 m between runs of the same
  build.
- **Configurations.** `configs/geode/rko_lio_{alpha,beta,gamma}_v9.yaml`.

ATE in metres, v8 → v9. Development sequences are marked *.

| GEODE sequence | v8 | v9 |
|---|---:|---:|
| Shield_tunnel1 (γ) | 303 | **97.6** |
| Shield_tunnel2 (γ) | 119 | **104.7** |
| Shield_tunnel6 (γ) | **0.56** | 25.3 |
| Shield_tunnel9 (β) * | 441 | **67.2** |
| Urban_Tunnel01–03, bridge01–02 (α) | same | same |
| bridge03 (α) | 1095 | **1035** |
| flat_surfaces_aggressive (γ) | incomplete | **2.62** |
| flat_surfaces_smooth (γ) * | 3.66 | **2.01** |
| median of 12 | 253 | **101** |

| ENWIDE sequence | v8 | v9 |
|---|---:|---:|
| IntersectionD | **0.38** | 0.52 |
| TunnelD * | **0.32** | 0.41 |
| FieldD * | **0.24** | 0.28 |
| RunwayD * | 2.34 | **1.70** |
| the other six | same | same |
| median of 10 | **0.29** | 0.35 |

On GEODE, v9 halves the median and is now the best method on flat_surfaces_aggressive,
where v8 lost track. But it loses Shield_tunnel6, the one sequence v8 tracked, and on
ENWIDE it costs up to 0.14 m. It still has no ENWIDE failure and stays below COIN-LIO on
every sequence.

The profile allows at most 2 % regression on sequences that are not degenerate. v9
regresses more than that on IntersectionD, so v8 stays the candidate. v9 is the
configuration for strongly degenerate scenes such as GEODE Hard.

The fallback fires per scan, so a single noisy scan switches registration even where
the bump terms were right. Requiring the slip to persist over several scans is the
obvious next step. The validation sequences above have now been seen, so that change
needs fresh validation, such as the six GEODE sequences not downloaded yet.

## Reproduction

```bash
python3 scripts/prepare_geode_ground_truth.py GEODE/groundtruth/traj GEODE/gt_tum
python3 scripts/convert_geode_bag.py SEQ/SEQ.bag SEQ/ros2 [--ros1-out SEQ/SEQ_lidar_imu.bag]
ros2 run rko_lio offline_node --ros-args --params-file configs/geode/rko_lio_DEVICE_v8.yaml \
  -p bag_path:=SEQ/ros2 -p imu_topic:=/imu/data -p lidar_topic:=LIDAR_TOPIC ...
python3 scripts/score_position_only_trajectory.py --reference GEODE/gt_tum/SEQ.txt \
  --estimate TRAJECTORY.tum --output score.json --max-time-gap 0.11
```

The ROS 1 rivals run in `docker/enwide_rivals_ros1.Dockerfile`. Each rival's `ouster64`,
`velodyne`/`velody16` or `avia` configuration is replaced by the matching file in
`configs/geode/rivals/`.

## Next

1. The six remaining Hard sequences, once Google Drive allows the downloads again.
2. A persistence condition for the v9 fallback, validated on sequences not yet seen.
