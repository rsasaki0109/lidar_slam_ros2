# GEODE Hard sequences: RKO-LIO v8 and the profile rivals (2026-10)

## Summary

GEODE ([thisparticle.github.io/geode](https://thisparticle.github.io/geode)) marks 18 of
its 64 sequences "Hard": flat surfaces, shield tunnels, urban tunnels and bridges. This
note runs RKO-LIO (v8, the ENWIDE candidate; v9, v10 and v7) and the five profile rivals
on all 18.

- On these sequences every method fails almost everywhere. Few runs stay under 20 % 10 m
  RTE: Shield_tunnel6 (v8, v10, v7, BIEVR-LIO), Shield_tunnel5 and Shield_tunnel10
  (BIEVR-LIO), and bridge01 (v8, v9, v10).
- Among the failures, v10 has the lowest median ATE (81 m), ahead of v7 (86 m), v9
  (89 m), Point-LIO (94 m), v8 (160 m) and BIEVR-LIO (180 m).
- v8's bump image registration makes it the best on the vehicle-mounted urban tunnels and
  bridges. The same registration hurts the handheld shield tunnels.
- v9 falls back to point-to-point when the bump terms turn the pose too far. On the four
  sequences that arrived after v9 was frozen, it beats v8 on two and loses on two.
- v10 falls back only when large corrections persist (3 of the last 10 scans). It keeps
  v8's results where v8 is right (Shield_tunnel6, the vehicle sequences, ENWIDE) and is
  the best RKO-LIO configuration on the two sequences that arrived after it was frozen,
  Shield_tunnel8 and 10.

No SOTA claim follows: the profile needs zero catastrophic failures, and no method has
that on GEODE Hard.

## Inputs

GEODE has three devices with the same Xsens MTi-30 IMU (`/imu/data`, 100 Hz):

| device | LiDAR | topic | sequences here |
|---|---|---|---|
| α | Velodyne VLP-16 | `/velodyne_points` | Urban_Tunnel01–03, bridge01–03 (vehicle) |
| β | Ouster OS1-64 | `/ouster/points` | Shield_tunnel7–10 (handheld) |
| γ | Livox AVIA | `/livox/lidar` (CustomMsg) | Shield_tunnel1–6, flat_surfaces_* (handheld) |

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

## Results (18 sequences)

ATE in metres.

- *: development sequences for v8, v9 and v10.
- †: arrived after v9 was frozen. ‡: arrived after v10 was frozen, so unseen by every
  configuration.
- *incomplete*: a run that diverged until it stopped producing poses (counted as the
  worst value in the medians).

| sequence (device) | v8 | v9 | v10 | v7 | COIN-LIO | BIEVR-LIO | FAST-LIO2 | Point-LIO |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Shield_tunnel1 (γ) | 303 | 97.6 | 82.9 | **78.3** | – | 995,841 | 295 | 93.7 |
| Shield_tunnel2 (γ) | 119 | 105 | 108 | **75.6** | – | 259,303 | 554 | 82.7 |
| Shield_tunnel3 (γ) † | 59.7 | 63.9 | 72.4 | **53.3** | – | 150,049 | 16,629 | 73.8 |
| Shield_tunnel4 (γ) † | 159 | **66.5** | 70.1 | 69.6 | – | 117,016 | 77,795 | 69.1 |
| Shield_tunnel5 (γ) † | 40.4 | 27.2 | 40.4 | 21.7 | – | **1.54** | 46,928 | 37.0 |
| Shield_tunnel6 (γ) | 0.56 | 25.3 | 0.56 | 4.12 | – | **0.47** | 39.9 | 32.2 |
| Shield_tunnel7 (β) † | **58.6** | 82.8 | 90.2 | 75.5 | 314 | 132 | 328 | 96.5 |
| Shield_tunnel8 (β) ‡ | 130 | 107 | 78.9 | 93.0 | 289 | **73.6** | 289 | 102 |
| Shield_tunnel9 (β) * | 441 | 67.2 | **56.6** | 59.1 | 852 | 83.7 | 254 | 83.1 |
| Shield_tunnel10 (β) ‡ | 227 | 95.7 | 86.6 | 93.1 | 349 | **7.10** | 358 | 94.6 |
| Urban_Tunnel01 (α) * | 560 | 560 | 560 | 888 | – | **169** | 1010 | 983 |
| Urban_Tunnel02 (α) | **260** | **260** | **260** | 816 | – | 680 | 1158 | 1173 |
| Urban_Tunnel03 (α) | 160.2 | 160.1 | **158.6** | 1202 | – | 69,283 | 1721 | 149,732 |
| bridge01 (α) | **46.2** | **46.2** | **46.2** | 167 | – | 109 | 933 | 984 |
| bridge02 (α) | 246 | 246 | 246 | 885 | – | **190** | 1830 | 23,649 |
| bridge03 (α) | 1095 | 1035 | 1088 | **697** | – | 879 | 1091 | 1100 |
| flat_surfaces_aggressive (γ) | incomplete | **2.62** | 3.02 | incomplete | – | 9304 | 1047 | 4.38 |
| flat_surfaces_smooth (γ) * | 3.66 | 2.01 | 2.00 | 2.47 | – | **1.34** | 2229 | 2.77 |
| median | 160 | 89.2 | **80.9** | 85.7 | 332 (β only) | 180 | 1029 | 94.2 |
| runs under 20 % RTE | 2 | 1 | 2 | 1 | 0 | 3 | 0 | 0 |

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

The GEODE numbers are in the results table above. The four sequences that arrived after
v9 was frozen are the only validation that no configuration has seen:

| unseen GEODE sequence | v8 | v9 |
|---|---:|---:|
| Shield_tunnel3 (γ) | **59.7** | 63.9 |
| Shield_tunnel4 (γ) | 159 | **66.5** |
| Shield_tunnel5 (γ) | 40.4 | **27.2** |
| Shield_tunnel7 (β) | **58.6** | 82.8 |

| ENWIDE sequence | v8 | v9 |
|---|---:|---:|
| IntersectionD | **0.38** | 0.52 |
| TunnelD * | **0.32** | 0.41 |
| FieldD * | **0.24** | 0.28 |
| RunwayD * | 2.34 | **1.70** |
| the other six | same | same |
| median of 10 | **0.29** | 0.35 |

On GEODE, v9 halves v8's median (75 m against 160 m over 16 sequences). It is the best
method on flat_surfaces_aggressive, where v8 lost track, but it loses Shield_tunnel6. On
the unseen sequences it is level with v8, two each. On ENWIDE it costs up to 0.14 m. It still has no ENWIDE failure and stays below COIN-LIO on
every sequence.

The profile allows at most 2 % regression on sequences that are not degenerate. v9
regresses more than that on IntersectionD, so v8 stays the candidate. v9 is the
configuration for strongly degenerate scenes such as GEODE Hard.

The fallback fires per scan, so a single noisy scan switches registration even where
the bump terms were right. v10 below adds a persistence condition.

## v10: fall back only when the slip persists

rko_lio #29 logs every scan's bump rotation correction (`bump_rotation_corrections.csv`)
and adds a persistence gate. The v8 logs on the six development sequences show that the
cases differ in persistence, not size:

| development sequence | v8 result | scans over 2° | longest run over 2° | largest correction |
|---|---|---:|---:|---:|
| GEODE Urban_Tunnel01 | best of all methods | 0 % | 0 | 1.8° |
| ENWIDE FieldD / RunwayD | good | 0.3 % | 1 | 3.1° |
| ENWIDE TunnelD | good | 1.6 % | 2 | 14.4° |
| GEODE flat_surfaces_smooth | fails | 46 % | 30 | 102° |
| GEODE Shield_tunnel9 | fails | 68 % | 34 | 48° |

- **What v10 does.** v10 is v9 plus `bump_image_rotation_fallback_window: 10` and
  `bump_image_rotation_fallback_min_count: 3`. A scan falls back only when at least 3 of
  the last 10 scans exceeded 2°.
- **How it was frozen.** Chosen on the development sequences and frozen before
  Shield_tunnel8 and 10 were read. A 20° hard limit (fall back at once) changed nothing
  there and was left out.
- **Check.** With the default 1 / 1 the new build reproduces v9 and v8 bit for bit.
- **Configurations.** `configs/geode/rko_lio_{alpha,beta,gamma}_v10.yaml`.

| development sequence | v8 | v9 | v10 |
|---|---:|---:|---:|
| ENWIDE FieldD | **0.24** | 0.28 | **0.24** |
| ENWIDE RunwayD | 2.34 | **1.70** | 2.34 |
| ENWIDE TunnelD | **0.32** | 0.41 | 0.34 |
| GEODE flat_surfaces_smooth | 3.66 | 2.01 | **2.00** |
| GEODE Urban_Tunnel01 | 560 | 560 | 560 |
| GEODE Shield_tunnel9 | 441 | 67.2 | **56.6** |

On Shield_tunnel8 and 10, unseen by every configuration, v10 is the best RKO-LIO
configuration (78.9 and 86.6 m against v8's 130 and 227 m, v9's 107 and 96 m, v7's 93 and
93 m). BIEVR-LIO is better on both (73.6 and 7.10 m) and is the only method that tracks
Shield_tunnel10.

On the sequences already seen during v9:

- **GEODE.** v10 keeps v8's Shield_tunnel6 (0.56 m, which v9 lost) and the vehicle
  results. Against v8 it wins 9 of 18 sequences and loses 2 (Shield_tunnel3 and 7).
  Against v9 it wins 6 and loses 7; v9 is better on the γ shield tunnels 1–5.
- **ENWIDE.** v10 equals v8 on FieldS, IntersectionD/S, KatzenseeD/S and RunwayS. It
  differs only on the development sequence TunnelD (0.34 against 0.32 m, a tunnel).
  TunnelS was not rerun because its bag had been deleted to save disk.

v10 is not yet the ENWIDE candidate. Replacing v8 would need the official ten-sequence
record with TunnelS.

## Why RKO-LIO slips where BIEVR-LIO does not (2026-10-11)

A windowed error profile shows how the shield-tunnel runs fail. Each 30 s window is aligned to the ground truth on its own. For each window it reports the local RMSE and the difference between the estimated and the true path length.

- **Shield_tunnel10.** BIEVR-LIO stays at about 0.2 m per window with the correct path length. v8 and v10 overestimate the path by 20–100 m per window: the trajectory runs ahead along the tunnel axis.
- **γ shield tunnels 1–5.** v7, v9 and v10 slip back and forth along the axis by 10–170 m per window. Which of them ends with the lowest ATE is chance among failing runs, not a systematic difference. BIEVR-LIO diverges from the start on Shield_tunnel1 and 3 but tracks 5 and 6.
- **Velocity window.** Lengthening `velocity_window_sec` from 0.3 to 2 s only changes the sign of the slip on Shield_tunnel10 (111 m against 86.6 m; Shield_tunnel9 63.5 against 56.6 m). The registration itself slips, presumably to a neighbouring ring of the periodic lining, rather than the velocity running away.

RKO-LIO uses the IMU for the prediction and for a gravity (roll/pitch) regularisation only; nothing in it constrains translation, velocity or accelerometer bias with the IMU. BIEVR-LIO optimises velocity, biases and gravity over a 10 s IMU window together with its bump-image registration.

An experiment added this to RKO-LIO (rko_lio branch `feat/inertial-velocity-filter`, not merged):

- **Velocity filter.** A Kalman filter on the world velocity and the accelerometer bias. It propagates with IMU preintegration and corrects with the registered displacement, weighted by the registration's Hessian.
- **Translation prior.** Optionally, the predicted position with its covariance is added to the registration.

It did not help on the development sequences:

| variant | FieldD | RunwayD | TunnelD | flat_surfaces_smooth | Shield_tunnel9 | Shield_tunnel10 |
|---|---:|---:|---:|---:|---:|---:|
| v10 | 0.24 | 2.34 | 0.34 | 2.00 | 56.6 | 86.6 |
| filter | 0.20 | 2.36 | 0.48 | 2.91 | diverges | 181 |
| filter + translation prior | – | – | – | – | diverges | – |

In a circular tunnel the roll about the axis is as unobservable as the translation along it. Once the registration turns the attitude, gravity leaks into the horizontal acceleration. The filter then estimates 4–7 m/s² of bias and integrates the leak into the velocity. Gating outlier updates and bounding the bias made it worse: Shield_tunnel9 diverged further and TunnelD reached 39 m, because a gated filter integrates the wrong acceleration alone.

Using the IMU to hold the translation therefore needs the attitude and gravity estimated jointly over a window, as BIEVR-LIO does. That is a tightly coupled estimator rather than an addition to RKO-LIO's per-scan registration.

### Attitude in the filter (2026-10-11)

A second experiment (rko_lio branch `feat/inertial-pose-filter`, not merged) adds the attitude. It is an error-state Kalman filter on attitude, position, velocity and accelerometer bias. The filter propagates with every IMU sample and is corrected by each registered pose, with a 6×6 covariance from the registration's Hessian. The fused pose replaces the registered one, and the filter's prediction is the registration's initial guess.

| variant | FieldD | TunnelD | flat_surfaces_smooth | Shield_tunnel9 |
|---|---:|---:|---:|---:|
| v10 | 0.24 | 0.34 | 2.00 | 56.6 |
| filter follows the registration (prediction as initial guess only) | 0.17 | 2.18 | 2.42 | 50.4 |
| registration rotation floor 0.1°, gyro 0.01 rad/s/√Hz | 0.24 | 0.49 | 2.55 | 73.4 |
| registration rotation floor 0.5°, gyro 0.005 rad/s/√Hz | 261 | 64.9 | 2.28 | 57.6 |

No setting improves on v10. Shield_tunnel9 differences of this size are within the run-to-run spread noted above. Four findings came out of the experiment:

- **The registration's roll in the shield tunnels is noise.** Per scan, v10's trajectory on Shield_tunnel9 turns about the tunnel axis by 9.7° RMS, and this does not follow the gyro: the residual against the integrated gyro is 9.6° in roll, against 1.7° and 0.8° in pitch and yaw. The bump and photometric terms set the roll, yet the Hessian reports it to about 0.1°.
- **Holding roll with the gyro does not stop the slip.** The 0.5° floor lets the gyro carry the attitude, and Shield_tunnel9 does not improve. The same setting diverges on FieldD and TunnelD, so the registration's rotation cannot be distrusted globally.
- **The covariance must be about the sensor.** The registration's left perturbation rotates about the world origin. A floor on that rotation becomes a position uncertainty of |t| times the angle (0.9 m for 0.5° at 100 m), and the filter then dead-reckons and diverges. The Hessian has to be moved to sensor-centred coordinates before the floors and caps.
- **The filter's velocity must not drive the deskew.** Feeding it to the deskew and the next prediction turned FieldD from 0.17 into 2.5 m, even with the filter following the registration exactly.

A per-block innovation gate, which leaves the position or attitude out of a scan's update when it disagrees, diverged on every sequence once the gated filter drifted. Inflating the block instead lets the filter creep toward whichever wrong value lands closer.

### BIEVR-LIO's inertial window (2026-10-12)

BIEVR-LIO's source shows that it is not tightly coupled in the pose:

- **Registration.** It registers each scan against its map without the IMU.
- **Fixed poses.** It keeps the registered pose as it is.
- **Window solve.** It then holds the last 10 s of poses fixed and solves only the velocity at every scan, the two IMU biases and the gravity direction from the IMU preintegration between them.
- **What the IMU feeds.** The IMU feeds only the next scan's prediction and deskew.

The third experiment adds that window to RKO-LIO. It was merged default-off as `inertial_window` (rko_lio #30), and `configs/enwide/rko_lio_os0_open_ground_v10_inertial_window.yaml` turns it on for ENWIDE-like data. It is a Gauss-Newton solve over the velocities and both biases with gravity fixed. The newest velocity replaces RKO-LIO's pose-difference velocity, and the biases replace `imu_bias`.

On ENWIDE it helps on several sequences (ATE in m, v10 from the profile record):

| | TunnelS | TunnelD | KatzenseeD | KatzenseeS | FieldD | FieldS | IntersectionD | IntersectionS | RunwayD | RunwayS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| v10 | 0.71 | 0.34 | 0.25 | 0.17 | 0.24 | 0.18 | 0.38 | 0.19 | 2.34 | 0.45 |
| window | 0.69 | 0.44 | 0.25 | 0.16 | 0.18 | 0.18 | 0.34 | 0.19 | 0.86 | 0.55 |

It is better on five sequences and worse on two, and the median does not move (0.295 m against 0.293 m).

On GEODE it is worse in the shield tunnels. Ten sequences ran: it wins on Shield_tunnel3, 4 and 9 and Urban_Tunnel01, and loses on the other six, by up to 108 → 895 m on Shield_tunnel2. Three findings came out of it:

- **The window fits the slip.** With an Xsens IMU, whose bias is a few hundredths of m/s², the window's accelerometer bias runs to the 0.5–1 m/s² limit in the shield tunnels. It explains the registered positions sliding along the axis as acceleration, and the bias then carries the slip into the prediction.
- **Anchoring the biases does not help.** A strong prior holding the biases at the static initialization keeps them at a few mm/s². Shield_tunnel2 and 8 still diverge (43 and 49 km), because the window's velocity between slipping positions feeds the slip back the same way.
- **The rotation residual matters.** Dropping it and fixing the gyroscope bias made TunnelD 20 m and Shield_tunnel9 diverge.

BIEVR-LIO succeeds on Shield_tunnel10 because its registration does not slip much, not because of its IMU window. Three ways of adding the IMU to RKO-LIO now agree on this: the velocity filter, the error-state filter and the inertial window. The shield-tunnel failure is in the registration, which locks to a neighbouring ring of the periodic lining, and the IMU cannot undo that after the fact.


### What the slip looks like, and the roll jitter (2026-10-12)

**The slip is a speed oscillation, not ring jumps.** Over 1 s windows, v10's estimated speed swings between a quarter and three to four times the true walking speed on Shield_tunnel1, 8 and 10. The median ratio is 1.0. On Shield_tunnel6, which succeeds, it stays within 0.86–1.15. The top 5 % of seconds carry about half of the excess distance. No single scan moves by a lining ring (>1 m), so the registration slides rather than jumps.

**RKO-LIO's attitude jitters; BIEVR-LIO's does not.** Per scan on Shield_tunnel10, the rotation between registered poses differs from the integrated gyro by this much:

| | roll | pitch | yaw |
|---|---:|---:|---:|
| BIEVR-LIO | 0.15° | 0.08° | 0.10° |
| v10 | 3.6° | 0.79° | 0.45° |

**The jitter comes from the gravity regularisation (`min_beta`).** First 80 s of Shield_tunnel10, gyro residual RMS (roll / pitch / yaw):

| v10 with… | roll | pitch | yaw |
|---|---:|---:|---:|
| nothing changed | 1.87° | 0.41° | 0.28° |
| photometric off | 1.83° | 0.39° | 0.29° |
| bump off | 1.91° | 0.42° | 0.29° |
| bump fallback off | 2.44° | 0.52° | 0.34° |
| gravity regularisation off | 0.29° | 0.10° | 0.13° |

In a circular tunnel nothing geometric fixes the roll, so the tilt read from the accelerometer, polluted by the walking motion, sets it. With the regularisation off, the attitude is as clean as BIEVR-LIO's and the node runs about twice as fast.

**A clean attitude alone does not stop the slip.** ATE in m:

| | Shield_tunnel8 | Shield_tunnel9 | Shield_tunnel10 |
|---|---:|---:|---:|
| v10 | 78.9 | 56.6 | 86.6 |
| gravity regularisation off | – | 68.2 | 111 |
| … plus inertial window | 126 | 216 | 229 |
| … bump only (no photometric, no fallback) | 75.5 | 37.4 | 108 |
| … bump only plus inertial window | 129 | 43.2 | 140 |
| BIEVR-LIO | 73.6 | 83.7 | 7.1 |

The last RKO-LIO rows use the same parts as BIEVR-LIO: bump-image registration with informed sampling, the same map parameters, no gravity term in the registration, and an inertial window. They still slip on Shield_tunnel10, although bump only is the best RKO-LIO result on Shield_tunnel9 (37.4 m, BIEVR-LIO 83.7 m). Turning the regularisation off on ENWIDE costs FieldD (0.24 → 0.43 m) and TunnelD's 10 m RTE (1.7 → 2.2 %).

The remaining difference is inside the registration step: the optimiser, the deskew or the sampling, not the choice of terms. The next step is to give both registrations the same scan, map and initial guess and compare one step.

### The registration step: Gauss-Newton against Levenberg-Marquardt (v11, 2026-10-12)

**A fixed-point test.** The test takes BIEVR-LIO's Shield_tunnel10 trajectory as the true pose:

- **Setup.** RKO-LIO builds its map from the true poses, registers every scan starting from the true pose, and logs how far the registration moves away.
- **Bump only.** The registration moves 0.8 mm per scan forward on average, with 38 mm RMS along the axis.
- **With the photometric terms.** It moves 5–8 mm per scan forward (3–5 % of the distance walked), with 93–101 mm RMS: the lining's periodic intensity pattern pulls the tracked patches.
- **Restoring force.** Started 0.2 m ahead or behind, bump-only registration comes back by 80–90 %, so it is not missing a basin.
- **Closed loop.** RKO-LIO's along-axis jitter is three times BIEVR-LIO's (second difference 148–172 mm against 56 mm).

**The optimisers differ.** RKO-LIO takes plain Gauss-Newton steps, `H dx = -b`, always applied. BIEVR-LIO takes Levenberg-Marquardt steps: damped, and accepted only when the cost drops. Along an axis the scan barely constrains, Gauss-Newton turns residual noise into large moves. rko_lio #31 adds Levenberg-Marquardt as `icp_levenberg_marquardt`, default off, damped in body coordinates. With it the fixed-point jitter falls from 15 to 11 mm over the first two minutes.

On Shield_tunnel10, each variant with Levenberg-Marquardt (ATE in m):

| variant | ATE |
|---|---:|
| v10 | 131 |
| without photometric | 64.1 |
| without gravity regularisation | 201 |
| without photometric and gravity regularisation | **0.76** |

That last row is **v11**, `configs/geode/rko_lio_{alpha,beta,gamma}_v11.yaml`: v10 plus `icp_levenberg_marquardt`, `min_beta: 1e9` and, for the Ouster β, no photometric terms. On Shield_tunnel10 every 30 s window has 0.1–0.2 m local RMSE and the right path length; BIEVR-LIO loses 7 m in one window.

The other development sequences confirmed it: flat_surfaces_smooth 2.20 m (v10 2.00), Urban_Tunnel01 334 m (560), Shield_tunnel9 24.1 m (56.6). v11 was then frozen. The rule set before running the other fifteen was to adopt v11 if the 18-sequence median improves on v10 and v11 wins more sequences than it loses.

| sequence | v10 | v11 | BIEVR-LIO |
|---|---:|---:|---:|
| flat_surfaces_smooth * | 2.00 | 2.20 | 1.34 |
| flat_surfaces_aggressive | 3.02 | 4.25 | 9304 |
| Urban_Tunnel01 * | 560 | 334 | 169 |
| Urban_Tunnel02 | 260 | 89.8 | 680 |
| Urban_Tunnel03 | 159 | 200 | 69,283 |
| bridge01 | 46.2 | 116 | 109 |
| bridge02 | 246 | 192 | 190 |
| bridge03 | 1088 | 681 | 879 |
| Shield_tunnel1 | 82.9 | 69.7 | 995,841 |
| Shield_tunnel2 | 108 | 102 | 259,303 |
| Shield_tunnel3 | 72.4 | 76.4 | 150,049 |
| Shield_tunnel4 | 70.1 | 546 | 117,016 |
| Shield_tunnel5 | 40.4 | 1.98 | 1.54 |
| Shield_tunnel6 | 0.56 | 0.58 | 0.47 |
| Shield_tunnel7 | 90.2 | 130 | 132 |
| Shield_tunnel8 | 78.9 | 109 | 73.6 |
| Shield_tunnel9 * | 56.6 | 24.1 | 83.7 |
| Shield_tunnel10 † | 86.6 | **0.76** | 7.10 |
| median | **80.9** | 95.9 | 180 |
| below 10 m | 3 | **5** | 4 |

\* development, † diagnostic (seen during development).

**Verdict.** v11 wins nine sequences and loses nine, and its median is worse, so it is not adopted; v10 stays the recommended configuration. v11 tracks more sequences than any method so far (five below 10 m, BIEVR-LIO four), and it fails harder elsewhere.

**Where v11 still fails.** On Shield_tunnel7 and 8 it no longer slides ahead; it stalls, with the speed near zero for a minute. BIEVR-LIO stalls in the same windows.

**ENWIDE.** Levenberg-Marquardt does not transfer:
- **v11's settings:** FieldD 0.93 m, RunwayD 17.3 m and TunnelD 19.1 m. ENWIDE's tunnels need the photometric terms and its open ground needs the gravity term.
- **Levenberg-Marquardt alone with v10:** FieldD 0.36 m and TunnelD 1.48 m.

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

1. The official ENWIDE record for v10 (all ten sequences, including TunnelS), to decide
   whether one configuration can replace both v8 and v9.
2. The γ shield tunnels, where v9 and v7 still beat v10, and Shield_tunnel10, where only
   BIEVR-LIO tracks.
3. IMU fusion is exhausted: the velocity filter, the error-state filter and BIEVR-LIO's
   inertial window all fail in the shield tunnels. What remains is the registration's lock
   onto the periodic lining itself.
