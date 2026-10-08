# ENWIDE TunnelD: photometric registration on LiDAR intensity (2026-10)

## Decision

Use opt-in photometric registration in RKO-LIO (rko_lio PRs #21 and #22,
after COIN-LIO) with the frozen `configs/enwide/rko_lio_os0_photometric_v7.yaml`.
On held-out TunnelS, never opened during development, RKO-LIO goes from 37.5 m
to 0.75 m ATE (10 m RTE 2.0 %). COIN-LIO reproduced on the same bag reaches
0.78 m (2.2 %); its paper reports 0.743 m (1.60 %). The TunnelD numbers below
are development results, because the weight and the scan-gap guard were found
there.

| TunnelS (held out, 251.6 m) | ATE | max | 10 m RTE | path (GT 252 m) |
|---|---:|---:|---:|---:|
| RKO-LIO, photometric off | 37.54 m | 88.1 m | 127.9 % | 407 m |
| **RKO-LIO, v7 (#21 and #22 identical: no LiDAR gap)** | **0.75 m** | 1.3 m | **2.0 %** | 248 m |
| COIN-LIO, reproduced | 0.78 m | 1.3 m | 2.2 % | 247 m |
| COIN-LIO, paper | 0.743 m | | 1.60 % | |

Sensitivity (reported, not used to choose): photometric_scale 0.002 gives
0.74 m (2.0 %) and 0.01 gives 1.44 m (4.1 %). The terms used about 53 patches
per scan on 2377 of 2379 scans.

This is still not a `degenerate_lio_sota_v1` claim. The profile's other
conditions are not met: the graph backend, three repetitions, the other
ENWIDE environments and the hidden tunnel.

Update 2026-10-08: v8 (below) adds three open-ground options. Over all ten ENWIDE
sequences it has no failure and a median ATE of 0.29 m, against 0.69 m for
COIN-LIO and 0.32 m for BIEVR-LIO, which diverges on three.

## Why the earlier configurations did nothing

All runs below use `offline_node` on the converted TunnelD bag. They are scored
against the Leica prism track with the IMU-to-prism lever arm, SE(3)
alignment without scale, and the 10 m RTE of the profile. The scorer
reproduces the July numbers.

| configuration | ATE | 10 m RTE | path (GT 180 m) |
|---|---:|---:|---:|
| plain RKO-LIO | 22.57 m | 80.3 % | 293 m |
| preregistered v1 (degeneracy solve, persistence gate, multiscan observability, intensity profile) | 22.57 m | 80.3 % | 293 m |
| kinematic velocity blend (speed caps 4-5 m/s, gates relaxed) | 22.57 m | 80.3 % | 293 m |
| voxel 0.1 m | 24.13 m | 75.0 % | 242 m |
| localizability weighting | 21.63 m | 72.4 % | 262 m |

v1 is byte-identical to plain because none of its features ever engages. The
degeneracy candidate appears once and is never confirmed, and the intensity
prior is never attempted. Per correspondence, the translation information of
point-to-point ICP is the identity, so the Hessian gates see no weak axis even
while the estimate slides. The velocity blend never anchors: handheld running
makes the IMU-propagated velocity disagree with ICP by up to 17 m/s. From
about 18 s the ICP reports half the true speed and then oscillates near the
tunnel entrance.

## COIN-LIO isolates the cause

COIN-LIO (ethz-asl/COIN-LIO, Docker, `mapping_enwide.launch`, real-time
playback) reproduces the paper on the same bag. Turning off only its
photometric terms leaves FAST-LIO2's point-to-plane IEKF, which fails like
RKO-LIO:

| system | ATE | 10 m RTE |
|---|---:|---:|
| COIN-LIO | 0.52 m | 1.7 % (paper 0.487 m, 1.59 %) |
| COIN-LIO, `photo_scale: 0` | 25.30 m | 102.8 % |

The photometric patches on the intensity image are what carry this sequence.

## The extrinsic of v1-v6 is wrong

The ROS clouds are published in `os_sensor`. In `os_enwide.json`,
`imu_to_sensor_transform` is the identity rotation with
[6.253, -11.775, 7.645] mm. The cloud-to-IMU rotation is therefore the
identity, as in COIN-LIO's `extrinsic_R`. Columns confirm it: column 0 sits at
about 170° azimuth, the os_sensor convention. v1-v6 derived
`T_imu_lidar = Rz(pi)` for `os_lidar`, which is not the frame the points are
in. Correcting it alone does not rescue the sequence (27.76 m ATE, 96.5 % RTE,
159 m path). v1-v6 stay unchanged as preregistration records. v7 uses
`[0, 0, 0, 1, -0.006253, 0.011775, -0.007645]`.

## Photometric registration in RKO-LIO

rko_lio PR #21 renders each organized scan as COIN-LIO's filtered intensity
image (line filter, brightness normalization, masks). It maps deskewed points
back to the pixels they were captured at. Patches are selected where the
image gradient sees motion along translation directions that few surface
normals face. Their Gauss-Newton terms are added to every ICP iteration with
weight `photometric_scale^2 / N_icp`.

| v7 with `photometric_scale` | ATE | max | 10 m RTE | path |
|---|---:|---:|---:|---:|
| off (corrected extrinsic) | 27.76 m | 45.8 m | 96.5 % | 159 m |
| 0.00095 (COIN-LIO's value) | 18.99 m | 32.1 m | 35.7 % | 169 m |
| 0.002 | 1.94 m | 4.7 m | 7.3 % | 181 m |
| **0.003 (v7)** | **1.70 m** | 4.6 m | **7.0 %** | 180 m |
| 0.005 | 1.78 m | 4.9 m | 7.5 % | 179 m |
| 0.01 | 2.12 m | 5.1 m | 7.2 % | 177 m |

The result is flat over a factor of five. RKO-LIO needs about three times
COIN-LIO's weight: point-to-point correspondences also hold the pose along the
weak axis, which point-to-plane residuals do not. Projecting the ICP
information off the weak axis gave the same 1.70 m. That pull is not what
separates RKO-LIO from COIN-LIO's 0.52 m.

Other measurements:
- Repeated runs are byte-identical.
- About 49 patches are used per scan, in 1184 of 1188 scans.
- The 119 s bag takes about 105 s with the terms and about 70 s without,
  startup excluded.
- Peak RSS is 422 MB against 390 MB.

## Where it does not help

NTNU Fyllingsdalen (OS0-128 at 512 columns, `fog_metadata.json` geometry,
line filter off) finds about 21 patches per scan. Reach is 93.7 m with the
terms and 106.8 m without, of about 505 m. The tunnel is lit by a continuous
LED strip and is bare concrete. As the July profile probes found, it carries
almost no along-axis intensity texture. Radar remains the answer there
(`tunnel_radar.ros.yaml`).

## Scan gaps (rko_lio PR #22, merged after TunnelS)

The 5.1 m loop-closure error of the 1.70 m run (GT 1.1 m, COIN-LIO 1.3 m)
comes from one scan. Relative to COIN-LIO's yaw, the estimate jumps 13° at
22.05 s, right after a 0.28 s gap in the bag's LiDAR stream, and keeps the
error. Without photometric terms the gap is harmless. rko_lio #22 skips the
photometric terms on a scan that follows a gap longer than 0.15 s:

| photometric_scale | #21 ATE | #22 ATE | final yaw vs COIN-LIO, #21 → #22 |
|---:|---:|---:|---:|
| 0.002 | 1.94 m | 2.38 m | −14.2° → +2.8° |
| 0.003 | 1.70 m | 0.34 m | about −14° → 0.0° |
| 0.01 | 2.12 m | 2.24 m | −13.3° → −23.0° |

The guard removes the jump at every weight, but the ATE gain appears only at
0.003. Both were found on TunnelD.

## Held-out TunnelS evaluation (declared before opening TunnelS; done)

- A: rko_lio #21 with `configs/enwide/rko_lio_os0_photometric_v7.yaml`.
- B: rko_lio #22 with the same file.
- Primary metric: ATE (SE(3), prism lever arm); also 10 m RTE and path length.
- Sensitivity, reported but not used to choose: photometric_scale 0.002 and 0.01 for A and B.
- Merge #22 only if B is not worse than A on TunnelS.

Result: A and B are identical (0.75 m), since TunnelS has no LiDAR gap and the
guard never fires, so #22 was merged.

## Profile record (`degenerate_lio_sota_v2`)

`run_enwide_sota_benchmark.sh --profile degenerate_lio_sota_v2` runs RKO-LIO with
the graph backend, three repetitions, input hashes checked. v2 differs from v1 only
in the frozen candidate (v7 at rko_lio e9441b3).

| sequence | runs complete | ATE median | 10 m RTE median | RTF | peak RSS |
|---|---:|---:|---:|---:|---:|
| TunnelS (held out) | 3/3 | 0.751 m | 1.98 % | 1.4-2.1 | 418 MB |
| TunnelD (development) | 3/3 | 0.336 m | 1.69 % | 2.3-3.0 | 421 MB |

All three repetitions are identical on both sequences.

The first attempts failed every repetition with exit 125, for an unrelated
harness reason. In no-map mode the harness rejects any map artifact, but
`lidarslam.yaml` lets the graph write the map bundle and `pose_graph.g2o` on an
accepted loop closure. The July runs never closed a loop. TunnelS now returns to
its start and does, so the harness sets the launch's existing
`M6A10_BENCHMARK_NO_MAP_ARTIFACTS` marker in no-map mode.

Two operational notes:
- Running other heavy jobs at the same time slowed registration to about
  320 ms per scan. A repetition then ended at the harness's quiescence timeout,
  before the offline node had written its dump.
- The node survived the process-group kill and kept publishing in the
  benchmark's ROS domain, which contaminated the following runs.

Run the profile on an otherwise quiet machine.

## Rivals on the same bags

BIEVR-LIO (ethz-asl/BIEVR-LIO 2306022, ROS 2 `process_bag`, its `enwide`
sensor config) is the profile's newest rival. Its paper (arXiv 2604.14421)
reports ENWIDE Intersection, Runway, Field and Katzensee, but not the tunnels.
On the tunnels it diverges here. It tracks for about 30 s, then slides along
the axis:

| | TunnelS ATE | TunnelS RTE | TunnelD ATE | TunnelD RTE |
|---|---:|---:|---:|---:|
| RKO-LIO v7 (profile record) | **0.751 m** | **1.98 %** | **0.336 m** | 1.69 % |
| COIN-LIO, reproduced | 0.78 m | 2.2 % | 0.52 m | **1.7 %** |
| BIEVR-LIO, reproduced | 483.34 m | 1194 % | 58.30 m | 214 % |

The BIEVR-LIO sensor config uses the same cloud-to-IMU extrinsic as v7: no
rotation, translation [-0.00625, 0.011775, -0.007645].

## All ten ENWIDE sequences

The other eight sequences use one offline run per system with the same scorer
and the same bag, downloaded once per sequence. The tunnel rows for RKO-LIO v7
are the profile record above; the rest of the tunnel rows are the reproductions
above. COIN-LIO and BIEVR-LIO reproduce their published KatzenseeD ATE
(0.592 m and 0.243 m). ATE in metres, 10 m RTE in percent:

| sequence | RKO-LIO v7 | RKO-LIO, photometric off | COIN-LIO | BIEVR-LIO |
|---|---:|---:|---:|---:|
| TunnelS (held out) | **0.75** / **2.0** | 37.5 / 128 | 0.78 / 2.2 | 483 / 1194 |
| TunnelD | **0.34** / **1.7** | 27.8 / 96.5 | 0.52 / **1.7** | 58.3 / 214 |
| KatzenseeD | 0.35 / 2.0 | 0.70 / 3.1 | 0.59 / 2.2 | **0.24** / **1.8** |
| KatzenseeS | **0.19** / **1.1** | 0.21 / 1.3 | 0.49 / 1.9 | **0.19** / **1.1** |
| FieldD | 7.22 / 30.2 | 28.6 / 77.9 | 0.83 / 4.4 | **0.17** / **1.3** |
| FieldS | 0.69 / 2.6 | 6.57 / 37.4 | 0.20 / 1.2 | **0.16** / **0.8** |
| IntersectionD | 4.52 / 9.4 | 49.0 / 95.8 | 1.86 / 4.1 | **0.39** / **1.4** |
| IntersectionS | 0.55 / 1.3 | 1.39 / 3.6 | 0.46 / 1.5 | **0.23** / **1.0** |
| RunwayD | 50.1 / 50.0 | 34.7 / 108 | **2.90** / **6.5** | 609 / 1013 |
| RunwayS | 28.9 / 41.1 | 40.1 / 89.0 | 2.80 / 4.6 | **0.73** / **3.1** |
| median ATE | 0.72 | 28.2 | 0.69 | 0.32 |
| runs over 20 % RTE | 3 | 7 | **0** | 3 |

- The photometric terms improve every sequence except RunwayD, mostly by an
  order of magnitude.
- RKO-LIO v7 is best on both tunnels, where BIEVR-LIO diverges.
- BIEVR-LIO is best on the open sequences, but it diverges on three.
- COIN-LIO is the only system that never fails.
- RKO-LIO v7 fails on FieldD and both runways: open, flat ground with little
  structure. Against BIEVR-LIO, its FieldD yaw drifts about 11° between 40 s and
  100 s.

No SOTA claim follows. The profile's win policy needs zero catastrophic failures.

## Open ground: v8 (2026-10-08)

`configs/enwide/rko_lio_os0_open_ground_v8.yaml` is v7 plus three opt-in rko_lio
options (rko_lio 8c77478):

- `bump_image_registration` (#23): BIEVR-LIO's voxel-wise height images replace
  the point-to-point residual. Relief of a few centimetres (grass, asphalt) then
  fixes the in-plane pose. The photometric terms stay on top.
- `velocity_window_sec: 0.3` (#26): the velocity is the pose difference over
  0.3 s instead of one scan. On RunwayD the sensor is spun at about 190 °/s. One
  0.33 m correction then turned the velocity from 2 into 5 m/s, and the weak
  in-plane constraint let it run away to 13 m/s.
- `skip_registration_after_gap_sec: 0.15` (#25): the scan after a LiDAR gap
  takes the IMU prediction. On TunnelD the bump registration otherwise turned
  the post-gap scan by 6.3° (it is constrained almost nothing in translation).

How the three were found:

- On the development sequences FieldD, RunwayD and TunnelD only.
- The configuration was frozen before any of the other seven sequences was run
  (one offline run each, same scorer, same bags as the rivals).

Two hypotheses failed first and are not in v8:

- **IMU biases.** RKO-LIO takes them from the first scan interval, and on these
  handheld starts that absorbs motion: up to 0.05 rad/s in the gyro. BIEVR-LIO's
  online estimate converges to about (-0.023, -0.017, 0.005) rad/s on every
  sequence. Tracking the gyro bias online, or fixing both biases at BIEVR-LIO's
  values, made FieldD worse (7.2 m to 14.5–30 m). The drift there comes from the
  registration, not from the biases.
- **Photometric weight on open ground.** Only 12–15 patches per scan survive on
  grass. A higher weight helps FieldD (5.7 m at 0.01) but breaks at 0.03.

ATE in metres / 10 m RTE in percent. Development sequences are marked *:

| sequence | RKO-LIO v8 | RKO-LIO v7 | COIN-LIO | BIEVR-LIO |
|---|---:|---:|---:|---:|
| TunnelS (held out) | **0.71** / **2.0** | 0.75 / **2.0** | 0.78 / 2.2 | 483 / 1194 |
| TunnelD * | **0.32** / **1.6** | 0.34 / 1.7 | 0.52 / 1.7 | 58.3 / 214 |
| KatzenseeD | 0.25 / **1.6** | 0.35 / 2.0 | 0.59 / 2.2 | **0.24** / 1.8 |
| KatzenseeS | **0.17** / **0.9** | 0.19 / 1.1 | 0.49 / 1.9 | 0.19 / 1.1 |
| FieldD * | 0.24 / 1.5 | 7.22 / 30.2 | 0.83 / 4.4 | **0.17** / **1.3** |
| FieldS | 0.18 / 1.0 | 0.69 / 2.6 | 0.20 / 1.2 | **0.16** / **0.8** |
| IntersectionD | **0.38** / 1.5 | 4.52 / 9.4 | 1.86 / 4.1 | 0.39 / **1.4** |
| IntersectionS | **0.19** / **0.9** | 0.55 / 1.3 | 0.46 / 1.5 | 0.23 / 1.0 |
| RunwayD * | **2.34** / **3.1** | 50.1 / 50.0 | 2.90 / 6.5 | 609 / 1013 |
| RunwayS | **0.45** / **1.7** | 28.9 / 41.1 | 2.80 / 4.6 | 0.73 / 3.1 |
| median ATE | **0.29** | 0.72 | 0.69 | 0.32 |
| runs over 20 % RTE | **0** | 3 | **0** | 3 |

On the seven sequences not used for development:

- v8 has the lowest ATE on five and is within 0.02 m of BIEVR-LIO on the
  other two (KatzenseeD, FieldS).
- No run of v8 exceeds 3.1 % RTE.
- COIN-LIO is the only rival without a failure, and v8 is below it on all ten.

Mean registration time per scan is 46–83 ms on the runs that had the machine to
themselves. The two Katzensee runs overlapped other jobs and took 115 and 299 ms.
Peak RSS is 530–630 MB. The profile run must measure the real-time factor on a
quiet machine.

This is still not a `degenerate_lio_sota_v1` claim. The remaining conditions are
the graph backend with three repetitions per sequence, FAST-LIO2 and Point-LIO,
GEODE and the hidden tunnel.

## Profile record (`degenerate_lio_sota_v3`, 2026-10-08)

`run_enwide_sota_benchmark.sh --profile degenerate_lio_sota_v3` ran v8 at rko_lio
8c77478 on all ten sequences:

- Graph backend, three repetitions each, input hashes checked.
- One sequence at a time, on a machine with nothing else heavy running.

| sequence | runs complete | ATE median | 10 m RTE median | RTF | peak RSS |
|---|---:|---:|---:|---:|---:|
| TunnelS (held out) | 3/3 | 0.708 m | 2.02 % | 1.29–1.32 | 484–627 MB |
| TunnelD * | 3/3 | 0.324 m | 1.62 % | 1.49–1.51 | 480–626 MB |
| KatzenseeD | 3/3 | 0.246 m | 1.57 % | 1.62–1.64 | 590–595 MB |
| KatzenseeS | 3/3 | 0.172 m | 0.95 % | 1.31–1.33 | 522–530 MB |
| FieldD * | 3/3 | 0.238 m | 1.55 % | 1.52–1.53 | 622–645 MB |
| FieldS | 3/3 | 0.177 m | 0.95 % | 1.35–1.37 | 613–628 MB |
| IntersectionD | 3/3 | 0.382 m | 1.53 % | 1.57–1.58 | 588–594 MB |
| IntersectionS | 3/3 | 0.194 m | 0.94 % | 1.27–1.28 | 586–593 MB |
| RunwayD * | 3/3 | 2.341 m | 3.08 % | 1.53–1.55 | 609–618 MB |
| RunwayS | 3/3 | 0.450 m | 1.73 % | 1.31–1.33 | 612–618 MB |

- All 30 runs completed with at least 99.6 % of the ground truth matched.
- The three repetitions are identical on every sequence and equal the offline
  runs above.
- The real-time factor is 1.27–1.64, so every sequence runs faster than real
  time.
- Median over the ten sequences: ATE 0.285 m, 10 m RTE 1.56 %.

The harness first scored the IMU trajectory without the 11 cm IMU-to-prism lever
arm (fixed in #490). The numbers above re-score the same saved trajectories with
it. That changed the ATE by at most 0.004 m. The runs are deterministic, so they
were not repeated. The v2 record above is still the uncorrected score.

This meets the profile's execution contract on ENWIDE. A SOTA claim still needs
FAST-LIO2 and Point-LIO on the same bags, GEODE and the hidden tunnel.

## All profile rivals (2026-10-08)

FAST-LIO2 (7cc4175) and Point-LIO (4b86a46), the profile's two remaining rivals,
ran on the same ROS 1 bags:

- Build: `docker/enwide_rivals_ros1.Dockerfile`.
- Run: `scripts/run_enwide_rival_ros1.sh`.
- Configuration: `configs/enwide/rivals/`. Each is the method's own `ouster64.yaml`
  with only the topics, 128 lines, the 0.65 m blind range of COIN-LIO's ENWIDE
  config and the os_sensor extrinsic changed.
- The `ring` field of these clouds is uint16 and the methods expect uint8. Only
  their feature-extraction path reads it, and that path is off for Ouster.

Every method below is scored the same way:

- `score_position_only_trajectory.py` on the IMU trajectory moved to the prism.
- v8 is the profile record (median of three identical runs).
- The rivals are one real-time playback each.

ATE in metres / 10 m RTE in percent:

| sequence | RKO-LIO v8 | COIN-LIO | BIEVR-LIO | FAST-LIO2 | Point-LIO |
|---|---:|---:|---:|---:|---:|
| TunnelS (held out) | **0.71** / **2.0** | 0.78 / 2.2 | 483 / 1194 | 38.5 / 177 | 37.6 / 93.5 |
| TunnelD * | **0.32** / **1.6** | 0.52 / 1.7 | 58.4 / 236 | 27.5 / 107 | 30.8 / 140 |
| KatzenseeD | 0.25 / **1.6** | 0.59 / 2.2 | **0.24** / 1.8 | 1.29 / 3.4 | 0.39 / 1.8 |
| KatzenseeS | **0.17** / **0.95** | 0.49 / 1.9 | 0.19 / 1.1 | 1.13 / 3.5 | 0.32 / 1.6 |
| FieldD * | 0.24 / 1.6 | 0.83 / 4.4 | **0.17** / **1.3** | 17.7 / 21.9 | 33.2 / 111 |
| FieldS | 0.18 / 0.95 | 0.20 / 1.2 | **0.16** / **0.8** | 0.19 / 1.4 | 0.32 / 3.1 |
| IntersectionD | **0.38** / 1.5 | 1.86 / 4.1 | 0.39 / **1.4** | 14.3 / 14.2 | 48.5 / 88.1 |
| IntersectionS | **0.19** / **0.94** | 0.46 / 1.5 | 0.23 / 1.0 | 13.1 / 22.1 | 4.15 / 14.5 |
| RunwayD * | **2.34** / **3.1** | 2.90 / 6.5 | 609 / 1012 | 68.0 / 121 | 52.2 / 107 |
| RunwayS | **0.45** / **1.7** | 2.80 / 4.6 | 0.73 / 3.1 | 44.6 / 103 | 892 / 1018 |
| median ATE | **0.29** | 0.69 | 0.32 | 16.0 | 32.0 |
| runs over 20 % RTE | **0** | **0** | 3 | 6 | 6 |

- Neither FAST-LIO2 nor Point-LIO holds the tunnels or the runways. Both fail on
  six sequences.
- v8 has no failure and the lowest median ATE.
- On the seven validation sequences v8 has the lowest ATE on five and is within
  0.02 m of BIEVR-LIO on the other two.

All four profile rivals are now reproduced on identical inputs. A SOTA claim
still needs GEODE's degenerate sequences and the hidden tunnel.

## Next

1. GEODE's degenerate sequences and the hidden tunnel, the profile's remaining
   claim conditions.
