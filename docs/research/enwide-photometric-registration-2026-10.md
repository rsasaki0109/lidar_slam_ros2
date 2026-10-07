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

## Next

1. Run v7 through `run_enwide_sota_benchmark.sh` (graph backend, three
   repetitions) on both tunnels for the profile record.
2. Evaluate the other ENWIDE environments (field, intersection, runway, ...),
   where geometry is degenerate in other ways.
3. Look for the remaining gap to COIN-LIO (1.70 m against 0.52 m). Candidates
   are IMU coupling of the photometric terms (COIN-LIO updates inside the
   IEKF), the patch reference refresh, and deskew of the reference patches.
