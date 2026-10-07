# ENWIDE TunnelD: photometric registration on LiDAR intensity (2026-10)

## Decision

Add opt-in photometric registration to RKO-LIO (rko_lio PR #21, after
COIN-LIO) and freeze `configs/enwide/rko_lio_os0_photometric_v7.yaml` for the
held-out TunnelS evaluation. On TunnelD it takes RKO-LIO from 27.8 m to
1.70 m ATE. That number comes from a weight chosen on TunnelD, so it is a
development result, not a benchmark claim under
`degenerate_lio_sota_v1` (`tuning_allowed: false`).

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

## Next

1. Evaluate the frozen v7 on TunnelS. It is held out: its bag was not
   downloaded during development.
2. Run v7 through `run_enwide_sota_benchmark.sh` (graph backend, three
   repetitions) for the profile record.
3. Look for the remaining gap to COIN-LIO (1.70 m against 0.52 m). Candidates
   are IMU coupling of the photometric terms (COIN-LIO updates inside the
   IEKF), the patch reference refresh, and deskew of the reference patches.
