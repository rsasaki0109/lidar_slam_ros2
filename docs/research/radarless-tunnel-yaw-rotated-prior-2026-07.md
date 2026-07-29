# Radar-less tunnel yaw-rotated velocity prior (2026-07-29)

## Decision

Adopt the yaw-rotated inertial velocity prior in the default-off
`lidarslam/param/presets/tunnel_imu_no_radar.ros.yaml`.

The candidate reaches 506.38 m on the approximately 500 m NTNU
Fyllingsdalen sequence, versus 415.59 m for the previous scene-gated preset
and 102.26 m for default-off RKO-LIO. Its final and maximum time-aligned reach
ratio against the radar-derived pseudo-GT is 1.004, below the 1.1 runaway
limit. Aligned translation RMSE improves from 48.74 m to 16.34 m.

Fog, HILTI exp07, and MID-360 driving candidate trajectories remain
byte-identical to their paired default-off controls.

## Failure mechanism

The scene-gated velocity blend still stalls between approximately 190 s and
250 s. Raw IMU inspection and the new diagnostics identify the sequence:

1. sustained yaw crosses the straight-corridor safety gate at 190.9 s;
2. the gate clears the propagation anchor;
3. ICP enters the tunnel zero-motion attractor before yaw settles;
4. `kinematic_blend_min_speed` then prevents a new anchor.

Discarding the full velocity prior on yaw loses useful speed information even
though IMU-supported orientation remains observable.

## Adopted behavior

During sustained yaw, translation correction remains suspended. Instead of
clearing the trusted velocity prior, the implementation rotates that prior by
the registered orientation delta. When yaw settles, the prior therefore has
the new world direction and retains its pre-turn speed.

A low LiDAR speed may keep using the prior only while all of these hold:

- a trusted anchor and independently propagated velocity already exist;
- within-scan raw acceleration-magnitude variance is at least 0.03
  `(m/s^2)^2`;
- scene and platform-speed gates still accept the scan.

The threshold separates the validation sequence cleanly: walking intervals,
including the 180-250 s ICP lock, remain above 0.03; the stationary interval
after 310 s remains below 0.01. The library default is zero, so activity-based
bridging is disabled unless a preset opts in.

Scene rejection, excessive platform speed, low speed without inertial
activity, and anchor expiry still clear the prior.

## A/B and holdouts

| sequence | control / previous | adopted candidate | result |
| --- | ---: | ---: | --- |
| NTNU tunnel endpoint | 415.59 m previous preset | 506.38 m | +90.79 m |
| NTNU tunnel aligned RMSE | 48.74 m | 16.34 m | -66.5% |
| NTNU tunnel max reach ratio | 0.983 | 1.004 | no runaway |
| NTNU fog | paired control | byte-identical | no corrections |
| HILTI exp07 | paired control | byte-identical | no corrections |
| MID-360 driving | paired control | byte-identical | no corrections |

Artifacts:

- `/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1/runs/radarless_tunnel_yaw_rotated_prior_v1`

## Rejected variants

- Keeping the first agreement-anchor axis through low speed was stopped at
  100.5 s: reach was 42.87 m versus 146.84 m reference, with 6.4 m vertical
  error. The stale axis fixed the wrong direction.
- Re-anchoring from the last low LiDAR velocity after yaw reached only
  305.59 m. Five re-anchors occurred, but the retained velocity had already
  collapsed toward the zero-motion solution.
- Activity gating without preserving the anchor was behavior-identical to the
  415.59 m preset because yaw had already removed the state before low-speed
  activity could retain it.

The final exp02, exp03, and exp21 holdouts remain untouched.
