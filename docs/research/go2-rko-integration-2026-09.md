# Go2 RKO integration, September 2026

RKO commit `3da6470f956278c39ff15acbaf61f4e024b18701` replaces
`1ef3e523604d929c76ac64be2a92789a8ca17c8b`. The six upstream commits retain
their individual history. Runtime code grows by 137 net lines; the remaining
additions are tests (290), build registration (18), and experiment notes (24).

The subsequent `12987f4` cleanup moves the queue test unchanged into `test/`,
replaces its obsolete experiment instructions with README documentation, and
updates two runtime comments. Runtime equations and settings are unchanged.
Its fresh Release build again passed 78 cases; Mask2 clouds/timestamps matched
all 669 inputs exactly, with pose differences below 2.8e-15 m and 3.4e-16 rad.

## Behavior

- A full LiDAR queue replaces its oldest pending scan with the newest valid
  scan. Conversion failures preserve the queued scan. Capacity must be positive;
  IMU readiness is recomputed under the queue lock after replacement.
- The gravity prior uses normalized measured direction and a fixed Tukey loss
  with a one-gravity innovation scale. Invalid or near-zero measurements provide
  no gravity constraint. This changes the numerical prior, not just its storage.
- Deskew rotation uses ordered gyro samples when the scan interval is covered.
  History is bounded by two seconds and 4096 knots; missing coverage falls back
  to the existing averaged-motion path. Translation and pose prediction retain
  their existing equations.
- With `initialization_phase=true`, neither the first LiDAR cloud/odometry pair
  nor IMU-rate odometry is published in a provisional world orientation. Stored
  trajectory poses retain their initialized frame. Explicitly disabling the
  initialization phase still publishes the first scan.

The Go2 evaluations used acceleration in SI units and initialization enabled.
Their converted bags must not be confused with original acceleration-in-g bags.
The local replay recipe is in the sibling workspace's
`jeplo_bench/GO2_LIVE_REPLAY.md`; it records queue, prediction, map, bag, and
runtime choices. This submodule update does not itself select that replay profile.

## Evidence and limits

All eight Go2 SLAM sequences achieved globally SE(3)-aligned ATE at most 0.10 m
with the evaluated candidate. That fit does not prove initial absolute accuracy:
Mask2's first-GT-anchor maximum error worsened from approximately 0.333 to 0.571 m.
The adaptive map-thickness p95 also worsened in seven of eight sequences.
Same-plane diagnostics showed that point density and mixed surfaces affect the
thickness statistic, so downstream localization was evaluated separately.

Five indoor leave-one-out maps each exclude the query session and merge the
other four source-session maps after source-only GT frame alignment. Both map
arms use the same candidate query frontend, native localizer, and replay profile.
These graph maps are distinct from the earlier dense deskew LOO maps.
The map-generation baseline is `5190642` (queue retention already present),
compared with `3da6470`; this table is not a direct run of the old main pin.

| Query | Old/candidate map ATE (m) | Old/candidate maximum error (m) |
|---|---:|---:|
| Box | 2.820414 / 0.092618 | 6.266322 / 0.279141 |
| Mask1 | 0.080652 / 0.055834 | 0.354302 / 0.236604 |
| Mask2 | 0.070558 / 0.045888 | 0.219270 / 0.145694 |
| Mix | 0.074143 / 0.058157 | 0.363911 / 0.340465 |
| Stairs | 0.073651 / 0.048662 | 0.668475 / 0.334238 |

Each is one normal, GT-seeded replay on a shared host. All expected inputs were
processed; paired frontend configurations and trajectories matched exactly.
Only the effective native map path differed. GT gaps remain unevaluated and
actual TF-listener delivery timing was not frozen. Alignment timing was mixed,
including a slower Mix result; this is not a general runtime-speed claim.

Single-source Mask2 maps did not generalize: the candidate caused a final Box
error of 2.716 m after the localizer's existing 30-rejection guard release, and
Stairs degraded. Outdoor cross-route results also include an ATE regression.
Earlier CPU-stress repetitions retained a long Box output gap. This integration
does not resolve those limitations or establish new-map fault robustness.

## Integration validation

A separate Jazzy Release build of the exact candidate passed all 11 CTest
targets, comprising 78 GTest cases with no failures or disabled cases. GTest
expectations remain active with `NDEBUG`. The fresh build's Mask2 replay matched
all 669 published local clouds and timestamps exactly; pose differences were
below 3.4e-15 m and 4.5e-16 rad. Pose arrays were not byte-identical.
This is targeted RKO validation, not a fresh full-workspace C++ CI run.

Local evidence roots under `/media/sasaki/aiueo2/jeplo_data/experiments/`:

- `go2_graph_final_all8`: paired SLAM outputs and source/input receipts.
- `go2_graph_loo_maps`, `go2_graph_loo_replay`: source exclusion, replay settings,
  full/common-time metrics, process outcomes, and artifact hashes.
- `go2_mask2_cross_route_box_stairs`: single-source failures and guard evidence.
- `go2_rko_integration_review`: independent build, test XML, source pins, and
  captured-cloud/pose comparison.
