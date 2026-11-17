# Refinement stability screen: results (plan.txt, pre-registered 2026-11-17)

Code: `refine_window_max_adjacent_change_m` / `_deg` (RB, hba_pyramid.hpp) and
`refine_relative_translation_sigma` / `_rotation_sigma_rad` (RA, plane_ba.hpp), offline runner
parameters, default 0 = off. With both off, all output files on HILTI exp04 are byte-identical to
the build before the change. Runs: `python -m lidarslam backend` (`tools/run.sh`), scores
`tools/score.py`, raw numbers `scores.json`.

## ATE (m) of trajectory_refined against the development references, and 1 um sensitivity (m)

| Run | pose graph | R0 | RB | RA | sens R0 | sens RB | sens RA |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| C1 GrandTour CON-1 | 0.3844 | 0.4497 | 0.3848 (18/20 gated) | 0.3931 | 0.7672 | 0.0004 | 0.0173 |
| C2 GrandTour CON-2 | 0.2907 | 0.3518 | 0.2907 (20/20 gated) | 0.3003 | 1.6208 | 0.0000 | 0.0079 |
| M1 MapsHD calib-room 1 | 0.2327 | 0.2294 | 0.2294 (0/23 gated) | 0.2304 | 0.0001 | 0.0001 | 0.0001 |
| H4 HILTI exp04 (weak) | 1.4225 | 1.4270 | 1.4263 (3/4 gated) | 1.4169 | 0.0000 | 0.0002 | 0.0000 |
| S2 Stadtgarten 2 | 0.7795 | 0.7818 | 0.7818 (0/33 gated) | 0.7826 | 0.0173 | 0.0173 | 0.0008 |

## Decision

| Clause | RB | RA |
| --- | --- | --- |
| a. C1 ATE <= 1.05 x pose graph (0.4036 m) | 0.3848 pass | 0.3931 pass |
| b. C2, M1, H4, S2 within 1.02 x R0 + 1 mm | pass on all four | pass on all four |
| c. C1 sensitivity <= 0.10 m | 0.0004 pass | 0.0173 pass |
| mean ATE / R0 over the five | 0.936 | 0.945 |

**Both pass; RB is recommended** (RA's mean ratio is not lower by 0.02). The defaults are
unchanged in this step.

## Reading

- The instability is not CON-1's alone: on CON-2 the current refinement is also worse than the
  pose graph (0.352 against 0.291 m) and a 1 um change moves it by 1.62 m.
- RB discards 18 of 20 windows on CON-1 and all 20 on CON-2 (the refinement then reports
  `no_improvement` and keeps the pose graph), and none on MapsHD calibration-room 1 or Stadtgarten
  2, where its result equals R0's byte for byte. On these GrandTour runs the gate effectively
  turns the refinement off, which is what their reference asks for.
- RA keeps every window, cuts the sensitivity (CON-1 0.77 m -> 17 mm, CON-2 1.62 m -> 8 mm,
  Stadtgarten 17 -> 0.8 mm) and still leaves the GrandTour runs 2-3 % worse than the pose graph.
- H4 is not informative: the submap trajectory (one pose per 1.5 m) is too sparse for its seven
  control points, so all its ATEs are about 1.42 m (the earlier exp04 scores used the dense
  frontend trajectory in the LiDAR frame).
- Wall time is not changed materially (RB/RA 1-18 s faster than R0, from fewer accepted steps).

## Default switched (maintainer's approval, 2026-11-17)

The maintainer chose the recommendation: the offline runner now defaults to
`refine_window_max_adjacent_change_m: 0.10` and `refine_window_max_adjacent_change_deg: 1.0`.
Checked with the build that carries the new default (`tools/run.sh`, variants DEF and OFF):

- no parameter given: `trajectory_refined.tum` identical to the RB run on all five runs;
- the gate set to 0: every output file identical to the R0 run on all five runs.

The map refinement report gains a `gated_windows` line while the gate is on. The live ROS 2
node does not run this refinement and is unchanged.
