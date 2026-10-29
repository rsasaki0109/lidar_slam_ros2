# Multi-session join (`graph_slam_multisession_join`)

Joins several runs (sessions) of one site into one pose graph. It needs no prior on how the
sessions relate: place recognition finds the transform between their odometry frames. It refuses to join
when the evidence does not hold over the whole map. A wrong cross-session join breaks the whole
map, so when the matches are in doubt the tool leaves the sessions apart.

Status: offline tool, development stage (SOTA v7 Phase 3). The method, its evaluation on
GrandTour and its limits are in `bench_report/REPORT.md` chapter 38 and
`bench_report/data/grandtour/multisession_2026-10-29/plan.txt`.

## Build

Built only when CMake finds small_gicp (`find_package(small_gicp)`; the same condition as
`registration_method: SMALL_GICP`). Without small_gicp the executable is not built.

## Inputs

For every session:

- its backend input bag: the frontend's odometry (`/rko_lio/odometry`) and deskewed scans
  (`/rko_lio/frame`) as recorded by `scripts/record_backend_input.sh`;
- the output directory of `graph_slam_offline_runner` on that bag (`trajectory_raw.tum`,
  `loop_edges.csv`).

The first session is the reference frame. All sessions must come from the same rig: one LiDAR extrinsic is given for all of them.

```bash
graph_slam_multisession_join --ros-args \
  --params-file lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml \
  -p "session_bag_paths:=['/data/run1/backend_input','/data/run2/backend_input']" \
  -p "session_backend_dirs:=['/out/run1','/out/run2']" \
  -p extrinsic_lidar2base_quat_xyzw_xyz:=[0,0,0,1,0,0,0] \
  -p output_dir:=/out/joined
```

The pose graph settings (adjacent and loop edge information, robust kernel) are read under the
runner's parameter names, so the params file of the single-session runs can be reused.

## Method

1. The runner's submaps are the keyframes. A keyframe's map is the scans within
   `multisession_keyframe_window_s` (1.0 s) of its stamp. The map is put in a gravity-aligned frame at
   the keyframe (its position and yaw), cropped to 40 m and thinned to 0.2 m.
2. Scan Context (the existing `ScanContext` class; 30 m, the map raised 2 m) proposes up to three
   keyframes of the reference session for each keyframe of the other. A candidate is kept if its distance is at most 0.4.
3. small_gicp GICP aligns each candidate from the Scan Context yaw (correspondence distance
   2.0, 1.0, then 0.5 m). A match is kept if at least 60 % of its points land within 0.3 m.
4. Every match implies one session transform. The largest set of matches that agree with one
   another (0.3 m at both matches' places, 1 deg) is found exactly (maximum clique). It must
   include 3 places at least 5 m apart.
5. Whole-map check: the session's keyframes are put into the reference session with the set's
   transform. At least 5 of them must land within 3 m of a reference keyframe, and at most 10 % of
   those may have less than half of their points within 0.3 m of the reference maps there.
   Otherwise the next largest set is tried, up to 3 sets. A site's repeated structure (columns at
   a fixed spacing along a hall) makes wrong matches that agree with each other. This check is what
   rejects them.
6. The joined sessions form one graph: each session's odometry chain (no adjacent edge across a
   session start, `SubmapNode::session_start`), its own loop edges, and one inter-session edge per
   joined place, optimized with the first submap fixed.

## Outputs (`output_dir`)

- `join_report.yaml`: per session, joined or not, candidates, accepted matches, every set tried
  with its whole-map check, and the transform to the reference session.
- `inter_session_edges.csv`: the inter-session edges (submaps and relative pose).
- `trajectory_session<i>.tum`: each joined session's optimized submap poses in the reference
  session's frame. Densify them onto the frontend trajectory with
  `scripts/densify_corrected_trajectory.py`.

## Measured (GrandTour development missions, two sites)

| Pair | Join error | Joint map: B's ATE with A's alignment vs B alone |
|---|---:|---:|
| con-1 + con-2 (both orders) | 11.5 / 12.5 mm | +2.0 / -0.1 mm |
| arc-5 + arc-6 (both orders) | 11.3 / 29.1 mm | +4.9 / +4.1 mm |
| different sites (2 pairs) | not joined | |
| the true matches removed (4 runs) | not joined | |

Limits: two sites and two total-station set-ups. The whole-map thresholds come from one of
these pairs, and its margin on the Construction site's wrong sets is thin (13 % of places against
the 10 % limit). Validate on more sites before unattended use. The `debug_exclude_transform`
parameter is a stress-test hook only.
