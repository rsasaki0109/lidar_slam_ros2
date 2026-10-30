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

### Joining a new run to a stored map

A session's self-similarity (method step 6) depends only on the session and the settings. With
`multisession_cache_dir`, the tool stores each session's result in that directory under the session's
fingerprint (`self_similarity_<fingerprint>.yaml`). Later runs use the stored result instead of a new
self-join. The fingerprint covers the keyframes (ids, frames, maps) and every setting of the check
except the number of threads. A changed session or setting therefore gets a new check, never a stale
result.

```bash
# once, when the map is made: check its self-similarity (one session is enough with the precheck)
graph_slam_multisession_join --ros-args --params-file <params> \
  -p "session_bag_paths:=['/data/map/backend_input']" -p "session_backend_dirs:=['/out/map']" \
  -p multisession_cache_dir:=/out/self_similarity -p multisession_precheck_self_similarity:=true \
  -p output_dir:=/out/map_check
# each new run: only the new run's self-similarity is checked
graph_slam_multisession_join --ros-args --params-file <params> \
  -p "session_bag_paths:=['/data/map/backend_input','/data/run7/backend_input']" \
  -p "session_backend_dirs:=['/out/map','/out/run7']" \
  -p multisession_cache_dir:=/out/self_similarity -p output_dir:=/out/map_run7
```

`join_report.yaml` ends with a `self_similarity` block. For each session it gives the fingerprint and
how the result was known: `stored`, `checked` (in this run, and then stored) or `none` (not needed).

Measured on the Construction pair (report 38.7). The outputs are byte-identical to computing the
check in the join.

| Run | Time |
|---|---:|
| Preparing the map (con-1) | 380 s |
| New run (con-2) joined to the stored map | 573 s (821 s without stored results) |
| Both stored | 75 s (73 s for a join that does not pass) |

A new run's self-join does not depend on the map either. Running the precheck when the run is
processed takes it out of the join.

`multisession_num_threads` (4) sets how many candidates are registered at once. Each registration
runs on one thread, so the outputs are the same for any number of threads (byte-identical on 4 and
1 threads, report 38.6).

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
6. Two refusals guard against places that repeat themselves:
   - **Ambiguity.** Every set that meets the spread criterion is checked, up to 3. If two of them pass with
     transforms that differ (1.0 m RMS over the session's keyframes, or 2 deg), there is no join. A symmetric
     room fits turned by 180 deg as well as it fits as it is.
   - **Self-similarity.** After a set passes, each session is joined to itself by the same method, with the
     matches near the identity left out (10 Scan Context candidates per keyframe; the 10 keyframes on either
     side are skipped). If either session joins itself, there is no join. Its place repeats itself, so a join to
     it cannot be told apart from a wrong one. In the self-join, a candidate is not refined after the first stage
     if less than 0.48 of its points lie within 1.0 m of the other keyframe's (`self_gate_fraction`). It could not
     be accepted anyway. Each session is checked once per run, however many sessions it is joined to. A join that
     passes still takes about ten times as long as one that does not: 821 s against 73 s on the Construction pair
     (report 38.6).
7. With more than two sessions, each session is joined to the map: the sessions joined before
   it, their keyframes placed in the first session's frame (`multisession_join_to: map`, the
   default; `first` joins every session to the first one only). The whole-map check runs over
   that map. Every session in the map was checked for self-similarity when it joined, so only the
   new session is checked. Sessions that do not join are tried again after the map has grown,
   until it stops growing.
8. The joined sessions form one graph: each session's odometry chain (no adjacent edge across a
   session start, `SubmapNode::session_start`), its own loop edges, and one inter-session edge per
   joined place, to the map keyframe's own session. It is optimized with the first submap fixed.

## Outputs (`output_dir`)

- `join_report.yaml`: per session, joined or not, candidates, accepted matches, every set tried
  with its whole-map check, and the transform to the reference session. `map_joins` gives each
  session's tries, the pass it joined in, and the map's sessions at its last try. `self_similarity`
  gives each session's fingerprint and how its self-similarity was known.
- `inter_session_edges.csv`: the inter-session edges (the session, the map keyframe's session
  `map_session`, both submaps, the relative pose).
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

Safety on more sites (report 38.5): 16 sessions were each split in two (`debug_split_gap_s`).
They come from GrandTour, MapsHD rooms, a corridor and garages, exp04 and Oxford. Every right match
was removed (`debug_exclude_transform`), so any join would be wrong.

- Without the two refusals, a symmetric room joined 20.7 m off: the room turned by 180 deg.
- With them there was no wrong join in any run.
- Two right joins were lost: one was ambiguous, the other had a self-similar half.

Joining to the map (report 38.8): there is no three-run set with a shared reference, so 18 sessions
were each cut into three parts. Each part was joined as a session, and the truth is the identity.
- Joining to the map joins 10 parts. Joining to the first part only joins 8.
- Two parts join only with the map, on a second try, once the third part is in the map (Oxford,
  pil-2).
- There was no wrong join in 72 runs, with or without the right matches.
- A part joined through the map inherits the placement error of the map's sessions: Oxford's
  transform was 0.13 m off the loop-closed reference. The joint optimization brings it to 0.035 m.

Limits:

- When only one session sees a symmetry (two identical rooms, each walked by a different session),
  neither rule can refuse the join.
- The whole-map thresholds come from the Construction pair.

`debug_exclude_transform`, `debug_split_gap_s` and `debug_split_parts` are test hooks only.
