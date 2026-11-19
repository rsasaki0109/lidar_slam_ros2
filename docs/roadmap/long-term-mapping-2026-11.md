# Long-term mapping: update a stored map while driving (2026-11)

Goal: a robot that drives through a place it has mapped before updates the stored map as it
goes. Where the place changed (furniture moved, a partition taken down, a wall added), the map
shows the place as it is now: new structure appears, and structure the robot sees is gone is
removed automatically. Nothing the robot did not see changes. The old map is kept, so an update
can be undone.

Decisions (maintainer, 2026-11-18): the update runs live, while the robot drives (not as a step
after the run); stale points are removed automatically (no human review step); the method is
developed and evaluated on public data only (this is open-source work with no robot of its own).

## Where the project is today

| Part | What exists | What is missing for this goal |
| --- | --- | --- |
| Map of one run | `lidarslam-map run/start` (ROS 2 or `--engine core`): an Autoware bundle (`pointcloud_map/`, metadata, trajectories) plus the recorded backend input | the bundle is the end product; nothing loads it back as a map to extend |
| Several runs | `lidarslam-map merge [--place-recognition]` and `start --join-map` (`scripts/map_merge.py`, `scripts/map_join.py`) | runs after the fact, never live. Each run's `map.pcd` is stacked with one rigid x, y, z, yaw transform, and in a voxel shared by two runs the **first** run's point is kept (`_deduplicate_records`, `scripts/map_merge.py`): a moved desk stays where it was, and new things only fill empty space |
| Place recognition between runs | `graph_slam_multisession_join` / `python -m lidarslam join` (`multisession_join.hpp`, `docs/architecture/multisession-join.md`): Scan Context candidates, small_gicp, max-clique, refusals (ambiguous sets, self-similar sessions, whole-map check), a joined pose graph | offline only, over finished runs. The re-optimized joined trajectories are not used for the merged points |
| Removing stale points | none. The 2026-11-08 trial (`bench_report/data/map_update_2026-11-08/plan.txt`, rays of a later run clear an earlier run's points) failed its preregistered rule | its test counted removed people as errors, and it cast rays from one frame in five |
| Live mapping | the ROS 2 nodes (`rko_lio`, `graph_based_slam`) | no ROS-free streaming API (task: ROS-free live mapping); no way to start from a stored map |
| Engine boundary | `GraphSlamApplication`: "the ordered, ROS-free workflow boundary shared by live and replay adapters" | a live adapter outside ROS 2 |

The product scripts reach each other through subprocesses (`lidarslam-map` dispatches to a dozen
scripts; `start` is `sensor_setup_wizard.py`, 3,935 lines, and also drives the join), and every
multi-session step exists twice (ROS 2 executables and `python -m lidarslam`). This design adds
the long-term mode to the package and the C++ core, not to that layer, and moves the join out of
it (see "Architecture cleanup").

## Design

### 1. The map store is the source of truth

A long-term map is a directory, `store/`, not an Autoware bundle. The bundle becomes an export.

```
store/
  store.json          format version, frame, the list of sessions (id, time, sensor setup, state)
  sessions/<id>/      keyframes: pose in the store frame, compressed deskewed scan (the backend's
                      cloud_compression format), Scan Context descriptor; the session's
                      self-similarity record (the join's fingerprint cache)
  graph/              the pose graph over all sessions: per-session odometry/loop edges and the
                      inter-session edges, with each edge's source
  evidence/           per-submap voxel evidence (below)
  export/             the Autoware bundle written from the store after each session
```

Sessions are append-only. A session is written to `sessions/<id>/` while it runs and becomes part
of the map only when `store.json` lists it as committed (one atomic rename). Rolling back an
update removes that entry; the older sessions' data was never changed.

### 2. A live session

1. **Load** the store: keyframes and submaps of the committed sessions (compressed in memory, as
   the backend keeps submaps today), their descriptors, the graph.
2. **Relocalize.** The first keyframes of the new session are matched against the stored
   keyframes with the join's place recognition and its refusals. Until a match is accepted the
   session maps in its own frame and its keyframes wait; when it is accepted, the session is
   anchored and the waiting keyframes are processed with the anchor. A start pose given by the
   caller (as Autoware's initial pose) narrows the search but is checked the same way. If no
   match is accepted the session is mapped and stored as a new, separate map island; it is never
   forced onto the old map.
3. **Track and close loops.** RKO-LIO odometry as today. The backend (`GraphSlamApplication`)
   holds the stored keyframes as nodes with their stored poses as priors and searches loops
   between new and stored submaps with the existing loop search. The graph is optimized
   incrementally; stored poses move only as far as their priors allow.
4. **Update the map (change detection).** For each new keyframe, the scan's rays are traced (3D
   DDA, C++) through the stored submaps near it. Evidence is kept **per submap, in the submap's
   own frame** (as SEAM, arXiv:2609.18819, anchors it), so a later pose correction moves a
   submap with its evidence and never invalidates it. Per voxel:
   - *hits* and *misses* of the current session, counted per frame;
   - the stored points' state: present, or removed by session `<id>` (a tombstone, so it can be
     undone).

   Rules:
   - **Removed (stale):** a stored voxel that rays of the current session crossed, short of their
     end point by a margin, in at least `k` frames from at least two viewpoints, with no hit of
     the session in it or its neighbours. Rays at grazing angles to a stored surface do not count.
   - **Added:** a voxel the current session hit in at least `m` frames.
   - **Not static (people, carts):** a voxel the current session both hit and saw through in
     different frames. It is left out of the map whichever session it came from, so people in the
     old map are removed as well.
   - **Not seen:** no change.

   The decision for a voxel is the latest session's: "the last session that looked wins".
5. **Publish.** After each keyframe the map change (voxels added and removed, in the store
   frame) is available to the caller: a Python callback, and later a ROS 2 topic.
6. **Commit.** At the end the session, its graph edges and its evidence are written and listed in
   `store.json`; the Autoware bundle is exported from the store.

### 3. Engine and API

- C++ (ROS-free, in `graph_based_slam`): the store, the live long-term adapter around
  `GraphSlamApplication`, the evidence map and ray tracing.
- Python (`lidarslam` package): `lidarslam.live.Session(store, params)` with `add_imu`,
  `add_scan`, `changes()`, `commit()`; `lidarslam-map live <store> <log> [--realtime]` replays a
  log at sensor rate through it (this is also the ROS-free live mapping task, simulated by
  real-time playback). A ROS 2 node over the same C++ adapter comes after the evaluation.
- The offline path (`run`, `start --engine core`) stays as it is and keeps its bytes.

### Architecture cleanup (in scope)

- Multi-session code moves into the package: one Python implementation of joining and merging
  over the C++ core, used by `merge`, `start --join-map` and the store. The ROS 2 executables stay
  as adapters; the duplicate subprocess paths in `map_join.py` go.
- `merge` is rebuilt on the store: `merge A B` = a store with session A, updated offline with
  session B (the same code as a live update, fed from a finished run). Its "first session wins"
  stacking goes.
- `sensor_setup_wizard.py` stops driving the join (`--join-map` becomes "update this store").
- Each step keeps the single-run outputs byte-identical (checked as in the ROS-free milestones).

## Evaluation (preregistered; fixed before any run of the method)

### Data

**LUH long-term indoor localization dataset**
([data.uni-hannover.de](https://data.uni-hannover.de/dataset/long-term-indoor-localization-in-dynamic-environment),
CC BY 4.0; paper doi:10.5194/isprs-archives-XLVIII-1-W6-2025-155-2025). A 40 x 10 m entrance hall,
recorded from a push cart (Ouster OS1, its IMU on `/ouster/imu`, Hesai Pandar64; ROS 1 bags) in
three layouts:

| Layout | Sequences | Contents |
| --- | --- | --- |
| A, extreme occluded | S0 (5.3 min), S1 (2.7 min) | eight poster walls forming a corridor |
| B, semi occluded | S2 (1.4), S3 (2.0), S4 (2.6) | L-shaped poster walls, tables, chairs |
| C, free space | S5 (2.2), S6 (2.0), S7 (2.0) | the empty hall, 10-15 pedestrians |

Reference: total-station positions of every sequence (prism tracked, offset to the LiDAR solved,
in the TLS frame) and a terrestrial laser scan of the **empty** hall (no poster walls, furniture
or people). Only the Ouster and its IMU are used (one sensor, as the product runs). The bags are
stream-converted to ROS 2 keeping those two topics, and handled as untrusted downloads.

### Split

Development uses S1, S3 and S6 only (their trajectories and the TLS scan may be looked at while
developing). Holdout: S0, S2, S4, S5, S7; their total-station trajectories stay unopened until the
method and its parameters are frozen and recorded in this file.

Update chains (base session first, each later session a live update of the store):

- development: S1 → S3 → S6, and S6 → S3 → S1
- holdout: S0 → S4 → S7, S7 → S4 → S0, S2 → S5, S5 → S2

### Measures

The store frame is aligned to the TLS frame once per chain, by the base session's estimated
positions against its total-station positions (Umeyama, rigid). Points are compared inside the
hall's floor area, between 0.1 m above the floor and 2.5 m (the poster walls' height band), and
only where the update session's own scans reach (within 10 m of its path).

- **G, ghosts after an update to layout C.** The share of map points farther than 0.10 m from
  the TLS scan. Everything that is not the empty hall is a ghost here: poster walls, furniture
  and people.
- **S, static structure kept.** The share of TLS points (same region) that have a map point within
  0.10 m, after the update, divided by the same share before it.
- **N, new structure after an update from layout C to A or B.** Reference: the map of the other
  sequence of the same layout, mapped alone, keeping its points farther than 0.20 m from the TLS
  scan (the poster walls and furniture). N is the share of those reference points with a map
  point within 0.10 m after the update. Uses S0/S1 and S2/S3/S4 as each other's reference; in
  development only S1 ↔ S3 cross references are used (no holdout map is built before the holdout
  run).
- **R, relocalization.** The update session anchors to the store within the first 10 s of data,
  with no wrong anchor (a wrong anchor: the anchored pose more than 0.5 m or 5 degrees from the
  total-station position and the TLS-registered heading).
- **L, localization.** Position RMSE of the update session's live (causal) poses against the
  total station, compared with the same sequence mapped alone.
- **T, run time.** Live mode at sensor rate on the development machine: no input dropped,
  processing time per scan below the scan period on average; peak memory reported.

Baseline for G, S and N: today's `lidarslam-map merge --place-recognition` on the same sessions.

### Decision rule (holdout)

The long-term mode becomes the product's update path (and `merge` is rebuilt on it) if, on every
holdout chain:

- G ≤ 0.02 after every update to layout C, and below the baseline's G;
- S ≥ 0.97 for every update;
- N ≥ 0.90 for every update from layout C to A or B;
- R holds for every update session;
- L: RMSE no more than 1.2 x the same sequence mapped alone, and ≤ 0.10 m;
- T holds.

If a criterion fails, the method stays an opt-in mode, the failure is recorded here with its
cause, and the next design is written as a new preregistration. Thresholds and measures in this
section do not change after the first holdout run.

### Data and machine notes

- The bags are 5-19 GB each (both LiDARs, bz2 chunks); the machine has about 11 GB free.
  `scripts/convert_luh_longterm_bag.py` reads a bag front to back over HTTP (resuming with Range
  requests, checking the dataset's md5) and writes a ROS 2 bag, so the original is never stored.
- What the bags hold (checked on the first 200 MB of S3): `/ouster/points` from an OS1-128 at
  2048 columns and 10 Hz (frame `ouster_lidar`; 262,144 points per scan, 22 % of them zero-range
  returns; fields x, y, z, intensity, `t` (uint32, ns after the header stamp, 0-100 ms),
  reflectivity, ring, ambient, range), `/ouster/imu` at about 100 Hz (frame `ouster/ouster_imu`),
  `/hesai/pandar`. The converter keeps x, y, z, intensity and `t`, drops zero-range returns and
  keeps every second column (the OS1's 1024-column mode): about 1.4 MB per scan with zstd
  instead of 12.6 MB.
- The bags have no `/tf_static`. The LiDAR-IMU extrinsic comes from the Ouster's documented
  sensor frames: the LiDAR frame turned 180 degrees about z, at (-0.006253, 0.011775, 0.028535) m
  in the IMU frame. On S3 (development) RKO-LIO odometry with it is closer to the total station
  than with no rotation (position RMSE 0.033 vs 0.041 m), so it is kept.
- The ground-truth files write each stamp with its tenth of a second fused on
  (`17410941326` is 1741094132.6 s); 10 Hz, one row per scan.

### L0 status (2026-11-18)

S3 converted (7.70 GB read, md5 as published, 1,189 scans and 11,886 IMU messages, 1.7 GB, 48
min at about 2.7 MB/s). RKO-LIO odometry alone (lidarslam 0.12.0 from PyPI, no ROS 2, voxel 0.25
m, the walking preset's registration settings): position RMSE 0.033 m, median 0.033 m, max 0.103
m against the total station over 1,183 matched scans and 44.5 m of path; 38 s for 119 s of data
on the development machine, 183 MB peak memory.

### L1 status (2026-11-18)

`lidarslam.live.Session` takes IMU and LiDAR messages one at a time and hands out every scan's
live pose as it is processed: the backend's causal pose (`OfflinePipeline::setLivePoseListener`,
the odometry corrected by the latest optimization the background worker had finished by the
scan's stamp). `python -m lidarslam live <log> ... --realtime` plays a log into a session at the
sensor's rate and writes `trajectory_live.tum` and `live_timing.json`. The CI test
(`lidarslam_core/test/test_python_live.py`) checks that the live poses are the causal trajectory
and that the session's files are byte-identical to `lidarslam map` with the causal output on.

S3 at sensor rate (development machine, 4 cores, walking backend preset): 118.7 s of data
streamed in 118.9 s with no message held back; a scan's live pose came 0.028 s after the scan's
data time (median; p95 0.26 s, max 0.67 s); the backend's remaining stages took 2.5 s after the
last message; peak memory 310 MB. Live poses against the total station: 0.033 m RMSE over 44.5
m (no loop edges in this sequence, so the live poses equal the odometry). The playback started
77 s after the command (opening the log and the session); to be looked at with the store (L2).

### L2 status (2026-11-18)

The store exists (`lidarslam.store.Store`, format version 1). A live session given a store
(`live.Session(..., store=)`, `python -m lidarslam live <log> ... --store STORE`) writes itself
to `sessions/<id>/` when it finishes and is then listed in `store.json` (one rename): the
backend's submaps in `submaps.bin` (stamp, odometry pose, optimized pose, the scan compressed as
the backend keeps it), `pose_graph.g2o`, and the parameters it was mapped with. `export/` is the
Autoware bundle written from the stored sessions (`python -m lidarslam store export STORE`);
`store rollback` takes the latest session off the list and leaves its files. `graph/` and
`evidence/` of the layout above come with L3 and L4. The C++ side is
`graph_based_slam/map_store.hpp` (session files, export) and `map_export::writeBundle`, now the
one map-bundle writer of both the backend and the store.

Check: the store's export of one session is byte-identical to the bundle the backend wrote. CI
(`lidarslam_core/test/test_python_store.py`, streamed and assembled map paths) and S3 (29
submaps; `map.pcd`, `map_bundle.yaml`, `map_projector_info.yaml`, `pose_graph.g2o` and the 15
`pointcloud_map/` files all identical). The S3 session takes 22 MB in the store; the export 4.8
MB. A store of two sessions is refused for export until they share a frame (L3), unless asked to
stack them.

The 77 s before playback in the L1 run did not come back: S3 at sensor rate with the store took
122.7 s from the command for 118.7 s of data (0.4 s before the first message, 3.4 s for the
backend's last stages and the store), with live poses 0.026 s after their scan's data time
(median; p95 0.033 s, max 0.045 s). The L1 run had shared the machine with the S6 conversion.

### L3 development status (2026-11-19)

`lidarslam.live.Session(..., store=..., initial_pose=...)` and `python -m lidarslam live
--store ... --initial-pose X Y Z QX QY QZ QW` verify a prior for the new odometry frame in the
store frame. Each scan window uses only messages received up to the decision. The initial
estimate must satisfy the 2 m and 20 degree bounds. A second distinct reference submap has to
support the same estimate. The confirmation search is centered on the estimate when the first
search returns one supporting submap. Stores written by the live path keep thinned reference
windows with their session files. Stores from L2 can be read using their individual submap scans.
An accepted session records `store_from_session` and exposes causal poses in `trajectory_store.tum`.

With the compiled matcher and recorded S3/S6 startup clouds, 76 of 91 perturbed nearby priors
joined for S3 → S6 and 60 of 91 for S6 → S3. All 278 wrong-site or wrong-heading priors and the
additional prior centered on the 4.75 m false match were refused.

Sensor-rate development chains with priors generated from the development ground truth:

| Chain | Update | Anchor wall time | Causal RMSE | Standalone RMSE |
| --- | --- | ---: | ---: | ---: |
| S1 → S3 → S6 | S3 | 1.58 s | 5.42 cm | 3.75 cm |
| S1 → S3 → S6 | S6 | 1.61 s | 3.42 cm | 1.55 cm |
| S6 → S3 → S1 | S3 | 1.68 s | 4.74 cm | 3.75 cm |
| S6 → S3 → S1 | S1 | 1.69 s | 8.40 cm | 3.71 cm |

All four first store poses met the position and heading tolerances. The standalone columns
use the same matched timestamps, with a rigid fit of each session's standalone causal path. The fixed
anchor exceeds the 1.2× localization ratio in all four updates. A causal replay of candidate
loop factors through g2o reduced RMSE in three of the four updates. The S6 and S1 updates in
the first and second chains still exceeded the ratio. The causal factor replay used recorded
keyframe windows and is a development experiment.

`live --track-store` (Python `live.Session(..., track_store=True)`) enables experimental live
tracking after startup. It verifies later windows against the store and adds the accepted
matches as robust inter-session factors. Observed reference submaps are fixed and active
session submaps are connected by odometry edges. The live pose stream uses the most recent
causal solution. Each accepted session saves its graph and absolute submap poses. Export and
later relocalization use those poses. This path still requires `--initial-pose`. Automatic
place recognition and a shared graph with movable stored poses and priors remain open in L3.

Sensor-rate replay through `live --track-store` on both development chains:

| Chain | Update | Accepted factors | Anchor wall time | Causal RMSE | Standalone RMSE | Ratio |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| S1 → S3 → S6 | S3 | 9 | 1.74 s | 4.11 cm | 3.75 cm | 1.10× |
| S1 → S3 → S6 | S6 | 22 | 1.64 s | 3.49 cm | 1.56 cm | 2.24× |
| S6 → S3 → S1 | S3 | 25 | 1.73 s | 4.27 cm | 3.75 cm | 1.14× |
| S6 → S3 → S1 | S1 | 30 | 1.67 s | 4.62 cm | 3.75 cm | 1.23× |

Each first store pose was within 0.025 m and 0.21 degrees. The driver reported zero trailing
frames, and median scan-to-pose lag was 0.02–0.03 s with p95 0.54–0.81 s. The two S3 updates
met the localization ratio. S6 had the tightest standalone comparator and slightly higher RMSE
than with the fixed anchor. Causally fitting one rigid transform to the accumulated, already
accepted factors in an offline diagnostic yielded 3.66 cm and 2.74 cm on the forward chain,
and 4.16 cm and 3.65 cm on the reverse chain. The diagnostic reused the live graph's accepted
factors. A live version would need a new replay because different corrections change later
matching priors. S6 still exceeds the ratio in the diagnostic.

`live --start-near-first-submap` is an experimental startup mode for a recording whose
operator independently knows it begins within 5 m of the first stored submap. It ranks Scan
Context matches in that area, then applies the existing geometric checks to the top eight
initializations. Conflicting verified poses cause a refusal. It derives the initializer from
the incoming LiDAR and the stored keyframes. A development replay of archived windows without
the area restriction found wrong-site aliases 8.68 m and 11.77 m away. Restricting the search
gave a correct, consistent result for all four archived development updates. The restriction
is an input assumption for the mode.

With incoming windows thinned to the density of the stored archive, both full development
chains ran at sensor rate using the launch-area search and the L4 persistence filter. No
ground-truth pose was supplied to the live runs. Scoring paired the live and standalone
estimates at common total-station timestamps:

| Update | Anchor data time | Causal RMSE | Standalone ratio | Active time / scan | Peak RSS |
| --- | ---: | ---: | ---: | ---: | ---: |
| S1 → S3 | 1.20 s | 4.07 cm | 1.09× | 70.8 ms | 813 MiB |
| S1 → S3 → S6 | 1.20 s | 1.99 cm | 1.28× | 77.4 ms | 950 MiB |
| S6 → S3 | 1.20 s | 4.32 cm | 1.16× | 68.6 ms | 881 MiB |
| S6 → S3 → S1 | 1.30 s | 4.20 cm | 1.12× | 63.6 ms | 996 MiB |

First-pose error stayed below 0.03 m and 0.20 degrees. There were zero trailing frames, with
one initializing scan per run. The earlier, unthinned startup probe refused S6 → S3. The
S6 update still exceeds the 1.2× L criterion.

Replaying the recorded factors at several uniform gains only reduced S6 to 1.97 cm. A candidate
that applied 10% of the position correction and 55% of the rotation correction looked promising
with recorded factors. On full unpaced sensor replay it gave 1.88 cm (1.208×) on S6 and
4.50 cm (1.202×) on the reverse S1 update. The sensor-rate runs above use the existing
35% correction for both, which remains the live setting.

On the S6 update, 11 of the 21 corrections after startup increased the error through the next
correction compared with keeping the preceding transform for that same interval. The correction
at 61.8 s raised interval RMSE from 2.52 to 3.41 cm and stayed in effect for 10.7 s. The one at
8.0 s raised it from 1.48 to 3.69 cm over 3.0 s. These comparisons use development ground truth
after playback and left the subsequent matching decisions fixed.

A shared-surface prototype builds a target from stored keyframe windows. It keeps points
within 0.10 m of a point in another aligned session, estimates local normals, and registers
each new window against the resulting near-horizontal and near-vertical surfaces. It uses
the existing verified startup anchor. The target is built without TLS or ground truth. On
recorded windows, using all stored planar surfaces without the agreement filter gave 2.27 cm
for S6 and 4.30 cm for S1 on the reverse chain. Shared surfaces gave 1.98 cm and 4.27 cm.
Using 35% of each correction at its keyframe gave 1.93 cm and 3.94 cm. These trials used the
same two committed reference sessions as the original live tracking runs.

Sensor-rate replay used the final update in each chain, with the committed output of each S3
run as store input:

| Final update | Live graph RMSE | Shared surfaces | Shared surfaces, 35% correction | Standalone ratio with 35% |
| --- | ---: | ---: | ---: | ---: |
| S1 → S3 → S6 | 3.49 cm | 1.99 cm | 1.93 cm | 1.24× |
| S6 → S3 → S1 | 4.62 cm | 4.32 cm | 3.98 cm | 1.06× |

The trials anchored within 1.73 s of playback start and reported zero trailing frames. The
35% variant had p95 scan-to-pose lag of 0.67 s on S6 and 0.53 s on S1. Building the shared
target took about 3.0–3.2 s in a separate timing probe. S6 remains above the 1.2× ratio.
The trial applied each correction directly to live poses.

`python -m lidarslam live --track-shared-surfaces` (Python `track_shared_surfaces=True`) now
includes shared-surface measurements in the tracking graph. It starts with the existing verified
anchor and builds the target from windows in aligned, committed sessions. When there are fewer
than two usable sessions it uses submap tracking. A match to the shared target is a robust SE(3)
edge from a fixed store-frame vertex (vertex 0 in `store_tracking_graph.g2o`). The optimizer
retains the full measurement and uses odometry edges between active submaps. Live output applies
35% of the graph's correction about the matched keyframe. Committing saves the graph's absolute
submap poses in `store_submap_poses.npy`, which both export and later relocalization read. The
session metadata records `shared_surface_observations`.

Full-rate tests appended a fourth session in a fresh process using the three-session store.
Both used a repeat of the S3 development recording as a revisit:

| Store and update | Shared factors | Anchor wall time | Causal RMSE | Standalone ratio |
| --- | ---: | ---: | ---: | ---: |
| S1 → S3 → S6 | 43 | 1.70 s | 1.98 cm | 1.27× |
| S1 → S3 → S6 → S3 | 28 | 1.56 s | 3.56 cm | 0.95× |
| S6 → S3 → S1 | 51 | 1.70 s | 4.16 cm | 1.11× |
| S6 → S3 → S1 → S3 | 28 | 1.61 s | 4.04 cm | 1.08× |

There were no trailing frames in the full runs. The new process read the exact absolute poses
from the third session, built a different shared target, and left its graph and submap files
byte-identical after committing the fourth. The S6 ratio is still above 1.2×.

A second test split the S6 development recording into disjoint intervals (0–43 s and 47–85 s).
The early interval contributed 14 shared factors, then the later interval reopened that store
in a separate process and appended 12 shared factors. With a prior derived from standalone S6
odometry and the development alignment, the later interval anchored 6.2 s after the first live
pose timestamp (7.26 s wall time after playback began). It had 2.24 cm causal RMSE over 314
matched poses and two pending LiDAR frames at the clip end. Both intervals come from one
development recording. The holdout remains unopened.

### L4 development status (2026-11-19)

An offline S3 → S6 ray-update probe used a fixed store frame, sampled TLS points and approximate
hall bounds. Requiring repeated free-space observations from separated viewpoints for clearing
old points retained more TLS coverage than treating every cross-frame hit and miss as dynamic.
The conservative variant retained 1.02× the base TLS coverage, with a ghost fraction of 0.479
in that probe. Tightening the approximate area to the TLS walls yielded 1.00× coverage and a
ghost fraction of 0.382. Of the 6,990 retained old ghost voxels in that area, 4,070 had enough
free-space observations from separated viewpoints but were protected by neighbouring hits, and
2,846 received a hit themselves. The same neighbour rule protected 537 old voxels within 0.10 m of
the TLS reference after repeated free-space observations. Requiring persistent neighbour hits
reduced G to 0.285. Excluding neighbours with repeated free-space observations lowered it to
0.229. Also allowing deletion after a single hit when repeated free-space evidence was available
yielded G = 0.146 and S = 0.998. These development variants used one scan per keyframe in the same
approximate area. With one scan in five, the same comparison was G = 0.429 and S = 1.009 for the
conservative rule, and G = 0.268 and S = 1.005 for the last variant. The planned per-submap evidence
and live change callbacks were outside that probe.

A second probe counted a ray crossing as free-space evidence only in a frame with no hit in
that voxel, then compared hit and free-space counts once the minimum number of separated
free-space observations was met. At one scan per keyframe, the best tested setting with S ≥ 0.97
gave G = 0.102 and S = 0.975. At one scan in five, the best such setting gave G = 0.091 and
S = 0.971. The two settings differ. In the keyframe run, 1,039 of the 1,166 remaining ghosts
came from the old map, at a median distance of 0.192 m from the sampled TLS. Both runs use the
same approximate area and fixed store frame. No L4 parameters have been frozen.

Of those 1,039 retained old ghosts, 706 were within 0.25 m of the sampled TLS and 856 were below
1 m above the floor. Only 24–27 passed a conservative test for a nearby parallel TLS wall
when the TLS neighbourhood used for the normal estimate was varied. This test leaves the other
points unclassified. For the same keyframe run, counting hits on an old voxel only within
0.09 m of its representative point yielded G = 0.100 and S = 0.983 at the best tested setting
with S ≥ 0.97. The same setting with one scan in five gave G = 0.088 and S = 0.988. A different
setting at that sampling rate gave G = 0.075 and S = 0.973. These are offline results in the
same approximate area. The ghost fraction remains above the 0.02 target.

`python -m lidarslam live --detect-changes` (Python `detect_changes=True`) is an experimental
old-map removal path. Once the prior is verified, it traces one deskewed scan per accepted
backend keyframe, including queued keyframes from startup. The C++ traversal uses a 0.15 m
world-grid index of old local-voxel centroids, a 0.30 m ray endpoint margin and a 40 m range
cap. Hits within 0.09 m of a centroid take precedence over free space in the same scan. A
voxel is suppressed when free-space votes outnumber hits by at least 8:1, with at least eight
free-space votes and viewpoints at least 0.50 m apart. Two or more hits with at least as many
hits as misses can restore an earlier suppression. Unobserved voxels keep their previous state.

`map_change_evidence.npz` belongs to the updating session. It records reference session IDs,
local submap voxel indices, per-scan vote counts, separated viewpoints and the resulting
decision. `Session.changes()` exposes the accumulating evidence. Export folds committed
decisions in session order. Rollback drops the latest decisions and regenerates the map from
the earlier sessions without rewriting their scans. This version applies masks to export;
tracking still uses the archived submaps. It uses the same world grid for candidate lookup and
ray traversal. Local ray intersection, grazing-angle rejection and change callbacks remain open.

Both development chains completed with the removal path enabled. The tables use the exported
0.10 m maps, the approximate wall bounds above, and a subsample of the TLS points. The control
exports use the same session poses with removal masks disabled.

| Chain update | Local voxels marked removed | S | G after update to C |
| --- | ---: | ---: | ---: |
| S1 → S3 | 41,724 | 1.171 | — |
| S1 → S3 → S6 | 84,457 | 1.018 | 0.451 |
| S6 → S3 | 21,491 | 1.013 | — |
| S6 → S3 → S1 | 48,889 | 1.005 | — |

At the S6 update, G was 0.653 for the unmasked control and 0.493 when S6 was mapped alone.
Old-map removal leaves substantial ghost returns in the new scans. For S6 → S3, the development
cross-layout N proxy was 0.198, versus 0.368 in the unmasked control. That proxy uses the S1
reference, so it is not a same-layout holdout measurement. The prescribed
`merge --place-recognition` baseline declined to join the S3 update in both directions and
produced no baseline maps. Both development stores were rolled back two steps in copies of
their indexes. At each step the exported bundle matched its saved predecessor byte for byte,
and the committed session files remained identical. All four runs reported zero trailing
frames. The concurrent replays and baseline jobs shared the machine, and the last updates
ran slower than sensor time. These results do not establish the runtime gate.

`python -m lidarslam live --filter-new-points` (Python `filter_new_points=True`) adds an
experimental persistence filter on top of old-map removal. It collects one localized scan
about every 0.5 s. Each 0.15 m store-frame cell can receive one hit or one free-space vote
per sampled scan, with hits taking precedence. Free-space votes begin after the first hit.
A new voxel is retained when a representative within 0.15 m has at least three hits and
at least as many hits as misses. Filtering covers new submap voxels within 40 m of the sensor
and requires at least three sampled scans. The sampled scans and keyframe poses are causal.
At commit, the capture-time pose maps each decision to an immutable submap-local voxel, so
it still masks the same points when export uses the optimized pose.

The updating session stores its suppression mask as the optional `new_rows` array in
`map_change_evidence.npz`. Older evidence files can still be read. Later observations can
restore a suppressed voxel, and rolling back the update discards its mask. A stationary
transient that receives repeated consistent hits can still pass this filter.

Both sensor-rate development chains completed with no trailing frames. An export of the same
sessions and poses that ignored only the `new_rows` arrays served as the old-removal-only
control. Scoring used the same sampled TLS and approximate wall bounds as above.

| Chain update | New local voxels suppressed | S | Absolute TLS coverage | Control coverage |
| --- | ---: | ---: | ---: | ---: |
| S1 → S3 | 24,205 | 1.132 | 85.48% | 88.44% |
| S1 → S3 → S6 | 47,656 | 1.025 | 91.76% | 92.35% |
| S6 → S3 | 20,763 | 1.010 | 92.54% | 92.78% |
| S6 → S3 → S1 | 30,542 | 1.002 | 92.99% | 93.51% |

S6 ghost fraction fell from 0.451 to 0.188. Of the 7,338 remaining ghost points, 1,963 had a
neighbour within 0.06 m only in the old submap export, 2,836 only in the new submap export,
and 2,538 in both. One had no neighbour in either component export. This is a proximity
diagnostic because the combined export downsamples the joint point cloud. The S6 → S3
cross-layout N proxy fell from 0.198 to 0.121. Of the 1,948 reference points that lost coverage,
1,174 were then more than 0.30 m from the map. Both stores again rolled back to the two prior
exports byte for byte with their session files intact. The runtime and quality gates for
holdout have not been established.

A sequential timing comparison replayed the same 43 s S6 development clip into copies of
the same two-session store. With old removal only and with the persistence filter, streaming
wall time was 43.00 s in both runs. Whole-process CPU time was 49.8 s and 53.3 s, and peak RSS
was 706 MiB and 735 MiB, respectively. Scan-to-pose p95 was 0.49 s and 0.45 s. The clip ended
with two pending LiDAR frames in each run. In the full-chain runs with the persistence filter,
scan-to-pose p95 ranged from 0.40 to 0.64 s.

`live --refine-visibility` is a separate L4 experiment for new submaps. It caches the 0.5 s
scan samples used by the persistence filter. The DDA first locates candidate voxels. The voxel
centroid must then lie within 0.085 m of the ray and before its endpoint margin for a free-space
vote. A hit within 0.10 m takes precedence in the same scan.

The new submap candidate index is complete before replaying the cached rays at commit.
This counts free space even when observed before a candidate's first hit. New voxels within
40 m remain when they have at least two hit votes and at least as many hits as misses.
The updating session stores submap-local masks and records `map_change_refined_visibility`
in the index.

In a same-recording diagnostic for S3, the first 58 s supplied candidate submaps and the
reference came from submaps captured after 64 s. With the same old-voxel rule and sampled
TLS, counting all misses reduced G from 0.192 to 0.181 and reduced reference coverage from
0.404 to 0.374. This time split uses archived poses and is a geometry diagnostic.

The initial prototype also used the same scan samples to score old voxels, with a four-to-one
free-space to hit threshold. The S1 → S3 → S6 and S6 → S3 updates were replayed at sensor rate
from the same pre-update stores and with the same launch-area startup as the persistence-filter
control.
Their saved submap pose arrays matched the control exactly. With the same development polygon
and all TLS points, G after S6 fell from 0.1533 to 0.1350. Absolute static coverage was 0.9180
and 0.9152, respectively. The S6 → S3 cross-layout N proxy fell from 0.109 to 0.091. Both
replays had zero trailing frames and averaged 73–87 ms per
scan. S6's post-replay processing and export took 55.5 s, compared with 19.4 s in the control
run. Each rollback restored the pre-update export byte for byte.

Recomposing the masks at the same poses isolated the two changes. Updating only the old-voxel
rule gave S6 G = 0.1544. Updating only the new-voxel rule gave G = 0.1337 and raised the S3
cross-layout N proxy from 0.0910 with both changes to 0.0982. This was the basis for keeping
the old removal policy and applying the refinement to new submaps.

The resulting implementation was replayed again at sensor rate on both updates. Its map
exports matched the full-TLS new-only reassembly byte for byte. S6 G fell from 15.33% to
13.37% and absolute TLS coverage fell from 91.80% to 91.45%. On S6 → S3 the cross-layout
N proxy was 9.82%, down from 10.89% with the persistence filter. Of 5,052 remaining S6 ghosts,
1,745 had a point within 0.06 m only in the old-map export, 1,612 only in the new-map export,
and 1,694 in both. One had no such neighbour. Both runs reported zero trailing frames and
56–64 ms of active wall time per scan. Post-replay processing and export took 31.6 s on
S3 and 56.9 s on S6. The map quality and commit cost remain open.

One offline plane-voting trial grouped neighbouring new points with aligned normals and
used multi-view hit and miss evidence to confirm or suppress each group. At the same poses
and with the same old-map points, it produced S6 G of 9.50% versus 9.42% for point voting.
The S3 cross-layout new-structure proxy was 9.41% for both. These maps were reconstructed
from saved vote geometry and scored with the full TLS cloud. The trial left the live
filter unchanged.

Recovering omitted points that agreed with a locally planar retained neighbourhood improved
the sampled development N proxy only from 0.121 to 0.125 at the widest tested radius. It
also increased G from 0.188 to 0.189. A separate filter that kept only well-supported planar
neighbourhoods lowered G to 0.120 at S = 0.973 with the sampled TLS. Stricter variants lost
more static structure. Both are offline probes and did not change the live filter.

### L5 evaluation tooling (2026-11-19)

- `scripts/evaluate_luh_trajectory.py` scores R and L against total-station poses and T from
  the live summary. It estimates the base-to-TLS transform once from the standalone base
  trajectory, matches the two localization paths at shared truth timestamps and records
  the transform in its report. Replay summaries include the first input stamp, active wall
  time per scan and process peak RSS. The T report lists the scan-to-pose count gap and the
  trailing-frame count. The initial LiDAR scan may initialize odometry without a pose.
  Published total-station timestamps sometimes repeat at 0.1 s resolution. Scoring uses the
  first sample for each repeated timestamp.
- `scripts/evaluate_luh_map_update.py` scores map exports with the full TLS cloud streamed
  in chunks. It takes the floor elevation, an explicit hall polygon, an update path in TLS
  coordinates and the fixed transform. N requires a separately supplied reference map and
  transform. A refusal log records an unavailable place-recognition baseline.
- `scripts/evaluate_luh_holdout.py` consumes a JSON manifest mapping each of the six
  preregistered update keys (such as `S0-S4-S7/S4`) to relative `map` and `motion` report
  paths. It applies the declared thresholds and reports the product or opt-in decision.
  A missing G comparison cannot demonstrate improvement over the baseline.

With the same approximate development polygon but the full TLS cloud, G after S6 was 0.153
with new-point filtering and 0.421 with old removal alone. Static coverage was 0.918 and
0.924, respectively. For S6 → S3, the S1 cross-layout N proxy was 0.109 versus 0.187. These
measurements exercise the evaluation path. The holdout measurement setup was subsequently
frozen below.

The full launch-area replays gave G = 0.153 after S6 and the same S6 → S3 development
N proxy of 0.109. This uses the same development polygon and full TLS cloud. No holdout
log or holdout ground truth was used in these trials.

### L5 holdout freeze (2026-11-19)

An S6 development replay with the two existing sessions profiled 55.77 s in commit,
including 39.75 s of new-voxel visibility replay and 7.23 s of export. One trial used
direct grid lookups for the same indexed voxels. It produced byte-identical stored
poses, evidence and map PCD, and took 53.47 s in the matching replay (4.1% less).
The original sparse lookup is used for holdout.

Use the existing `live` store path at sensor rate with `--start-near-first-submap`,
`--track-shared-surfaces` and `--refine-visibility` on every update. Start the base
session in an empty store. Use the known launch area within 5 m of the first stored
submap for the startup prior. Supply no pose from ground truth and do not retry
relocalization with a different prior. A refused update fails R. A successor that
requires an unaligned predecessor is recorded as blocked.

For every sequence, use `bench_report/data/long_term_2026-11/rko_lio_luh_os1.yaml`,
`lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml` and converter column step
2 with the published MD5. Use the full published empty-hall TLS cloud. Use the same
approximate hall polygon as development, with TLS XY vertices (11, 7.7), (51.5, 7.7),
(51.5, 14.4), (11, 14.4) and floor elevation −1.39 m. Apply the height band and the
10 m window around the update's total-station path from the preregistration.

Fit the store-to-TLS transform once from the base session's causal odometry. Use
standalone offline maps for the new-structure reference, with S1 for S0 and S3 for
S2 and S4. Compare each update with the existing `lidarslam-map merge --place-recognition
--engine core` on the same prefix of the chain. Record a baseline refusal as a
missing comparison. Keep the preregistered thresholds. The holdout total-station
trajectories had yet to be opened at freeze.

### L5 holdout results (2026-11-19)

| Chain | Update | G (%) | S (%) | N (%) | R (s) | L (cm, ratio) | T (ms/scan) | Offline merge |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| S0 → S4 → S7 | S4 | — | 117.80 | — | 1.30 | 10.54, 7.541 | 58.0 | Refused |
| S0 → S4 → S7 | S7 | 37.56 | 102.22 | — | 1.30 | 10.06, 6.898 | 58.4 | Refused |
| S7 → S4 → S0 | S4 | — | 100.32 | 21.80 | 1.30 | 1.52, 1.091 | 59.6 | Refused |
| S7 → S4 → S0 | S0 | — | 75.98 | — | 1.30 | 15.05, 1.334 | 49.6 | Refused |
| S2 → S5 | S5 | 11.56 | 100.44 | — | 1.20 | 1.84, 1.151 | 63.6 | Refused |
| S5 → S2 | S2 | — | 100.97 | 34.28 | 1.20 | 5.58, 1.040 | 64.0 | Refused |

All six updates passed R and T, with zero trailing frames dropped. The two
three-session stores each exported a map with three aligned sessions. Commit and
export took 23.9–80.1 s after replay.

Five updates passed S. In S7 → S4 → S0, absolute TLS coverage fell from 91.17% to
69.27% after S0. The three updates involving S0 as base or update exceeded both L
limits. S0's standalone RMSE was 11.28 cm.

G exceeded 2% in both updates to layout C. N was below 90% in both updates directly
from layout C to B. Offline place recognition refused all six comparisons. In each
case the accepted matches provided too little support for one transform across three
places spaced at least 5 m apart.

The six-report evaluator returned `opt-in`. L5 evaluation is complete. The
preregistered product-adoption rule keeps the mode experimental.

### S0 trajectory follow-up

The causal odometry used to create the S0 base store had 11.28 cm RMSE against
the total station after a rigid fit over 3,147 poses. The table compares each
update on the same matched timestamps with the frozen L5 store-to-TLS transform
and with an independent rigid fit of the saved update trajectory to TLS.

| Update | Matched poses | L5 RMSE (cm) | Independently fitted update (cm) | Independently fitted standalone (cm) |
| --- | ---: | ---: | ---: | ---: |
| S0 → S4 | 1,554 | 10.54 | 1.53 | 1.40 |
| S0 → S4 → S7 | 1,171 | 10.06 | 1.50 | 1.46 |
| S7 → S4 → S0 | 3,136 | 15.05 | 11.03 | 11.28 |

S4 and S7 retained close to standalone trajectory accuracy after a per-session
rigid fit. Most of their measured error is associated with placement in the fixed
frame established by the S0 base. For S0 as the update, about 11 cm of error was
still present after the independent fit, consistent with its standalone result.
The independent fits use ground truth for diagnosis. The L5 scores use the
original fixed transform.

### Review and roll back an experimental update

Copy the last accepted store while it is idle. Use fresh paths for the copy,
the reference export and the run output. The example assumes a known launch
area within 5 m of the first stored submap and uses the replay's sensor parameters.

```bash
accepted=/path/to/accepted-store
candidate=/path/to/candidate-store
before=/path/to/pre-update-export
bag=/path/to/update-bag
run=/path/to/update-run
odometry_params=/path/to/odometry.yaml
backend_params=/path/to/backend.yaml
python -m lidarslam store export "$accepted" --out "$before"
cp -a "$accepted" "$candidate"
python -m lidarslam live "$bag" --store "$candidate" --out "$run" \
  --odometry-params "$odometry_params" --backend-params "$backend_params" \
  --realtime --start-near-first-submap --track-shared-surfaces --refine-visibility
python -m lidarslam store info "$candidate"
```

Before using the candidate for the next session, check that its latest session
matches the replay's `store_session`, that `store_anchor_stamp_sec` is set, and
that `dropped_trailing_frames` is zero. Compare `$candidate/export/map.pcd` with
`$before/map.pcd` in a point-cloud viewer. Check alignment and inspect permanent
surfaces and expected changes in the area just surveyed. A session without an
anchor can appear in the store index while the export still shows the previous map.

When the latest update should be undone, confirm its ID with `store info` and run
the rollback once on the candidate. Run the export explicitly to cover the case
where the remaining sessions have no change evidence.

```bash
python -m lidarslam store rollback "$candidate"
python -m lidarslam store info "$candidate"
python -m lidarslam store export "$candidate"
cmp "$before/map.pcd" "$candidate/export/map.pcd"
```

Confirm that the committed session list and map match the pre-update state before
using the candidate again. Rollback removes the latest entry from the index and
keeps its session files. The three-session S7 → S4 → S0 store was tested on a copy.
Rolling back S0 and then S4 reproduced both earlier map PCDs byte for byte, with
all session file checksums intact.

## Milestones

Each milestone is one reviewable change with its own checks.

1. **L0, this document**, the data converter for LUH (Ouster + IMU, ROS 2 bag), and a first
   look at S1/S3/S6 with the existing single-run mapping (does each map alone; ATE against the
   total station).
2. **L1, ROS-free live mapping.** `lidarslam.live.Session` over `GraphSlamApplication` and the
   RKO-LIO driver, fed scan by scan; `lidarslam-map live <log> --realtime`. Check: with no store,
   the live run's trajectory equals the causal output of the offline backend on the same input,
   and real time holds on the development sequences.
3. **L2, the store.** Write a store from a live session; load it; export the Autoware bundle from
   it (byte-identical to the single-run bundle when the store has one session).
4. **L3, relocalization and the live multi-session graph.** Anchor a new session to the store,
   loops between new and stored submaps, priors on stored poses. Check: R and L on the
   development chains.
5. **L4, change detection.** Per-submap evidence, ray tracing, the four rules, tombstones,
   rollback. Check: G, S and N on the development chains; parameters frozen and recorded here.
6. **L5, holdout evaluation.** Holdout chains once, under the decision rule above; then the
   product decision.
7. **L6, product.** `lidarslam-map update` and `merge` on the store, the wizard's `--join-map`,
   docs; a ROS 2 node over the same adapter.

## Risks

- **Relocalization in a changed layout.** Layout A hides most of the hall; Scan Context may find
  no confident match against a layout C map (the join then refuses, by design). A caller-given
  start pose (the cart starts at the same spot in most sequences) is the fallback that is
  checked, not trusted.
- **Removing what is only out of view.** Thin structures (table legs, chair backs) are hit by few
  rays and crossed by many; the neighbour and grazing-angle rules exist for this, and S measures
  it.
- **One dataset.** All public data with a known changed layout and a reference scan found so far
  is this one hall. Static-structure checks on other multi-session data (no change expected) are
  added when it is downloaded again; they can only lower confidence, not replace the holdout.

## Agent review, development trial (preregistered 2026-11-19, before any measurement)

Question: does reviewing an update's candidates with the map review tools
(`docs/map-review.md`, `python -m lidarslam mcp`) improve the map over the automatic change
detection alone? Development data only (S1, S3, S6); no holdout data is used.

- **Stores.** Two chains, each a base session in an empty store and one update, both with
  `live --start-near-first-submap --detect-changes --filter-new-points` (walking backend preset,
  `rko_lio_luh_os1.yaml`, column step 2): S3 → S6 (update to layout C) and S6 → S3 (update to
  layout B).
- **Arms.** A: the store's export after the update, no edits. B: the same store after one
  agent review, committed as one `agent:claude` edit.
- **Reviewer.** Claude, through `lidarslam.map_review.Review` only (the MCP tools'
  implementation: `info`, `list_candidates`, `describe`, `view_region`, `decide`,
  `decide_box`, `commit`). It sees no TLS, total-station or other reference data and no score
  until its edit is committed. Budget: the 20 largest candidates of each kind (60 per chain),
  in list order; it may leave any undecided. It decides from the tool's numbers and pictures with
  the instructions of the MCP server (`lidarslam/mcp_server.py`, `INSTRUCTIONS`) as written at
  this commit.
- **Measures.** `scripts/evaluate_luh_map_update.py` with the full TLS cloud
  (`Reference_Map_TLS.pcd`, md5 `231eb6939447570e407ebad56029e151`), the L5 hall polygon and
  floor (−1.39 m), the update's path, and the store-to-TLS transform fitted once per chain
  from the base session's odometry against its total-station positions (Umeyama, rigid). G
  and S on both chains; N on S6 → S3 against S1 mapped alone (the development N proxy).
- **Rule.** Review helps on a chain if, against A, G falls by at least 0.02 (absolute) while S
  stays at least 0.97 and, on S6 → S3, N falls by no more than 0.02. Both chains are reported
  whatever the outcome, with the decisions and the time the review took.

### Agent review, development results (2026-11-19)

Both reviews were committed before any TLS or total-station data was read. The
store-to-TLS fits used 1,183 (S3) and 1,218 (S6) matched poses, with 3.7 cm and 1.6 cm RMSE;
the S1 reference fit 1,626 poses at 3.8 cm. The TLS md5 matched. "Before" is the base session
exported alone.

| Chain | Arm | G (%) | S (%) | N (%) | Map points in region |
| --- | --- | ---: | ---: | ---: | ---: |
| S3 → S6 | A, no edits | 14.45 | 100.71 | — | 34,463 |
| S3 → S6 | B, agent edit | 13.91 | 100.66 | — | 34,085 |
| S6 → S3 | A, no edits | 29.55 | 100.79 | 11.04 | 41,205 |
| S6 → S3 | B, agent edit | 27.52 | 100.81 | 9.55 | 39,851 |

Decisions (60 candidates per chain): S3 → S6 kept 13, removed 34, left 13 undecided
(16,801 voxels in the edit); S6 → S3 kept 19, removed 31, left 10 undecided (13,701 voxels).
Removing a `removed_old` or `suppressed_new` candidate only confirms the evidence; the map
changes come from keeps on those kinds and removes on `added`. The review time was not
recorded.

Under the rule:

- **S3 → S6: does not help.** G fell by 0.0054, below 0.02. S held.
- **S6 → S3: helps, at the margin.** G fell by 0.0203. S held at 100.81%. N fell by 0.0149,
  within 0.02.

Both arms leave G far above the 2% L5 target, and N on S6 → S3 is about 10% in both: the
automatic update misses most new structure, and review, which only sees what the update
produced, cannot add it back. The review removed some points that the S1 reference counts as
new structure. One chain at the margin on development data is not enough to change the
product. A further check would need a fresh public holdout, since the LUH holdout ground
truth is open.
