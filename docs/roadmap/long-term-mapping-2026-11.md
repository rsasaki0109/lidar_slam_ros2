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
