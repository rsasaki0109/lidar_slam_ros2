# README media

## Live map building (`lidarslam/images/slam_koide_outdoor_hard_02a.gif`)

**Data:** [Hard Point Cloud Localization Dataset](https://zenodo.org/records/10122133) (Koide et al., CC BY 4.0), sequence `outdoor_hard_02a`.
- Handheld Livox MID-360, 363 s.
- About 554 m of ground-truth path in the evaluated window.

The dataset publishes IMU acceleration in g, and RKO-LIO expects m/s². The first step therefore rescales `/livox/imu` and copies `/livox/points` unchanged.

**Run:**

```bash
python3 tools/readme_media/scale_imu_bag.py <dataset>/sequences/outdoor_hard_02a outdoor_hard_02a_scaled
bash scripts/run_rko_lio_graph_benchmark.sh \
  --bag outdoor_hard_02a_scaled --lidar-topic /livox/points --imu-topic /livox/imu \
  --base-frame livox_frame \
  --rko-param lidarslam/param/rko_lio_mid360_handheld_outdoor.yaml \
  --lidarslam-param lidarslam/param/lidarslam.yaml \
  --reference-tum gt_02a.tum --reference-meta ref_meta.json --skip-reference-gen \
  --publish-static-tf false --quiescence-secs 60 --save-timeout-secs 420 \
  --output-dir run_02a
```

- `gt_02a.tum` is the dataset ground truth for this sequence in TUM format. `ref_meta.json` is `{}`.
- RKO-LIO uses `lidarslam/param/rko_lio_mid360_handheld_outdoor.yaml`. Its two changes from the earlier outdoor settings (`voxel_size` 0.5, gravity alignment over a 40 s window) were **selected on this sequence**. They were then checked unchanged on `outdoor_hard_02b` and `01b`. Three later changes were checked on all sequences: `max_scan_delta_sec` of 3 s fixes a heading failure on `01b` and leaves this sequence unchanged, re-levelling the local map lowers vertical error on all three, and `min_icp_keypoints: 100` skips occluded scans (held-out `01a` 1.56 → 0.72 m, this sequence 0.53 → 0.55 m). See [the profile study](research/koide-handheld-outdoor-profile-2026-10.md).
- `scanmatcher` and `graph_based_slam` were built from `acffebf`. Its C++ sources, launch files and parameters are identical to `develop` at `1c1bc322`; only two Python tests differ.
- RKO-LIO was built from the pinned `e5f731f`.

**Result** (`metrics.json`, Umeyama SE(3) alignment):

| Metric | Value |
| --- | --- |
| Scans tracked | 2870 / 2880 (99.7%) |
| Raw APE RMSE / max | 0.55 m / 1.42 m (previous outdoor settings: 1.03 m / 2.08 m) |
| Corrected APE RMSE | 0.58 m |
| Loop closures | 0 |
| Real-time factor | 0.44 (shared workstation) |
| Autoware map check | 8 PASS / 0 FAIL |

**Render:**

```bash
python3 tools/readme_media/render_lidar_demo.py --mode slam --overview \
  --bag outdoor_hard_02a_scaled --estimate run_02a/traj_corrected.tum \
  --out frames/x.gif --frames 300 --zcut 4.0 --title '...' --caption '...'
ffmpeg -framerate 15 -i frames/frame_%04d.png \
  -vf "split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
  slam_koide_outdoor_hard_02a.gif
```

- Each scan is placed at the corrected SLAM pose of its timestamp and added to a 0.25 m voxel map. Points are coloured by the time they were mapped, and the current scan is drawn in magenta.
- Points more than 4 m above the sensor at insertion time are hidden, so canopy does not cover the ground.
- The ground truth is used only for the metrics above. It is not drawn.

## Stairs and a metro tunnel (GEODE)

Files: `lidarslam/images/slam_geode_stairs_beta.gif` and `lidarslam/images/slam_geode_tunnel4_beta.gif`.

**Data:** [GEODE](https://github.com/PengYu-Team/GEODE_dataset) (Chen et al., "Heterogeneous LiDAR Dataset for Benchmarking Robust Localization in Diverse Degenerate Scenarios", IJRR 2025, [arXiv:2409.04961](https://arxiv.org/abs/2409.04961)), device β: Ouster OS1-64 and Xsens MTi-30. The dataset pages state no licence.
- `stairs_beta`: handheld, 331 s, 300 m. The walk goes down one stairwell (21 m), along the lower level and up a second stairwell.
- Metro Tunneling `tunnel4_beta`: UGV, 232 s, 208 m.

**Run:** the RKO-LIO frontend alone, as in the [GEODE note](research/geode-hard-evaluation-2026-10.md#reproduction):

```bash
python3 scripts/convert_geode_bag.py stairs_beta/stairs_beta.bag stairs_beta/ros2
python3 -c "import sys, yaml; print(yaml.safe_dump({'/**': {'ros__parameters': yaml.safe_load(open(sys.argv[1]))}}))" \
  configs/geode/rko_lio_beta_v18.yaml > beta_v18.ros.yaml
ros2 run rko_lio offline_node --ros-args --params-file beta_v18.ros.yaml \
  -p bag_path:=stairs_beta/ros2 -p imu_topic:=/imu/data -p lidar_topic:=/ouster/points \
  -p base_frame:=base_link -p dump_results:=true -p results_dir:=run_stairs -p run_name:=stairs_beta
```

- `tunnel4_beta` uses `configs/geode/rko_lio_beta_ugv_v19.yaml`. Its trajectory is the run from [the sealed half](research/geode-hard-evaluation-2026-10.md#the-sealed-half-results-2026-10-15), not a new run.
- RKO-LIO was built from the pinned `80f3c1c`.
- `offline_node` does not exit at the end of the bag. Stop it with SIGINT once its CPU time stops moving.

**Result** (`scripts/score_position_only_trajectory.py`, SE(3) alignment):

| | `stairs_beta` | `tunnel4_beta` |
| --- | ---: | ---: |
| Configuration | handheld v18 | UGV v19 |
| ATE RMSE | 0.24 m | 0.38 m |
| RTE over 10 m | 1.0 % | 2.0 % |
| Held out | no: v18 was developed on this sequence | yes |
| Rivals (ATE) | BIEVR-LIO 0.23 m, FAST-LIO2 0.35 m, Point-LIO 0.29 m | BIEVR-LIO, FAST-LIO2 and Point-LIO 0.38 m, COIN-LIO 1.62 m |

**Render:**

```bash
EXT="-0.006892256666756416 -0.002318844449334277 -0.025993963174297075 0.9996356504431766 -0.027172 -0.034873 0.062643"
python3 tools/readme_media/render_lidar_demo.py --mode slam --overview \
  --bag stairs_beta/ros2 --topic /ouster/points --estimate run_stairs/stairs_beta_0/stairs_beta_tum_0.txt \
  --lidar-to-base $EXT --scan-stride 8 --out stairs/x.gif --frames 300 --width 640 --height 360 \
  --zcut 1.0 --title '...' --caption '...'
python3 tools/readme_media/render_lidar_demo.py --mode slam \
  --bag Tunneling_tunnel4_beta/ros2 --topic /ouster/points \
  --estimate run_tunnel4/Tunneling_tunnel4_beta_0/Tunneling_tunnel4_beta_tum_0.txt \
  --lidar-to-base $EXT --scan-stride 8 --out tunnel/x.gif --frames 300 --width 640 --height 360 \
  --zcut 0.8 --chase-dist 22 --chase-height 15 --title '...' --caption '...'
ffmpeg -framerate 15 -i stairs/frame_%04d.png \
  -vf "select='not(mod(n\,2))',setpts=N/(10*TB),split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
  -r 10 slam_geode_stairs_beta.gif
```

- The tunnel GIF is encoded the same way with `max_colors=128`. Every second frame is kept at 10 fps, which keeps the stairs GIF near the size of the one above.
- The poses are odometry only: no loop closure or graph optimization.
- The estimate is the pose of the base frame (the IMU), so each scan is placed through the LiDAR-to-base extrinsic of the configuration (`--lidar-to-base`, in the order of `extrinsic_lidar2base_quat_xyzw_xyz`).
- The current scan is drawn with every eighth point (`--scan-stride 8`). Drawn in full, the OS1-64's scan hides the map.
- Points more than 1.0 m (stairs) or 0.8 m (tunnel) above the sensor at insertion time are hidden, so ceilings do not cover the walk.
- The ground truth is used only for the metrics above. It is not drawn.

## Three walks joined into one map (`docs/assets/images/multisession_oxford_new_college.gif`)

**Data:** [Oxford Spires](https://huggingface.co/datasets/ori-drs/oxford_spires_dataset) (CC BY-NC-SA 4.0), sequences `2024-07-09-new-college-03`, `-04` and `-01`.
- Handheld rig: Hesai LiDAR (`/hesai/pandar`) and Alphasense IMU.
- Only the LiDAR clouds and the IMU file were fetched. No ground truth was fetched or used.
- The GIF is made from the dataset, so it carries the dataset's licence (CC BY-NC-SA 4.0), not the repository's BSD-2.

**Run:** each walk alone, then the join. These are bench scripts with the development machine's paths (`/home/user/...`):

```bash
for s in 03 04 01; do
  WORKERS=12 DROP_BAG=1 bash bench_report/scripts/p3_spires_session.sh 2024-07-09-new-college-$s nc$s
done
cd /home/user/bench/spires_ms
bash /home/user/lidar_slam_ros2/bench_report/scripts/p3_spires_join.sh join_nc03_nc04_nc01 nc03 nc04 nc01
```

- `p3_spires_session.sh` converts the sequence to a ROS 2 bag (`bench_report/scripts/oxford_to_ros2.py`). It then runs the RKO-LIO frontend with the Phase 1 configuration and records its odometry and deskewed scans (thinned to 0.1 m) as the backend input. Last, it runs `graph_slam_offline_runner` with `lidarslam/param/lidarslam_mid360_rko_graph_walking.yaml`.
- `p3_spires_join.sh` runs [`graph_slam_multisession_join`](architecture/multisession-join.md) with the same parameter file. Walk 03 is the reference, and each walk joins the map built so far. The tool is given nothing about where the walks start relative to each other.

**Result** (`bench_report/data/oxford/multisession_gif_2026-10-31/new_college_join_report.yaml`):

| | 03 | 04 | 01 |
| --- | ---: | ---: | ---: |
| Length (odometry) | 799 m | 368 m | 221 m |
| Submaps | 512 | 236 | 142 |
| Joined | reference | first try, to 03 at 87 places | first try, to 03 and 04 at 123 places |
| Route within 5 m of another walk | 43 % | 82 % | 100 % |

- No set was ambiguous and no walk joined itself. Either one would have made the tool refuse the join.
- Walk 01 follows routes that 03 and 04 also walk. 04 adds about 70 m of route that the other walks do not cover.
- New College 02 was tried first. Only 22 of its 474 candidate matches with 03 and 04 were accepted, too few for a set, so it was not joined. Other sites are in report 38.10.

**Render:**

```bash
python3 bench_report/scripts/p3_multisession_glow_gif.py join_nc03_nc04_nc01/join \
  "Three separate walks through New College, Oxford" nc_glow_en3 \
  "New College 03=nc03" "New College 04=nc04" "New College 01=nc01"
```

- Points are every fifth deskewed scan of each walk's backend input, from 1.1 m below to 16 m above the sensor, thinned to 0.2 m. Each walk has its own colour.
- Each walk is first drawn in its own odometry frame, painted along its route. The walks are set apart only for display.
- The walks then move onto the map in the order the tool joined them. The lines show the matched places. At the end, the joined map is shown in 3-D.
- The GIF is 720 px, 15 fps and 128 colours (4.4 MB). The script also writes a 960×540 MP4.
- The captions are in English (`GIF_LANG=ja` gives Japanese).
