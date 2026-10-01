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
  --rko-param tools/readme_media/rko_lio_koide_outdoor.yaml \
  --lidarslam-param lidarslam/param/lidarslam.yaml \
  --reference-tum gt_02a.tum --reference-meta ref_meta.json --skip-reference-gen \
  --publish-static-tf false --quiescence-secs 60 --save-timeout-secs 420 \
  --output-dir run_02a
```

- `gt_02a.tum` is the dataset ground truth for this sequence in TUM format. `ref_meta.json` is `{}`.
- The RKO-LIO settings are the outdoor settings of an earlier full-dataset completion sweep. No parameter was tuned on this run.
- `scanmatcher` and `graph_based_slam` were built from `acffebf`. Its C++ sources, launch files and parameters are identical to `develop` at `1c1bc322`; only two Python tests differ.
- RKO-LIO was built from `a8a7ac6`. It differs from the pinned `d4eb79f` only in the node shutdown-callback lifetime fix.

**Result** (`metrics.json`, Umeyama SE(3) alignment):

| Metric | Value |
| --- | --- |
| Scans tracked | 2870 / 2880 (99.7%) |
| Raw APE RMSE / max | 1.03 m / 2.08 m |
| Corrected APE RMSE | 1.04 m |
| Loop closures | 0 |
| Real-time factor | 0.75 (shared workstation, load average ~19) |
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
